#include "flash_if.h"

void Flash_Init(void)
{
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_FLASH_Lock();
}

HAL_StatusTypeDef Flash_EraseAppArea(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t pageError = 0;

    uint32_t firstPage = (APP_FLASH_START - FLASH_START_ADDR) / FLASH_PAGE_SIZE;
    uint32_t lastPage  = (APP_FLASH_END   - FLASH_START_ADDR) / FLASH_PAGE_SIZE - 1U;
    uint32_t nPages    = lastPage - firstPage + 1U;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page      = firstPage;
    erase.NbPages   = nPages;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &pageError);
    HAL_FLASH_Lock();

    return st;
}

HAL_StatusTypeDef Flash_ProgramBytes(uint32_t addr, const uint8_t *data, uint32_t len)
{
    HAL_StatusTypeDef st = HAL_OK;
    uint8_t buf[8];

    HAL_FLASH_Unlock();

    while (len > 0 && st == HAL_OK) {
        uint32_t chunk = (len >= 8) ? 8 : len;

        for (uint32_t i = 0; i < 8; ++i) {
            buf[i] = (i < chunk) ? data[i] : 0xFF;
        }

        uint64_t dw =
            ((uint64_t)buf[0])        |
            ((uint64_t)buf[1] << 8)   |
            ((uint64_t)buf[2] << 16)  |
            ((uint64_t)buf[3] << 24)  |
            ((uint64_t)buf[4] << 32)  |
            ((uint64_t)buf[5] << 40)  |
            ((uint64_t)buf[6] << 48)  |
            ((uint64_t)buf[7] << 56);

        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, dw);
        addr += 8;
        data += chunk;
        len  -= chunk;
    }

    HAL_FLASH_Lock();
    return st;
}

void Flash_ReadMeta(bl_meta_t *meta)
{
    const bl_meta_t *src = (const bl_meta_t *)BL_META_ADDR;
    *meta = *src;
}

HAL_StatusTypeDef Flash_WriteMeta(const bl_meta_t *meta)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t pageError = 0;
    uint32_t page = (BL_META_ADDR - FLASH_START_ADDR) / FLASH_PAGE_SIZE;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page      = page;
    erase.NbPages   = 1;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &pageError);
    if (st != HAL_OK) {
        HAL_FLASH_Lock();
        return st;
    }

    const uint64_t *p = (const uint64_t *)meta;
    uint32_t addr = BL_META_ADDR;

    for (uint32_t i = 0; i < sizeof(bl_meta_t)/8; ++i) {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, p[i]);
        if (st != HAL_OK)
            break;
        addr += 8;
    }

    HAL_FLASH_Lock();
    return st;
}

uint32_t Flash_ComputeAppCrc(uint32_t size)
{
    const uint8_t *p = (const uint8_t *)APP_FLASH_START;
    return crc32_compute(p, size);
}

int Flash_IsAppValid(bl_meta_t *meta_out)
{
    bl_meta_t meta;
    Flash_ReadMeta(&meta);

    if (meta.magic != BL_META_MAGIC) {
        return 0;
    }
    if (meta.size == 0 || meta.size > APP_MAX_SIZE) {
        return 0;
    }

    if (Flash_ComputeAppCrc(meta.size) != meta.crc32) {
        return 0;
    }

    if (meta_out != 0) {
        *meta_out = meta;
    }
    return 1;
}
