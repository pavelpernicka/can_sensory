#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_conf.h"
#include "bl_config.h"
#include "flash_if.h"
#include "bl_can.h"

CAN_HandleTypeDef hcan1;

static int bl_led_inited = 0;
static uint8_t bl_led_toggle_remaining = 0;
static uint32_t bl_led_toggle_interval_ms = 0;
static uint32_t bl_led_next_toggle_ms = 0;

static void SystemClock_Config(void);
static void Error_Handler(void);

volatile uint32_t bl_tick_ms = 0;

void SysTick_Handler(void)
{
    HAL_IncTick();
    bl_tick_ms++;
}

static uint32_t bl_millis(void)
{
    return bl_tick_ms;
}

static void BL_EnsureMetaDeviceId(void)
{
    bl_meta_t meta = {0};

    if (!Flash_IsAppValid(&meta)) {
        return;
    }

    if (BL_META_RESERVED_HAS_DEVICE_ID(meta.reserved)) {
        uint8_t id = BL_META_RESERVED_GET_DEVICE_ID(meta.reserved);
        if (id == (uint8_t)BL_DEVICE_ID) {
            return;
        }
    }

    meta.reserved = BL_META_RESERVED_ENCODE_DEVICE_ID(BL_DEVICE_ID);
    (void)Flash_WriteMeta(&meta);
}

static int BL_ConsumeStayMagic(void)
{
    volatile uint32_t *magic = (volatile uint32_t *)BL_STAY_MAGIC_ADDR;
    if (*magic == BL_STAY_MAGIC_VALUE) {
        *magic = 0U;
        return 1;
    }
    return 0;
}

/* LED utils */
static void BL_LedInit(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = BL_LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BL_LED_GPIO_PORT, &gpio);
    bl_led_inited = 1;
}

static void BL_LedSet(int on)
{
    GPIO_PinState st = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
#if BL_LED_ACTIVE_LOW
    st = (st == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif
    HAL_GPIO_WritePin(BL_LED_GPIO_PORT, BL_LED_PIN, st);
}

static void BL_LedToggle(void)
{
    HAL_GPIO_TogglePin(BL_LED_GPIO_PORT, BL_LED_PIN);
}

static void BL_BlinkBlocking(uint32_t count, uint32_t delay_ms)
{
    for (uint32_t i = 0; i < count; ++i) {
        HAL_Delay(delay_ms);
        BL_LedToggle();
        HAL_Delay(delay_ms);
        BL_LedToggle();
    }
    BL_LedSet(0);
}

static void BL_QueueBlink(uint32_t count, uint32_t delay_ms, int force)
{
    uint8_t toggles = (uint8_t)(count * 2U);
    if (toggles == 0U) {
        return;
    }
    if (!force && bl_led_toggle_remaining != 0U) {
        return;
    }

    bl_led_toggle_remaining = toggles;
    bl_led_toggle_interval_ms = delay_ms;
    bl_led_next_toggle_ms = bl_millis();
}

static void BL_ServiceLedQueue(void)
{
    if (bl_led_toggle_remaining == 0U) {
        return;
    }

    uint32_t now = bl_millis();
    if (now < bl_led_next_toggle_ms) {
        return;
    }

    BL_LedToggle();
    bl_led_toggle_remaining--;
    bl_led_next_toggle_ms = now + bl_led_toggle_interval_ms;

    if (bl_led_toggle_remaining == 0U) {
        BL_LedSet(0);
    }
}

static void BL_HandleCanEvents(uint32_t events)
{
    if ((events & BL_CAN_EVENT_PING) != 0U) {
        BL_QueueBlink(BL_PING_BLINK_COUNT, BL_PING_BLINK_DELAY_MS, 1);
    } else if ((events & BL_CAN_EVENT_ACTIVITY) != 0U) {
        BL_QueueBlink(BL_ACTIVITY_BLINK_COUNT, BL_ACTIVITY_BLINK_DELAY_MS, 0);
    }
}

static void BL_CanStandbyInit(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = BL_CAN_S_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BL_CAN_S_GPIO_PORT, &gpio);

    HAL_GPIO_WritePin(BL_CAN_S_GPIO_PORT, BL_CAN_S_PIN, BL_CAN_S_NORMAL_LEVEL);
}

// jumping to application
typedef void (*pFunction)(void);

static int BL_IsValidRamAddress(uint32_t addr)
{
    if (addr >= SRAM1_BASE && addr <= (SRAM1_BASE + SRAM1_SIZE_MAX)) {
        return 1;
    }
    if (addr >= SRAM2_BASE && addr <= (SRAM2_BASE + 0x4000UL)) {
        return 1;
    }
    return 0;
}

static uint8_t JumpToApplication(void)
{
    uint32_t appStack = *(uint32_t *)APP_FLASH_START;
    uint32_t appEntry = *(uint32_t *)(APP_FLASH_START + 4U);
    uint32_t irq_words = ((uint32_t)sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]));

    if (!Flash_IsAppValid(0)) {
        return (uint8_t)BL_BOOTERR_APP_INVALID;
    }
    if (appStack == 0xFFFFFFFFU || appEntry == 0xFFFFFFFFU) {
        return (uint8_t)BL_BOOTERR_VECTOR_EMPTY;
    }

    // Basic vector-table sanity checks before handoff.
    if ((appStack & 0x3U) != 0U) {
        return (uint8_t)BL_BOOTERR_STACK_ALIGN;
    }
    if (!BL_IsValidRamAddress(appStack)) {
        return (uint8_t)BL_BOOTERR_STACK_RANGE;
    }
    if ((appEntry & 0x1U) == 0U || appEntry < APP_FLASH_START || appEntry >= APP_FLASH_END) {
        return (uint8_t)BL_BOOTERR_ENTRY_RANGE;
    }

    BL_BlinkBlocking(BL_JUMP_BLINK_COUNT, BL_JUMP_BLINK_DELAY_MS);

    __disable_irq();

    HAL_CAN_DeInit(&hcan1);
    HAL_RCC_DeInit();
    HAL_DeInit();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // Disable and clear all NVIC IRQs before switching vector table
    for (uint32_t i = 0; i < irq_words; ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }

    __set_MSP(appStack);
    SCB->VTOR = APP_FLASH_START;
    __DSB();
    __ISB();

    pFunction appReset = (pFunction)appEntry;
    __enable_irq();
    appReset();

    return (uint8_t)BL_BOOTERR_RETURNED;
}

int main(void)
{
    int stay_magic = 0;

    HAL_Init();
    BL_LedInit();
    SystemClock_Config();
    BL_LedSet(1);
    BL_BlinkBlocking(BL_START_BLINK_COUNT, BL_START_BLINK_DELAY_MS);
    BL_CanStandbyInit();

    Flash_Init();
    BL_EnsureMetaDeviceId();
    stay_magic = BL_ConsumeStayMagic();

    BL_CAN_Init(&hcan1);
    if (stay_magic) {
        BL_CAN_ForceStayInBootloader();
    }

    // if valid FW, wait before jumping
    if (Flash_IsAppValid(0)) {
        uint32_t start = bl_millis();
        while (bl_millis() - start < BL_AUTORUN_WAIT_MS) {
            uint32_t events = BL_CAN_Poll();
            BL_HandleCanEvents(events);
            BL_ServiceLedQueue();

            // stay after can event
            if ((events & BL_CAN_EVENT_ACTIVITY) != 0U) {
                BL_CAN_ForceStayInBootloader();
            }

            if (BL_CAN_StayInBootloaderRequested())
                break;
        }

        if (!BL_CAN_StayInBootloaderRequested() && !BL_FORCE_STAY_IN_BOOTLOADER) {
            uint8_t boot_err = JumpToApplication();
            if (boot_err != (uint8_t)BL_BOOTERR_NONE) {
                BL_CAN_SetLastBootError(boot_err);
            }
        }
    }

    // bootloader loop
    while (1) {
        uint32_t events = BL_CAN_Poll();
        BL_HandleCanEvents(events);
        BL_ServiceLedQueue();

        if (BL_BootToAppRequest) {
            uint8_t boot_err;
            BL_BootToAppRequest = 0;
            boot_err = JumpToApplication();
            if (boot_err != (uint8_t)BL_BOOTERR_NONE) {
                BL_CAN_ReportBootError(boot_err);
            }
        }
    }
}

// clock: HSI16 as SYSCLK, no PLL (16 MHz)
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

static void Error_Handler(void)
{
    if (!bl_led_inited) {
        BL_LedInit();
    }
    while (1) {
        BL_LedToggle();
        for (volatile uint32_t i = 0; i < 100000; ++i) ;
    }
}
