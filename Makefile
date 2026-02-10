# STM32L432KBUx

# Toolchain
CC      := arm-none-eabi-gcc
AS      := arm-none-eabi-gcc
LD      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

# Pathes
CUBE_DIR   := third_party/STM32CubeL4
SRCDIR     := src
INCDIR     := inc
LINKERDIR  := linker

CMSIS_CORE_INC := $(CUBE_DIR)/Drivers/CMSIS/Include
CMSIS_DEV_INC  := $(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32L4xx/Include
HAL_INC        := $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Inc

# Output
TARGET  := bootloader
ELF     := $(TARGET).elf
BIN     := $(TARGET).bin

# Sources

# This project
SRCS = \
  $(SRCDIR)/main.c \
  $(SRCDIR)/bl_can.c \
  $(SRCDIR)/flash_if.c \
  $(SRCDIR)/crc32.c \
  $(SRCDIR)/stm32l4xx_hal_msp.c

# HAL and CMSIS
SRCS += \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
  $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
  $(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/system_stm32l4xx.c

# Startup
ASMS = \
  $(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/startup_stm32l432xx.s

OBJS = $(SRCS:.c=.o) $(ASMS:.s=.o)

# Flags
MCU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=soft

CDEFS   := -DSTM32L432xx

CINCS   := -I$(INCDIR) \
           -I$(CMSIS_CORE_INC) \
           -I$(CMSIS_DEV_INC) \
           -I$(HAL_INC)

CFLAGS  := $(MCU) $(CDEFS) $(CINCS) \
           -O2 -g3 -Wall -ffunction-sections -fdata-sections

LDFLAGS := $(MCU) -specs=nosys.specs -Wl,--gc-sections \
           -T$(LINKERDIR)/stm32l432kbu_boot.ld

# Targets
all: $(BIN)

$(ELF): $(OBJS)
	$(LD) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(AS) $(MCU) -c $< -o $@

clean:
	rm -f $(OBJS) $(ELF) $(BIN)

flash_st: $(BIN)
	st-flash write $(BIN) 0x08000000

flash_openocd: $(BIN)
	openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
	        -c "program $(BIN) verify reset exit 0x08000000"

.PHONY: all clean flash flash_openocd
