CC      := arm-none-eabi-gcc
AS      := arm-none-eabi-gcc
LD      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

CUBE_DIR   := ../third_party/STM32CubeL4
SRCDIR     := src
INCDIR     := inc
LINKERDIR  := linker

CMSIS_CORE_INC := $(CUBE_DIR)/Drivers/CMSIS/Include
CMSIS_DEV_INC  := $(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32L4xx/Include
HAL_INC        := $(CUBE_DIR)/Drivers/STM32L4xx_HAL_Driver/Inc

TARGET  := app_firmware
ELF     := $(TARGET).elf
BIN     := $(TARGET).bin

SRCS = \
  $(SRCDIR)/main.c \
  $(SRCDIR)/app_can.c \
  $(SRCDIR)/calibration.c \
  $(SRCDIR)/sensor_i2c.c \
  $(SRCDIR)/sensor_hmc5883l.c \
  $(SRCDIR)/sensor_lis3dhtr.c \
  $(SRCDIR)/sensor_aht20.c \
  $(SRCDIR)/sensors.c \
  $(SRCDIR)/ws2812.c \
  $(SRCDIR)/event_detector.c \
  $(SRCDIR)/events.c \
  $(SRCDIR)/stm32l4xx_hal_msp.c

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

ASMS = \
  $(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/startup_stm32l432xx.s

OBJS = $(SRCS:.c=.o) $(ASMS:.s=.o)

MCU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=soft
CDEFS   := -DSTM32L432xx
CINCS   := -I$(INCDIR) -I$(CMSIS_CORE_INC) -I$(CMSIS_DEV_INC) -I$(HAL_INC)

CFLAGS  := $(MCU) $(CDEFS) $(CINCS) -O2 -g3 -Wall -ffunction-sections -fdata-sections
LDFLAGS := $(MCU) -specs=nosys.specs -Wl,--gc-sections -T$(LINKERDIR)/stm32l432kbu_app.ld -lm

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

flash_openocd: $(BIN)
	openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
		-c "program $(BIN) verify reset exit 0x08004000"

.PHONY: all clean flash_openocd
