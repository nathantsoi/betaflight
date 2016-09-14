STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F10x_StdPeriph_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = stm32f10x_crc.c \
                  stm32f10x_cec.c \
                  stm32f10x_can.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

# Search path and source files for the CMSIS sources
VPATH           := $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC       = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
                  $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/CM3/CoreSupport \
                   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k.ld
ARCH_FLAGS      = -mthumb -mcpu=cortex-m3

ifeq ($(DEVICE_FLAGS),)
DEVICE_FLAGS    = -DSTM32F10X_MD
endif
DEVICE_FLAGS   += -DSTM32F10X

TARGET_SRC := \
            startup_stm32f10x_md_gcc.S \
            drivers/adc_stm32f10x.c \
            drivers/bus_i2c_stm32f10x.c \
            drivers/dma.c \
            drivers/gpio_stm32f10x.c \
            drivers/inverter.c \
            drivers/serial_softserial.c \
            drivers/serial_uart_stm32f10x.c \
            drivers/system_stm32f10x.c \
            drivers/timer_stm32f10x.c \
						$(TARGET_SRC)
