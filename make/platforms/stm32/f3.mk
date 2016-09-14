STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F30x_StdPeriph_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = stm32f30x_crc.c \
                  stm32f30x_can.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))
DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

VPATH           := $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC       = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
                  $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/CM1/CoreSupport \
                   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x

ifneq ($(filter VCP, $(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(USBFS_DIR)/inc \
                   $(ROOT)/src/main/vcp

VPATH           := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC)\
                        $(USBPERIPH_SRC)
endif

ifneq ($(filter SDCARD, $(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(FATFS_DIR) \

VPATH           := $(VPATH):$(FATFS_DIR)
endif

LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS    = -DSTM32F303xC -DSTM32F303


# Hardfault debugging startup script
ifeq ($(DEBUG_HARDFAULTS),F3)
CFLAGS              += -DDEBUG_HARDFAULTS
STARTUP_SCRIPT_SRC  = startup_stm32f3_debug_hardfault_handler.S
else
STARTUP_SCRIPT_SRC  = startup_stm32f30x_md_gcc.S
endif


TARGET_SRC := $(STARTUP_SCRIPT_SRC) \
            target/system_stm32f30x.c \
            drivers/adc_stm32f30x.c \
            drivers/bus_i2c_stm32f30x.c \
            drivers/dma.c \
            drivers/gpio_stm32f30x.c \
            drivers/light_ws2811strip_stm32f30x.c \
            drivers/serial_uart_stm32f30x.c \
            drivers/system_stm32f30x.c \
            drivers/timer_stm32f30x.c \
						$(TARGET_SRC)

