# STM32 build info
128K_TARGETS  = $(F1_TARGETS)
256K_TARGETS  = $(F3_TARGETS)
512K_TARGETS  = $(F411_TARGETS)
1024K_TARGETS = $(F405_TARGETS)

# Configure default flash sizes for the targets (largest size specified gets hit first) if flash not specified already.
ifeq ($(FLASH_SIZE),)
ifeq ($(TARGET),$(filter $(TARGET),$(1024K_TARGETS)))
FLASH_SIZE = 1024
else ifeq ($(TARGET),$(filter $(TARGET),$(512K_TARGETS)))
FLASH_SIZE = 512
else ifeq ($(TARGET),$(filter $(TARGET),$(256K_TARGETS)))
FLASH_SIZE = 256
else ifeq ($(TARGET),$(filter $(TARGET),$(128K_TARGETS)))
FLASH_SIZE = 128
else
$(error FLASH_SIZE not configured for target $(TARGET))
endif
endif


# Search path for sources
VPATH           := $(VPATH):$(SRC_DIR)/startup
USBFS_DIR       = $(ROOT)/lib/main/STM32_USB-FS-Device_Driver
USBPERIPH_SRC   = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))
FATFS_DIR       = $(ROOT)/lib/main/FatFS
FATFS_SRC       = $(notdir $(wildcard $(FATFS_DIR)/*.c))


# OPBL
ifeq ($(OPBL),yes)
TARGET_FLAGS := -DOPBL $(TARGET_FLAGS)
ifeq ($(TARGET), $(filter $(TARGET),$(F405_TARGETS)))
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f405_opbl.ld
else ifeq ($(TARGET), $(filter $(TARGET),$(F411_TARGETS)))
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f411_opbl.ld
else ifeq ($(TARGET), $(filter $(TARGET),$(F3_TARGETS)))
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k_opbl.ld
else ifeq ($(TARGET), $(filter $(TARGET),$(F1_TARGETS)))
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k_opbl.ld
endif
.DEFAULT_GOAL := binary
else
.DEFAULT_GOAL := hex
endif


# Target specific
# F1 targets
ifeq ($(TARGET),$(filter $(TARGET),$(F1_TARGETS)))
include make/platforms/stm32/f1.mk
# F3 targets
else ifeq ($(TARGET),$(filter $(TARGET), $(F3_TARGETS)))
include make/platforms/stm32/f3.mk
# F4 targets
else ifeq ($(TARGET),$(filter $(TARGET), $(F4_TARGETS)))
include make/platforms/stm32/f4.mk
# Unknown target
else
$(error STM32 Target '$(TARGET)' has not specified a valid STM32 group, must be one of '$(F1_TARGETS), $(F3_TARGETS), $(F4_TARGETS)')
endif


# Assign target src
ifeq ($(TARGET),$(filter $(TARGET),$(F4_TARGETS) $(F3_TARGETS)))
TARGET_SRC += $(HIGHEND_SRC)
else ifneq ($(filter HIGHEND,$(FEATURES)),)
TARGET_SRC += $(HIGHEND_SRC)
endif

TARGET_SRC += $(CMSIS_SRC) \
							$(DEVICE_STDPERIPH_SRC)


# Search path and source files for the ST stdperiph library
VPATH        := $(VPATH):$(STDPERIPH_DIR)/src
