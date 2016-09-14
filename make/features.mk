ifneq ($(filter ONBOARDFLASH,$(FEATURES)),)
TARGET_SRC += \
            drivers/flash_m25p16.c \
            io/flashfs.c
endif

ifneq ($(filter SDCARD,$(FEATURES)),)
TARGET_SRC += \
            drivers/sdcard.c \
            drivers/sdcard_standard.c \
            io/asyncfatfs/asyncfatfs.c \
            io/asyncfatfs/fat_standard.c
endif

ifneq ($(filter VCP,$(FEATURES)),)

# F1 VCP specific includes, non exlusive
ifeq ($(TARGET),$(filter $(TARGET), $(F1_TARGETS)))
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(USBFS_DIR)/inc \
                   $(ROOT)/src/main/vcp

VPATH           := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC) \
                        $(USBPERIPH_SRC)
endif


# F4 VCP specifics sources, exclusive
ifeq ($(TARGET),$(filter $(TARGET), $(F4_TARGETS)))
VCP_SRC = \
            vcpf4/stm32f4xx_it.c \
            vcpf4/usb_bsp.c \
            vcpf4/usbd_desc.c \
            vcpf4/usbd_usr.c \
            vcpf4/usbd_cdc_vcp.c \
            drivers/serial_usb_vcp.c
# F1, F3 VCP sources, exclusive of F4
else
VCP_SRC = \
            vcp/hw_config.c \
            vcp/stm32_it.c \
            vcp/usb_desc.c \
            vcp/usb_endp.c \
            vcp/usb_istr.c \
            vcp/usb_prop.c \
            vcp/usb_pwr.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c
endif
TARGET_SRC += $(VCP_SRC)
endif
