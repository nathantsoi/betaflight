F405_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH
#ifdef OBF4_REV1
FEATURES       += SDCARD
#else
FEATURES       += ONBOARDFLASH
#endif

TARGET_SRC = \
            drivers/accgyro_spi_mpu6000.c \
            drivers/barometer_bmp280.c \
            drivers/barometer_spi_bmp280.c \
            drivers/compass_hmc5883l.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f4xx.c \
            drivers/max7456.c \
            io/osd.c

