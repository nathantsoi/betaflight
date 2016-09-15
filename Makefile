###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the betaflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
#
###############################################################################


# Things that the user might override on the commandline
#

# The target to build, see VALID_TARGETS below
TARGET    ?= NAZE

# Compile-time options
OPTIONS   ?=

# compile for OpenPilot BootLoader support
OPBL      ?= no

# Debugger optons, must be empty or GDB
DEBUG     ?=

# Insert the debugging hardfault debugger
# releases should not be built with this flag as it does not disable pwm output
DEBUG_HARDFAULTS ?=

# Serial port/Device for flashing
SERIAL_DEVICE   ?= $(firstword $(wildcard /dev/ttyUSB*) no-port-found)

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=

## Set verbosity level based on the V= parameter
## V=0 Low
## v=1 High
export AT := @

ifndef V
export V0    :=
export V1    := $(AT)
else ifeq ($(V), 0)
export V0    := $(AT)
export V1    := $(AT)
else ifeq ($(V), 1)
endif

###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME      = betaflight

# Working directories
ROOT            := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR         = $(ROOT)/src/main
OBJECT_DIR      = $(ROOT)/obj/main
BIN_DIR         = $(ROOT)/obj
CMSIS_DIR       = $(ROOT)/lib/main/CMSIS
INCLUDE_DIRS    = $(SRC_DIR) \
                  $(ROOT)/src/main/target
LINKER_DIR      = $(ROOT)/src/main/target

## Build tools, so we all share the same versions
# import macros common to all supported build systems
include $(ROOT)/make/system-id.mk
# developer preferences, edit these at will, they'll be gitignored
include $(ROOT)/make/local.mk

# configure some directories that are relative to wherever ROOT_DIR is located
TOOLS_DIR := $(ROOT)/tools
BUILD_DIR := $(ROOT)/build
DL_DIR := $(ROOT)/downloads

export RM := rm

# import macros that are OS specific
include $(ROOT)/make/$(OSFAMILY).mk

# include the tools makefile
include $(ROOT)/make/tools.mk

# default xtal value for F4 targets
HSE_VALUE       = 8000000

# used for turning on features like VCP and SDCARD
FEATURES        =

ALT_TARGETS     = $(sort $(filter-out target, $(basename $(notdir $(wildcard $(ROOT)/src/main/target/*/*.mk)))))
OPBL_TARGETS    = $(filter %_OPBL, $(ALT_TARGETS))

#VALID_TARGETS  = $(F1_TARGETS) $(F3_TARGETS) $(F4_TARGETS)
VALID_TARGETS   = $(dir $(wildcard $(ROOT)/src/main/target/*/target.mk))
VALID_TARGETS  := $(subst /,, $(subst ./src/main/target/,, $(VALID_TARGETS)))
VALID_TARGETS  := $(VALID_TARGETS) $(ALT_TARGETS) $(DESKTOP_TARGETS)
VALID_TARGETS  := $(sort $(VALID_TARGETS))

ifeq ($(filter $(TARGET),$(ALT_TARGETS)), $(TARGET))
BASE_TARGET    := $(firstword $(subst /,, $(subst ./src/main/target/,, $(dir $(wildcard $(ROOT)/src/main/target/*/$(TARGET).mk)))))
-include $(ROOT)/src/main/target/$(BASE_TARGET)/$(TARGET).mk
else
BASE_TARGET    := $(TARGET)
endif

ifeq ($(filter $(TARGET),$(OPBL_TARGETS)), $(TARGET))
OPBL            = yes
endif

# silently ignore if the file is not present. Allows for target specific.
-include $(ROOT)/src/main/target/$(BASE_TARGET)/target.mk

F4_TARGETS      = $(F405_TARGETS) $(F411_TARGETS)

ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS). Have you prepared a valid target.mk?)
endif


# Calculate Version
REVISION = $(shell git log -1 --format="%h")

FC_VER_MAJOR := $(shell grep " FC_VERSION_MAJOR" src/main/build/version.h | awk '{print $$3}' )
FC_VER_MINOR := $(shell grep " FC_VERSION_MINOR" src/main/build/version.h | awk '{print $$3}' )
FC_VER_PATCH := $(shell grep " FC_VERSION_PATCH" src/main/build/version.h | awk '{print $$3}' )

FC_VER := $(FC_VER_MAJOR).$(FC_VER_MINOR).$(FC_VER_PATCH)


# Source
TARGET_DIR     = $(ROOT)/src/main/target/$(BASE_TARGET)
TARGET_DIR_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_DIR)

VPATH           := $(VPATH):$(TARGET_DIR):$(SRC_DIR)

BASE_SRC = \
            build/build_config.c \
            build/debug.c \
            build/version.c \
            $(TARGET_DIR_SRC) \
            main.c \
            fc/mw.c \
            common/encoding.c \
            common/filter.c \
            common/maths.c \
            common/printf.c \
            common/streambuf.c \
            common/typeconversion.c \
            config/config.c \
            config/config_eeprom.c \
            config/feature.c

COMMON_SRC = \
            $(BASE_SRC) \
            fc/runtime_config.c \
            drivers/adc.c \
            drivers/buf_writer.c \
            drivers/bus_i2c_soft.c \
            drivers/bus_spi.c \
            drivers/exti.c \
            drivers/gyro_sync.c \
            drivers/io.c \
            drivers/light_led.c \
            drivers/pwm_mapping.c \
            drivers/pwm_output.c \
            drivers/pwm_rx.c \
            drivers/rcc.c \
            drivers/serial.c \
            drivers/serial_uart.c \
            drivers/sound_beeper.c \
            drivers/system.c \
            drivers/timer.c \
            flight/altitudehold.c \
            flight/failsafe.c \
            flight/imu.c \
            flight/mixer.c \
            flight/pid.c \
            flight/pid_legacy.c \
            flight/pid_betaflight.c \
            io/beeper.c \
            fc/rc_controls.c \
            fc/rc_curves.c \
            io/serial.c \
            io/serial_4way.c \
            io/serial_4way_avrootloader.c \
            io/serial_4way_stk500v2.c \
            io/serial_cli.c \
            io/serial_msp.c \
            io/statusindicator.c \
            rx/ibus.c \
            rx/jetiexbus.c \
            rx/msp.c \
            rx/pwm.c \
            rx/rx.c \
            rx/sbus.c \
            rx/spektrum.c \
            rx/sumd.c \
            rx/sumh.c \
            rx/xbus.c \
            scheduler/scheduler.c \
            scheduler/scheduler_tasks.c \
            sensors/acceleration.c \
            sensors/battery.c \
            sensors/boardalignment.c \
            sensors/compass.c \
            sensors/gyro.c \
            sensors/initialisation.c

HIGHEND_SRC = \
            blackbox/blackbox.c \
            blackbox/blackbox_io.c \
            common/colorconversion.c \
            drivers/display_ug2864hsweg01.c \
            flight/gtune.c \
            flight/navigation.c \
            flight/gps_conversion.c \
            io/gps.c \
            io/ledstrip.c \
            io/display.c \
            sensors/sonar.c \
            sensors/barometer.c \
            telemetry/telemetry.c \
            telemetry/frsky.c \
            telemetry/hott.c \
            telemetry/smartport.c \
            telemetry/ltm.c


# Feature source
include make/features.mk


# Platform / target specific includes
ifeq ($(TARGET), DESKTOP)
PLATFORM = DESKTOP
include make/platforms/desktop.mk
else ifeq ($(filter $(TARGET),$(F1_TARGETS) $(F3_TARGETS) $(F4_TARGETS)), $(TARGET))
PLATFORM = STM32
include make/platforms/stm32.mk
else
$(error Target '$(TARGET)' has not specified a valid DESKTOP or STM group, must be one of DESKTOP, F1, F3, F405, or F411. Have you prepared a valid target.mk?)
endif

TARGET_SRC += $(COMMON_SRC)

# Target / device flags
ifneq ($(BASE_TARGET), $(TARGET))
TARGET_FLAGS  := $(TARGET_FLAGS) -D$(BASE_TARGET)
endif

ifneq ($(FLASH_SIZE),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)
endif

ifneq ($(HSE_VALUE),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DHSE_VALUE=$(HSE_VALUE)
endif


###############################################################################
# Things that might need changing to use different tools
#

include make/toolchain.mk

###############################################################################
# No user-serviceable parts below
###############################################################################

CPPCHECK        = cppcheck $(CSOURCES) --enable=all --platform=unix64 \
                  --std=c99 --inline-suppr --quiet --force \
                  $(addprefix -I,$(INCLUDE_DIRS)) \
                  -I/usr/include -I/usr/include/linux

#
# Things we will build
#
TARGET_BIN      = $(BIN_DIR)/$(FORKNAME)_$(FC_VER)_$(TARGET).bin
TARGET_HEX      = $(BIN_DIR)/$(FORKNAME)_$(FC_VER)_$(TARGET).hex
TARGET_ELF      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).elf
TARGET_OBJS     = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(TARGET_SRC))))
TARGET_DEPS     = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(TARGET_SRC))))
TARGET_MAP      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

CLEAN_ARTIFACTS := $(TARGET_BIN)
CLEAN_ARTIFACTS += $(TARGET_HEX)
CLEAN_ARTIFACTS += $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(V0) $(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	$(V0) $(OBJCOPY) -O binary $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(V1) echo LD $(notdir $@)
	$(V1) $(CC) -o $@ $^ $(LDFLAGS)
	$(V0) $(SIZE) $(TARGET_ELF)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) echo %% $(notdir $<)
	$(V1) $(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	$(V1) mkdir -p $(dir $@)
	$(V1) echo %% $(notdir $<)
	$(V1) $(CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	$(V1) mkdir -p $(dir $@)
	$(V1) echo %% $(notdir $<)
	$(V1) $(CC) -c -o $@ $(ASFLAGS) $<


## all               : Build all valid targets
all: $(VALID_TARGETS)

$(VALID_TARGETS):
		$(V0) echo "" && \
		echo "Building $@" && \
		$(MAKE) binary hex TARGET=$@ && \
		echo "Building $@ succeeded."



CLEAN_TARGETS = $(addprefix clean_,$(VALID_TARGETS) )
TARGETS_CLEAN = $(addsuffix _clean,$(VALID_TARGETS) )

## clean             : clean up temporary / machine-generated files
clean:
	$(V0) echo "Cleaning $(TARGET)"
	$(V0) rm -f $(CLEAN_ARTIFACTS)
	$(V0) rm -rf $(OBJECT_DIR)/$(TARGET)
	$(V0) echo "Cleaning $(TARGET) succeeded."

## clean_test        : clean up temporary / machine-generated files (tests)
clean_test:
	$(V0) cd src/test && $(MAKE) clean || true

## clean_<TARGET>    : clean up one specific target
$(CLEAN_TARGETS) :
	$(V0) $(MAKE) -j TARGET=$(subst clean_,,$@) clean

## <TARGET>_clean    : clean up one specific target (alias for above)
$(TARGETS_CLEAN) :
	$(V0) $(MAKE) -j TARGET=$(subst _clean,,$@) clean

## clean_all         : clean all valid targets
clean_all:$(CLEAN_TARGETS)

## all_clean         : clean all valid targets (alias for above)
all_clean:$(TARGETS_CLEAN)


flash_$(TARGET): $(TARGET_HEX)
	$(V0) stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	$(V0) echo -n 'R' >$(SERIAL_DEVICE)
	$(V0) stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## flash             : flash firmware (.hex) onto flight controller
flash: flash_$(TARGET)

st-flash_$(TARGET): $(TARGET_BIN)
	$(V0) st-flash --reset write $< 0x08000000

## st-flash          : flash firmware (.bin) onto flight controller
st-flash: st-flash_$(TARGET)

binary:
	$(V0) $(MAKE) -j $(TARGET_BIN)

hex:
	$(V0) $(MAKE) -j $(TARGET_HEX)

unbrick_$(TARGET): $(TARGET_HEX)
	$(V0) stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	$(V0) stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## unbrick           : unbrick flight controller
unbrick: unbrick_$(TARGET)

## cppcheck          : run static analysis on C source code
cppcheck: $(CSOURCES)
	$(V0) $(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	$(V0) $(CPPCHECK) --xml-version=2 2> cppcheck-result.xml

## mkdirs
$(DL_DIR):
	mkdir -p $@

$(TOOLS_DIR):
	mkdir -p $@

$(BUILD_DIR):
	mkdir -p $@

## help              : print this help message and exit
help: Makefile
	$(V0) @echo ""
	$(V0) @echo "Makefile for the $(FORKNAME) firmware"
	$(V0) @echo ""
	$(V0) @echo "Usage:"
	$(V0) @echo "        make [V=<verbosity>] [TARGET=<target>] [OPTIONS=\"<options>\"]"
	$(V0) @echo "Or:"
	$(V0) @echo "        make <target> [V=<verbosity>] [OPTIONS=\"<options>\"]"
	$(V0) @echo ""
	$(V0) @echo "Valid TARGET values are: $(VALID_TARGETS)"
	$(V0) @echo ""
	$(V0) @sed -n 's/^## //p' $<

## targets           : print a list of all valid target platforms (for consumption by scripts)
targets:
	$(V0) @echo "Valid targets: $(VALID_TARGETS)"
	$(V0) @echo "Target:        $(TARGET)"
	$(V0) @echo "Base target:   $(BASE_TARGET)"

## test              : run the cleanflight test suite
test:
	$(V0) cd src/test && $(MAKE) test || true

# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# include auto-generated dependencies
-include $(TARGET_DEPS)
