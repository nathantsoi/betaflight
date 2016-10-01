#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "desktop.h"

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "blackbox/blackbox_io.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/io.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/max7456.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/escservo.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "io/ledstrip.h"
#include "io/gps.h"
#include "io/osd.h"
#include "io/vtx.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"

#include "fc/runtime_config.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"


// Implementations should go in their own files probably
void failureMode(failureMode_e mode) {
  printf("Failure Mode %d, exiting", mode);
  exit(-1);
}


// MOCKS BELOW THIS LINE
//
pwmOutputConfiguration_t pwmOutputConfiguration = {
  .servoCount = 0,
  .motorCount = 0
};

void systemInit(void) { }

// used by light_led
void IOInitGlobal() { }
void IOConfigGPIO(IO_t io, ioConfig_t cfg) { }
void IOInit(IO_t io, resourceOwner_t owner, resourceType_t resource, uint8_t index) { }
void IOToggle(IO_t io) { }
void IOWrite(IO_t io, bool value) { }

// e.g. battery read
uint16_t adcGetChannel(uint8_t channel) {
  return 0;
}

// time
void delay(uint32_t i) { }
void delayMicroseconds(uint32_t i) { }
void timerStart(void) { }
void timerInit(void) { }
uint32_t micros(void) { return 0; }
uint32_t millis(void) { return 0; }

// dma
void dmaInit(void) { }

// cli
uint8_t cliMode = 0;
void cliInit(serialConfig_t *serialConfig) { }

// config / eeprom
void initEEPROM(void) { }
void writeEEPROM(void) { }
bool isEEPROMContentValid(void) {
  return true;
}
//void readEEPROM(void) { }
// MOCK!
void readEEPROM() {
  resetConf();
  activateConfig();
/*
  flight3DConfig_t flight3DConfigToUse;
  escAndServoConfig_t escAndServoConfigToUse;
  mixerConfig_t mixerConfigToUse;
  airplaneConfig_t airplaneConfigToUse;
  rxConfig_t rxConfigToUse;

  memset(&flight3DConfigToUse, 0, sizeof(flight3DConfigToUse));
  memset(&escAndServoConfigToUse, 0, sizeof(escAndServoConfigToUse));
  memset(&mixerConfigToUse, 0, sizeof(mixerConfigToUse));
  memset(&rxConfigToUse, 0, sizeof(rxConfigToUse));

  mixerUseConfigs(
    &flight3DConfigToUse,
    &escAndServoConfigToUse,
    &mixerConfigToUse,
    &airplaneConfigToUse,
    &rxConfigToUse
  );
  */
}

// serial
const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT];
serialPort_t *findNextSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction) { }
serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function) { }
serialPort_t *findSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction) { } 
void handleSerial(void) { }
bool isSerialConfigValid(serialConfig_t *serialConfig) {
  return true;
}
void mspSerialReleasePortIfAllocated(serialPort_t *serialPort) { }
serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e function, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, portOptions_t options) { }
void serialInit(serialConfig_t *initialSerialConfig, bool softserialEnabled, serialPortIdentifier_e serialPortToDisable) { }


// pwm / rx
bool isPWMDataBeingReceived(void) { return false; }
bool isPPMDataBeingReceived(void) { return false; }
uint16_t ppmRead(uint8_t channel) { return 0; }
void pwmCompleteOneshotMotorUpdate(uint8_t motorCount) { }
pwmOutputConfiguration_t *pwmInit(drv_pwm_config_t *init) {
  return &pwmOutputConfiguration;
}
void pwmRxInit(inputFilteringMode_e initialInputFilteringMode) { }
void pwmShutdownPulsesForAllMotors(uint8_t motorCount) { }
void pwmWriteMotor(uint8_t index, uint16_t value) { }
void pwmWriteServo(uint8_t index, uint16_t value) { }
void resetPPMDataReceivedState(void) { }
uint16_t pwmRead(uint8_t channel) {
  return 0;
}

