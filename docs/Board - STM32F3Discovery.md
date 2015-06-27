# Board - STM32F3Discovery

A powerful development board with enough flash to compile CleanFlight with debug flags, a built in STLink debugger, gyro, VCP USB Port and UART1 USB

Flying this board is untested, but it is theoretically possible. It's great for debugging.

# Pinouts

Check `timer.c` for details.

## Inputs

| Pin | Function  | Notes                            |
| --- | --------- | -------------------------------- |
| PA8 | PWM1      |                                  |
| PB8 | PWM2      | |
| PB9 | PWM3      | |
| PC6 | PWM4      | |
| PC7 | PWM5      | |
| PC8 | PWM6      | |
| PB1 | PWM7      | |
| PA2 | PWM8      | |

### Serial 1Wire passthrough

PC1: RX -- Connect to UART TX
PC3: TX -- Connect to UART RX


## Outputs

| Pin  | Function  | Notes                            |
| ---- | --------- | |
| PD12 | PWM9      | |
| PD13 | PWM10     | |
| PD14 | PWM11     | |
| PD15 | PWM12     | |
| PA1  | PWM13     | |
| PA2  | PWM14     | |
