# 1-wire passthrough esc programming

Currently supported on the STM32F3DISCOVERY, NAZE (and variants) and CC3D.

The general idea is to connect a UART/USB converter to your computer and the UART pins defined in `target.h`, and a ground of course, to program your ESCs in place.

It should be possible on boards with built in UART/USB converters (and maybe VCPs) to avoid using the external UART adapter.

## Configuration

The following parameters can be used to enable and configure this in the related target.h file:

    USE_SERIAL_1WIRE              Enables the 1wire code, defined in target.h

## Usage

  - Start the board normally and connect to the CLI, then use the `1wire` command followed with the ESC you wish to program.

    ie. to connect to the esc on your flight controller's port #2:

    ```
    1wire 2
    ```

    TODO: `1wire 0` to write all escs at once, read from esc 1

  - Attach your 1-wire or 4w programmer (made via the BlHeli suite) to the pin specified in the board documentation.

    ie. CC3D boards should have the PWM output (motor out) pin 6 attached to the 1-wire line from the Arduino and the ground line from the Arduino attached to any ground pin on the CC3D

  - Open the BlHeli Suite

  - Pick the BlHeli Bootloader (USB/Com) option

  - Use BlHeli suite as normal

## Implementing

  - For new targets

    - in `target.h`

        ```
        // Turn on serial 1wire passthrough
        #define USE_SERIAL_1WIRE
        // How many escs does this board support?
        #define ESC_COUNT 6
        // STM32F3DISCOVERY TX - PC3 connects to UART RX
        #define S1W_TX_GPIO         GPIOC
        #define S1W_TX_PIN          GPIO_Pin_3
        #define S1W_TX_PERIPH       RCC_AHBPeriph_GPIOC
        // STM32F3DISCOVERY RX - PC1 connects to UART TX
        #define S1W_RX_GPIO         GPIOC
        #define S1W_RX_PIN          GPIO_Pin_1
        #define S1W_RX_PERIPH       RCC_AHBPeriph_GPIOC
        ```

    - in `serial_1wire.c`

       ```
       // Define your esc hardware
       const escHardware_t escHardware[ESC_COUNT] = {
           { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_12 },
           { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_13 },
           { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_14 },
           { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_15 },
           { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_1 },
           { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_2 }
       };
       ```

       - it might be possible to pull this from the timer hardware, but I couldn't find the periphs...
