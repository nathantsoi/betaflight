# 1-wire passthrough esc programming

Currently supported on CC3D. More boards coming soon.

## Configuration

The following parameters can be used to enable and configure this in the related target.h file:

    USE_SERIAL_1WIRE              Enables the 1wire code, defined in target.h
    SERIAL_1WIRE_MOTOR_COUNT      Total motor count, defined in target.h

## Usage

  - Start the board normally and connect to the CLI, then use the `1wire` command followed with the ESC you wish to program.

    ie. to connect to the esc on your flight controller's port #2:

    ```
    1wire 2
    ```

  - Close the CleanFlight configurator

  - Open the BlHeli Suite

  - Pick the 1-wire programmer interface, the COM port of your Flight Control Board and BAUD: 115200

  - Use BlHeli suite as normal

  - Turn off the flight controller and start over for the next ESC

## Implementing

  - For new targets

    - in `target.h`

        ```
        #define USE_SERIAL_1WIRE
        #define SERIAL_1WIRE_MOTOR_COUNT 6
        ```

    - in `serial_1wire.h` define `serialOut` from `timer.c`

## Notes

  - Programming each ESC indepently provides more flexibility than an all-at-once approach, programming all at once should be possible with a little modification
