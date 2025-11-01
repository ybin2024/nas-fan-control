| Example Target | ESP32-H2 |
| -------------- | -------- |

# About

(See the README.md file in the upper level 'examples' directory for more information about examples.)

A fan controller for hard disks on your DIY NAS base on ESP32H2. By adjusting PWM according to the current consumption of 12V as well as temperature from NTC sensor to keep quiet.

The project is built ESP-IDF Example of ADC oneshot_read.

# Features

* Fan diagnostic on start-up, report rota start threshold PWM
* 12V Hard disks current sencing from ADC
* Hard disks TEMP sencing(NTC) from ADC
* Hard envirenment TEMP sencing(ESP temp sensor)
* current consumption and temp data fusion
* ESP sleep managment enable

# How to use example

See circuit upload from PNG later on.

## Hardware Required

* ESP32-H2 SupperMini development board
* AD8418 current sensing
* Pegboard for integrated components and DIY circuit

In this example, you need to connect a voltage source (e.g. a DC power supply) to the GPIO pins specified in `oneshot_read_main.c` (see the macros defined on the top of the source file). Feel free to modify the pin setting.


### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

Running this example, you will see the following log output on the serial monitor:

```
I (6338516) FAN_CONTROL: Room-Temp: 35.1℃, Cell-Temp: 39.6℃,Current: 34mA
I (6338516) FAN_CONTROL: T->duty: 0.22, I->duty: 0.03, =>Merger: 0.24
I (6340516) FAN_CONTROL: Room-Temp: 35.1℃, Cell-Temp: 39.7℃,Current: 33mA
I (6340516) FAN_CONTROL: T->duty: 0.23, I->duty: 0.03, =>Merger: 0.24
I (6340516) FAN_CONTROL: DUTY_INV: 75%, Duty_PWM: 24%, Fan => START
I (6342516) FAN_CONTROL: Room-Temp: 35.1℃, Cell-Temp: 39.7℃,Current: 42mA
I (6342516) FAN_CONTROL: T->duty: 0.23, I->duty: 0.04, =>Merger: 0.25
I (6342516) FAN_CONTROL: DUTY_INV: 75%, Duty_PWM: 24%, RPM=1200
I (6344516) FAN_CONTROL: Room-Temp: 35.1℃, Cell-Temp: 39.7℃,Current: 48mA
I (6344516) FAN_CONTROL: T->duty: 0.23, I->duty: 0.05, =>Merger: 0.25
I (6344516) FAN_CONTROL: DUTY_INV: 75%, Duty_PWM: 24%, RPM=2140
I (6346516) FAN_CONTROL: Room-Temp: 35.1℃, Cell-Temp: 39.7℃,Current: 54mA
I (6346516) FAN_CONTROL: T->duty: 0.23, I->duty: 0.05, =>Merger: 0.25
I (6346516) FAN_CONTROL: DUTY_INV: 74%, Duty_PWM: 25%, RPM=1920
...
```

## Troubleshooting

If following warning is printed out, it means the calibration required eFuse bits are not burnt correctly on your board. The calibration will be skipped. Only raw data will be printed out.
```
W (300) ADC_ONESHOT: eFuse not burnt, skip calibration
I (1310) ADC_ONESHOT: ADC1 Channel[2] Raw Data: 0
```# nas-fan-control
# nas-fan-control
