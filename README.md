# LT BMS

LT BMS is a Battery Management System (BMS) based on Linear Technology's (LT's) LTC6804 Multicell Battery Monitor Chips and the Arduino Uno (or Linduino One) microcontroller board. This repository includes:

- Two C++ classes for controlling stacks of addressed LTC6804-2 chips or DC9143C demonstration boards connected on a multidrop isoSPI bus. These include functionality to write configuration registers, read battery cell voltages and status registers, and control General-Purpose Input-Output pins on each chip (e.g. to interface thermistors).
- Several Arduino sketches that demonstrate the functionality of the LTC6804 and DC9143C classes.
- An Ardiono sketch that controls a Manzanita Micro PFC charger based on cell voltage readings from DC9143C demonstration boards.

## Instalation

1. Download the [Arduino IDE](https://www.arduino.cc/en/Main/Software).
2. Clone this repository onto your machine.
3. Either change the Arduino Sketchbook location to the cloned repository (`lt_bms` directory) or add the folders under `libraries` to your Arduino libraries folder.
