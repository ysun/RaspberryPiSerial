#!/bin/bash

sed  -i -e s/#define\ AXIS.*/#define\ AXIS\ 0/ firmware.ino
make ARDUINO_PORT=/dev/ArduinoX upload

sed  -i -e s/#define\ AXIS.*/#define\ AXIS\ 1/ firmware.ino
make ARDUINO_PORT=/dev/ArduinoY upload

sed  -i -e s/#define\ AXIS.*/#define\ AXIS\ 2/ firmware.ino
make ARDUINO_PORT=/dev/ArduinoZ upload

sed  -i -e s/#define\ AXIS.*/#define\ AXIS\ 3/ firmware.ino
make ARDUINO_PORT=/dev/ArduinoW upload
