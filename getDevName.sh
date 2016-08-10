#!/bin/bash
#set -x

count_dev=`ls -l /dev/ |grep ttyUSB |grep -v Arduino |wc | awk '{print $1}'`

if [[ $count_dev == 1 ]]; then
	echo ArduinoX
elif [[ $count_dev == 2 ]]; then
	echo ArduinoY
elif [[ $count_dev == 3 ]]; then
	echo ArduinoZ
fi
