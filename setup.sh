#!/bin/sh

export SLOTS=/sys/devices/platform/bone_capemgr/slots

config-pin overlay cape-universaln 

config-pin P9_31 pruout
config-pin P8_11 pruout
config-pin P8_12 pruout

config-pin P9_22 spi
config-pin P9_21 spi
config-pin P9_18 spi
config-pin P9_17 spi

config-pin P9_25 qep
config-pin P9_27 qep
config-pin P9_41 in+
config-pin P9_91 qep
config-pin P9_42 in+
config-pin P9_92 qep

echo 0 > /sys/devices/virtual/graphics/fbcon/cursor_blink

cat $SLOTS

/home/debian/lathe/lathe
