#!/bin/sh

config-pin P9_30 pruout
config-pin P8_11 pruout
config-pin P8_12 pruout

config-pin P9_25 qep
config-pin P9_27 qep
config-pin P9_41 in+
config-pin P9_91 qep
config-pin P9_42 in+
config-pin P9_92 qep

echo 0 > /sys/devices/virtual/graphics/fbcon/cursor_blink

/home/debian/lathe/lathe
