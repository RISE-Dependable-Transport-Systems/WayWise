#!/bin/bash


#Read Scaled angle
high_byte=$(i2cget -y 1 0x36 0x0E)
low_byte=$(i2cget -y 1 0x36 0x0F)
result=$(( (high_byte << 8) | low_byte ))
echo "Scaled angle is  $result"


# Read raw angle
high_byte=$(i2cget -y 1 0x36 0x0C)
low_byte=$(i2cget -y 1 0x36 0x0D)
result=$(( (high_byte << 8) | low_byte ))
echo "Raw angle is  $result "

