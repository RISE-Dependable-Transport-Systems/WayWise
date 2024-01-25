#!/bin/bash

# Read start potision
high_byte=$(i2cget -y 1 0x36 0x01)
low_byte=$(i2cget -y 1 0x36 0x02)
result=$(( (high_byte << 8) | low_byte ))
echo "Start poistion is $result"


# Read stop potision
high_byte=$(i2cget -y 1 0x36 0x03)
low_byte=$(i2cget -y 1 0x36 0x04)
result=$(( (high_byte << 8) | low_byte ))
echo "Stop poistion is $result"
