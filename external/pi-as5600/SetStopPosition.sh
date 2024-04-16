#!/bin/bash

# Read high and low bytes for stop position
high_byte=$(i2cget -y 1 0x36 0x0C)
low_byte=$(i2cget -y 1 0x36 0x0D)

# Set stop position using i2cset (MPOS Register)
i2cset -y 1 0x36 0x03 $high_byte
if [ $? -eq 0 ]; then
    echo "High byte set successfully."
else
    echo "Error setting high byte. Exiting."
    exit 1
fi

i2cset -y 1 0x36 0x04 $low_byte
if [ $? -eq 0 ]; then
    echo "Low byte set successfully."
else
    echo "Error setting low byte. Exiting."
    exit 1
fi

echo "Stop position set successfully."