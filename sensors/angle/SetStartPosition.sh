#!/bin/bash

# Read high and low bytes for start position
high_byte=$(i2cget -y 1 0x36 0x0C)
low_byte=$(i2cget -y 1 0x36 0x0D)

# Set start position using i2cset (ZPOS Register)
i2cset -y 1 0x36 0x01 $high_byte
if [ $? -eq 0 ]; then
    echo "High byte set successfully"
else
    echo "Error setting high byte. Exiting."
    exit 1
fi

i2cset -y 1 0x36 0x02 $low_byte
if [ $? -eq 0 ]; then
    echo "Low byte set successfully."
else
    echo "Error setting low byte. Exiting."
    exit 1
fi

echo "Start position set successfully."