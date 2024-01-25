# READ RAW ANGLE
high_byte=$(i2cget -y 1 0x36 0x0C)
low_byte=$(i2cget -y 1 0x36 0x0D)
result=$(( (high_byte << 8) | low_byte ))
echo $result

#READ AND SET START POSITION STEP3
high_byte=$(i2cget -y 1 0x36 0x0C)
low_byte=$(i2cget -y 1 0x36 0x0D)
i2cset -y 1 0x36 0x01 $high_byte
i2cset -y 1 0x36 0x02 $low_byte

# READ AND WRITE STOP POSITION
high_byte=$(i2cget -y 1 0x36 0x0C)
low_byte=$(i2cget -y 1 0x36 0x0D)
i2cset -y 1 0x36 0x03 $high_byte
i2cset -y 1 0x36 0x04 $low_byte


#READ STOP POSITION
high_byte=$(i2cget -y 1 0x36 0x03)
low_byte=$(i2cget -y 1 0x36 0x04)
result=$(( (high_byte << 8) | low_byte ))
echo $result

i2cget -y 1 0x36 0x0B



# READ  START POSITION
high_byte=$(i2cget -y 1 0x36 0x01)
low_byte=$(i2cget -y 1 0x36 0x02)
result=$(( (high_byte << 8) | low_byte ))
echo $result



# READ RAW ANGLE
high_byte=$(i2cget -y 1 0x36 0x0C)
low_byte=$(i2cget -y 1 0x36 0x0D)
result=$(( (high_byte << 8) | low_byte ))
echo $result




#READ ANGLE
high_byte=$(i2cget -y 1 0x36 0x0E)
low_byte=$(i2cget -y 1 0x36 0x0F)
result=$(( (high_byte << 8) | low_byte ))
echo $result