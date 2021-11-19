# Manages I2C between scripts on SCUTTLE w/ Jetson
# Created Nov 10 2021 - Carson F

import smbus2

i2c_bus = smbus2.SMBus(0)

def get_i2c():                           # Returns the i2c_bus
    return i2c_bus

def close():
    i2c_bus.close()