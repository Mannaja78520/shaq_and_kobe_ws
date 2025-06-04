#!/usr/bin/env python3


import smbus2
import time

# I2C bus number (usually 1 on Raspberry Pi)
bus = smbus2.SMBus(1)

# I2C address of the UPS
UPS_ADDRESS = 0x36

def read_voltage():
    raw = bus.read_word_data(UPS_ADDRESS, 0x02)
    # Swap bytes because the data is little endian
    raw = ((raw & 0xFF) << 8) | (raw >> 8)
    voltage = raw * 1.25 / 1000 / 16  # Convert to volts
    return voltage

def read_capacity():
    raw = bus.read_word_data(UPS_ADDRESS, 0x04)
    raw = ((raw & 0xFF) << 8) | (raw >> 8)
    return raw / 256  # Convert to %

while True:
    voltage = read_voltage()
    capacity = read_capacity()
    print(f"Voltage: {voltage:.2f} V, Capacity: {capacity:.2f} %")
    time.sleep(2)
