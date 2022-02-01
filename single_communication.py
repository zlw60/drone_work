import sys
import smbus
import time
bus = smbus.SMBus(1)

x_arduino = 0x04

# Convert a string in bytes
def ConvertStringsToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted

def main():
    # Send data to the Arduino
    value = 17
    bus.write_byte_data(x_arduino,0)
    print("The value that was sent was 17")

    # Read data from Arduino
    read = bus.read_byte_data(x_arduino,1)
    print("Data Received: " + read)

    if (read == value):
        print("Communication was a success!")

    return 0
