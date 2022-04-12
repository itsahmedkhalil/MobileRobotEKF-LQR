#! /usr/bin/python3
import serial

# Set up a Serial port connection. Port is '/dev/ttyUSB0' (for Arduino Nano on USB)
port = serial.Serial('/dev/ttyUSB0',57600)
port.flush()  # Flush any data currently on the port

try:
    while True:
        line = port.read(100)   # read 100 characters from the port
        print(line.decode())   # "decode" the line from raw bytes into a string and print it. 
except:
    pass;
