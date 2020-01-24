#!/usr/bin/env python3
#
# Read temperature and humidity values from Arduino board
# which is connected to a DH22 sensor over a serial link
# with baudrate 115200 and the following measurement scheme:
# Humidity: XX.XX%  Temperature: XX.XX°C  Heat index: XX.XX°C
#

import sys
import os
import serial
import re
import json

EXAMPLE_TEMP = 19.40
EXAMPLE_HUMIDITY = 56.70
EXAMPLE_HEAT_INDEX = 18.88
EXAMPLE_MEASUREMENT = "Humidity: 56.70%  Temperature: 19.40°C  Heat index: 18.88°C"
SERIAL_BAUDRATE = 115200
SERIAL_DEVICE = "/dev/ttyACM0"



class sensor_msg:
    def __init__(self, temperature: float, humidity: float, heat_index: float):
        self.temperature = temperature
        self.humidity = humidity
        self.heat_index = heat_index
    def __repr__(self) -> str:
        return "Temperature: {}, Humidity: {}, Heat index: {}".format(self.temperature, self.humidity, self.heat_index)


class serial_reader:
    def __init__(self, serial_device: str = SERIAL_DEVICE, serial_baudrate: int = SERIAL_BAUDRATE):
        if not serial_device.startswith("/dev/tty"):
            print("{} is not a valid serialport!".format(serial_device))
            sys.exit(-1)

        try:
            self.serial_dev = serial.Serial(serial_device, serial_baudrate, timeout=5)
        except Exception as e:
            print("Couldn't open serial port {}!".format(serial_device))
            print(e)
            sys.exit(-1)

        # Flush all buffers
        self.serial_dev.flushInput()
        self.serial_dev.flushOutput()

    def read_current_msg(self) -> str:
        line = self.serial_dev.readline().decode('utf-8')
        return line


def get_samples_from_message(msg: str) -> list:
    measurements = re.findall("\d+.\d+", msg)
    measurements = list(map(lambda x: float(x), measurements))
    if len(measurements) == 3:
        return sensor_msg(humidity=measurements[0], temperature=measurements[1], heat_index=measurements[2])
    else:
        return sensor_msg(0., 0., 0.)


def test_format(message_str: str) -> bool:
    samples = get_samples_from_message(message_str)
    
    # Testcase: Check numeric values
    if samples.humidity == EXAMPLE_HUMIDITY and samples.temperature == EXAMPLE_TEMP and samples.heat_index == EXAMPLE_HEAT_INDEX:
        return True
    else:
        print("Test failed: received different temp / humidity / heat index values!")
        print("True: {}, Actual: {}".format([EXAMPLE_TEMP,EXAMPLE_HUMIDITY,EXAMPLE_HEAT_INDEX], samples))
        return False



if __name__ == "__main__":
    print("Unit test result: {}".format("successful" if test_format(EXAMPLE_MEASUREMENT) else  "unsuccessful"))

    reader = serial_reader(SERIAL_DEVICE, SERIAL_BAUDRATE)
    samples = {"temperature" : [],
               "humidity":     [],
               "heat_index":   []}
    try:
        while True:
            sample = reader.read_current_msg()
            msg = get_samples_from_message(sample)
            print(msg)
            samples["temperature"].append(msg.temperature)
            samples["humidity"].append(msg.humidity)
            samples["heat_index"].append(msg.heat_index)   
    except KeyboardInterrupt:
        print("Exit!")
        print(json.dumps(samples, sort_keys=False, indent=2))
