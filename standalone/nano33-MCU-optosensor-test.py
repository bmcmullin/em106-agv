# nano33-MCU-optosensor-test.py
# barry.mcmullin@dcu.ie

# Test script for optical sensors on EM106 nano33 MCU

import time
from machine import Pin, ADC

print("Starting...")

# See nano33 pin mappings here:
# https://docs.arduino.cc/tutorials/nano-33-ble/ble-python-api
nano33_pins = {
    "A0" : 4,
    "A1" : 5,
    "A2" : 30,
    "A3" : 29,
    "A4" : 31,
    "A5" : 2,
    "A6" : 28,
    "A7" : 3,
    "TX" : 35,
    "RX" : 42,
    "D2" : 43,
    "D3" : 44,
    "D4" : 47,
    "D5" : 45,
    "D6" : 46,
    "D7" : 23,
    "D8" : 21,
    "D9" : 27,
    "D10" : 34,
    "D11" : 33,
    "D12" : 40,
    "D13" : 13,
    "D14" : 4,
    "D15" : 5,
    "D16" : 30,
    "D17" : 29,
    "D18" : 31,
    "D19" : 2,
    "D20" : 28,
    "D21" : 3}

left_sensor_pin = Pin(nano33_pins["A3"], Pin.IN, None) # Discrete load resistor 4.7k on MCU
right_sensor_pin = Pin(nano33_pins["A2"], Pin.IN, None) # Discrete load resistor 4.7k on MCU

u16_max = float((2**16)-1) # Largest 16-bit integer, converted to float representation

# Possibly replace parameters below with actual circuit values (as measured)
Vss = 3.3

sample=0
while(True) :
    left_sensor_u16 = ADC(left_sensor_pin).read_u16()
    left_sensor_volts = (float(left_sensor_u16) / u16_max) * Vss
    print("Sample: {:05d} Left: u16={:05} volts={:5.2f} V".format(
        sample, left_sensor_u16, left_sensor_volts))

    right_sensor_u16 = ADC(right_sensor_pin).read_u16()
    right_sensor_volts = (float(right_sensor_u16) / u16_max) * Vss
    print("Sample: {:05d} Right: u16={:05} volts={:5.2f} V".format(
        sample, right_sensor_u16, right_sensor_volts))

    time.sleep_ms(500)
    sample += 1
