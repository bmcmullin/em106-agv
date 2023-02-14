# nano33-MCU-PWM-test.py
# barry.mcmullin@dcu.ie

# Test script for PWM on nano33 MCU

# NB: The nano33 pwm support in micropython is pretty idiosyncratic. It specifically
# does NOT match the generic API documented for <machine.PWM>. Some (incomplete) discussion
# herer: https://forums.openmv.io/t/unable-to-import-pwm-module-in-nano-33-ble-sense/7028

import time
from machine import Pin, PWM

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

# All the digital pins on Arduino Nano 33 BLE sense are PWM-enabled pins, numbered from D0 to D13
steer_pin = Pin(nano33_pins["D10"], mode=Pin.OUT)
traction_pin = Pin(nano33_pins["D11"], mode=Pin.OUT)

freq_hz = 50
clock_prescale = PWM.FREQ_125KHZ
clock_hz = 125000
top_counter = clock_hz//freq_hz
top_usec = (10**6)//freq_hz
print("top_counter: {:d} top_usec: {:d}".format(top_counter, top_usec))
count_1500usec = (1500*top_counter)//top_usec
count_500usec = ((500)*top_counter)//top_usec
print("count_1500s: {:d} count_500usec: {:d} ".format(
    count_1500usec, count_500usec))

def pwm_init_percent(nrf_pwm_id, pin, percent) :
    pulse_count = count_1500usec + ((percent*count_500usec)//100)
    print("pwm_percent: {:d}% (pulse_count: {:d})".format(percent, pulse_count))
    p = PWM(nrf_pwm_id, pin=pin,
        freq=clock_prescale, period=top_counter, pulse_width=pulse_count)
    p.init()
    return(p) # Allow for deinit()!

def sweep(nrf_pwm_id, pin, delay_ms):
    percent = 0
    while (percent <= 100) :
        p = pwm_init_percent(nrf_pwm_id, pin, percent)
        time.sleep_ms(delay_ms)
        p.deinit()
        percent += 1

    percent = 99
    while (percent >= -100) :
        p = pwm_init_percent(nrf_pwm_id, pin, percent)
        time.sleep_ms(delay_ms)
        p.deinit()
        percent -= 1

    percent = -99
    while (percent <=0) :
        p = pwm_init_percent(nrf_pwm_id, pin, percent)
        time.sleep_ms(delay_ms)
        p.deinit()
        percent += 1

print("Starting sweep()...")
sweep(1, steer_pin, 100)

print("We're done...")
