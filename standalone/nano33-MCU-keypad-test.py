# nano33-MCU-keypad-test.py
# barry.mcmullin@dcu.ie

# Test script for keypad on EM106 nano33 MCU

import time
from machine import Pin

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

ROWS = 4
COLS = 4

hexaKeys = [
  ['1', '2', '3', 'A'],
  ['4', '5', '6', 'B'],
  ['7', '8', '9', 'C'],
  ['*', '0', '#', 'D']
]

print("setup: configuring rowPins (inputs)")

rowPins = ['D12','D8','D7','D6'] # inputs
for row in range(ROWS) :
    pin_num = nano33_pins[rowPins[row]]
    rowPins[row] = Pin(pin_num, Pin.IN, Pin.PULL_DOWN)
    print(rowPins[row])

print("setup: configuring colPins (outputs)")

colPins = ['D5','D4','D3','D2'] # outputs
for col in range(COLS) :
    pin_num = nano33_pins[colPins[col]]
    colPins[col] = Pin(pin_num, Pin.OUT)
    print(colPins[col])

rowStates = [0, 0, 0, 0]
keyPressed = False
activeRow = None

def readRows() :
    global keyPressed, activeRow, rowStates
    keyPressed = False;
    activeRow = None;
    for row in range(ROWS) :
        state = rowPins[row].value()
        rowStates[row] = state
        keyPressed = keyPressed or state
        if(state) :
            activeRow = row

    return(keyPressed)

print("setup: Setting all cols HIGH")
for col in range(COLS) :
    colPins[col].high()

print("setup: waiting for keyPressed")
while(not readRows()) :
    pass

print(rowStates, keyPressed, activeRow)

print("setup: Setting all cols LOW")
for col in range(COLS) :
    colPins[col].low()

readRows()
print(rowStates, keyPressed, activeRow)

print("setup: entering loop ")

# TODO: there is an argument that only one col should be *driven* at a time (others should be
# tristate): else, accidentally pressing two keys simultaneously could connect two cols, having
# different levels, which would be *bad* in general... effectively we want a wired-OR kind of
# setup for each row (pulled low active, but can be driven high). Not sure if this requires
# continuously reconfiguring the col pins, or if there a wire-OR configuration that can be done
# once off...

i = 0
while True :
    for col in range(COLS) :
        colPins[col].high()
        time.sleep_ms(1) # Wait for signal to settle
        readRows()
        if(keyPressed) :
            key = hexaKeys[activeRow][col]
            print("i: {:05d} col: {:d} row: {:d} key: {:s} ".format(i, col, activeRow, key))
            while(keyPressed) : # Wait for key release
                time.sleep_ms(1)
                readRows()
        colPins[col].low()
        i += 1
        time.sleep_ms(10) # Wait for at least this time between detecting key presses

print("Done...")
