# EKeypad.py
# uasyncio event based keypad class
#
# barry.mcmullin@dcu.ie
# Last modified: 12 Feb 2023

import uasyncio as asyncio
from machine import Pin

# To be tested!
from em106-AGV import nano33_pins

class EKeypad:
    debounce_ms = 50
    ROWS = 4
    COLS = 4
    hexaKeys = [
      ['1', '2', '3', 'A'],
      ['4', '5', '6', 'B'],
      ['7', '8', '9', 'C'],
      ['*', '0', '#', 'D']
    ]

    def __init__(self):
        self.keyPressed = asyncio.Event()
        self._key = None
        self._rowPins = ['D12','D8','D7','D6'] # inputs
        for row in range(EKeypad.ROWS) :
            pin_num = nano33_pins[self._rowPins[row]]
            self._rowPins[row] = Pin(pin_num, Pin.IN, Pin.PULL_DOWN)
        self._colPins = ['D5','D4','D3','D2'] # outputs
        for col in range(EKeypad.COLS) :
            pin_num = nano33_pins[self._colPins[col]]
            self._colPins[col] = Pin(pin_num, Pin.OUT)
            self._colPins[col].low()
        self._rowStates = [0, 0, 0, 0]
        self._someKeyPressed = False
        self._activeRow = None

        asyncio.create_task(self._poll())

    def _readRows(self) :
        self._someKeyPressed = False;
        self._activeRow = None;
        for row in range(EKeypad.ROWS) :
            state = self._rowPins[row].value()
            self._rowStates[row] = state
            self._someKeyPressed = self._someKeyPressed or state
            if(state) :
                self._activeRow = row
        return()

    async def _poll(self):
        while True:
            for col in range(EKeypad.COLS) :
                self._colPins[col].high()
                await asyncio.sleep_ms(1) # Wait for signal to settle
                self._readRows()
                if(self._someKeyPressed) :
                    self._key = EKeypad.hexaKeys[self._activeRow][col]
                    self.keyPressed.set() # Should be cleared by key handling task
                    while(self._someKeyPressed) : # Wait for key release
                        await asyncio.sleep_ms(1)
                        self._readRows()
                self._colPins[col].low()
                await asyncio.sleep_ms(EKeypad.debounce_ms)  # Wait out bounce

    # ***** API *****
    # Return last key pressed
    def __call__(self):
        return self._key

    def deinit(self):
        self._poll.cancel()
        self.keyPressed.clear()

# standalone test: remove from module version?
#keypad = EKeypad()

#async def main():
    #while True:
        #await keypad.keyPressed.wait()
        #key = keypad()
        #keypad.keyPressed.clear()
        #print("keyPressed: {:s}".format(key))

#asyncio.run(main())
