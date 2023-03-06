# as-nano33MCU-agv-INLINE.py
# barry.mcmullin@dcu.ie

# Test asyncio script for AGV functionality on EM106 nano33 MCU.
# Libraries INLINE for dev/test purposes in OpenMV IDE only.

import time
import sys

import uasyncio as asyncio
from primitives import Delay_ms, WaitAny, ESwitch

from machine import Pin, I2C, PWM

## from em106-AGV import nano33Pins
## INLINE for test purposes
# See nano33 pin mappings here:
# https://docs.arduino.cc/tutorials/nano-33-ble/ble-python-api
nano33Pins = {
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

## from em106-AGV import as_lcd_api.py
## INLINE for test purposes

# as_lcd_api.py
# asyncio version of lcd_api. Some functionality pruned.
#
# barry.mcmullin@dcu.ie
# Last modified: 12 Feb 2023

#import uasyncio as asyncio

# Very minimal changes for asyncio compatibility.
#
# Original made some use of time.sleep_us(). In practice this was invoked with only a
# 40us delay which is arguably tolerable for asyncio applications. Nonetheless, it is replaced
# here with await asyncio.sleep_ms(), with a guaranteed minimum delay of 1ms (being potentially
# much longer, of course). But *extra* delay should not cause any issues with the code here.

class LcdApi:
    """Implements the API for talking with HD44780 compatible character LCDs.
    This class only knows what commands to send to the LCD, and not how to get
    them to the LCD.

    It is expected that a derived class will implement the hal_xxx functions.
    """

    # The following constant names were lifted from the avrlib lcd.h
    # header file, however, I changed the definitions from bit numbers
    # to bit masks.
    #
    # HD44780 LCD controller command set

    LCD_CLR = 0x01              # DB0: clear display
    LCD_HOME = 0x02             # DB1: return to home position

    LCD_ENTRY_MODE = 0x04       # DB2: set entry mode
    LCD_ENTRY_INC = 0x02        # --DB1: increment
    LCD_ENTRY_SHIFT = 0x01      # --DB0: shift

    LCD_ON_CTRL = 0x08          # DB3: turn lcd/cursor on
    LCD_ON_DISPLAY = 0x04       # --DB2: turn display on
    LCD_ON_CURSOR = 0x02        # --DB1: turn cursor on
    LCD_ON_BLINK = 0x01         # --DB0: blinking cursor

    LCD_MOVE = 0x10             # DB4: move cursor/display
    LCD_MOVE_DISP = 0x08        # --DB3: move display (0-> move cursor)
    LCD_MOVE_RIGHT = 0x04       # --DB2: move right (0-> left)

    LCD_FUNCTION = 0x20         # DB5: function set
    LCD_FUNCTION_8BIT = 0x10    # --DB4: set 8BIT mode (0->4BIT mode)
    LCD_FUNCTION_2LINES = 0x08  # --DB3: two lines (0->one line)
    LCD_FUNCTION_10DOTS = 0x04  # --DB2: 5x10 font (0->5x7 font)
    LCD_FUNCTION_RESET = 0x30   # See "Initializing by Instruction" section

    LCD_CGRAM = 0x40            # DB6: set CG RAM address
    LCD_DDRAM = 0x80            # DB7: set DD RAM address

    LCD_RS_CMD = 0
    LCD_RS_DATA = 1

    LCD_RW_WRITE = 0
    LCD_RW_READ = 1

    def __init__(self, num_lines, num_columns): # Executes *synchronously*
        self.num_lines = num_lines
        if self.num_lines > 4:
            self.num_lines = 4
        self.num_columns = num_columns
        if self.num_columns > 40:
            self.num_columns = 40
        self.cursor_x = 0
        self.cursor_y = 0
        self.implied_newline = False
        self.backlight = True

    async def startup(self) : # Potentially slow - use as awaitable
        self.display_off()
        await self.clear()
        self.hal_write_command(self.LCD_ENTRY_MODE | self.LCD_ENTRY_INC)
        self.hide_cursor()
        self.display_on()
        #self.putstr("reset complete!")
        #print("LcdApi.reset() completing...")

    async def home(self):  # Potentially slow - use as awaitable
        """Moves the cursor to the top left corner.
        """
        self.hal_write_command(self.LCD_HOME)
        self.cursor_x = 0
        self.cursor_y = 0
        await asyncio.sleep_ms(5)

    async def clear(self):  # Potentially slow - use as awaitable
        """Clears the LCD display and moves the cursor to the top left
        corner.
        """
        await self.home()
        self.hal_write_command(self.LCD_CLR)
        await asyncio.sleep_ms(5)

    def show_cursor(self):
        """Causes the cursor to be made visible."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR)

    def hide_cursor(self):
        """Causes the cursor to be hidden."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def blink_cursor_on(self):
        """Turns on the cursor, and makes it blink."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR | self.LCD_ON_BLINK)

    def blink_cursor_off(self):
        """Turns on the cursor, and makes it no blink (i.e. be solid)."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR)

    def display_on(self):
        """Turns on (i.e. unblanks) the LCD."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def display_off(self):
        """Turns off (i.e. blanks) the LCD."""
        self.hal_write_command(self.LCD_ON_CTRL)

    def move_to(self, cursor_x, cursor_y):
        """Moves the cursor position to the indicated position. The cursor
        position is zero based (i.e. cursor_x == 0 indicates first column).
        """
        self.cursor_x = cursor_x
        self.cursor_y = cursor_y
        addr = cursor_x & 0x3f
        if cursor_y & 1:
            addr += 0x40    # Lines 1 & 3 add 0x40
        if cursor_y & 2:    # Lines 2 & 3 add number of columns
            addr += self.num_columns
        self.hal_write_command(self.LCD_DDRAM | addr)

    def putchar(self, char):
        """Writes the indicated character to the LCD at the current cursor
        position, and advances the cursor by one position.
        """
        if char == '\n':
            if self.implied_newline:
                # self.implied_newline means we advanced due to a wraparound,
                # so if we get a newline right after that we ignore it.
                #pass
                self.implied_newline = False
            else:
                self.cursor_x = self.num_columns
        else:
            self.hal_write_data(ord(char))
            self.cursor_x += 1
            self.implied_newline = False
        if self.cursor_x >= self.num_columns:
            self.cursor_x = 0
            self.cursor_y += 1
            self.implied_newline = (char != '\n')
        if self.cursor_y >= self.num_lines:
            self.cursor_y = 0
        self.move_to(self.cursor_x, self.cursor_y)

    def putstr(self, string):
        """Write the indicated string to the LCD at the current cursor
        position and advances the cursor position appropriately.
        """
        #print("LcdApi.putstr('{:s}')".format(string))
        for char in string:
            self.putchar(char)

    def clearline(self, line):
        line = line % self.num_lines
        self.move_to(0,line)
        for col in range(0,self.num_columns) :
            self.putchar(' ')
        self.move_to(0,line)

    def putline(self, line, string):
        self.clearline(line)
        self.putstr(string)

    def custom_char(self, location, charmap):
        """Write a character to one of the 8 CGRAM locations, available
        as chr(0) through chr(7).
        """
        location &= 0x7
        self.hal_write_command(self.LCD_CGRAM | (location << 3))
        self.hal_sleep_us(40)
        for i in range(8):
            self.hal_write_data(charmap[i])
            #self.hal_sleep_us(40)
            time.sleep_us(40)
        self.move_to(self.cursor_x, self.cursor_y)

    def hal_write_command(self, cmd):
        """Write a command to the LCD.

        It is expected that a derived HAL class will implement this
        function.
        """
        raise NotImplementedError

    def hal_write_data(self, data):
        """Write data to the LCD.

        It is expected that a derived HAL class will implement this
        function.
        """
        raise NotImplementedError

## from em106-AGV import mcp23008
## INLINE for test purposes

# mcp23008.py
# MCP23008 class
#
# Some functionality pruned.
#
# barry.mcmullin@dcu.ie
# Last modified: 12 Feb 2023

#from machine import Pin, I2C

class MCP23008(object):
    """A convenience class to interact with the MCP23008.
    Based on the datasheet found at http://ww1.microchip.com/downloads/en/DeviceDoc/21919e.pdf
    Author: joe@crankshaw.org """

    #Some defaults and init
    def __init__(self, i2c = None, address = 32 ):
        """Initialise the MCP23008 object, defaults to address 32 / "\\x20" and
        scl pin 4, sda pin 5 if no i2c object passed."""

        # Register values
        self._registers = {}
        self._registers['IODIR'] = 0   # b'\x00'
        self._registers['IPOL'] = 1    # b'\x01'
        self._registers['GPINTEN'] = 2 # b'\x02'
        self._registers['DEFVAL'] = 3  # b'\x03'
        self._registers['INTCON'] = 4  # b'\x04'
        self._registers['IOCON'] = 5   # b'\x05'
        self._registers['GPPU'] = 6    # b'\x06'
        self._registers['INTF'] = 7    # b'\x07'
        self._registers['INTCAP'] = 8  # b'\x08'
        self._registers['GPIO'] = 9    # b'\x09'
        self._registers['OLAT'] = 10   # b'\x0A'

        # Premake some buffers for interrupt safeness, only the likely ones for now
        self._intcap_buf = bytearray(1)
        self._intf_buf = bytearray(1)
        self._gpio_buf = bytearray(1)

        # I2C Address
        self._address = address        # b'\x20' = 32.  so 32+ for addresses

        # I2C Object
        if i2c == None:                # A default that is useful during dev
            self._i2c = I2C(scl=Pin(4), sda=Pin(5), freq=100000)
        else:
            self._i2c = i2c

    @property
    def IODIR(self):
        """Controls the direction of the data I/O.
        When a bit is set, the corresponding pin becomes an
        input. When a bit is clear, the corresponding pin
        becomes an output"""

        #print ("IODIR getter called")
        # read i2c
        return self._i2c.readfrom_mem(self._address, self._registers['IODIR'], 1 )

    @IODIR.setter
    def IODIR(self, value):
        #print ("IODIR setter called with value: %s" % value)

        # write to i2c
        self._i2c.writeto_mem(self._address, self._registers['IODIR'], value)

    @property
    def GPIO(self):
        """The GPIO register reflects the value on the port.
        Reading from this register reads the port. Writing to this
        register modifies the Output Latch (OLAT) register."""
        # print ("GPIO getter called")
        self._i2c.readfrom_mem_into(self._address, self._registers['GPIO'], self._gpio_buf )
        return self._gpio_buf

    @GPIO.setter
    def GPIO(self, value):
        # print ("GPIO setter called")
        self._i2c.writeto_mem(self._address, self._registers['GPIO'], value)

## from em106-AGV import as_i2c_lcd_mcp23008.py
## INLINE for test purposes

# as_i2c_lcd_mcp23008.py
# asyncio version of i2c_lcd_mcp23008.
#
# barry.mcmullin@dcu.ie
# Last modified: 12 Feb 2023

#import gc
#import uasyncio as asyncio
#from mcp23008 import MCP23008

# DCU EM106 nano33-MCU pin definitions (via MCP23008 I2C port expander)
MASK_RS = 0x01       # P0
MASK_RW = 0x02       # P1
MASK_E  = 0x04       # P2

SHIFT_DATA = 3       # P3-P6

class I2cLcd(LcdApi):

    # Implements a HD44780 character LCD connected via MCP23008

    def __init__(self, mcp, num_lines, num_columns): # Executes *synchronously*
        self.mcp = mcp
        self.mcp.IODIR=bytes(b"\x80") # MCP GPIO P0-P6 as outputs
        self.mcp.GPIO=bytes([0])
        LcdApi.__init__(self, num_lines, num_columns)

    async def reset(self) : # Slow: use as awaitable
        await asyncio.sleep_ms(20)   # Allow LCD time to powerup
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        await asyncio.sleep_ms(5)    # Need to delay at least 4.1 msec
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        await asyncio.sleep_ms(1)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        await asyncio.sleep_ms(1)
        self.hal_write_init_nibble(self.LCD_FUNCTION)
        await asyncio.sleep_ms(1)
        cmd = self.LCD_FUNCTION
        if self.num_lines > 1:
            cmd |= self.LCD_FUNCTION_2LINES
        self.hal_write_command(cmd)
        await LcdApi.reset(self)
        print("lcd.reset() completing...")

    def hal_write_init_nibble(self, nibble):
        # Writes an initialization nibble to the LCD.
        # This particular function is only used during initialization.
        byte = ((nibble >> 4) & 0x0f) << SHIFT_DATA
        self.mcp.GPIO = bytes([byte | MASK_E])
        self.mcp.GPIO = bytes([byte])

    def hal_write_command(self, cmd):
        # Write a command to the LCD. Data is latched on the falling edge of E.
        byte = (((cmd >> 4) & 0x0f) << SHIFT_DATA)
        self.mcp.GPIO = bytes([byte | MASK_E])
        self.mcp.GPIO = bytes([byte])
        byte = ((cmd & 0x0f) << SHIFT_DATA)
        self.mcp.GPIO = bytes([byte | MASK_E])
        self.mcp.GPIO = bytes([byte])
        #if cmd <= 3:
            ## The home and clear commands require a worst case delay of 4.1 msec
            #print("hal_write_command({:02x}): await...".format(cmd))
            #await asyncio.sleep_ms(5)
            #time.sleep_ms(5)
            #print("...done")

    def hal_write_data(self, data):
        # Write data to the LCD. Data is latched on the falling edge of E.
        byte = (MASK_RS |
                (((data >> 4) & 0x0f) << SHIFT_DATA))
        self.mcp.GPIO = bytes([byte | MASK_E])
        self.mcp.GPIO = bytes([byte])
        byte = (MASK_RS |
                ((data & 0x0f) << SHIFT_DATA))
        self.mcp.GPIO = bytes([byte | MASK_E])
        self.mcp.GPIO = bytes([byte])

## from em106-AGV import ESteerSensors
## INLINE for test purposes
#import uasyncio as asyncio
#from machine import Pin
#from primitives import Switch

class EOptoSensors:

    def __init__(self, leftPinID = "A3", rightPinID = "A2"):
        # NB: may be *mounted* opposite way around...
        # MCU allocates analog pins, but we use in digital mode only here as ESwitch devices:
        # https://github.com/peterhinch/micropython-async/blob/master/v3/docs/EVENTS.md#61-eswitch
        # External 4.7k phototransitor pullups present on MCU: don't
        # activate internal pull up or down as well!
        self.leftPin=Pin(nano33Pins[leftPinID],Pin.IN, None)
        self.left = ESwitch(self.leftPin)
        self.left.debounce_ms = 5
        self.rightPin=Pin(nano33Pins[rightPinID],Pin.IN, None)
        self.right = ESwitch(self.rightPin)
        self.right.debounce_ms = 5

class rcPWM:

    # Set up to generate PWM signal suitable for RC ("remote control") type
    # interface devices. Pulse frequency 50Hz. Pulse width varying from
    # 1ms (-100%) to 1.5ms (0%) to 2ms (+100%).

    # The current micropython nrf port has very poor/dysfunctional implementation
    # of the hardware PWM. It specifically does NOT match the generic API documented
    # for <machine.PWM>. Some (incomplete) discussion here:
    # https://forums.openmv.io/t/unable-to-import-pwm-module-in-nano-33-ble-sense/7028
    # The effect is that, every time we want to change the pulse width, we have to create
    # a new PWM object. Yuk.

    freqHz = 50
    clkPrescale = PWM.FREQ_125KHZ
    clkHz = 125000
    topCnt = clkHz // freqHz
    top_usec = (10**6)//freqHz
    count_1500usec = (1500*topCnt)//top_usec
    count_500usec = ((500)*topCnt)//top_usec

    def __init__(self, pinID, nrfPWM_ID):
        # NRF52840 has four independent PWM blocks, numbered 0-3
        self.pinID = pinID
        self.pin = Pin(nano33Pins[pinID], mode=Pin.OUT)
        self.nrfPWM_ID = nrfPWM_ID
        self._pwm = None
        self.percent = 0

    @property
    def percent(self):
        return self._percent

    @percent.setter
    def percent(self, percentValue):
        self._percent = percentValue
        pulseCnt = rcPWM.count_1500usec + (
            (percentValue * rcPWM.count_500usec)//100)
        if self._pwm is not None :
            self._pwm.deinit() # Defensive: may not be necessary/useful?
        self._pwm = PWM(self.nrfPWM_ID, pin=self.pin,
            freq=rcPWM.clkPrescale, period=rcPWM.topCnt, pulse_width=pulseCnt)
        self._pwm.init()

## from em106-AGV import EKeypad
## INLINE for test purposes
#import uasyncio as asyncio
#from machine import Pin

class EKeypad:
    ROWS = 4
    COLS = 4
    hexaKeys = [
      ['1', '2', '3', 'A'],
      ['4', '5', '6', 'B'],
      ['7', '8', '9', 'C'],
      ['*', '0', '#', 'D']
    ]

    def __init__(self, debounce_ms = 20):
        self.debounce_ms = debounce_ms
        self.keyPressed = asyncio.Event()
        self._key = None
        self._rowPins = ['D12','D8','D7','D6'] # inputs
        for row in range(EKeypad.ROWS) :
            self._rowPins[row] = Pin(
                nano33Pins[self._rowPins[row]], Pin.IN, pull=None)
        self._colPins = ['D5','D4','D3','D2'] # outputs
        for col in range(EKeypad.COLS) :
            self._colPins[col] = Pin(
                #nano33Pins[self._colPins[col]], Pin.IN, pull=None)
                # leave as hi-impedance by default?
                nano33Pins[self._colPins[col]], Pin.OUT)
            self._colPins[col].low()
        self.rowStates = [0, 0, 0, 0]
        self._someKeyPressed = False
        self.activeRow = None
        self.activeCol = None

        asyncio.create_task(self._poll())

    async def _readRows(self) :
        self._someKeyPressed = False;
        #self.activeRow = None;
        for row in range(EKeypad.ROWS) :
            state = self._rowPins[row].value()
            if(state) :
                await asyncio.sleep_ms(self.debounce_ms)
                    # noise reduction: must *stay* active for this time or be discarded
                state = self._rowPins[row].value()
                # Uncomment to show rejected transient key activations...
                #if not state :
                    #print("keypad transient rejected [row: {:d} col: {:d} key: {:s}!".format(
                        #row, self.activeCol, EKeypad.hexaKeys[row][self.activeCol]))
            self.rowStates[row] = state
            if (state) :
                self.activeRow = row
            self._someKeyPressed = self._someKeyPressed or state

    async def _poll(self):
        while True:
            for col in range(EKeypad.COLS) :
                #self._colPins[col].init(mode=Pin.OUT)
                self._colPins[col].high()
                self.activeCol = col
                await asyncio.sleep_ms(1) # Wait for signal to settle
                await self._readRows()
                if(self._someKeyPressed) :
                    self._key = EKeypad.hexaKeys[self.activeRow][self.activeCol]
                    self.keyPressed.set() # Should be cleared by key handling task
                    while(self._someKeyPressed) : # Wait for key release
                        await asyncio.sleep_ms(1)
                        self._someKeyPressed = self._rowPins[self.activeRow].value()
                self._colPins[col].low()
                #self.activeCol = None
                #self._colPins[col].init(mode=Pin.IN,pull=None)
                await asyncio.sleep_ms(self.debounce_ms)  # Wait out bounce

    # ***** API *****
    # Return last key pressed
    def __call__(self):
        return self._key

    def deinit(self):
        self._poll.cancel()
        self.keyPressed.clear()

## from em106-AGV import AGV.py?
## INLINE for test purposes

class AGV :

    # Encapsulates standard EM106 nano33 MCU devices and functionality.

    def __init__(self,splashQuitTimeout=30):
        self.__version__ = "0.04rc"
        self.keypad = EKeypad()
        self.optoSensors = EOptoSensors()
        self.optoSensorStates = {}
        self.I2C_ADDR = 0x20
        self.LCD_ROWS = 4
        self.LCD_COLS = 20
        self.i2c = I2C(0,scl=Pin("I2C_SCL"), sda=Pin("I2C_SDA"))
        self.mcp = MCP23008(self.i2c, address=self.I2C_ADDR)
        self.lcd = I2cLcd(self.mcp, self.LCD_ROWS, self.LCD_COLS)
        self.steerPWM = rcPWM("D10",0)
        self.tractionPWM = rcPWM("D11",1)

    def updateDuration(self,duration,key):
        if key == 'A' :
            duration += 1
        elif key == 'B' :
            duration = max(duration-1,0)
        elif key == 'C' :
            duration += 10
        elif key == 'D' :
            duration = max(duration-10,0)
        else :
            # ignore other key values
            pass
        return duration

    async def getDuration(self,prompt,duration) :
        await self.lcd.clear()
        self.lcd.putline(0,"{:s}:".format(prompt))
        self.lcd.putline(1,"{:4d} s".format(duration))
        while True :
            await self.keypad.keyPressed.wait()
            self.keypad.keyPressed.clear()
            key = self.keypad()
            old_duration = duration
            duration = self.updateDuration(duration,key)
            if duration != old_duration :
                self.lcd.putline(1,"{:4d} s".format(duration))
            elif key == '*':
                return(duration)
            else :
                # ignore other key values
                pass

    def updatePercent(self,percent,key):
        if key == 'A' :
            percent = min(percent+1, 100)
        elif key == 'B' :
            percent = max(percent-1, -100)
        elif key == 'C' :
            percent = min(percent+10, 100)
        elif key == 'D' :
            percent = max(percent-10, -100)
        else :
            # ignore other key values
            pass
        return percent

    async def getPercent(self, prompt, percent):
        await self.lcd.clear()
        self.lcd.putline(0,"{:s} : ".format(prompt))
        self.lcd.putline(1,"{:4d}%".format(percent))
        while True :
            await self.keypad.keyPressed.wait()
            self.keypad.keyPressed.clear()
            key = self.keypad()
            oldPercent = percent
            percent = self.updatePercent(percent,key)
            if percent != oldPercent :
                self.lcd.putline(1,"{:4d}%".format(percent))
            elif key == '*':
                return(percent)
            else :
                # ignore other key values
                pass

    def updateBool(self,value,key):
        if key == 'A' :
            value = not value
        return value

    async def getBool(self, prompt, value):
        await self.lcd.clear()
        self.lcd.putline(0,"{:s} : ".format(prompt))
        self.lcd.putline(1,"{:5s}".format(str(value)))
        while True :
            await self.keypad.keyPressed.wait()
            self.keypad.keyPressed.clear()
            key = self.keypad()
            oldValue = value
            value = self.updateBool(value,key)
            if value != oldValue :
                self.lcd.putline(1,"{:5s}".format(str(value)))
            elif key == '*':
                return(value)
            else :
                # ignore other key values
                pass

    def latchOptoSensorStates(self) :
        self.optoSensorStates['left'] = self.optoSensors.left()
        self.lcd.putline(3,"Left: {:d}   ".format(self.optoSensorStates['left']))
        self.optoSensorStates['right'] = self.optoSensors.right()
        self.lcd.putstr("Right: {:d}".format(self.optoSensorStates['right']))

    async def autoCapture(self, tractionPercent, steerGain, captureEvent, stopLineEvent) :
        # Implement line capture, *assuming* both sensors are initially fully
        # to left *or* right of line (but don't know which) and steering straight will
        # encounter line at shallow enough angle to then transition directly
        # to autoSteer()/line following.
        #
        # tractionPercent and steerGain are passed in for possible *future* use to
        # modulate the algorithm here. For the moment however, they are ignored.
        steerPercent = 0
        self.steerPWM.percent = steerPercent
        self.lcd.putline(2,"Steer: {:d}".format(steerPercent))
        self.latchOptoSensorStates()
        while (not self.optoSensorStates['left']) and (not self.optoSensorStates['right']) :
            event = await WaitAny((
                self.optoSensors.left.close, self.optoSensors.left.open,
                self.optoSensors.right.close, self.optoSensors.right.open)).wait()
            event.clear()
            self.latchOptoSensorStates()
        if (self.optoSensorStates['left'] and self.optoSensorStates['right']) : # STOP LINE
            stopLineEvent.set()
            return # Exit autoCapture
        elif self.optoSensorStates['left'] :
            waitSensor = 'left'
            #steerPercent = steerGain // 2# Assume we will overshoot
        else :
            waitSensor = 'right'
            #steerPercent = -steerGain // 2 # Assume we will overshoot
        self.lcd.putline(2,"Steer: {:d}".format(steerPercent))
        self.steerPWM.percent = steerPercent
        while (self.optoSensorStates[waitSensor]) : # keep updating the state display
            event = await WaitAny((
                self.optoSensors.left.close, self.optoSensors.left.open,
                self.optoSensors.right.close, self.optoSensors.right.open)).wait()
            event.clear()
            self.latchOptoSensorStates()
        captureEvent.set()

    async def autoSteer(self, tractionPercent, steerGain, stopLineEvent) :
        # tractionPercent is passed in for possible *future* use to
        # modulate the algorithm here. For the moment however, it is ignored.
        steerMap = [
          [0, +1],
          [-1, 0]]
        self.latchOptoSensorStates()
        steerPercent = (steerMap[self.optoSensorStates['left']]
            [self.optoSensorStates['right']]
            * steerGain)
        self.steerPWM.percent = steerPercent
        self.lcd.putline(2,"Steer: {:d}".format(steerPercent))
        while not (self.optoSensorStates['left'] and self.optoSensorStates['right']) :
            event = await WaitAny((
                self.optoSensors.left.close, self.optoSensors.left.open,
                self.optoSensors.right.close, self.optoSensors.right.open)).wait()
            event.clear()
            self.latchOptoSensorStates()
            steerPercent = (steerMap[self.optoSensorStates['left']]
                [self.optoSensorStates['right']]
                * steerGain)
            self.steerPWM.percent = steerPercent
            self.lcd.putline(2,"Steer: {:d}".format(steerPercent))
        # STOP LINE - exit autoSteer
        stopLineEvent.set()

    def lineTest(self,capture=False,tractionPercent=20,steerGain=30) :
        capture = await self.getBool("Line tst capture",capture)
        tractionPercent = await self.getPercent("Line tst tract",tractionPercent)
        steerGain = await self.getPercent("Line tst gain",steerGain)
        await self.lcd.clear()
        self.lcd.putline(0,"Line test")
        self.lcd.putline(1,"[{:d}, {:d}]".format(tractionPercent,steerGain))
        if capture :
            self.lcd.move_to(10,1)
            self.lcd.putstr(" (capture)")
        stopLineEvent = asyncio.Event()
        self.tractionPWM.percent = tractionPercent
        if capture :
            captureEvent = asyncio.Event()
            autoCaptureTask = asyncio.create_task(
                self.autoCapture(tractionPercent, steerGain, captureEvent, stopLineEvent))
            event = await WaitAny((stopLineEvent,captureEvent,self.keypad.keyPressed)).wait()
            event.clear()
            autoCaptureTask.cancel()
            if event is not captureEvent :
                self.steerPWM.percent = 0
                self.tractionPWM.percent = 0
                if event is stopLineEvent :
                    self.lcd.putline(1,"STOP LINE")
                else :
                    self.lcd.putline(3,"Aborted by key!")
                await asyncio.sleep(2)
                return
        self.lcd.move_to(10,1)
        self.lcd.putstr(" (follow) ")
        autoSteerTask = asyncio.create_task(
            self.autoSteer(tractionPercent, steerGain, stopLineEvent))
        event = await WaitAny((stopLineEvent,self.keypad.keyPressed)).wait()
        event.clear()
        autoSteerTask.cancel()
        self.steerPWM.percent = 0
        self.tractionPWM.percent = 0
        if event is stopLineEvent :
            self.lcd.putline(1,"STOP LINE")
        else :
            self.lcd.putline(3,"Aborted by key!")
        await asyncio.sleep(2)

    async def arcSegment(self,tractionPercent,steerPercent,duration,arcEvent) :
        self.steerPWM.percent = steerPercent
        self.tractionPWM.percent = tractionPercent
        delay = Delay_ms(duration=duration*1000)
        delay.trigger()
        event = await WaitAny((delay,self.keypad.keyPressed)).wait()
        event.clear()
        self.tractionPWM.percent = 0
        self.steerPWM.percent = 0
        arcEvent.set()

    async def turnTest(self,tractionPercent=30,steerPercent=60,duration=3) :
        tractionPercent = await self.getPercent("Turn tst tract",tractionPercent)
        steerPrecent = await self.getPercent("Turn tst steer",steerPercent)
        duration = await self.getDuration("Turn tst duration",duration)
        await self.lcd.clear()
        self.lcd.putline(0,"Multi-pt turn test")
        #self.lcd.putline(1,"NOT IMPLEMENTED")
        #self.lcd.putline(2,"Exiting...")
        self.lcd.putline(1,"Running [{:d}, {:d}]".format(tractionPercent,steerPercent))
        arcEvent = asyncio.Event()
        arcSegmentTask = asyncio.create_task(
            self.arcSegment(-tractionPercent, steerPercent, duration, arcEvent))
        event = await WaitAny((arcEvent,self.keypad.keyPressed)).wait()
        event.clear()
        arcSegmentTask.cancel() # in case of key press
        if event is arcEvent :
            arcSegmentTask = asyncio.create_task(
                self.arcSegment(tractionPercent, -steerPercent, duration, arcEvent))
            event = await WaitAny((arcEvent,self.keypad.keyPressed)).wait()
            event.clear()
            arcSegmentTask.cancel() # in case of key press
        if event is arcEvent :
            self.lcd.putline(1,"Completed...")
        else :
            self.lcd.putline(1,"Aborted by key!")
        await asyncio.sleep(2)

    async def circleTest(self,tractionPercent=0,steerPercent=0,duration=10) :
        tractionPercent = await self.getPercent("Circ tst tract",tractionPercent)
        steerPercent = await self.getPercent("Circ tst steer",steerPercent)
        duration = await self.getDuration("Circ tst duration",duration)
        await self.lcd.clear()
        self.lcd.putline(0,"Circle Test")
        self.lcd.putline(1,"Running...")
        arcEvent = asyncio.Event()
        arcSegmentTask = asyncio.create_task(
            self.arcSegment(tractionPercent, steerPercent, duration, arcEvent))
        event = await WaitAny((arcEvent,self.keypad.keyPressed)).wait()
        event.clear()
        arcSegmentTask.cancel() # in case of key press
        if event is arcEvent :
            self.lcd.putline(1,"Completed...")
        else :
            self.lcd.putline(1,"Aborted by key!")
        await asyncio.sleep(2)

    async def pwmTest(self,pwmname,pwm):
        percent = 0
        await self.lcd.clear()
        self.lcd.putline(0,"{:s} test: {:4d}%".format(pwmname,percent))
        pwm.percent = percent
        while True :
            await self.keypad.keyPressed.wait()
            self.keypad.keyPressed.clear()
            key = self.keypad()
            oldPercent = percent
            percent = self.updatePercent(percent,key)
            if percent != oldPercent :
                self.lcd.putline(0,"{:s} test: {:4d}%".format(pwmname,percent))
                pwm.percent = percent
            elif key == '*' :
                pwm.percent = 0
                self.lcd.putline(0,"{:s} test: exit...".format(pwmname))
                await asyncio.sleep(2)
                return
            else :
                # Some other key: ignore!
                pass

    async def optoSensorTest(self):
        await self.lcd.clear()
        self.lcd.putline(0,"optosensors: test")
        leftState = self.optoSensors.left()
        self.lcd.putline(1,"Left:  {:d}".format(leftState))
        rightState = self.optoSensors.right()
        self.lcd.putline(2,"Right: {:d}".format(rightState))
        while True :
            event = await WaitAny((
                self.optoSensors.left.close, self.optoSensors.left.open,
                self.optoSensors.right.close, self.optoSensors.right.open,
                self.keypad.keyPressed)).wait()
            event.clear()
            leftState = self.optoSensors.left()
            self.lcd.putline(1,"Left:  {:d}".format(leftState))
            rightState = self.optoSensors.right()
            self.lcd.putline(2,"Right: {:d}".format(rightState))
            if event is self.keypad.keyPressed :
                self.lcd.putline(0,"optosensors: exit...")
                await asyncio.sleep(2)
                return
            else :
                # Can't happen: should really raise an exception?
                pass

    def displayKeyPress(self, starCountdown):
        self.lcd.move_to(14,0)
        self.lcd.putline(0,"Keypad test:")
        self.lcd.putline(1,"activeRow: {:d}".format(self.keypad.activeRow))
        self.lcd.putline(2,"activeCol: {:d}".format(self.keypad.activeCol))
        self.lcd.putline(3,'key code: "{:s}"'.format(self.keypad()))
        self.lcd.move_to(14,3)
        self.lcd.putstr("[*: {:d}]".format(starCountdown))

    async def keypadTest(self):
        starCountdown = 3
        await self.lcd.clear()
        self.lcd.putline(0,"Keypad test:")
        self.lcd.move_to(14,3)
        self.lcd.putstr("[*: {:d}]".format(starCountdown))
        while True :
            await self.keypad.keyPressed.wait()
            self.keypad.keyPressed.clear()
            key = self.keypad()
            if key == '*' :
                starCountdown -= 1
            else :
                starCountdown = 3
            self.displayKeyPress(starCountdown)
            if starCountdown == 0 :
                self.lcd.putline(0,"keypad test: exit...")
                await asyncio.sleep(2)
                return

    async def displayMainMenu(self, starCountdown):
        await self.lcd.clear()
        self.lcd.putline(0,"1: keypad 2: opto")
        self.lcd.putline(1,"3: servo  4: tract")
        self.lcd.putline(2,"5: circle 6: line")
        self.lcd.putline(3,"7: turn")
        self.lcd.move_to(14,3)
        self.lcd.putstr("[*: {:d}]".format(starCountdown))

    async def cmdloop(self):
        starCountdown = 3
        await self.displayMainMenu(starCountdown)
        while True:
            await self.keypad.keyPressed.wait()
            self.keypad.keyPressed.clear()
            key = self.keypad()

            if (key == '1') :
                await self.keypadTest()
                starCountdown = 3
                await self.displayMainMenu(starCountdown)
            elif (key == '2') :
                await self.optoSensorTest()
                starCountdown = 3
                await self.displayMainMenu(starCountdown)
            elif (key == '3') :
                await self.pwmTest("servo",self.steerPWM)
                starCountdown = 3
                await self.displayMainMenu(starCountdown)
            elif (key == '4') :
                await self.pwmTest("tract",self.tractionPWM)
                starCountdown = 3
                await self.displayMainMenu(starCountdown)
            elif (key == '5') :
                await self.circleTest()
                starCountdown = 3
                await self.displayMainMenu(starCountdown)
            elif (key == '6') :
                await self.lineTest()
                starCountdown = 3
                await self.displayMainMenu(starCountdown)
            elif (key == '7') :
                await self.turnTest()
                starCountdown = 3
                await self.displayMainMenu(starCountdown)
            elif (key == '*') :
                starCountdown -= 1
                self.lcd.move_to(14,3)
                self.lcd.putstr("[*: {:d}]".format(starCountdown))
                self.lcd.move_to(0,3)
                self.lcd.putstr("            ".format(key))
            else :
                self.lcd.move_to(0,3)
                self.lcd.putstr("{:s}: not valid".format(key))
                starCountdown = 3
                self.lcd.move_to(14,3)
                self.lcd.putstr("[*: {:d}]".format(starCountdown))
                await asyncio.sleep(2)
                self.lcd.move_to(0,3)
                self.lcd.putstr("7: turns    ")

            if starCountdown == 0 :
                await self.lcd.clear()
                self.lcd.putline(1," (pyAGV user exit)")
                await asyncio.sleep(2)
                return

    async def splash(self):
        await self.lcd.startup()
        self.lcd.putline(1,"   pyAGV  v {:s}".format(self.__version__))
        self.lcd.putline(2," (any key to start)")

    async def run(self,splashQuitTimeout=30):
        await self.splash()
        splashQuit = Delay_ms(duration=splashQuitTimeout*1000)
        splashQuit.trigger()
        event = await WaitAny((splashQuit,self.keypad.keyPressed)).wait()
        event.clear()
        if event is self.keypad.keyPressed :
            await self.cmdloop()
        else :
            await self.lcd.clear()
            self.lcd.putline(1,"       TIMEOUT")
            self.lcd.putline(2,"    (pyAGV abort)")
            await asyncio.sleep(2)

# standalone test: remove from module version?

async def main():
    print("main: initialising agv obj...")
    agv = AGV()
    print("main: starting agv.run()...")
    await agv.run(splashQuitTimeout=10)
    print("agv.run() exit detected...")

print("starting main()")
asyncio.run(main())
print("main() exited...")
