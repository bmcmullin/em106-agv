# as-nano33-MCU-keypad-display-test.py
# barry.mcmullin@dcu.ie

# Test asyncio script for keypad+display on EM106 nano33 MCU
# Libraries INLINE for test purposes only

import time

import uasyncio as asyncio
from primitives import Delay_ms, WaitAny

from machine import Pin, I2C

## from em106-AGV import nano33_pins
## INLINE for test purposes
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

## from em106-AGV import EKeypad
## INLINE for test purposes
#import uasyncio as asyncio
#from machine import Pin

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
        self.rowStates = [0, 0, 0, 0]
        self.someKeyPressed = False
        self.activeRow = None
        self.activeCol = None

        asyncio.create_task(self._poll())

    def _readRows(self) :
        self.someKeyPressed = False;
        self.activeRow = None;
        for row in range(EKeypad.ROWS) :
            state = self._rowPins[row].value()
            self.rowStates[row] = state
            self.someKeyPressed = self.someKeyPressed or state
            if(state) :
                self.activeRow = row

    async def _poll(self):
        while True:
            for col in range(EKeypad.COLS) :
                self._colPins[col].high()
                await asyncio.sleep_ms(1) # Wait for signal to settle
                self._readRows()
                if(self.someKeyPressed) :
                    self.activeCol = col
                    self._key = EKeypad.hexaKeys[self.activeRow][col]
                    self.keyPressed.set() # Should be cleared by key handling task
                    while(self.someKeyPressed) : # Wait for key release
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

    async def reset(self) : # Potentially slow - use as awaitable
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
        #time.sleep_ms(20)
        # Send reset 3 times
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        await asyncio.sleep_ms(5)    # Need to delay at least 4.1 msec
        #time.sleep_ms(5)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        await asyncio.sleep_ms(1)
        #time.sleep_ms(1)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        await asyncio.sleep_ms(1)
        #time.sleep_ms(1)
        # Put LCD into 4-bit mode
        self.hal_write_init_nibble(self.LCD_FUNCTION)
        await asyncio.sleep_ms(1)
        #time.sleep_ms(1)
        cmd = self.LCD_FUNCTION
        if self.num_lines > 1:
            cmd |= self.LCD_FUNCTION_2LINES
        self.hal_write_command(cmd)
        await LcdApi.reset(self)
        print("lcd.reset() completing...")
        #gc.collect()

    def hal_write_init_nibble(self, nibble):
        # Writes an initialization nibble to the LCD.
        # This particular function is only used during initialization.
        byte = ((nibble >> 4) & 0x0f) << SHIFT_DATA
        self.mcp.GPIO = bytes([byte | MASK_E])
        self.mcp.GPIO = bytes([byte])
        #gc.collect()

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
        #gc.collect()

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
        #gc.collect()

# standalone test: remove from module version?

# This toy init() and loop() architecture is very clumsy for now. Really need a class (AGV?)
# that will initialise and encapsulate all required objects (keypad, lcd, sensors,
# steer and motor pwm); though __init__ can only do the "synchronous" bits - will
# need an awaitable to do anything non-synchronous...

async def init():
    keypad = EKeypad()

    I2C_ADDR     = 0x20
    I2C_NUM_ROWS = 4
    I2C_NUM_COLS = 20

    print("\nRunning LCD test")

    print("Creating i2c object...")
    i2c = I2C(0,scl=Pin("I2C_SCL"), sda=Pin("I2C_SDA"))

    print("Creating mcp object...")
    mcp = MCP23008(i2c, address=I2C_ADDR)

    print("Creating I2cLcd object...")
    lcd = I2cLcd(mcp, I2C_NUM_ROWS, I2C_NUM_COLS)
    #print("main: await asyncio.sleep_ms(5) ...")
    #await asyncio.sleep_ms(5000)
    #print("main: ... done")
    print("Running lcd.reset()...")
    await lcd.reset()

    msg = "LCD Initialised\n"
    #print("calling lcd.putstr('{}')?".format(msg))
    lcd.putstr(msg)

    return((keypad,lcd))


def display_key_press(keypad,lcd,star_countdown):
        lcd.move_to(14,0)
        lcd.putstr("{:d}".format(star_countdown))
        lcd.move_to(0,1)
        lcd.putstr("activeRow: {:d}".format(keypad.activeRow))
        lcd.move_to(0,2)
        lcd.putstr("activeCol: {:d}".format(keypad.activeCol))
        lcd.move_to(0,3)
        lcd.putstr('keyPressed: "{:s}"'.format(keypad()))

async def display_main_menu(lcd):
    await lcd.clear()
    lcd.putstr("Main menu:\n")
    lcd.putstr("0: keypad test\n")

async def loop(keypad,lcd,deadManSwitch):

    print("loop: Respond to user commands...")

    state = "main"
    await display_main_menu(lcd)

    while True:
        await keypad.keyPressed.wait()
        keypad.keyPressed.clear()
        deadManSwitch.trigger()
        key = keypad()

        if state == "main" :
            if (key == '0') :
                state = "keypad test"
                star_countdown = 3 # Multi star key to exit keypad test
                await lcd.clear()
                lcd.putstr("Press key:")
                lcd.move_to(14,0)
                lcd.putstr("{:d}".format(star_countdown))
            else :
                lcd.clearline(3)
                lcd.putstr("{:s}: not valid".format(key))
        elif state == "keypad test" :
            if key == '*' :
                star_countdown -= 1
            else :
                star_countdown = 3
            display_key_press(keypad,lcd,star_countdown)
            if star_countdown == 0 :
                await asyncio.sleep(1)
                lcd.clearline(0)
                lcd.putstr("keypad test: exit")
                await asyncio.sleep(1)
                state = "main"
                await display_main_menu(lcd)


async def main():
    print("initialising...")
    (keypad,lcd) = await init()

    deadManSwitch = Delay_ms(duration=20000)

    print("starting loop...")
    looptask = asyncio.create_task(loop(keypad,lcd,deadManSwitch))
    deadManSwitch.trigger()
    print("main(): awaiting deadManSwitch...")
    await deadManSwitch.wait()
    print("main(): deadManSwitch detected...")
    await lcd.clear()
    lcd.putstr("deadManSwitch:\nExiting...")

asyncio.run(main())

print("main() exited...")
