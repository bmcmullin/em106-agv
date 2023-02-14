# nano_i2c_lcd_test-INLINE.py
# barry.mcmullin@dcu.ie

# Test script for HD44780 LCD display (via I2C) on EM106 nano33 MCU

# Derived from: https://peppe8o.com/using-i2c-lcd-display-with-raspberry-pi-pico-and-micropython/

# i2c_lcd.py modified for mcp23008 port expander (DCU EM106 nano33-MCU) using
# mcp23008.py from: https://github.com/CrankshawNZ/Micropython/blob/master/mcp23008.py

# Modified to put third party packages inline (to facilitate direct execution in OpenMV)

import gc

import time
    # Non u- version preferred in general!
    # see: https://docs.micropython.org/en/latest/library/index.html#python-standard-libraries-and-micro-libraries
    # NB: nrf mp port has only a limited subset of time functionality <shrug>:
    #  https://github.com/openmv/micropython/blob/f72d922af0351ec48e0846be95f1179bc94cff73/ports/nrf/modules/utime/modutime.c
    # sleep_ms(), sleep_us(), ticks_ms(), ticks_us(), ticks_add(), ticks_diff()

from machine import Pin, I2C

#from mcp23008 import MCP23008
# INLINE (unused functionality pruned)

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

# from lcd_api import LcdApi
# INLINE (unused functionality pruned)
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

    def __init__(self, num_lines, num_columns):
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
        self.display_off()
        self.clear()
        self.hal_write_command(self.LCD_ENTRY_MODE | self.LCD_ENTRY_INC)
        self.hide_cursor()
        self.display_on()

    def clear(self):
        """Clears the LCD display and moves the cursor to the top left
        corner.
        """
        self.hal_write_command(self.LCD_CLR)
        self.hal_write_command(self.LCD_HOME)
        self.cursor_x = 0
        self.cursor_y = 0

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
                pass
            else:
                self.cursor_x = self.num_columns
        else:
            self.hal_write_data(ord(char))
            self.cursor_x += 1
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
        for char in string:
            self.putchar(char)

    def custom_char(self, location, charmap):
        """Write a character to one of the 8 CGRAM locations, available
        as chr(0) through chr(7).
        """
        location &= 0x7
        self.hal_write_command(self.LCD_CGRAM | (location << 3))
        self.hal_sleep_us(40)
        for i in range(8):
            self.hal_write_data(charmap[i])
            self.hal_sleep_us(40)
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

    def hal_sleep_us(self, usecs):
        """Sleep for some time (given in microseconds)."""
        time.sleep_us(usecs)

#from i2c_lcd_mcp23008 import I2cLcd
# INLINE (unused functionality pruned)

# DCU nano33-MCU pin definitions
MASK_RS = 0x01       # P0
MASK_RW = 0x02       # P1
MASK_E  = 0x04       # P2

SHIFT_DATA = 3       # P3-P6

class I2cLcd(LcdApi):

    # Implements a HD44780 character LCD connected via MCP23008

    def __init__(self, mcp, num_lines, num_columns):
        self.mcp = mcp
        self.mcp.IODIR=bytes(b"\x80") # MCP GPIO P0-P6 as outputs
        self.mcp.GPIO=bytes([0])
        time.sleep_ms(20)   # Allow LCD time to powerup
        # Send reset 3 times
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        time.sleep_ms(5)    # Need to delay at least 4.1 msec
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        time.sleep_ms(1)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        time.sleep_ms(1)
        # Put LCD into 4-bit mode
        self.hal_write_init_nibble(self.LCD_FUNCTION)
        time.sleep_ms(1)
        LcdApi.__init__(self, num_lines, num_columns)
        cmd = self.LCD_FUNCTION
        if num_lines > 1:
            cmd |= self.LCD_FUNCTION_2LINES
        self.hal_write_command(cmd)
        gc.collect()

    def hal_write_init_nibble(self, nibble):
        # Writes an initialization nibble to the LCD.
        # This particular function is only used during initialization.
        byte = ((nibble >> 4) & 0x0f) << SHIFT_DATA
        self.mcp.GPIO = bytes([byte | MASK_E])
        self.mcp.GPIO = bytes([byte])
        gc.collect()

    def hal_write_command(self, cmd):
        # Write a command to the LCD. Data is latched on the falling edge of E.
        byte = (((cmd >> 4) & 0x0f) << SHIFT_DATA)
        self.mcp.GPIO = bytes([byte | MASK_E])
        self.mcp.GPIO = bytes([byte])
        byte = ((cmd & 0x0f) << SHIFT_DATA)
        self.mcp.GPIO = bytes([byte | MASK_E])
        self.mcp.GPIO = bytes([byte])
        if cmd <= 3:
            # The home and clear commands require a worst case delay of 4.1 msec
            time.sleep_ms(5)
        gc.collect()

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
        gc.collect()



############################

# MCU LCD test code "proper"

print("\nStarting...")

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

itworks = "It Works! Starting test cycle..."
print("lcd.putstr('{}')?".format(itworks))
lcd.putstr(itworks)
time.sleep_ms(2000)

test = 0
while True:
    # Loop until interrupted via OpenMV!
    lcd.clear()
    if test == 0:
        tstr = "Test #{:1d}: Turning cursor on".format(test)
        print(tstr)
        lcd.putstr(tstr)
        time.sleep_ms(1000)
        lcd.show_cursor()
    if test == 1:
        tstr = "Test #{:1d}: Turning cursor off".format(test)
        print(tstr)
        lcd.putstr(tstr)
        time.sleep_ms(1000)
        lcd.hide_cursor()
    if test == 2:
        tstr = "Test #{:1d}: Turning blink cursor on".format(test)
        print(tstr)
        lcd.putstr(tstr)
        time.sleep_ms(1000)
        lcd.blink_cursor_on()
    if test == 3:
        tstr = "Test #{:1d}: Turning blink cursor off".format(test)
        print(tstr)
        lcd.putstr(tstr)
        time.sleep_ms(1000)
        lcd.blink_cursor_off()
    if test == 4:
        tstr = "Test #{:1d}: Turning display off".format(test)
        print(tstr)
        lcd.putstr(tstr)
        time.sleep_ms(1000)
        lcd.display_off()
    if test == 5:
        lcd.display_on()
        tstr = "Test #{:1d}: Turned display on".format(test)
        print(tstr)
        lcd.putstr(tstr)
        time.sleep_ms(1000)
    if test == 6:
        lcd.display_on()
        tstr = "Test #{:1d}: Filling display".format(test)
        print(tstr)
        lcd.putstr(tstr)
        time.sleep_ms(1000)
        lcd.clear()
        string = ""
        for x in range(32, 32+I2C_NUM_ROWS*I2C_NUM_COLS):
            string += chr(x)
        lcd.putstr(string)
    test = ((test + 1) % 7) # cycle through 0-6
    time.sleep_ms(2000)
