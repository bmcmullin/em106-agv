# as_i2c_lcd_mcp23008.py
# asyncio version of i2c_lcd_mcp23008.
#
# barry.mcmullin@dcu.ie
# Last modified: 12 Feb 2023

import gc
import uasyncio as asyncio
from mcp23008 import MCP23008

# DCU EM106 nano33-MCU pin definitions (via MCP23008 I2C port expander)
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
        await asyncio.sleep_ms(20)   # Allow LCD time to powerup
        # Send reset 3 times
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        await asyncio.sleep_ms(5)    # Need to delay at least 4.1 msec
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        await asyncio.sleep_ms(1)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        await asyncio.sleep_ms(1)
        # Put LCD into 4-bit mode
        self.hal_write_init_nibble(self.LCD_FUNCTION)
        await asyncio.sleep_ms(1)
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
            await asyncio.sleep_ms(5)
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
