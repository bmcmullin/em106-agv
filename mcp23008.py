# mcp23008.py
# MCP23008 class
#
# Some functionality pruned.
#
# barry.mcmullin@dcu.ie
# Last modified: 12 Feb 2023

from machine import Pin, I2C

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
