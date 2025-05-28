# mcp3008.py Read ADC using SPI.
# For CS use any valid output pin.
# Channel numbers are 0-7.
# Returns a 10 bit integer.
# Based on a C program by Michalt Zalewski <lcamtuf@coredump.cx>
# http://dangerousprototypes.com/docs/MCP3008_8-channel_10-bit_ADC
# Author: Peter Hinch
# Copyright Peter Hinch 2016-2020
# Released under the MIT License (MIT) - see LICENSE file
# V0.5 20 Mar 2020 Support for arbitrarily assigned SPI pins

import os
from machine import SPI, Pin

# Initialise and return an SPI bus object.
# If a bus object is passed, ensure it is an SPI bus, else build a new one.
# This helper function is part of the library but not strictly needed if you pre-initialize SPI
# For simplicity in your main code, we pre-initialize SPI.
# However, the class __init__ below is flexible.

class MCP3008:
    def __init__(self, spi_bus, cs_pin, sck_pin_id=None, mosi_pin_id=None, miso_pin_id=None,
                 baudrate=100000, ref_voltage=3.3):
        """
        Args:
            spi_bus: SPI bus ID (int, e.g. 0 or 1) OR an initialised SPI object.
            cs_pin: chip select machine.Pin object.
            sck_pin_id, mosi_pin_id, miso_pin_id: Physical pin ID's for SPI bus.
                        Required if spi_bus is an int (bus ID). Ignored if spi_bus is an object.
            baudrate=100000: SPI clock speed.
            ref_voltage=3.3: Vref for Volts conversion.
        """
        if isinstance(spi_bus, int): # If spi_bus is an integer (bus ID)
            if sck_pin_id is None or mosi_pin_id is None or miso_pin_id is None:
                raise ValueError("If spi_bus is an ID, sck, mosi, and miso pins must be provided.")
            # Note: MicroPython's SPI constructor can take Pin objects directly for sck, mosi, miso
            self._spi = SPI(spi_bus, baudrate=baudrate, polarity=0, phase=0,
                            sck=Pin(sck_pin_id), mosi=Pin(mosi_pin_id), miso=Pin(miso_pin_id))
        elif isinstance(spi_bus, SPI): # If spi_bus is already an initialized SPI object
            self._spi = spi_bus
            # Optionally, re-initialize if baudrate is different or to ensure settings
            # self._spi.init(baudrate=baudrate, polarity=0, phase=0) # Be cautious with re-init
        else:
            raise ValueError("spi_bus must be an SPI bus ID (int) or an initialised machine.SPI object.")

        if not isinstance(cs_pin, Pin):
            raise ValueError("cs_pin must be a machine.Pin object.")
        self._cs = cs_pin
        self._cs.init(Pin.OUT, value=1)  # Ensure CS is output and deselects (high)

        self._out_buf = bytearray(3)
        self._in_buf = bytearray(3)
        self._ref_voltage = ref_voltage

    def _read_adc(self, channel_id):
        if not 0 <= channel_id <= 7:
            raise ValueError('MCP3008 channel must be 0-7')

        self._cs.value(0)  # Select slave (CS low)

        # Construct the command bytes
        # Byte 1: Start bit (always 0x01)
        self._out_buf[0] = 0x01
        # Byte 2: Configuration byte
        # SGL/DIFF = 1 (single-ended)
        # D2, D1, D0 = channel bits
        # (0b1000 | channel_id) then shifted left by 4 bits
        self._out_buf[1] = (0x08 | channel_id) << 4
        # Byte 3: Don't care (used to clock out data)
        self._out_buf[2] = 0x00

        self._spi.write_readinto(self._out_buf, self._in_buf) # Perform SPI transaction

        self._cs.value(1)  # Deselect slave (CS high)

        # Parse the 10-bit result from the received bytes
        # _in_buf[0] is often ignored or contains part of the command echo
        # _in_buf[1] contains the 2 most significant bits (and 6 leading null/garbage bits)
        # _in_buf[2] contains the 8 least significant bits
        result = ((self._in_buf[1] & 0x03) << 8) | self._in_buf[2]
        return result

    def read(self, channel_id):
        """Read the raw 10-bit ADC value from the specified channel."""
        return self._read_adc(channel_id)

    def reference_voltage(self, val=None):
        """Get or set the reference voltage for voltage calculations."""
        if val is not None:
            self._ref_voltage = val
        return self._ref_voltage

    def read_voltage(self, channel_id):
        """Read the ADC value and convert it to a voltage."""
        raw_value = self._read_adc(channel_id)
        return (raw_value * self._ref_voltage) / 1023.0 # Use 1023.0 for float division

    # Differential mode is less common for simple line followers, but included for completeness
    def read_diff(self, diff_pair_id): # diff_pair_id = 0..3
        # (CH0-CH1), (CH1-CH0), (CH2-CH3), (CH3-CH2), etc.
        # This is a simplified example, actual command byte for diff mode is different
        # For MCP3008, differential uses SGL/DIFF=0.
        # Command for CH_P = D2,D1,D0 and CH_N = D2',D1',D0'
        # For single chip it's usually set pairs (e.g., CH0+ CH1-)
        # Example: pair 0 means (CH0 as IN+, CH1 as IN-)
        # Command byte 2: (0b000 | diff_pair_id for MCP3002/4, more complex for MCP3008 diff) << 4
        # For MCP3008, it's simpler to just select single-ended for most applications.
        # This implementation detail for diff might need more specific MCP3008 datasheet reference
        # if true differential is needed. The existing single-ended `read` is robust.
        raise NotImplementedError("Differential read for MCP3008 needs specific command byte review for pairs.")