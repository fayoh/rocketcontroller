# MIT License
#
# Copyright (c) 2018 Daniel Bengtsson
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Implementation of the RFM69HCW driver class.
    Currently depends on raspberry pi gpio lib, but it should be trivial
    to change to a generic lib or direct manipulation through sysfs
"""

import time
import RPi.GPIO as GPIO
import spidev

class RFM69HCW:
    """RFM69HCW driver
    The DEFAULTSETTINGS member can be used to set register values that shall be
    written on init.
    """

    #Version of the chip the module is tested on
    # Bits 3-0 is metal mask revision and may change within compatible chips?
    CHIPVERSION = 0x24

    REGFIFO = 0x00
    REGOPMODE = 0x01
    REGDATAMODUL = 0x02
    REGBITRATEMSB = 0x03
    REGBITRATELSB = 0x04
    REGFDEVMSB = 0x05
    REGFDEVLSB = 0x06
    REGFRFMSB = 0x07
    REGFRFMID = 0x08
    REGFRFLSB = 0x09
    REGOSC1 = 0x0A
    REGAFCCTRL = 0x0B
    REGLISTEN1 = 0x0D
    REGLISTEN2 = 0x0E
    REGLISTEN3 = 0x0F
    REGVERSION = 0x10
    REGPALEVEL = 0x11
    REGPARAMP = 0x12
    REGOCP = 0x13
    REGLNA = 0x18
    REGRXBW = 0x19
    REGAFCBW = 0x1A
    REGOOKPEAK = 0x1B
    REGOOKAVG = 0x1C
    REGOOKFIX = 0x1D
    REGAFCFEI = 0x1E
    REGAFCMSB = 0x1F
    REGAFCLSB = 0x20
    REGFEIMSB = 0x21
    REGFEILSB = 0x22
    REGRSSICONFIG = 0x23
    REGRSSIVALUE = 0x24
    REGDIOMAPPING1 = 0x25
    REGDIOMAPPING2 = 0x26
    REGIRWFLAGS1 = 0x27
    REGIRQFLAGS2 = 0x28
    REGRSSITHRESH = 0x29
    REGRXTIMEOUT1 = 0x2A
    REGRXTIMEOUT2 = 0x2B
    REGPREAMBLEMSB = 0x2C
    REGPREAMBLELSB = 0x2D
    REGSYNCCONFIG = 0x2E
    REGSYNCVALUE1 = 0x2F
    REGSYNCVALUE2 = 0x30
    REGSYNCVALUE3 = 0x31
    REGSYNCVALUE4 = 0x32
    REGSYNCVALUE5 = 0x33
    REGSYNCVALUE6 = 0x34
    REGSYNCVALUE7 = 0x35
    REGSYNCVALUE8 = 0x36
    REGPACKETCONFIG1 = 0x37
    REGPAYLOADLENGTH = 0x38
    REGNODEADDRS = 0x39
    REGBROADCASTADRS = 0x3A
    REGAUTOMODES = 0x3B
    REGFIFOTHRESH = 0x3C
    REGPACKETCONFIG2 = 0x3D
    REGAESKEY1 = 0x3E
    REGAESKEY2 = 0x3F
    REGAESKEY3 = 0x40
    REGAESKEY4 = 0x41
    REGAESKEY5 = 0x42
    REGAESKEY6 = 0x43
    REGAESKEY7 = 0x44
    REGAESKEY8 = 0x45
    REGAESKEY9 = 0x46
    REGAESKEY10 = 0x47
    REGAESKEY11 = 0x48
    REGAESKEY12 = 0x49
    REGAESKEY13 = 0x4A
    REGAESKEY14 = 0x4B
    REGAESKEY15 = 0x4C
    REGAESKEY16 = 0x4D
    REGTEMP1 = 0x4E
    REGTEMP2 = 0x4F
    REGTESTLNA = 0x58
    REGTESTPA1 = 0x5A
    REGTESTPA2 = 0x5C
    REGTESTDAGC = 0x6F
    REGTESTAFC = 0x71

    REGLNARECVAL = 0x88
    REGRXBWRECVAL = 0x55
    REGAFCBWRECVAL = 0x8B
    REGDIOMAPPING2RECVAL = 0x07
    REGRSSITHRESHRECVAL = 0xE4
    REGSYNCVALUERECVAL = 0x01
    REGFIFOTHRESHRECVAL = 0xF
    REGTESTDAGCRECVAL = 0x30

    # Recommended register values from the datasheet
    RECVALS = {REGLNA:REGLNARECVAL,
               REGRXBW:REGRXBWRECVAL,
               REGAFCBWRECVAL:REGAFCBWRECVAL,
               REGDIOMAPPING2RECVAL:REGDIOMAPPING2RECVAL,
               REGRSSITHRESHRECVAL:REGRSSITHRESHRECVAL,
               REGSYNCVALUERECVAL:REGSYNCVALUERECVAL,
               REGFIFOTHRESHRECVAL:REGFIFOTHRESHRECVAL,
               REGTESTDAGCRECVAL:REGTESTDAGCRECVAL}
    DEFAULTSETTINGS = {}

    def __init__(self, spibus, rxpayloadreadypin, resetpin):
        self.spi = None
        self.rxpayloadreadypin = rxpayloadreadypin
        self.resetpin = resetpin
        self.rx_callbacks = []
        self.package_size = 10
        self.node_address = 0x01
        self.broadcast_address = 0xFF
        self.rssi = -127
        self.mode = 'rx'
        self._init_spi(spibus)
        self._init_gpio(rxpayloadreadypin, resetpin)

    def init_rfm69hcw(self):
        """
        Reset the chip and write default register values
        If you know that your chip is connected and setup correctly you
        can skip this
        The chip will be in rx mode after return of this
        """
        # Start with resetting the module to a known state
        self._reset()
        # Verify that a corect rfm69 chip is attached to the spi bus
        if (self._read_reg(RFM69HCW.REGVERSION) != RFM69HCW.CHIPVERSION):
            raise Exception("Incompatible chip version")
        # Write recommended values and any overridden values
        for reg, val in  RFM69HCW.RECVALS:
            self._write_reg(reg, val)
        for reg, val in  RFM69HCW.DEFAULTSETTINGS:
            self._write_reg(reg, val)
        self.rx_mode()

    def _reset(self):
        """
        Reset the chip to a known state by toggling the reset pin
        high for 100us and then waiting 5ms
        """
        # It seems like python can not reliably sleep for less than
        # tenish milliseconds, so let's use 15
        GPIO.output(self.resetpin, GPIO.HIGH)
        time.sleep(0.015)
        GPIO.output(self.resetpin, GPIO.LOW)
        time.sleep(0.015)

    def _init_spi(self, spibus):
        self.spi = spidev.SpiDev(spibus)
        self.spi.max_speed_hz = 500000

    def _init_gpio(self, txreadypin, resetpin):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(resetpin, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(txreadypin, GPIO.IN)
        GPIO.add_event_detect(txreadypin,
                              GPIO.RISING,
                              callback=self._rx_payload_ready)

    def _rx_payload_ready(self, _channel):
        print('Payload ready to read')
        # Read RSSI
        self.rssi = -self._read_reg(RFM69HCW.REGRSSIVALUE)/2
        data = self._read_reg(RFM69HCW.REGFIFO, self.package_size)
        for callback in self.rx_callbacks:
            callback(data)

    def _write_reg(self, reg, data):
        write_reg = 0x80 | reg
        self.spi.writebytes(data.insert(0, write_reg))

    def _read_reg(self, reg, length=1):
        data = [reg]+[0x0]*length
        return self.spi.xfer2(data)

    def subscribe_to_rx_data(self, callback):
        """Add a callback function for when a package is received"""
        self.rx_callbacks.append(callback)

    def send_package(self, data):
        """Send a package over the wireless link"""
        # to tx mode
        self._write_reg(RFM69HCW.REGFIFO, data)
        # Do stuff to actually send it

    def rx_mode(self):
        """Set module in RX (listening)  mode"""
        # Find modeval
        self._write_reg(RFM69HCW.REGOPMODE, 0x01)
        self.mode = 'rx'

    def tx_mode(self):
        """Set module in TX (transmitting) mode"""
        # Find modeval
        self._write_reg(RFM69HCW.REGOPMODE, 0x02)
        self.mode = 'tx'

    def set_spi_max_clock(self, clkspeed):
        """Set maximum clock frequency for the SPI bus"""
        self.spi.max_speed_hz = clkspeed
