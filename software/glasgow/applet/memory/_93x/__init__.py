import logging
import argparse
import struct

from ...interface.microwire_master import MicrowireMasterApplet
from ....support.logging import *
from ... import *


class Memory93xInterface:
    def __init__(self, interface, logger, address_width=8, data_width=16):
        self.lower       = interface
        self._logger     = logger
        self._level      = logging.DEBUG if self._logger.name == __name__ else logging.TRACE
        self._addr_width = address_width
        self._data_width = data_width

    def _log(self, message, *args):
        self._logger.log(self._level, "93x: " + message, *args)

    def _gen_cmd(self, opcode, addr, data=None):
        fields = [
            (1, 1),
            (opcode, 2),
            (addr, self._addr_width),
        ]

        if data is not None:
            fields.append( (data, self._data_width) )

        nbits  = sum([x[1] for x in fields])
        nbytes = (nbits + 7) >> 3
        vbytes = bytearray(nbytes)

        ofs = 0
        for f in fields:
            for i in range(f[1]):
                byte_idx = (ofs + i) >> 3
                bit_idx  = 7 - ((ofs + i) & 7)
                if f[0] & (1 << (f[1] - i - 1)):
                    vbytes[byte_idx] |= 1 << bit_idx
            ofs += f[1]

        return vbytes, nbits

    def _get_data(self, vbytes):
        data = 0
        ofs = 1 + 2 + self._addr_width
        for i in range(self._data_width):
            byte_idx = (ofs + i) >> 3
            bit_idx  = 7 - ((ofs + i) & 7)
            if vbytes[byte_idx] & (1 << bit_idx):
                data |= 1 << (self._data_width - i - 1)
        return data

    def _to_bytes(self, words):
        return struct.pack('<%d%s' % (len(words), 'H' if self._data_width==16 else 'B'), *words)

    def _to_words(self, data):
        return struct.unpack('<%d%s' % (len(data) / (self._data_width/8), 'H' if self._data_width==16 else 'B'), data)

    async def read(self, addr, length):
        self._log("addr=%#06x", addr)

        data_words = []

        while len(data_words) < length:
            cmd_data, cmd_len = self._gen_cmd(2, addr+len(data_words), 0)

            resp = await self.lower.transfer(cmd_data, cmd_len)
            if resp is None:
                return None

            self._log("%d:%s:%s %04x", cmd_len, dump_hex(cmd_data), dump_hex(resp), self._get_data(resp))

            data_words.append( self._get_data(resp) )

        return self._to_bytes(data_words)

    async def write(self, addr, data):
        self._log("addr=%#06x", addr)

        data_words = self._to_words(data)
        ofs = 0

        while ofs < len(data_words):
            cmd_data, cmd_len = self._gen_cmd(1, addr+ofs, data_words[ofs])

            resp = await self.lower.transfer(cmd_data, cmd_len)
            if resp is None:
                return False

            self._log("%d:%s:%s %04x", cmd_len, dump_hex(cmd_data), dump_hex(resp), self._get_data(resp))

            for i in range(100):
                resp = await self.lower.transfer(b'\x00', 8)
                if resp is None:
                    return False

                if resp == b'\xff':
                    break

            if resp != b'\xff':
                return False

            ofs += 1

        return True


class Memory93xApplet(MicrowireMasterApplet, name='memory-93x'):
    logger = logging.getLogger(__name__)
    help = "read and write 93-series microwire EEPROM memories"
    default_addr_width = 8
    default_data_width = 16
    description = """
     Read and write memories compatible with 93-series EEPROM memories.

     # Layout

     Different memories can have different layouts (data width x8 or x16) and so
     different address length. This can depend on the model of the chip, or of the
     state of an `ORG` pin.

     The default configuration of this applet is an address width of {addr_width}
     with a data width (word size) of {data_width} bits. If this doesn't match your chip,
     you'll need to use the --addr-width and/or --data-width to adjust.

     (note that 16 bits word will be loaded/written to disk as little-endian)

     # Pinout

     The pinout of a typical 93-series IC in DIP/SOIC is as follows:

         * 8-pin (DIP/SOIC/SSOP) : CS=1, CLK=2, DI=3, DO=4, GND=5, ORG=6, NC=7, VCC=8
         * 8-pin (rotated SOIC)  : NC=1, VCC=2, CS=3, CLK=4, DI=5, DO=6, GND=7, ORG=8
         * 6-pin (SOT-23)        : DO=1, GND=2, DI=3, CLK=4, CS=5, VCC=6
    """.format(addr_width=default_addr_width, data_width=default_data_width)

    @classmethod
    def add_run_arguments(cls, parser, access):
        super().add_run_arguments(parser, access)

        parser.add_argument(
            "-A", "--address-width", type=int, default=cls.default_addr_width,
            help="number of address bits")
        parser.add_argument(
            "-D", "--data-width", type=int, choices=[8,16], default=cls.default_data_width,
            help="number of data bits / word-size")

    async def run(self, device, args):
        microwire_iface = await super().run(device, args)
        return Memory93xInterface(
            microwire_iface, self.logger,
            address_width = args.address_width,
            data_width = args.data_width,
        )

    @classmethod
    def add_interact_arguments(cls, parser):
        def address(arg):
            return int(arg, 0)
        def length(arg):
            return int(arg, 0)
        def hex_bytes(arg):
            return bytes.fromhex(arg)

        p_operation = parser.add_subparsers(dest="operation", metavar="OPERATION")

        p_read = p_operation.add_parser(
            "read", help="read memory")
        p_read.add_argument(
            "address", metavar="ADDRESS", type=address,
            help="read memory starting at address ADDRESS, with wraparound")
        p_read.add_argument(
            "length", metavar="LENGTH", type=length,
            help="read LENGTH bytes from memory")
        p_read.add_argument(
            "-f", "--file", metavar="FILENAME", type=argparse.FileType("wb"),
            help="write memory contents to FILENAME")

        p_write = p_operation.add_parser(
            "write", help="write memory")
        p_write.add_argument(
            "address", metavar="ADDRESS", type=address,
            help="write memory starting at address ADDRESS")
        g_data = p_write.add_mutually_exclusive_group(required=True)
        g_data.add_argument(
            "-d", "--data", metavar="DATA", type=hex_bytes,
            help="write memory with DATA as hex bytes")
        g_data.add_argument(
            "-f", "--file", metavar="FILENAME", type=argparse.FileType("rb"),
            help="write memory with contents of FILENAME")

    async def interact(self, device, args, m93x_iface):
        if args.operation == "read":
            data = await m93x_iface.read(args.address, args.length)
            if data is None:
                raise GlasgowAppletError("error while reading from memory")

            if args.file:
                args.file.write(data)
            else:
                print(data.hex())

        if args.operation == "write":
            if args.data is not None:
                data = args.data
            if args.file is not None:
                data = args.file.read()

            success = await m93x_iface.write(args.address, data)
            if not success:
                raise GlasgowAppletError("error while writing to memory")
