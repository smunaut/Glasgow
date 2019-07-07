import argparse
import logging
import math
import struct
from migen import *
from migen.genlib.cdc import *

from ....support.logging import *
from ....gateware.clockgen import *
from ... import *


CMD_XFER = 0x01


class MicrowireMasterBus(Module):
    def __init__(self, pads):
        self.cs   = Signal(reset=0)
        self.clk  = Signal(reset=0)
        self.mosi = Signal(reset=0)
        self.miso = Signal()

        self.comb += [
            pads.cs_t.oe.eq(1),
            pads.cs_t.o.eq(self.cs),
            pads.clk_t.oe.eq(1),
            pads.clk_t.o.eq(self.clk),
            pads.mosi_t.oe.eq(1),
            pads.mosi_t.o.eq(self.mosi),
        ]

        self.specials += MultiReg(pads.miso_t.i, self.miso)


class MicrowireMasterSubtarget(Module):
    def __init__(self, pads, out_fifo, in_fifo, period_cyc):
        self.submodules.bus = MicrowireMasterBus(pads)
        self.submodules.clkgen = ResetInserter()(ClockGen(period_cyc))

        oreg  = Signal(8)
        ireg  = Signal(8)

        self.comb += [
            self.bus.clk.eq(self.clkgen.clk),
            self.bus.mosi.eq(oreg[oreg.nbits - 1]),
        ]
        self.sync += [
            If(self.clkgen.stb_f,
                oreg[1:].eq(oreg),
                ireg.eq(Cat(self.bus.miso, ireg)),
            )
        ]

        cmd   = Signal(8)
        cnt_bytes = Signal(6)   # Number of bytes - 2 (-1 => last byte)
        cnt_bits  = Signal(3)   # Number of bits in last byte - 2
        bitno = Signal(4)

        self.submodules.fsm = FSM(reset_state="RECV-COMMAND")
        self.fsm.act("RECV-COMMAND",
            in_fifo.flush.eq(1),
            If(out_fifo.readable,
                out_fifo.re.eq(1),
                NextValue(cmd, out_fifo.dout),
                NextState("RECV-COUNT"),
            ),
        )
        self.fsm.act("RECV-COUNT",
            If(out_fifo.readable,
                out_fifo.re.eq(1),
                If(out_fifo.dout[0:3] == 0,
                    NextValue(cnt_bytes, out_fifo.dout[3:8] - 2),
                ).Else(
                    NextValue(cnt_bytes, out_fifo.dout[3:8] - 1),
                ),
                NextValue(cnt_bits,  out_fifo.dout[0:3] - 2),
                If(out_fifo.dout == 0,
                    NextState("RECV-COMMAND"),
                ).Else(
                    NextState("RECV-DATA"),
                ),
            ),
        )
        self.fsm.act("RECV-DATA",
            NextValue(self.bus.cs, 1),
            NextValue(oreg, out_fifo.dout),
            If(out_fifo.readable,
                out_fifo.re.eq(1),
                NextValue(cnt_bytes, cnt_bytes - 1),
                If(cnt_bytes[5],
                    NextValue(bitno, cnt_bits),
                ).Else(
                    NextValue(bitno, 6),
                ),
                NextState("TRANSFER"),
            ),
        )
        self.comb += self.clkgen.reset.eq(~self.fsm.ongoing("TRANSFER"))
        self.fsm.act("TRANSFER",
            If(self.clkgen.stb_f,
                NextValue(bitno, bitno - 1),
                If(bitno[3] == 1,
                    NextState("SEND-DATA")
                ),
            ),
        )
        self.comb += in_fifo.din.eq(ireg)
        self.fsm.act("SEND-DATA",
            If(in_fifo.writable,
                in_fifo.we.eq(1),
                If(cnt_bytes[5] & ~cnt_bytes[0],
                    NextValue(self.bus.cs, 0),
                    NextState("RECV-COMMAND"),
                ).Else(
                    NextState("RECV-DATA")
                )
            )
        )


class MicrowireMasterInterface:
    def __init__(self, interface, logger):
        self.lower   = interface
        self._logger = logger
        self._level  = logging.DEBUG if self._logger.name == __name__ else logging.TRACE

    def _log(self, message, *args):
        self._logger.log(self._level, "Microwire: " + message, *args)

    async def reset(self):
        self._log("reset")
        await self.lower.reset()

    async def transfer(self, data, nbits):
        assert nbits <= 256
        assert len(data) == (nbits + 7) >> 3

        self._log("xfer-out=%d:<%s>", nbits, dump_hex(data))

        await self.lower.write(struct.pack('BB', CMD_XFER, nbits))

        if nbits:
            await self.lower.write(data)
            data = await self.lower.read(len(data))
            if nbits & 7:
                data[-1] = (data[-1] << (8 - (nbits & 7))) & 0xff

        self._log("xfer-in=%d<%s>", nbits, dump_hex(data))

        return data


class MicrowireMasterApplet(GlasgowApplet, name="microwire-master"):
    logger = logging.getLogger(__name__)
    help = "initiate Microwire transactions"
    description = """
    Initiate transactions on the Microwire bus.

    Maximum transfer length is 256 bits.
    """

    __pins = ("cs", "clk", "mosi", "miso")

    @classmethod
    def add_build_arguments(cls, parser, access):
         super().add_build_arguments(parser, access)

         access.add_pin_argument(parser, "cs",   required=True)
         access.add_pin_argument(parser, "clk",  required=True)
         access.add_pin_argument(parser, "mosi", required=True)
         access.add_pin_argument(parser, "miso", required=True)

         parser.add_argument(
            "-b", "--bit-rate", metavar="FREQ", type=int, default=100,
            help="set Microwire bit rate to FREQ kHz (default: %(default)s)")


    def build(self, target, args, pins=__pins):
        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
        return iface.add_subtarget(MicrowireMasterSubtarget(
            pads=iface.get_pads(args, pins=pins),
            out_fifo=iface.get_out_fifo(),
            in_fifo=iface.get_in_fifo(auto_flush=False),
            period_cyc=self.derive_clock(input_hz=target.sys_clk_freq,
                                         output_hz=args.bit_rate * 1000,
                                         clock_name="master",
                                         min_cyc=4),
        ))

    async def run(self, device, args):
        iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)
        microwire_iface = MicrowireMasterInterface(iface, self.logger)
        return microwire_iface

    @classmethod
    def add_interact_arguments(cls, parser):
        def hex(arg): return bytes.fromhex(arg)

        parser.add_argument(
            "data", metavar="DATA", type=hex,
            help="hex bytes to transfer to the device")
        parser.add_argument(
            "nbits", metavar="N", type=int,
            help="number of bits to transfer to the device")

    async def interact(self, device, args, microwire_iface):
        data = await microwire_iface.transfer(args.data, args.nbits)
        print(data.hex())
