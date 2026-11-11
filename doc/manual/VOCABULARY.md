# Vocabulary of the LiteICLink manual

Project-specific writing rules, completing the generic `STYLE.md` of the manual kit.

## Names of the parts

- *the project* is LiteICLink; its two halves are *the SerDes wrappers* (`liteiclink/serdes/`) and
  *SerWB* (`liteiclink/serwb/`).
- *transceiver* is the hard high-speed serial block of the FPGA (GTP, GTX, GTH, GTY, DCU, GTR12).
  *SerDes wrapper* (or *wrapper*) is a LiteICLink class that drives one transceiver channel
  (`GTX`, `GTY`, `SerDesECP5`, `GTR12`). "SerDes" alone is the generic function (serializer plus
  deserializer), not a class.
- *channel* is one TX and RX lane pair of a transceiver; *quad* (Xilinx, Gowin) and *dual* (Lattice
  DCU) are the vendor groups of channels that share common PLLs; keep the vendor word per family.
- PLLs as the vendors name them: *CPLL* (channel PLL), *QPLL* (quad PLL; *QPLL0* and *QPLL1* on
  UltraScale and Gowin), *PLL0* and *PLL1* on the GTP. The *PLL solver* is the search done by the
  PLL classes (`compute_config`).
- *reference clock* in prose, never "refclk" (allowed in code: `refclk`, `refclk_freq`).
- *line rate* in prose, in Gbps (`linerate` and `line_rate` only as code identifiers).
- *user clock* is the parallel-side clock of a channel, exposed as the `tx` and `rx` clock
  domains; a *word* is the data of one user-clock beat (20, 40 or 80 bits); a *symbol* is one
  10-bit 8b/10b code group; *raw symbols* are words sent or received without the fabric 8b/10b.
- *comma* is the K28.5 symbol used for alignment; a *K character* is a control symbol of 8b/10b;
  *PRBS7*, *PRBS15*, *PRBS31*; *BER* is the bit error ratio.
- *init FSM* is a reset and initialization sequence of a wrapper (`GTXTXInit`, `SerdesInit`); the
  *watchdog* is the timer that restarts it.
- *DRP* (Xilinx dynamic reconfiguration port), *SCI* (Lattice serial client interface), *UPAR*
  (the Gowin runtime register access port of a GTR12 quad): capitals, defined once in the
  transceiver primer; the expansion of UPAR is not spelled out.
- SerWB roles: *link master* and *link slave* are the PHY roles (`SERWBPHY(mode="master")` drives
  the link clock and runs the initialization, `mode="slave"` follows). *SoC side* is
  `SERWBCore(mode="slave")`, which exposes a bus slave to the local SoC; *remote side* is
  `SERWBCore(mode="master")`, which drives a bus master in the other FPGA.
- SerWB PHYs: *high-speed PHY* (`liteiclink/serwb/phy.py`, vendor IO serializers) and *generic
  PHY* (`liteiclink/serwb/genphy.py`, one bit per cycle on plain IOs).
- SerWB objects: *packet*, *port* (the destination number of a packet), *request*, *response*,
  *native protocol* (MMAP, the default), *Etherbone protocol* (deprecated); *SerIO* for the
  tunnelled IOs of `SERIOCore`.
- *register* in prose, never "CSR" (except the bus name "CSR bus" and identifiers).
- *target* is a board script of `bench/`; *host script* is a script of `bench/` run on the PC
  (`test_prbs.py`, `test_serwb.py`).

## Protocol names

- 8b/10b symbols as K28.5, D21.5 (no code font); line codes as *8b/10b*, *64b/66b*,
  *128b/132b*; running disparity as *RD* after its first use.
- Loopbacks as *near-end PCS*, *near-end PMA*, *far-end PMA*, *far-end PCS*.
- Bus names: *Wishbone*, *AXI*, *AXI-Lite*.
- Vendor documents by number (UG482, UG476, UG576, UG578, TN1261), with a section only
  when sure.

## Placeholders

- `<target>` is a target script name of `bench/serdes/` or `bench/serwb/` without `.py` (`kc705`,
  `nexys_video`). Defined once in the Conventions of the front matter.

## Hardware names

- FPGA families: Artix-7, Kintex-7, Virtex-7, Spartan-7, Zynq-7000, Kintex UltraScale, Kintex
  and Virtex UltraScale+, Zynq UltraScale+, Lattice ECP5-5G, Gowin GW5AST and GW5AT, Efinix
  Titanium and Trion, Lattice iCE40.
- Transceivers: GTP (Artix-7), GTX (Kintex-7, Virtex-7), GTH (Virtex-7 GTHE2, UltraScale GTHE3,
  UltraScale+ GTHE4), GTY (UltraScale+ GTYE4), DCU (ECP5-5G), GTR12 (GW5).
- Boards as LiteX-Boards names them: KC705, KCU105, XCU1525, SQRL Acorn CLE-215+ (then "the
  Acorn"), Versa ECP5, ECPIX-5, Nexys Video, Trion T120 BGA576 Dev Kit, Titanium Ti60 F225 Dev
  Kit, iCEBreaker, TrellisBoard, ULX3S, Tang Mega 138K Pro, Alibaba XCKU3P (then "the XCKU3P
  board").
