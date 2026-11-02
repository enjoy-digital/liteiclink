# AGENTS.md: LiteICLink

Inter-chip links for LiteX: SerWB (Wishbone over a serial link) and transceiver wrappers (Xilinx
GTP/GTX/GTH/GTY, Lattice ECP5, Gowin GW5 GTR12) used by LiteEth, LiteSATA, LiteJESD204, USB3-PIPE.

Read the LiteX agent page first: https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents (framework map, conventions, how to
verify a change, contribute back). Index of the whole wiki for agents: https://github.com/enjoy-digital/litex/wiki/llms.txt.

## Where things are

- `liteiclink/serdes/`: transceiver wrappers per family (`gtp_7series.py`, `gtx_7series.py`, `gth*_ultrascale.py`, `gty_ultrascale.py`, `serdes_ecp5.py`, `gtr12_gw5.py`) and their init FSMs.
- `liteiclink/serwb/`: SerWB (PHY, packetizer, Etherbone-like core); `liteiclink/transceiver.py`.
- `bench/`: hardware benches; `test/`.

## Verify a change

```sh
python3 -m pytest -v                                                # what CI runs
```

- Transceiver wrappers are shared by several cores: a parameter change can affect LiteEth,
  LiteSATA, LiteJESD204 and USB3-PIPE users; say which families and rates were rebuilt or run.
- Reference clocks: routing a reference clock through the fabric into a GTP common can give
  build-dependent RX quality; prefer the dedicated clock paths (`refclk_from_fabric` options).
- GW5 GTR12 UPAR accesses need the CSR mode used by the vendor wrappers; GTY DRP access goes through
  a single DRP interface (debug tools must select it).

## Rules

- Style: https://github.com/enjoy-digital/litex/blob/master/doc/coding_style.md (match the
  surrounding file; `module: Description of the change.` commit titles; one change per commit).
- Never edit generated files (`build/`, `csr.h`, `csr.csv`, the Verilog): change the Python.
- One topic per pull request, from the default development branch, with what was tested and on
  which hardware; say that an agent wrote it and who supervised. Do not rename options or
  reformat unrelated code.
- Hardware: never flash a board or write its SPI flash unless the user asked; `--load` (volatile)
  is the default for experiments.

## Debug on hardware

Method: https://github.com/enjoy-digital/litex/wiki/Debug-A-SoC-On-Hardware (check the link and `--ident` first, registers before
traces, narrow LiteScope captures). `litex_mcp` gives the same loop as MCP tools (CSRs by name,
LiteScope captures, builds with parsed timing, simulation): https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents#drive-a-soc-through-mcp.
