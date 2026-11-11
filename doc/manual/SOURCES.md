# Sources of the manual's facts

Every number and name in the manual is taken from the repository at the documented revision. This
file records where each chapter's facts come from, so that a reviewer can re-check them and a
maintainer knows what to update when the code changes.

| Chapter / section | Sources |
|---|---|
| 00 Front matter | `doc/manual/manual.yaml`, `README.md` |
| 01 Introduction | `README.md`, `setup.py`, `CONTRIBUTORS`, `liteiclink/serdes/*.py`, `liteiclink/serwb/*.py`, `liteiclink/serdes/serdes_ecp5.py` |
| 02 Serial links primer | `liteiclink/serdes/{gtx_7series,clock_aligner,serdes_ecp5,gtr12_gw5}.py`, `bench/serdes/test_prbs.py`; LiteX `litex/soc/cores/{prbs,code_8b10b}.py` |
| 03 FPGA transceivers | `liteiclink/serdes/*_init.py`, `liteiclink/serdes/{gtp_7series,gty_ultrascale,gth4_ultrascale,common,serdes_ecp5,gtr12_gw5}.py`, `bench/serdes/{kc705,sqrl_acorn}.py`; LiteICLink history (Acorn reference clock) |
| 04 Wishbone over serial | `liteiclink/serwb/{phy,s7serdes,kuserdes,genphy,datapath,common,core,mmap}.py`, `doc/serwb.md` |
| 05 SerDes architecture | `liteiclink/serdes/{common,clock_aligner,gtx_7series,gtp_7series,gty_ultrascale}.py`, `liteiclink/serdes/*_init.py`, `bench/serdes/test_prbs.py`, `test/test_serdes_xilinx.py`; LiteX `litex/soc/cores/{prbs,code_8b10b}.py` |
| 06 Xilinx 7-Series | `liteiclink/serdes/{gtp,gtx,gth}_7series.py` and `*_7series_init.py`, `bench/serdes/{sqrl_acorn,kc705}.py`, `test/test_serdes_{xilinx,init}.py` |
| 07 Xilinx UltraScale | `liteiclink/serdes/{gth3,gth4,gth,gty}_ultrascale.py` and `*_ultrascale_init.py`, `bench/serdes/{kcu105,xcu1525}.py`, `test/test_gty_qpll_config.py`, `test/test_serdes_xilinx.py` |
| 08 Lattice ECP5 | `liteiclink/serdes/serdes_ecp5.py`, `bench/serdes/{ecpix5,versa_ecp5}.py`, `test/test_serdes_ecp5.py`; USB3-PIPE `usb3_pipe/serdes.py`, LiteSATA `litesata/phy/ecp5sataphy.py`; LitePCIe `litepcie/phy/soft/serdes/ecp5.py`; Yumewatari `yumewatari/gateware/platform/lattice_ecp5.py` and katsuo-pcie `katsuo/pcie/phy/ecp5_serdes.py` (PCI Express mode attributes); Lattice TN1261 (receiver detection) |
| 09 Gowin GW5 | `liteiclink/serdes/gtr12_gw5.py`, `test/test_serdes_gtr12.py`; USB3-PIPE `usb3_pipe/serdes.py`, `usb3_pipe/gw5/` (location of the settings) |
| 10 SerWB architecture | `liteiclink/serwb/{core,packet,mmap,phy,genphy,s7serdes,kuserdes,efinixserdes}.py`, `bench/serwb/{nexys_video,demo/*}.py`, `doc/serwb.md`; LiteX `litex/soc/interconnect/packet.py` |
| 11 SerWB PHYs | `liteiclink/serwb/{phy,s7serdes,kuserdes,efinixserdes,genphy,common,datapath,scrambler}.py`, `bench/serwb/{nexys_video,ti60_f225_dev_kit,t120_bga576_dev_kit}.py`, `doc/serwb.md` |
| 12 SerWB protocol | `liteiclink/serwb/{packet,mmap,etherbone,core}.py`, `doc/serwb.md`, `test/test_serwb_{link,packet}.py` |
| 13 Boards | `bench/serdes/*.py`, `bench/serwb/*.py`, `bench/serwb/demo/*.py`; LiteX-Boards platforms and targets of the benches; USB3-PIPE `README.md`; LiteX WR-NIC SPEC-A7 platform (photo) |
| 14 Getting started | `bench/serdes/{kc705,test_prbs}.py`, `bench/serwb/{sim,ulx3s,test_serwb}.py`, `test/test_serwb_link.py`, `.github/workflows/ci.yml`; LiteX `litex_repos.py`, `litex_client.py`, `litex_server.py` |
| 15 Integration | `liteiclink/serdes/{gtx_7series,gtp_7series,gty_ultrascale,gtr12_gw5,serdes_ecp5,common}.py`, `liteiclink/serwb/{phy,s7serdes,kuserdes,genphy,core,mmap}.py`, `bench/serwb/{nexys_video,demo/ecpix5,demo/icebreaker}.py`, `doc/serwb.md`, `doc/serdes.md`; LiteX `soc/integration/{builder,soc}.py` (`add_master`, `add_remapper`), `soc/interconnect/wishbone.py`, `soc/interconnect/axi/{axi_lite_to_wishbone,axi_full_to_wishbone}.py` (`SLVERR`, LiteX commit `4e0d72b84`); LiteX-Boards targets using LiteICLink |
| 16 Benches and examples | `bench/serdes/*.py` (targets, `test_prbs.py`), `bench/serwb/*.py` (targets, `test_serwb.py`, `resources.py`), `bench/serwb/demo/` (`ecpix5.py`, `icebreaker.py`, `README.md`), `liteiclink/serdes/serdes_ecp5.py` (1-bit `loopback`), `liteiclink/serdes/gtx_7series.py` (`ALIGN_COMMA_WORD`), LiteX-Boards platforms of the targets; elaboration of the SerDes targets without `--build` |
| 17 Simulation and tests | `test/*.py`, `bench/serwb/sim.py`, `.github/workflows/ci.yml`, `.github/workflows/doc.yml`; run of `bench/serwb/sim.py --test` and `--min` |
| 18 Debugging | `liteiclink/serdes/*_7series.py`, `*_ultrascale.py` (controls, clock cycles, DRP mux), `liteiclink/serdes/*_init.py` (watchdogs), `liteiclink/serdes/clock_aligner.py`, `liteiclink/serdes/common.py` (`DRPControl`), `liteiclink/serdes/serdes_ecp5.py` (`SerdesInit`, `SerDesECP5SCIReconfig`), `liteiclink/serdes/gtr12_gw5.py` (`add_status_csr`, `GTR12UPAR.add_csr`), `liteiclink/serwb/phy.py` (`_SerdesInit`), `liteiclink/serwb/common.py`, `liteiclink/serwb/core.py`, `liteiclink/serdes/gty_ultrascale.py` (`RXCDRHOLD`), `liteiclink/serwb/datapath.py`, `bench/serdes/test_prbs.py`, LiteX `litex/soc/cores/prbs.py` (`PRBSRX`), `analyzer.csv` of `bench/serdes/ecpix5.py`; litescope `litescope_cli.py` (default `--csr-csv`) |
| A Register reference | Generated: `doc/manual/registers.py` (manual-only SoCs), `doc/manual/csr/*.json`, `tools/csr_doc.py`; register definitions of `liteiclink/serdes/*.py`, `liteiclink/serwb/common.py`, `liteiclink/serwb/core.py` |
| B Build options | Constructors of `liteiclink/serdes/*.py` (PLLs, wrappers, `DRPControl`), `liteiclink/serwb/phy.py`, `liteiclink/serwb/genphy.py`, `liteiclink/serwb/core.py` (`SERWBCore`, `SERIOCore`), `liteiclink/serwb/mmap.py` |
| C Glossary | The chapters it points to |
| D Performance and resources | Computed from `liteiclink/serdes/*.py` (word widths), `liteiclink/serwb/packet.py`, `liteiclink/serwb/mmap.py`, `doc/serwb.md` (PHY rates, word counts, resource comparison); recorded results: commits `939f4ad`, `107782b`, `095d2a0`, `f29cd43`, `578074a`, `16f1e31`, `5d098d2`, `816996a`, `1ef0fff` |
| E Maintaining this manual | Manual kit |
