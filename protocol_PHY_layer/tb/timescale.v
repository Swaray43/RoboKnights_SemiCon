// Compiled FIRST by run_all.py so every module in a bench gets the same
// 1ns/1ps timescale (the same setting tb_pcie_phy_top already uses).
// Without it Icarus defaults to 1 s units for files with no `timescale,
// which crashed vvp on some Windows builds (exit code 0xC0000005).
`timescale 1ns/1ps
