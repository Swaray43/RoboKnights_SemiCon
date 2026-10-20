# quartus_create_project.tcl
# Usage:  quartus_sh -t <repo>/bridge_top/quartus_create_project.tcl
# Optional env vars:  QUARTUS_FAMILY (default "Arria 10"), QUARTUS_DEVICE (default AUTO)
# The design is 512 bits wide with unrolled CRC-64/FEC; Cyclone parts are likely too small.
# Arria 10 / Stratix / Agilex need Quartus Prime Standard or Pro.

load_package flow
set here [file normalize [file dirname [info script]]]
set root [file normalize "$here/.."]
set family [expr {[info exists ::env(QUARTUS_FAMILY)] ? $::env(QUARTUS_FAMILY) : "Arria 10"}]
set device [expr {[info exists ::env(QUARTUS_DEVICE)] ? $::env(QUARTUS_DEVICE) : "AUTO"}]

file mkdir "$root/quartus_bridge"
cd "$root/quartus_bridge"
project_new semicon_bridge -overwrite

set_global_assignment -name FAMILY $family
set_global_assignment -name DEVICE $device
set_global_assignment -name TOP_LEVEL_ENTITY bridge_top
set_global_assignment -name SEARCH_PATH "$root/src-link_layer/hdl"
set_global_assignment -name VERILOG_INPUT_VERSION SYSTEMVERILOG_2005
set_global_assignment -name SDC_FILE "$here/bridge_top.sdc"

set fh [open "$here/bridge_top.f" r]
foreach line [split [read $fh] "\n"] {
    set line [string trim $line]
    if {$line eq "" || [string match "//*" $line] || [string match "+incdir+*" $line]} continue
    set_global_assignment -name VERILOG_FILE "$root/$line"
}
close $fh

project_close
puts "Project created: $root/quartus_bridge/semicon_bridge.qpf  (family $family, top bridge_top)"
puts "Compile with:  quartus_sh --flow compile semicon_bridge   (from that folder)"
puts "Simulate with Questa: vlog +define+SIMULATION -f bridge_top/bridge_top.f -f bridge_top/bridge_top_tb.f"
