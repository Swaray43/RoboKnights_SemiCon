# vivado_create_project.tcl
# Usage (from anywhere):  vivado -mode batch -source <repo>/bridge_top/vivado_create_project.tcl
#                    or:  in the Vivado Tcl console:  source <repo>/bridge_top/vivado_create_project.tcl
# Optional: pick the part first, e.g. in the Tcl console:  set ::env(VIVADO_PART) xcu250-figd2104-2L-e
# Otherwise the script picks an installed part (Alveo first). The design is 512 bits wide with
# unrolled CRC-64/FEC, so small Artix parts will likely run out of LUTs.

set here [file normalize [file dirname [info script]]]
set root [file normalize "$here/.."]
# Part selection: VIVADO_PART env var if set, otherwise the first of these that is
# installed in this Vivado (Alveo parts first: the Alveo license covers only those),
# otherwise the largest UltraScale+ / 7-series part that is installed.
set candidates {xcu250-figd2104-2L-e xcu200-fsgd2104-2-e xcu280-fsvh2892-2L-e xcu55c-fsvh2892-2L-e
                xcu50-fsvh2104-2-e xcvu9p-flga2104-2L-e xcku5p-ffvb676-2-e xc7k325tffg900-2}
set part ""
if {[info exists ::env(VIVADO_PART)]} {
    set part $::env(VIVADO_PART)
    if {[llength [get_parts -quiet $part]] == 0} {
        puts "ERROR: VIVADO_PART=$part is not installed. Installed parts: [lrange [get_parts] 0 20] ..."
        return -code error "part not installed"
    }
} else {
    foreach c $candidates {
        if {[llength [get_parts -quiet $c]] > 0} { set part $c; break }
    }
    if {$part eq ""} {
        set all [get_parts -quiet]
        if {[llength $all] == 0} { return -code error "No devices installed in this Vivado" }
        set part [lindex $all 0]
        puts "WARNING: none of the preferred parts is installed; using $part (may be too small for this design)"
    }
}
puts "Using part: $part"

create_project -force semicon_bridge "$root/vivado_bridge" -part $part

proc read_list {root f} {
    set out {}
    set fh [open $f r]
    foreach line [split [read $fh] "\n"] {
        set line [string trim $line]
        if {$line eq "" || [string match "//*" $line] || [string match "+incdir+*" $line]} continue
        lappend out "$root/$line"
    }
    close $fh
    return $out
}

# ---- synthesis sources ----
add_files -norecurse -fileset sources_1 [read_list $root "$here/bridge_top.f"]
add_files -norecurse -fileset sources_1 "$root/src-link_layer/hdl/flit_defs.vh"
set_property file_type {Verilog Header} [get_files flit_defs.vh]
set_property include_dirs [list "$root/src-link_layer/hdl"] [get_filesets sources_1]
set_property top bridge_top [get_filesets sources_1]

# ---- constraints ----
add_files -norecurse -fileset constrs_1 "$here/bridge_top.xdc"

# ---- simulation ----
add_files -norecurse -fileset sim_1 [read_list $root "$here/bridge_top_tb.f"]
set_property include_dirs [list "$root/src-link_layer/hdl"] [get_filesets sim_1]
set_property verilog_define {SIMULATION} [get_filesets sim_1]
set_property top tb_bridge_top [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {all} -objects [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
puts "Project created in $root/vivado_bridge  (part $part, top bridge_top, sim top tb_bridge_top)"
puts "Run: launch_simulation    or    launch_runs synth_1 -jobs 4"
