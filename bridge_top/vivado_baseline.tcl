# vivado_baseline.tcl -- Phase 2 Step 2 baseline: regression gate, synthesis, implementation, reports
#
# Usage (Vivado 2026.1, batch):
#   vivado -mode batch -source <repo>/bridge_top/vivado_baseline.tcl
# or in the Tcl console:  source <repo>/bridge_top/vivado_baseline.tcl
#
# Knobs (environment variables, all optional):
#   VIVADO_PART=<part>   part to use (else vivado_create_project.tcl picks an installed Alveo part)
#   BASELINE_SIM=0       skip the XSim regression gate (tb_bridge_top with perf_monitor, +define+PERF)
#   BASELINE_IMPL=0      stop after synthesis (no place & route)
#   BASELINE_JOBS=<n>    parallel jobs (default 8)
#
# Out-of-context build: bridge_top has ~1,900 signal ports (two 512-bit inputs, a 512-bit
# output), more than any package has pins. It is an IP block, so it is synthesized and
# placed out of context: no I/O buffers, no pin assignment, clocks from bridge_top.xdc.
#
# Everything lands in <repo>/vivado_bridge/baseline/:
#   sim_tb_bridge_top.log             XSim output incl. RESULT and PERF lines
#   synth_util.rpt / synth_util_hier.rpt / synth_timing.rpt
#   route_util.rpt / route_util_hier.rpt / route_timing.rpt / route_timing_paths.rpt
#   ram_util.rpt, clock_interaction.rpt, cdc.rpt
#   baseline_summary.txt              the numbers to paste into the Phase 2 doc
#   baseline_summary_<commit>.txt     same, named after the commit measured (Step 4/5 history)

set here [file normalize [file dirname [info script]]]
set root [file normalize "$here/.."]
proc envd {name def} { if {[info exists ::env($name)]} { return $::env($name) } else { return $def } }
set do_sim  [envd BASELINE_SIM 1]
set do_impl [envd BASELINE_IMPL 1]
set jobs    [envd BASELINE_JOBS 8]

# ---- 1. fresh project (same file lists / include dirs as the regular project) ----
source "$here/vivado_create_project.tcl"
set out "$root/vivado_bridge/baseline"
file mkdir $out

# ---- 2. regression gate: tb_bridge_top + perf_monitor in XSim ----
if {$do_sim} {
    add_files -norecurse -fileset sim_1 "$here/tb/perf_monitor.v"
    set_property file_type SystemVerilog [get_files perf_monitor.v]   ;# uses 'final'; xvlog reads .v as Verilog-2001
    set_property verilog_define {SIMULATION PERF} [get_filesets sim_1]
    set_property top tb_bridge_top [get_filesets sim_1]
    update_compile_order -fileset sim_1
    launch_simulation -simset sim_1 -mode behavioral     ;# runs to $finish (xsim.simulate.runtime = all)
    close_sim
    set simlog [glob -nocomplain "$root/vivado_bridge/semicon_bridge.sim/sim_1/behav/xsim/simulate.log"]
    if {$simlog ne ""} { file copy -force $simlog "$out/sim_tb_bridge_top.log" }
    set fh [open "$out/sim_tb_bridge_top.log" r]; set simtxt [read $fh]; close $fh
    if {![regexp {RESULT: PASS} $simtxt]} {
        puts "BASELINE: tb_bridge_top did not PASS in XSim -- fix that before trusting any numbers."
        return -code error "regression gate failed"
    }
    puts "BASELINE: tb_bridge_top PASS in XSim"
}

# ---- 3. synthesis, out of context ----
set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} -value {-mode out_of_context} -objects [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt [get_runs synth_1]
reset_run synth_1
launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { return -code error "synth_1 failed: see vivado_bridge/semicon_bridge.runs/synth_1/runme.log" }

open_run synth_1 -name synth_1
report_utilization                                           -file "$out/synth_util.rpt"
report_utilization -hierarchical -hierarchical_depth 5       -file "$out/synth_util_hier.rpt"
report_timing_summary -max_paths 10 -report_unconstrained    -file "$out/synth_timing.rpt"
catch { report_ram_utilization                               -file "$out/ram_util.rpt" }
report_clock_interaction                                     -file "$out/clock_interaction.rpt"
catch { report_cdc -details                                  -file "$out/cdc.rpt" }
set stage synth

# ---- 4. implementation (place & route), out of context ----
if {$do_impl} {
    launch_runs impl_1 -to_step route_design -jobs $jobs
    wait_on_run impl_1
    if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { return -code error "impl_1 failed: see vivado_bridge/semicon_bridge.runs/impl_1/runme.log" }
    open_run impl_1 -name impl_1
    report_utilization                                       -file "$out/route_util.rpt"
    report_utilization -hierarchical -hierarchical_depth 5   -file "$out/route_util_hier.rpt"
    report_timing_summary -max_paths 10                      -file "$out/route_timing.rpt"
    report_timing -max_paths 20 -nworst 1 -sort_by group -input_pins -file "$out/route_timing_paths.rpt"
    set stage route
}

# ---- 5. summary ----
set fs [open "$out/baseline_summary.txt" w]
proc both {fs s} { puts $s; puts $fs $s }
# commit being measured (Step 4/5: results are matched to commits by this hash)
set commit "unknown"
catch { set commit [string trim [exec git -C $root rev-parse --short HEAD]] }
set dirty ""
catch { if {[string trim [exec git -C $root status --porcelain --untracked-files=no]] ne ""} { set dirty " (uncommitted changes!)" } }
both $fs "Bridge baseline  part [get_property PART [current_design]]  stage $stage  [clock format [clock seconds] -format {%Y-%m-%d %H:%M}]"
both $fs "  commit $commit$dirty   Vivado [version -short]"
set u [report_utilization -return_string]
# pick a row out of report_utilization's table: Used | ... | Available | Util%
proc util_row {u label site_re} {
    if {![regexp -line "^\\|\\s*(${site_re})\\s*\\|.*$" $u row]} { return "  [format %-10s $label] (not in report_utilization)" }
    set f {}
    foreach x [split [string trim $row "| "] "|"] { lappend f [string trim $x] }
    return [format "  %-10s %10s of %8s  (%s%%)" $label [lindex $f 1] [lindex $f end-1] [lindex $f end]]
}
foreach {label site_re} {
    LUTs       {(?:CLB|Slice) LUTs\*?}
    Registers  {(?:CLB|Slice) Registers}
    BRAM_tiles {Block RAM Tile}
    URAM       {URAM}
    DSPs       {DSPs}
} { both $fs [util_row $u $label $site_re] }
foreach clk {clk aclk} {
    set p [get_timing_paths -quiet -max_paths 1 -nworst 1 -setup -group $clk]
    if {[llength $p] == 0} { both $fs "  $clk: no timed paths"; continue }
    set slack  [get_property SLACK $p]
    set period [get_property PERIOD [get_clocks $clk]]
    set fmax   [expr {1000.0 / ($period - $slack)}]
    both $fs [format "  %-4s WNS %7.3f ns at %.3f ns  -> Fmax ~ %.0f MHz   levels %s" $clk $slack $period $fmax [get_property LOGIC_LEVELS $p]]
    both $fs "       from [get_property STARTPOINT_PIN $p]"
    both $fs "       to   [get_property ENDPOINT_PIN $p]"
}
# Step 4: how widespread the failures are, and where. Bins every failing
# setup endpoint (up to 20000) by its module path (5 levels deep).
foreach clk {clk aclk} {
    set fails [get_timing_paths -quiet -setup -group $clk -max_paths 20000 -nworst 1 -slack_lesser_than 0]
    set n [llength $fails]
    set tns 0.0
    array unset bin
    array unset worst
    foreach p $fails {
        set sl [get_property SLACK $p]
        set tns [expr {$tns + $sl}]
        set parts [split [get_property ENDPOINT_PIN $p] /]
        set mod [join [lrange $parts 0 [expr {min(5, [llength $parts] - 3)}]] /]
        if {$mod eq ""} { set mod "(top)" }
        if {![info exists bin($mod)]} { set bin($mod) 0; set worst($mod) 0.0 }
        incr bin($mod)
        if {$sl < $worst($mod)} { set worst($mod) $sl }
    }
    both $fs [format "  %-4s failing endpoints %d%s   TNS %.1f ns" $clk $n [expr {$n >= 20000 ? "+ (capped)" : ""}] $tns]
    set rows {}
    foreach m [array names bin] { lappend rows [list $m $bin($m) $worst($m)] }
    foreach r [lrange [lsort -real -index 2 $rows] 0 14] {
        both $fs [format "       %6d  worst %7.3f  %s" [lindex $r 1] [lindex $r 2] [lindex $r 0]]
    }
}
set h [get_timing_paths -quiet -max_paths 1 -hold]
if {[llength $h]} { both $fs [format "  WHS %7.3f ns" [get_property SLACK $h]] }
both $fs "Per-module area: $out/[expr {$stage eq {route} ? {route} : {synth}}]_util_hier.rpt   Simulation numbers: PERF lines in sim_tb_bridge_top.log"
close $fs
file copy -force "$out/baseline_summary.txt" "$out/baseline_summary_$commit.txt"
puts "BASELINE done: $out/baseline_summary.txt"