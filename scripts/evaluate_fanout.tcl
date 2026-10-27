# Evaluate unchanged, Sizer-repaired, and OpenROAD reference designs with the
# same ASAP7 libraries, SDC, and benchmark placement RC. Invoked by the runner.
proc evaluate {} {
    set platform_dir $::env(PLATFORM_DIR)
    set design_path $::env(DESIGN_PATH)
    source ${design_path}/lib_setup.tcl
    foreach lef $lefs { read_lef $lef }
    foreach lib $libbest { read_liberty $lib }
    read_def $::env(INPUT_DEF)
    read_sdc ${design_path}/contest.sdc
    set_ideal_network [all_clocks]
    source $rc_file
    set_thread_count 8
    set_max_fanout $::env(MAX_FANOUT) [current_design]
    set physical [expr {[info exists ::env(CHECK_PLACEMENT)] && $::env(CHECK_PLACEMENT)}]
    if {$physical} {
        # Native Sizer exports the accepted, already legal placement. Check it
        # without silently repairing a broken output in the evaluation process.
        if {$::env(EVAL_MODE) ni {sizer legacy_sizer} || !$::env(NATIVE_REPAIR)} {
            detailed_placement
        }
        check_placement -verbose
    }
    puts "EVAL_STAGE estimate_parasitics"
    estimate_parasitics -placement
    puts "EVAL_STAGE repair"
    set start [clock milliseconds]
    if {$::env(EVAL_MODE) eq "openroad_design"} {
        repair_design
    } elseif {$::env(EVAL_MODE) eq "openroad_buffer"} {
        repair_design
        # Match gbi_scripts/ISPD26/evaluation_setup.tcl's setup scope.
        repair_timing -setup -sequence "buffer" -skip_pin_swap \
            -skip_gate_cloning -skip_size_down -skip_last_gasp \
            -skip_vt_swap -skip_crit_vt_swap
    }
    set repair_ms [expr {[clock milliseconds] - $start}]
    if {$physical && $::env(EVAL_MODE) in {openroad_design openroad_buffer}} {
        detailed_placement
        check_placement -verbose
    }
    estimate_parasitics -placement
    set_cmd_units -time ns -capacitance pF
    puts "EVAL_STAGE metrics"
    # Initialize fanout checking before the low-level count query. In the
    # vendored STA revision the count API does not run checkFanoutPreamble().
    report_check_types -max_slew -max_capacitance -max_fanout -digits 6
    set block [ord::get_db_block]
    set area 0.0
    set unplaced 0
    set added 0
    set supply_errors 0
    set buffer_masters [dict create]
    foreach inst [$block getInsts] {
        foreach pin [$inst getITerms] {
            set type [$pin getSigType]
            if {$type in {POWER GROUND}} {
                set net [$pin getNet]
                if {$net eq "NULL" || [$net getSigType] ne $type} { incr supply_errors }
            }
        }
        set master [$inst getMaster]
        set width [$block dbuToMicrons [$master getWidth]]
        set height [$block dbuToMicrons [$master getHeight]]
        set area [expr {$area + $width * $height}]
        if {![$inst isPlaced]} { incr unplaced }
        if {[string match sizer_fanout_buf* [$inst getName]] || [string match sizer_drv_buf* [$inst getName]] || [string match sizer_setup_buf* [$inst getName]] || [string match sizer_vg_buf* [$inst getName]]} {
            incr added
            dict incr buffer_masters [$master getName]
        }
    }
    # Same OpenSTA default activity assumptions for every independently loaded design.
    lassign [sta::design_power [sta::cmd_scene]] internal switching leakage total
    set fp [open $::env(OUTPUT_DIR)/metrics.tsv w]
    foreach {key value} [list leakage_power_w $leakage total_power_w $total \
        internal_power_w $internal switching_power_w $switching instances [llength [$block getInsts]] \
        nets [llength [$block getNets]] area_um2 $area unplaced $unplaced \
        inserted_sizer_buffers $added repair_ms $repair_ms \
        placement_checked $physical supply_pin_errors $supply_errors \
        setup_wns_ns [sta::worst_negative_slack -max] \
        setup_tns_ns [sta::total_negative_slack -max] \
        hold_wns_ns [sta::worst_negative_slack -min] \
        hold_tns_ns [sta::total_negative_slack -min] \
        slew_violations [sta::max_slew_violation_count] \
        cap_violations [sta::max_capacitance_violation_count] \
        fanout_violations [sta::max_fanout_violation_count]] {
        puts $fp "$key\t$value"
        puts "METRIC $key $value"
    }
    close $fp
    puts "INSERTED_BUFFER_MASTERS $buffer_masters"
    report_checks -path_delay max -digits 6
    source $::env(EQUIV_DIR)/or_utils.tcl
    write_node_and_net_files $::env(OUTPUT_DIR)/node.csv $::env(OUTPUT_DIR)/nets.csv
    write_def $::env(OUTPUT_DIR)/evaluated.def
    write_verilog $::env(OUTPUT_DIR)/evaluated.v
}
if {[catch {evaluate} message options]} {
    puts stderr [dict get $options -errorinfo]
    exit 1
}
exit 0
