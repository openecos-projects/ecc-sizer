# Evaluate an ECC/ics55 placement with the same OpenROAD/STA setup used by
# the Sizer runner.  All paths are supplied through the environment so this
# file does not depend on the external ECC workspace layout.

proc split_env_paths {name} {
    if {![info exists ::env($name)] || $::env($name) eq ""} {
        return {}
    }
    return [split $::env($name) "\n"]
}

proc evaluate {} {
    foreach lef [split_env_paths LEF_FILES] { read_lef $lef }
    foreach lib [split_env_paths LIB_FILES] { read_liberty $lib }
    read_def $::env(INPUT_DEF)
    read_sdc $::env(INPUT_SDC)
    set_ideal_network [all_clocks]
    set_wire_rc -signal -layer $::env(RC_LAYER)
    set_wire_rc -clock -layer $::env(RC_LAYER)
    set_thread_count $::env(STA_THREADS)

    check_placement -verbose
    estimate_parasitics -placement
    set_cmd_units -time ns -capacitance pF
    report_check_types -max_slew -max_capacitance -max_fanout -digits 6

    set block [ord::get_db_block]
    set area 0.0
    set unplaced 0
    set components [$block getInsts]
    foreach inst $components {
        set master [$inst getMaster]
        set area [expr {$area + [$block dbuToMicrons [$master getWidth]] *
                        [$block dbuToMicrons [$master getHeight]]}]
        if {![$inst isPlaced]} { incr unplaced }
    }

    lassign [sta::design_power [sta::cmd_scene]] internal switching leakage total
    set fp [open $::env(OUTPUT_DIR)/metrics.tsv w]
    foreach {key value} [list \
        leakage_power_w $leakage total_power_w $total \
        internal_power_w $internal switching_power_w $switching \
        instances [llength $components] nets [llength [$block getNets]] \
        area_um2 $area unplaced $unplaced \
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
    report_checks -path_delay max -digits 6
}

if {[catch {evaluate} message options]} {
    puts stderr [dict get $options -errorinfo]
    exit 1
}
exit 0
