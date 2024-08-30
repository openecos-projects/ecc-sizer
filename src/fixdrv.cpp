#include "sizer.h"
#include <omp.h>
#include <stdlib.h>
#include <algorithm>
#include <cassert>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <ostream>
#include <sstream>
#include "ckt.h"
#include "odb/db.h"
#include "ord/ordMain.hh"
#include "utils.h"
#include <iostream>
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include <arpa/inet.h>
#include "rsz/Resizer.hh"
#include "ord/Design.h"
#include "ord/Timing.h"
#include <gperftools/profiler.h>
#include <gperftools/heap-profiler.h>
#include "ord/OpenRoad.hh"
#include "dpl/Opendp.h"
#include "grt/GlobalRouter.h"

unsigned Sizer::FwdFixCapViolation(unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    unsigned change = 0;

    cout << "Fwd fix cap violation .. for view " << view << " " << std::endl;

    double remains = 0.0;
    double origins = 0.0;  // origin is not always the same current cap. fix can
                           // create additional violations.
    ofstream ofs("fwdFixCap.log");
    for(unsigned i = 0; i < topolist.size(); i++) {
        unsigned cur = topolist[i];

        // should account for flipflop for fwd fix
        // if(isff(cells[cur]))
        //     continue;

        LibCellInfo *lib_cell_info = getLibCellInfo(cells[cur], corner);

        if(lib_cell_info == NULL) {
            continue;
        }

        for(unsigned k = 0; k < cells[cur].outpins.size(); ++k) {
            double maxCap = 0.0;
            int out_pin_id = cells[cur].outpins[k];

            maxCap = lib_cell_info->pins[pins[view][out_pin_id].lib_pin]
                         .maxCapacitance;
            if(use_margin) {
                maxCap *= cap_margin;
            }
            unsigned outnet = pins[view][out_pin_id].net;
            if(nets[corner][outnet].is_clock) {
                continue;
            }
            double loadCap = 0;
            for(unsigned j = 0; j < nets[corner][outnet].outpins.size(); j++) {
                loadCap += pins[view][nets[corner][outnet].outpins[j]].cap;
            }
            // if(fabs(nets[corner][outnet].cap + loadCap -
            //         pins[view][cells[cur].outpins[k]].totcap) > 1e-3) {
            //     printf(
            //         "Error: totcap not equal to net cap totcap %f, recalc
            //         %f\n", pins[view][cells[cur].outpins[k]].totcap,
            //         nets[corner][outnet].cap + loadCap);
            // }

            if(pins[view][out_pin_id].totcap > maxCap) {
                origins += (pins[view][out_pin_id].totcap - maxCap);
                ofs << getFullPinName(pins[view][out_pin_id])
                    << " max cap vio: " << pins[view][out_pin_id].totcap << " "
                    << nets[corner][outnet].cap << " "
                    << nets[corner][outnet].name << " " << maxCap << endl;
            }

            while(pins[view][out_pin_id].totcap > maxCap) {
                double delta_target = pins[view][out_pin_id].totcap - maxCap;

                set< entry > targets;

                for(unsigned j = 0; j < nets[corner][outnet].outpins.size();
                    j++) {
                    unsigned curinpin = nets[corner][outnet].outpins[j];
                    unsigned curfo = pins[view][curinpin].owner;

                    if(curfo == UINT_MAX) {
                        continue;
                    }

                    if(getLibCellInfo(cells[curfo], corner) == NULL ||
                       isff(cells[curfo]) || cells[curfo].isDontTouch) {
                        continue;
                    }

                    entry tmpEntry;
                    tmpEntry.id = curfo;

                    double slack = .0;

                    for(unsigned l = 0; l < cells[curfo].outpins.size(); ++l) {
                        slack = min(slack,
                                    pins[view][cells[curfo].outpins[l]].rslk);
                        slack = min(slack,
                                    pins[view][cells[curfo].outpins[l]].fslk);
                    }

                    if(slack >= 0) {
                        double delta_impact_size = 0.0, delta_impact_type = 0.0;
                        double delta_cap_size = 0.0, delta_cap_type = 0.0;
                        // downsizing
                        if(!isMin(cells[curfo])) {
                            CELL &cell = cells[curfo];
                            // double delta_slack = EstDeltaSlackNEW(cell, -1,
                            // 0, view);

                            LibCellInfo *lib_cell_info =
                                sizing_progression(cell, -1, 0, view);

                            if(lib_cell_info != NULL) {
                                delta_cap_size =
                                    lib_cell_info
                                        ->pins[pins[view][curinpin].lib_pin]
                                        .capacitance -
                                    pins[view][curinpin].cap;
                            }
                            else {
                                delta_cap_size = 0.0;
                            }

                            if(delta_cap_size >= 0) {
                                delta_impact_size = 0.0;
                            }
                            else {
                                // delta_impact_size =
                                // delta_cap_size/delta_slack;
                                delta_impact_size = delta_cap_size;
                            }
                        }
                        // downgrading
                        if(r_type(cells[curfo]) != 0) {
                            CELL &cell = cells[i];
                            // double delta_slack = EstDeltaSlackNEW(cell, 0,
                            // -1, view);

                            LibCellInfo *lib_cell_info =
                                sizing_progression(cell, 0, -1, view);

                            if(lib_cell_info != NULL) {
                                delta_cap_type =
                                    lib_cell_info
                                        ->pins[pins[view][curinpin].lib_pin]
                                        .capacitance -
                                    pins[view][curinpin].cap;
                            }
                            else {
                                delta_cap_type = 0.0;
                            }

                            if(delta_cap_type <= 0) {
                                delta_impact_type = 0.0;
                            }
                            else {
                                delta_impact_type = delta_cap_type;
                            }
                        }

                        tmpEntry.delta_impact = 0.0;

                        if(delta_impact_size == 0.0) {
                            tmpEntry.change = DNTYPE;
                            tmpEntry.delta_impact = delta_impact_type;
                            tmpEntry.tie_break = delta_cap_type;
                        }
                        else if(delta_impact_type == 0.0) {
                            tmpEntry.change = DNSIZE;
                            tmpEntry.delta_impact = delta_impact_size;
                            tmpEntry.tie_break = delta_cap_size;
                        }
                        else if(delta_impact_size != 0.0 &&
                                delta_impact_type != 0.0) {
                            if(delta_impact_size < delta_impact_type) {
                                tmpEntry.change = DNSIZE;
                                tmpEntry.delta_impact = delta_impact_size;
                                tmpEntry.tie_break = delta_cap_size;
                            }
                            else {
                                tmpEntry.change = DNTYPE;
                                tmpEntry.delta_impact = delta_impact_type;
                                tmpEntry.tie_break = delta_cap_type;
                            }
                        }

                        if(tmpEntry.delta_impact != 0.0)
                            targets.insert(tmpEntry);
                    }
                }

                change = targets.size();

                for(set< entry >::iterator it = targets.begin();
                    it != targets.end(); it++) {
                    unsigned cur = it->id;

                    if(it->change == DNSIZE) {
                        cell_resize(cells[cur], -1);
                    }
                    else if(it->change == DNTYPE) {
                        cell_retype(cells[cur], -1);
                    }

                    OneTimer(cells[cur], STA_MARGIN, true);

                    delta_target += it->tie_break;
                    if(delta_target < 0) {
                        break;
                    }
                }

                double loadCap = 0.;
                for(unsigned j = 0; j < nets[corner][outnet].outpins.size();
                    j++) {
                    loadCap += pins[view][nets[corner][outnet].outpins[j]].cap;
                }

                pins[view][cells[cur].outpins[k]].totcap =
                    nets[corner][outnet].cap + loadCap;
#ifdef DEBUG
                cout << "load cap now is = "
                     << pins[view][cells[cur].outpins[k]].totcap << " ("
                     << maxCap << ") " << endl;
#endif
                if(targets.size() == 0) {
                    if(pins[view][cells[cur].outpins[k]].totcap > maxCap)
                        remains +=
                            (pins[view][cells[cur].outpins[k]].totcap - maxCap);
                    break;
                }
            }
#ifdef DEBUG
            if(pins[view][cells[cur].outpins[k]].totcap > maxCap) {
                if(lib_cell_info) {
                    cout << "nothing more can be done.. ";
                    cout << cells[cur].name << " " << lib_cell_info->name
                         << " FOs: " << cells[cur].fos.size() << " "
                         << pins[view][cells[cur].outpins[k]].totcap << "fF"
                         << endl;
                }
            }
#endif
        }
    }
    ofs.close();

    printf("Fwd FixCap %d cells were changed\n", change);
    cout << remains << " fF remains, origins " << origins << " fF . " << endl;
    if(fabs(cap_violation - origins) > 3) {
        printf("cap violation %f, origins %f\n", cap_violation, origins);
        printf("fwd fix cap not equal to remains\n");
        // exit(0);
    }
    cap_violation = remains;
    return change;
}

// increase the size of the cells to increase the maxCap, furthermore fix the
// cap violation.
unsigned Sizer::BwdFixCapViolation(unsigned view) {
    unsigned change = 0;
    cout << "Bwd fix cap violation .. for view " << view << " ";
    unsigned corner = 0;  // mmmcViewList[view].corner;

    double remains = 0.0;
    double origins = 0.0;  // origin is not always the same current since cap
                           // fix can create additional violations.

    for(unsigned i = 0; i < rtopolist.size(); i++) {
        unsigned cur = rtopolist[i];

        if(cells[cur].isClockCell)
            continue;
        if(cells[cur].isDontTouch)
            continue;

        LibCellInfo *lib_cell_info = getLibCellInfo(cells[cur], corner);

        if(lib_cell_info == NULL) {
            continue;
        }
        if(isff(cells[cur])) {
            continue;
        }
        for(unsigned k = 0; k < cells[cur].outpins.size(); ++k) {
            double maxCap = 0.0;
            maxCap =
                lib_cell_info->pins[pins[view][cells[cur].outpins[k]].lib_pin]
                    .maxCapacitance;
            if(use_margin) {
                maxCap *= cap_margin;
            }
            unsigned outnet = pins[view][cells[cur].outpins[k]].net;
            if(nets[corner][outnet].is_clock) {
                continue;
            }
            double loadCap = 0.;

            for(unsigned j = 0; j < nets[corner][outnet].outpins.size(); j++)
                loadCap += pins[view][nets[corner][outnet].outpins[j]].cap;

            pins[view][cells[cur].outpins[k]].totcap =
                nets[corner][outnet].cap + loadCap;

            if(pins[view][cells[cur].outpins[k]].totcap > maxCap)
                origins += (pins[view][cells[cur].outpins[k]].totcap - maxCap);

            while(pins[view][cells[cur].outpins[k]].totcap > maxCap) {
                if(!isMax(cells[cur])) {
                    cell_resize(cells[cur], 1);
                    ++change;
                }
                else if(r_type(cells[cur]) != (numVt - 1)) {
                    bool ok = cell_retype(cells[cur], 1);
                    if(!ok) {
                        break;
                    }
                    ++change;
                }
                else {
                    if(pins[view][cells[cur].outpins[k]].totcap > maxCap)
                        remains +=
                            (pins[view][cells[cur].outpins[k]].totcap - maxCap);
                    break;
                }

                lib_cell_info = getLibCellInfo(cells[cur], corner);

                if(lib_cell_info != NULL) {
                    maxCap =
                        lib_cell_info
                            ->pins[pins[view][cells[cur].outpins[k]].lib_pin]
                            .maxCapacitance;
                    if(use_margin) {
                        maxCap *= cap_margin;
                    }
                }
            }
#ifdef DEBUG
            if(pins[view][cells[cur].outpins[k]].totcap > maxCap) {
                cout << endl
                     << "--cap violation at : " << cells[cur].name
                     << " max cap: " << maxCap
                     << " totcap: " << pins[view][cells[cur].outpins[k]].totcap
                     << endl;
            }
#endif
        }
    }
    cout << remains << " fF out of " << origins << " fF remains. " << endl;
    cap_violation = remains;
    return change;
}

// FIXME: This function invoke calc_stats for each cell. This is not efficient.
// It should be invoked only once for all cells.
// FIXME: calcStats needs to be updated to handle one cell at a time.
unsigned Sizer::FwdFixSlewViolation(double maxTranRatio, unsigned view) {
    unsigned change = 0;
    unsigned thread_id = 0;

    cout << "Fwd fix slew violation .. for view " << view << endl;
    unsigned corner = 0;  // mmmcViewList[view].corner;
    double prev_tns, cur_tns = 0.0;
    bool old_updatePinAcc = updatePinAcc;
    updatePinAcc = true;
    for(unsigned i = 0; i < topolist.size(); i++) {
        unsigned cur = topolist[i];

        // if(cells[cur].isClockCell) {
        //     continue;
        // }
        // if(cells[cur].isDontTouch)
        //     continue;
        if(getLibCellInfo(cells[cur], corner) == NULL) {
            continue;
        }

        for(unsigned j = 0; j < cells[cur].outpins.size(); j++) {
            unsigned curpin = cells[cur].outpins[j];
            unsigned outnet = pins[view][curpin].net;

            if(outnet == UINT_MAX) {
                continue;
            }

            if(pins[view][curpin].waiveTran) {
                continue;
            }
            // if(pins[view][fopin].name == "CLK" ||
            //    pins[view][fopin].name == "clk") {
            //     continue;
            // }
            double cur_max_tran = pins[view][curpin].max_tran;
            // for(unsigned k = 0; k < nets[corner][outnet].outpins.size(); ++k)
            // {
            //     unsigned fopin = nets[corner][outnet].outpins[k];
            //     timing_lookup wire_delay = get_wire_delay(outnet, fopin,
            //     view); cur_max_tran =
            //         std::min(sqrt(pow(pins[view][fopin].max_tran, 2) -
            //                       pow(log(9) * wire_delay.rise, 2)),
            //                  cur_max_tran);
            //     cur_max_tran =
            //         std::min(sqrt(pow(pins[view][fopin].max_tran, 2) -
            //                       pow(log(9) * wire_delay.fall, 2)),
            //                  cur_max_tran);
            // }

            if(!IsTranVio(pins[view][curpin], cur_max_tran)) {
                continue;
            }

            //            cout << "MAX TRAN " <<
            //            getFullPinName(pins[view][curpin]) << " "
            //                << max(pins[view][curpin].rtran,
            //                pins[view][curpin].ftran) << "/" <<
            //                pins[view][curpin].max_tran << endl;

            // downsizing fanouts
            // for(unsigned k = 0; k < nets[corner][outnet].outpins.size(); ++k)
            // {
            //     unsigned focell =
            //         pins[view][nets[corner][outnet].outpins[k]].owner;
            //     if(focell == UINT_MAX) {
            //         continue;
            //     }
            //     if(getLibCellInfo(cells[focell], corner) == NULL) {
            //         continue;
            //     }
            //     if(cells[focell].isClockCell) {
            //         continue;
            //     }
            //     if(cells[focell].isDontTouch)
            //         continue;
            //     // if(isff(cells[focell])) {
            //     //     continue;
            //     // }
            //     // CalcStats((unsigned)thread_id, false, "", view, false);
            //     prev_tns = viewTNS[view];

            //     if(cell_resize(cells[focell], -1)) {
            //         OneTimer(cells[focell], 1, true);
            //         // CalcStats((unsigned)thread_id, false, "", view,
            //         // false);
            //         cur_tns = viewTNS[view];
            //         change++;
            //         if(cur_tns > prev_tns) {
            //             cell_resize(cells[focell], 1);
            //             cells[focell].isChanged -= 2;
            //             change--;
            //             OneTimer(cells[focell], 1, true);
            //         }
            //         else if(!IsTranVio(pins[view][curpin], cur_max_tran)) {
            //             break;
            //         }
            //     }
            // }

            // upsizing target cell
            while(IsTranVio(pins[view][curpin], cur_max_tran)) {
                if(cells[cur].isDontTouch)
                    break;
                if(r_type(cells[cur]) == numVt - 1 && isMax(cells[cur])) {
                    break;
                }

                double delta_impact_size = 0.0, delta_impact_type = 0.0;
                double prev_tran =
                    max(pins[view][curpin].rtran, pins[view][curpin].ftran);
                string prev_type = cells[cur].type;

                if(!isMax(cells[cur])) {
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    // prev_tns = CalcSlackViolation(view);
                    prev_tns = viewTNS[view];

                    bool change_size = cell_resize(cells[cur], 1);
                    if(change_size) {
                        OneTimer(cells[cur], STA_MARGIN, true);
                    }
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    // cur_tns = CalcSlackViolation(view);
                    cur_tns = prev_tns;
                    double delta_tran = 0.0;

                    if(cur_tns > prev_tns) {
                        delta_tran = 0.0;
                    }
                    else {
                        delta_tran = max(pins[view][curpin].rtran,
                                         pins[view][curpin].ftran) -
                                     prev_tran;
                    }

                    if(delta_tran > 0.0)
                        delta_tran = 0;

                    if(change_size) {
                        cell_resize(cells[cur], -1);
                        cells[cur].isChanged -= 2;
                        OneTimer(cells[cur], STA_MARGIN, true);
                    }

                    double delta_sw_power, delta_leak, delta_int;

                    if(ALPHA != 0.0) {
                        delta_sw_power =
                            LookupDeltaSwitchPower(cells[cur], 1, 0);
                        delta_int = LookupDeltaIntPower(cells[cur], 1, 0);
                    }
                    else {
                        delta_sw_power = 0.0;
                        delta_int = 0.0;
                    }
                    delta_leak = LookupDeltaLeak(cells[cur], 1, 0);

                    double delta_power = ALPHA * (delta_sw_power + delta_int) +
                                         (1 - ALPHA) * delta_leak;
                    delta_impact_size = delta_tran / delta_power;
                }

                if(r_type(cells[cur]) != (numVt - 1)) {
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    prev_tns = viewTNS[view];

                    bool change_type = cell_retype(cells[cur], 1);

                    OneTimer(cells[cur], STA_MARGIN, true);
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    cur_tns = viewTNS[view];

                    double delta_tran = 0.0;

                    if(cur_tns > prev_tns) {
                        delta_tran = 0.0;
                    }
                    else {
                        delta_tran = max(pins[view][curpin].rtran,
                                         pins[view][curpin].ftran) -
                                     prev_tran;
                    }
                    if(delta_tran > 0.0)
                        delta_tran = 0;

                    if(change_type) {
                        cell_retype(cells[cur], -1);
                        cells[cur].isChanged -= 2;
                    }

                    OneTimer(cells[cur], STA_MARGIN, true);

                    double delta_sw_power, delta_leak, delta_int;

                    if(ALPHA != 0.0) {
                        delta_sw_power =
                            LookupDeltaSwitchPower(cells[cur], 1, 0);
                        delta_int = LookupDeltaIntPower(cells[cur], 1, 0);
                    }
                    else {
                        delta_sw_power = 0.0;
                        delta_int = 0.0;
                    }
                    delta_leak = LookupDeltaLeak(cells[cur], 1, 0);
                    double delta_power = ALPHA * (delta_sw_power + delta_int) +
                                         (1 - ALPHA) * delta_leak;
                    delta_impact_type = delta_tran / delta_power;
                }

                if(delta_impact_size == 0 && delta_impact_size == 0) {
                    break;
                }

                if(delta_impact_size < delta_impact_type ||
                   delta_impact_type == 0) {
                    cell_resize(cells[cur], 1);
                    change++;
                    // cout << "UPSIZED CELL " << cells[cur].name << " "
                    //    << prev_type << " --> " << cells[cur].type << endl;
                }
                else {
                    cell_retype(cells[cur], 1);
                    change++;
                    // cout << "UPTYPED CELL " << cells[cur].name << " "
                    //    << prev_type << " --> " << cells[cur].type << endl;
                }

                OneTimer(cells[cur], STA_MARGIN, true);
            }
            //            cout << "AFTER MAX TRAN " <<
            //            getFullPinName(pins[view][curpin]) << " "
            //                << max(pins[view][curpin].rtran,
            //                pins[view][curpin].ftran) << "/" <<
            //                pins[view][curpin].max_tran << endl;
        }

        for(unsigned j = 0; j < cells[cur].inpins.size(); j++) {
            unsigned curpin = cells[cur].inpins[j];

            if(pins[view][curpin].waiveTran) {
                continue;
            }

            if(!IsTranVio(pins[view][curpin])) {
                continue;
            }
            //            cout << "MAX TRAN " <<
            //            getFullPinName(pins[view][curpin]) << " "
            //                << max(pins[view][curpin].rtran,
            //                pins[view][curpin].ftran) << "/" <<
            //                pins[view][curpin].max_tran << endl;

            int curnet = pins[view][cells[cur].inpins[j]].net;
            unsigned ficell = UINT_MAX;
            if(cells[cur].inpins[j] != UINT_MAX) {
                if(curnet != UINT_MAX) {
                    if(nets[corner][curnet].inpin != UINT_MAX) {
                        ficell = pins[view][nets[corner][curnet].inpin].owner;
                    }
                }
            }

            if(ficell == UINT_MAX) {
                continue;
            }

            if(getLibCellInfo(cells[ficell], corner) == NULL) {
                continue;
            }
            if(cells[ficell].isClockCell) {
                continue;
            }
            if(cells[ficell].isDontTouch)
                continue;

            unsigned max_upsize = 5;
            unsigned iter = 0;
            // upsizing fanin cell
            while(IsTranVio(pins[view][curpin])) {
                if(iter > max_upsize) {
                    break;
                }

                iter++;

                if(r_type(cells[ficell]) == numVt - 1 && isMax(cells[ficell])) {
                    break;
                }

                double delta_impact_size = 0.0, delta_impact_type = 0.0;
                double prev_tran =
                    max(pins[view][curpin].rtran, pins[view][curpin].ftran);
                string prev_type = cells[ficell].type;

                if(!isMax(cells[ficell])) {
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    prev_tns = viewTNS[view];

                    bool change_size = cell_resize(cells[ficell], 1);
                    if(change_size) {
                        OneTimer(cells[ficell], STA_MARGIN, true);
                    }
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    cur_tns = viewTNS[view];

                    double delta_tran = 0.0;

                    if(cur_tns > prev_tns) {
                        delta_tran = 0.0;
                    }
                    else {
                        delta_tran = max(pins[view][curpin].rtran,
                                         pins[view][curpin].ftran) -
                                     prev_tran;
                    }

                    if(delta_tran > 0.0)
                        delta_tran = 0;

                    if(change_size) {
                        cell_resize(cells[ficell], -1);
                        cells[ficell].isChanged -= 2;
                    }

                    OneTimer(cells[ficell], STA_MARGIN, true);

                    double delta_sw_power, delta_leak, delta_int;
                    if(ALPHA != 0.0) {
                        delta_sw_power =
                            LookupDeltaSwitchPower(cells[cur], 1, 0);
                        delta_int = LookupDeltaIntPower(cells[cur], 1, 0);
                    }
                    else {
                        delta_sw_power = 0.0;
                        delta_int = 0.0;
                    }
                    delta_leak = LookupDeltaLeak(cells[cur], 1, 0);
                    double delta_power = ALPHA * (delta_sw_power + delta_int) +
                                         (1 - ALPHA) * delta_leak;
                    delta_impact_size = delta_tran / delta_power;
                }

                if(r_type(cells[ficell]) != (numVt - 1)) {
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    prev_tns = viewTNS[view];

                    bool change_type = cell_retype(cells[ficell], 1);

                    OneTimer(cells[ficell], STA_MARGIN, true);
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    cur_tns = viewTNS[view];

                    double delta_tran = 0.0;

                    if(cur_tns > prev_tns) {
                        delta_tran = 0.0;
                    }
                    else {
                        delta_tran = max(pins[view][curpin].rtran,
                                         pins[view][curpin].ftran) -
                                     prev_tran;
                    }
                    if(delta_tran > 0.0)
                        delta_tran = 0;

                    if(change_type) {
                        cell_retype(cells[ficell], -1);
                        cells[ficell].isChanged -= 2;
                    }

                    OneTimer(cells[ficell], STA_MARGIN, true);

                    double delta_sw_power, delta_leak, delta_int;

                    if(ALPHA != 0.0) {
                        delta_sw_power =
                            LookupDeltaSwitchPower(cells[cur], 1, 0);
                        delta_int = LookupDeltaIntPower(cells[cur], 1, 0);
                    }
                    else {
                        delta_sw_power = 0.0;
                        delta_int = 0.0;
                    }
                    delta_leak = LookupDeltaLeak(cells[cur], 1, 0);
                    double delta_power = ALPHA * (delta_sw_power + delta_int) +
                                         (1 - ALPHA) * delta_leak;
                    delta_impact_type = delta_tran / delta_power;
                }

                if(delta_impact_size == 0 && delta_impact_size == 0) {
                    break;
                }

                if(delta_impact_size < delta_impact_type ||
                   delta_impact_type == 0) {
                    cell_resize(cells[ficell], 1);
                    change++;
                    // cout << "UPSIZED CELL " << cells[ficell].name << " "
                    //    << prev_type << " --> " << cells[ficell].type << endl;
                }
                else {
                    cell_retype(cells[ficell], 1);
                    change++;
                    // cout << "UPTYPED CELL " << cells[ficell].name << " "
                    //    << prev_type << " --> " << cells[ficell].type << endl;
                }

                OneTimer(cells[ficell], STA_MARGIN, true);
            }
#if 0
            if(curnet == UINT_MAX) {
                continue;
            }
            for(auto brother_pin : nets[corner][curnet].outpins) {
                if(brother_pin == curpin) {
                    continue;
                }
                unsigned brother_cell = pins[view][brother_pin].owner;
                if(brother_cell == UINT_MAX) {
                    continue;
                }
                if(getLibCellInfo(cells[brother_cell], corner) == NULL) {
                    continue;
                }
                if(cells[brother_cell].isClockCell) {
                    continue;
                }
                if(cells[brother_cell].isDontTouch) {
                    continue;
                }
                // if(IsTranVio(pins[view][brother_pin])) {
                //     continue;
                // }
                if(cell_resize(cells[brother_cell], -1)) {
                    OneTimer(cells[brother_cell], STA_MARGIN, view);
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    cur_tns = viewTNS[view];
                    change++;
                    if(cur_tns > prev_tns) {
                        cell_resize(cells[brother_cell], 1);
                        cells[brother_cell].isChanged -= 2;
                        change--;
                        OneTimer(cells[brother_cell], STA_MARGIN, view);
                    }
                    else if(!IsTranVio(pins[view][curpin])) {
                        break;
                    }
                }
            }
#endif
            // cout << "AFTER MAX TRAN " << getFullPinName(pins[view][curpin])
            // << " "
            //    << max(pins[view][curpin].rtran, pins[view][curpin].ftran) <<
            //    "/" <<
            //    pins[view][curpin].max_tran << endl;
        }
    }
    updatePinAcc = old_updatePinAcc;
    cout << "finished." << endl;
    return change;
}

// FIXME: This function invoke calc_stats for each cell. This is not efficient.
// It should be invoked only once for all cells.
// FIXME: calcStats needs to be updated to handle one cell at a time.
unsigned Sizer::BwdFixSlewViolation(double maxTranRatio, unsigned view) {
    unsigned change = 0;
    unsigned thread_id = 0;

    cout << "Bwd fix slew violation .. for view " << view << endl;
    unsigned corner = 0;  // mmmcViewList[view].corner;
    double prev_tns, cur_tns = 0.0;

    for(unsigned i = 0; i < rtopolist.size(); i++) {
        unsigned cur = rtopolist[i];

        // if(cells[cur].isClockCell) {
        //     continue;
        // }
        // if(cells[cur].isDontTouch)
        //     continue;
        if(getLibCellInfo(cells[cur], corner) == NULL) {
            continue;
        }

        for(unsigned j = 0; j < cells[cur].inpins.size(); j++) {
            unsigned curpin = cells[cur].inpins[j];

            if(curpin == UINT_MAX || pins[view][curpin].waiveTran) {
                continue;
            }

            if(!IsTranVio(pins[view][curpin])) {
                continue;
            }

            unsigned max_upsize = 20;
            unsigned iter = 0;
            // upsizing fanin cell
            while(IsTranVio(pins[view][curpin])) {
                if(iter > max_upsize) {
                    break;
                }

                iter++;
                unsigned ficell = FindAvailablePreCell(curpin, 15, view);

                if(ficell == UINT_MAX || isMax(cells[ficell])) {
                    break;
                }

                double delta_impact_size = 0.0, delta_impact_type = 0.0;
                double prev_tran =
                    max(pins[view][curpin].rtran, pins[view][curpin].ftran);
                string prev_type = cells[ficell].type;

                if(!isMax(cells[ficell])) {
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    prev_tns = viewTNS[view];

                    bool change_size = cell_resize(cells[ficell], 1);
                    if(change_size) {
                        OneTimer(cells[ficell], STA_MARGIN, true);
                    }
                    change += 1;
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                }

                // OneTimer(cells[ficell], STA_MARGIN, true);
            }
            // cout << "AFTER MAX TRAN " << getFullPinName(pins[view][curpin])
            // << " "
            //    << max(pins[view][curpin].rtran, pins[view][curpin].ftran) <<
            //    "/" <<
            //    pins[view][curpin].max_tran << endl;
        }
    }
    cout << "finished. " << change << " times has been changed" << endl;
    return change;
}

// FIXME: This function invoke calc_stats for each cell. This is not efficient.
// It should be invoked only once for all cells.
// FIXME: calcStats needs to be updated to handle one cell at a time.
unsigned Sizer::FwdFixSlewViolationPost(double maxTranRatio, unsigned view) {
    unsigned change = 0;
    unsigned thread_id = 0;
    bool old_updatePinAcc = updatePinAcc;
    updatePinAcc = true;
    cout << "Fwd fix slew violation .. for view " << view << endl;
    unsigned corner = 0;  // mmmcViewList[view].corner;
    double prev_tns, cur_tns = 0.0;

    for(unsigned i = 0; i < rtopolist.size(); i++) {
        unsigned cur = rtopolist[i];

        // if(cells[cur].isClockCell) {
        //     continue;
        // }
        // if(cells[cur].isDontTouch)
        //     continue;
        if(getLibCellInfo(cells[cur], corner) == NULL) {
            continue;
        }

        for(unsigned j = 0; j < cells[cur].inpins.size(); j++) {
            unsigned curpin = cells[cur].inpins[j];
            unsigned outnet = pins[view][curpin].net;

            if(outnet == UINT_MAX) {
                continue;
            }

            if(pins[view][curpin].waiveTran) {
                continue;
            }
            if(nets[corner][outnet].is_clock) {
                continue;
            }
            if(!IsTranVio(pins[view][curpin])) {
                continue;
            }
            timing_lookup wire_delay;
            double r_tran, f_tran = 0.0;
            calc_one_net_delay(outnet, WIRE_METRIC, true, view);
            wire_delay = get_wire_delay(outnet, curpin, view);
            r_tran = log(9) * wire_delay.rise;
            f_tran = log(9) * wire_delay.fall;
            double old_tran = max(r_tran, f_tran);
            // downsizing fanouts
            set< entry > targets;
            for(unsigned k = 0; k < nets[corner][outnet].outpins.size(); ++k) {
                // int step = 1;
                unsigned focell =
                    pins[view][nets[corner][outnet].outpins[k]].owner;
                bool ok = false;
                entry tmpEntry;
                tmpEntry.id = focell;
                tmpEntry.change = DNSIZE;
                tmpEntry.delta_impact = DBL_MAX;
                if(focell == UINT_MAX) {  //|| focell != cur
                    continue;
                }
                if(getLibCellInfo(cells[focell], corner) == NULL) {
                    continue;
                }
                if(cells[focell].isClockCell) {
                    continue;
                }
                if(cells[focell].isDontTouch)
                    continue;

                double prev_slack = min(GetCellSlack(cells[focell], view),
                                        GetFICellSlack(cells[focell], view));
                double prev_tran = GetCellTran(cells[focell], view) +
                                   GetFICellTran(cells[focell], view) +
                                   GetFOCellTran(cells[focell], view) +
                                   GetNetFOTran(outnet, view) +
                                   GetCellCapVio(cells[focell], view);
                //    GetFOCellTran(cells[focell], view);
                int size_num =
                    main_lib_cell_tables[corner][cells[focell].main_lib_cell_id]
                        ->lib_vt_size_table.size();
                for(int new_size = 0; new_size < size_num; new_size++) {
                    int step = new_size - cells[focell].c_size;
                    if(step == 0) {
                        continue;
                    }
                    bool change_size = cell_resize(cells[focell], step);

                    if(change_size) {
                        OneTimer(cells[focell], 0.1, true);
                    }
                    calc_one_net_delay(outnet, WIRE_METRIC, true, view);
                    timing_lookup new_wire_delay =
                        get_wire_delay(outnet, curpin, view);
                    double new_r_tran = log(9) * new_wire_delay.rise;
                    double new_f_tran = log(9) * new_wire_delay.fall;
                    double new_tran = max(new_r_tran, new_f_tran);

                    double cur_slack = min(GetCellSlack(cells[focell], view),
                                           GetFICellSlack(cells[focell], view));

                    cur_tns = prev_tns;

                    double now_tran = GetCellTran(cells[focell], view) +
                                      GetFICellTran(cells[focell], view) +
                                      GetFOCellTran(cells[focell], view) +
                                      GetNetFOTran(outnet, view) +
                                      GetCellCapVio(cells[focell], view);
                    //   GetFOCellTran(cells[focell], view);
                    double delta_tran =
                        now_tran - prev_tran;  // + new_tran - old_tran;
                    double delta_slack = 0;
                    double benefit = -delta_slack + slew_gamma * delta_tran;
                    if(benefit < tmpEntry.delta_impact) {
                        tmpEntry.delta_impact = benefit;
                        tmpEntry.step = step;
                    }
                    if(change_size) {
                        cell_resize(cells[focell], -step);
                        cells[focell].isChanged -= 2;
                        OneTimer(cells[focell], 0.1, true);
                    }
                }
                if(tmpEntry.delta_impact < -0.001) {
                    printf("cell %s size %d, step %d, delta_impact %f\n",
                           cells[focell].name.c_str(), cells[focell].c_size,
                           tmpEntry.step, tmpEntry.delta_impact);
                    bool change_size =
                        cell_resize(cells[tmpEntry.id], tmpEntry.step);
                    OneTimer(cells[focell], 0.1, true);
                    change++;
                    // targets.insert(tmpEntry);
                }
            }
            // if(targets.size() == 0) {
            //     break;
            // }
            // int ii = 0;
            // for(int kk = 0; kk < 5; kk++) {
            //     int iter = 0;
            //     int max_iter = 10;
            //     if(!IsTranVio(pins[view][curpin])) {
            //         break;
            //     }
            //     for(auto &target : targets) {
            //         unsigned focell = target.id;
            //         if(iter++ > max_iter) {
            //             break;
            //         }
            //         double prev_slack =
            //             min(GetCellSlack(cells[focell], view),
            //                 GetFICellSlack(cells[focell], view));
            //         double prev_tran = GetCellTran(cells[focell], view) +
            //                            GetFICellTran(cells[focell], view) +
            //                            GetFOCellTran(cells[focell], view);
            //         bool change_size = cell_resize(cells[focell],
            //         target.step);

            //         if(change_size) {
            //             OneTimer(cells[focell], 1, true);
            //         }
            //         double cur_slack = min(GetCellSlack(cells[focell], view),
            //                                GetFICellSlack(cells[focell],
            //                                view));

            //         cur_tns = prev_tns;
            //         double now_tran = GetCellTran(cells[focell], view) +
            //                           GetFICellTran(cells[focell], view) +
            //                           GetFOCellTran(cells[focell], view);

            //         double delta_tran = now_tran - prev_tran;

            //         double delta_slack = cur_slack - prev_slack;

            //         if(-delta_slack + slew_gamma * delta_tran > -0.001) {
            //             change_size =
            //                 cell_resize(cells[target.id], -target.step);
            //             OneTimer(cells[focell], 1, true);
            //         }
            //         else {
            //             change++;
            //         }
            //     }
            // }
        }
    }
    updatePinAcc = old_updatePinAcc;
    cout << "Post fix slew " << change << " cells were changed." << endl;
    cout << "finished." << endl;
    return change;
}

// FIXME: This function invoke calc_stats for each cell. This is not efficient.
// It should be invoked only once for all cells.
// FIXME: calcStats needs to be updated to handle one cell at a time.
unsigned Sizer::FwdFixSlackViolation(double maxTranRatio, unsigned view) {
    unsigned change = 0;
    unsigned thread_id = 0;

    cout << "Fwd fix slew violation .. for view " << view << endl;
    unsigned corner = 0;  // mmmcViewList[view].corner;
    double prev_tns, cur_tns = 0.0;

    for(unsigned i = 0; i < topolist.size(); i++) {
        unsigned cur = topolist[i];

        if(cells[cur].isClockCell) {
            continue;
        }
        if(cells[cur].isDontTouch)
            continue;
        if(isff(cells[cur])) {
            continue;
        }
        if(getLibCellInfo(cells[cur], corner) == NULL) {
            continue;
        }

        for(unsigned j = 0; j < cells[cur].outpins.size(); j++) {
            unsigned curpin = cells[cur].outpins[j];
            unsigned outnet = pins[view][curpin].net;

            if(outnet == UINT_MAX) {
                continue;
            }

            if(pins[view][curpin].waiveTran) {
                continue;
            }

            if(!IsTranVio(pins[view][curpin])) {
                continue;
            }

            //            cout << "MAX TRAN " <<
            //            getFullPinName(pins[view][curpin]) << " "
            //                << max(pins[view][curpin].rtran,
            //                pins[view][curpin].ftran) << "/" <<
            //                pins[view][curpin].max_tran << endl;

            // downsizing fanouts
            for(unsigned k = 0; k < nets[corner][outnet].outpins.size(); ++k) {
                unsigned focell =
                    pins[view][nets[corner][outnet].outpins[k]].owner;
                if(focell == UINT_MAX) {
                    continue;
                }
                if(getLibCellInfo(cells[focell], corner) == NULL) {
                    continue;
                }
                if(cells[focell].isClockCell) {
                    continue;
                }
                if(cells[focell].isDontTouch)
                    continue;

                // CalcStats((unsigned)thread_id, false, "", view, false);
                prev_tns = viewTNS[view];

                if(cell_resize(cells[focell], -1)) {
                    OneTimer(cells[focell], STA_MARGIN, true);
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    cur_tns = viewTNS[view];
                    change++;
                    if(cur_tns > prev_tns) {
                        cell_resize(cells[focell], 1);
                        cells[focell].isChanged -= 2;
                        change--;
                        OneTimer(cells[focell], STA_MARGIN, true);
                    }
                    else if(!IsTranVio(pins[view][curpin])) {
                        break;
                    }
                }
            }

            // upsizing target cell
            while(IsTranVio(pins[view][curpin])) {
                if(r_type(cells[cur]) == numVt - 1 && isMax(cells[cur])) {
                    break;
                }

                double delta_impact_size = 0.0, delta_impact_type = 0.0;
                double prev_tran =
                    max(pins[view][curpin].rtran, pins[view][curpin].ftran);
                string prev_type = cells[cur].type;

                if(!isMax(cells[cur])) {
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    // prev_tns = CalcSlackViolation(view);
                    prev_tns = viewTNS[view];

                    bool change_size = cell_resize(cells[cur], 1);
                    if(change_size) {
                        OneTimer(cells[cur], STA_MARGIN, true);
                    }
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    // cur_tns = CalcSlackViolation(view);
                    cur_tns = prev_tns;
                    double delta_tran = 0.0;

                    if(cur_tns > prev_tns) {
                        delta_tran = 0.0;
                    }
                    else {
                        delta_tran = max(pins[view][curpin].rtran,
                                         pins[view][curpin].ftran) -
                                     prev_tran;
                    }

                    if(delta_tran > 0.0)
                        delta_tran = 0;

                    if(change_size) {
                        cell_resize(cells[cur], -1);
                        cells[cur].isChanged -= 2;
                        OneTimer(cells[cur], STA_MARGIN, true);
                    }

                    double delta_sw_power, delta_leak, delta_int;

                    if(ALPHA != 0.0) {
                        delta_sw_power =
                            LookupDeltaSwitchPower(cells[cur], 1, 0);
                        delta_int = LookupDeltaIntPower(cells[cur], 1, 0);
                    }
                    else {
                        delta_sw_power = 0.0;
                        delta_int = 0.0;
                    }
                    delta_leak = LookupDeltaLeak(cells[cur], 1, 0);

                    double delta_power = ALPHA * (delta_sw_power + delta_int) +
                                         (1 - ALPHA) * delta_leak;
                    delta_impact_size = delta_tran / delta_power;
                }

                if(r_type(cells[cur]) != (numVt - 1)) {
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    prev_tns = viewTNS[view];

                    bool change_type = cell_retype(cells[cur], 1);

                    OneTimer(cells[cur], STA_MARGIN, true);
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    cur_tns = viewTNS[view];

                    double delta_tran = 0.0;

                    if(cur_tns > prev_tns) {
                        delta_tran = 0.0;
                    }
                    else {
                        delta_tran = max(pins[view][curpin].rtran,
                                         pins[view][curpin].ftran) -
                                     prev_tran;
                    }
                    if(delta_tran > 0.0)
                        delta_tran = 0;

                    if(change_type) {
                        cell_retype(cells[cur], -1);
                        cells[cur].isChanged -= 2;
                    }

                    OneTimer(cells[cur], STA_MARGIN, true);

                    double delta_sw_power, delta_leak, delta_int;

                    if(ALPHA != 0.0) {
                        delta_sw_power =
                            LookupDeltaSwitchPower(cells[cur], 1, 0);
                        delta_int = LookupDeltaIntPower(cells[cur], 1, 0);
                    }
                    else {
                        delta_sw_power = 0.0;
                        delta_int = 0.0;
                    }
                    delta_leak = LookupDeltaLeak(cells[cur], 1, 0);
                    double delta_power = ALPHA * (delta_sw_power + delta_int) +
                                         (1 - ALPHA) * delta_leak;
                    delta_impact_type = delta_tran / delta_power;
                }

                if(delta_impact_size == 0 && delta_impact_size == 0) {
                    break;
                }

                if(delta_impact_size < delta_impact_type ||
                   delta_impact_type == 0) {
                    cell_resize(cells[cur], 1);
                    change++;
                    // cout << "UPSIZED CELL " << cells[cur].name << " "
                    //    << prev_type << " --> " << cells[cur].type << endl;
                }
                else {
                    cell_retype(cells[cur], 1);
                    change++;
                    // cout << "UPTYPED CELL " << cells[cur].name << " "
                    //    << prev_type << " --> " << cells[cur].type << endl;
                }

                OneTimer(cells[cur], STA_MARGIN, true);
            }
            //            cout << "AFTER MAX TRAN " <<
            //            getFullPinName(pins[view][curpin]) << " "
            //                << max(pins[view][curpin].rtran,
            //                pins[view][curpin].ftran) << "/" <<
            //                pins[view][curpin].max_tran << endl;
        }

        for(unsigned j = 0; j < cells[cur].inpins.size(); j++) {
            unsigned curpin = cells[cur].inpins[j];

            if(pins[view][curpin].waiveTran) {
                continue;
            }

            if(!IsTranVio(pins[view][curpin])) {
                continue;
            }
            //            cout << "MAX TRAN " <<
            //            getFullPinName(pins[view][curpin]) << " "
            //                << max(pins[view][curpin].rtran,
            //                pins[view][curpin].ftran) << "/" <<
            //                pins[view][curpin].max_tran << endl;

            unsigned ficell = UINT_MAX;
            if(cells[cur].inpins[j] != UINT_MAX) {
                if(pins[view][cells[cur].inpins[j]].net != UINT_MAX) {
                    if(nets[corner][pins[view][cells[cur].inpins[j]].net]
                           .inpin != UINT_MAX) {
                        ficell = pins[view]
                                     [nets[corner]
                                          [pins[view][cells[cur].inpins[j]].net]
                                              .inpin]
                                         .owner;
                    }
                }
            }

            if(ficell == UINT_MAX) {
                continue;
            }

            if(getLibCellInfo(cells[ficell], corner) == NULL) {
                continue;
            }
            if(cells[ficell].isClockCell) {
                continue;
            }
            if(cells[ficell].isDontTouch)
                continue;

            unsigned max_upsize = 5;
            unsigned iter = 0;
            // upsizing fanin cell
            while(IsTranVio(pins[view][curpin])) {
                if(iter > max_upsize) {
                    break;
                }

                iter++;

                if(r_type(cells[ficell]) == numVt - 1 && isMax(cells[ficell])) {
                    break;
                }

                double delta_impact_size = 0.0, delta_impact_type = 0.0;
                double prev_tran =
                    max(pins[view][curpin].rtran, pins[view][curpin].ftran);
                string prev_type = cells[ficell].type;

                if(!isMax(cells[ficell])) {
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    prev_tns = viewTNS[view];

                    bool change_size = cell_resize(cells[ficell], 1);
                    if(change_size) {
                        OneTimer(cells[ficell], STA_MARGIN, true);
                    }
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    cur_tns = viewTNS[view];

                    double delta_tran = 0.0;

                    if(cur_tns > prev_tns) {
                        delta_tran = 0.0;
                    }
                    else {
                        delta_tran = max(pins[view][curpin].rtran,
                                         pins[view][curpin].ftran) -
                                     prev_tran;
                    }

                    if(delta_tran > 0.0)
                        delta_tran = 0;

                    if(change_size) {
                        cell_resize(cells[ficell], -1);
                        cells[ficell].isChanged -= 2;
                    }

                    OneTimer(cells[ficell], STA_MARGIN, true);

                    double delta_sw_power, delta_leak, delta_int;
                    if(ALPHA != 0.0) {
                        delta_sw_power =
                            LookupDeltaSwitchPower(cells[cur], 1, 0);
                        delta_int = LookupDeltaIntPower(cells[cur], 1, 0);
                    }
                    else {
                        delta_sw_power = 0.0;
                        delta_int = 0.0;
                    }
                    delta_leak = LookupDeltaLeak(cells[cur], 1, 0);
                    double delta_power = ALPHA * (delta_sw_power + delta_int) +
                                         (1 - ALPHA) * delta_leak;
                    delta_impact_size = delta_tran / delta_power;
                }

                if(r_type(cells[ficell]) != (numVt - 1)) {
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    prev_tns = viewTNS[view];

                    bool change_type = cell_retype(cells[ficell], 1);

                    OneTimer(cells[ficell], STA_MARGIN, true);
                    // CalcStats((unsigned)thread_id, false, "", view, false);
                    cur_tns = viewTNS[view];

                    double delta_tran = 0.0;

                    if(cur_tns > prev_tns) {
                        delta_tran = 0.0;
                    }
                    else {
                        delta_tran = max(pins[view][curpin].rtran,
                                         pins[view][curpin].ftran) -
                                     prev_tran;
                    }
                    if(delta_tran > 0.0)
                        delta_tran = 0;

                    if(change_type) {
                        cell_retype(cells[ficell], -1);
                        cells[ficell].isChanged -= 2;
                    }

                    OneTimer(cells[ficell], STA_MARGIN, true);

                    double delta_sw_power, delta_leak, delta_int;

                    if(ALPHA != 0.0) {
                        delta_sw_power =
                            LookupDeltaSwitchPower(cells[cur], 1, 0);
                        delta_int = LookupDeltaIntPower(cells[cur], 1, 0);
                    }
                    else {
                        delta_sw_power = 0.0;
                        delta_int = 0.0;
                    }
                    delta_leak = LookupDeltaLeak(cells[cur], 1, 0);
                    double delta_power = ALPHA * (delta_sw_power + delta_int) +
                                         (1 - ALPHA) * delta_leak;
                    delta_impact_type = delta_tran / delta_power;
                }

                if(delta_impact_size == 0 && delta_impact_size == 0) {
                    break;
                }

                if(delta_impact_size < delta_impact_type ||
                   delta_impact_type == 0) {
                    cell_resize(cells[ficell], 1);
                    change++;
                    // cout << "UPSIZED CELL " << cells[ficell].name << " "
                    //    << prev_type << " --> " << cells[ficell].type << endl;
                }
                else {
                    cell_retype(cells[ficell], 1);
                    change++;
                    // cout << "UPTYPED CELL " << cells[ficell].name << " "
                    //    << prev_type << " --> " << cells[ficell].type << endl;
                }

                OneTimer(cells[ficell], STA_MARGIN, true);
            }
            // cout << "AFTER MAX TRAN " << getFullPinName(pins[view][curpin])
            // << " "
            //    << max(pins[view][curpin].rtran, pins[view][curpin].ftran) <<
            //    "/" <<
            //    pins[view][curpin].max_tran << endl;
        }
    }
    cout << "finished." << endl;
    return change;
}

int Sizer::FwdFixSlewViolationCell(bool corr_pt, unsigned option, unsigned cur,
                                   double maxTran, unsigned view) {
    cout << "Fwd fix slew violation cell .. for view " << view << endl;
    unsigned corner = 0;  // mmmcViewList[view].corner;

    int swap_cell_cnt = 0;
    for(unsigned j = 0; j < cells[cur].inpins.size(); j++) {
        double pin_slew = 0;
        unsigned pin_index = cells[cur].inpins[j];
        if(corr_pt) {
            pin_slew = max(
                T[view]->getRiseTran(getFullPinName(pins[view][pin_index])),
                T[view]->getFallTran(getFullPinName(pins[view][pin_index])));
        }
        else {
            pin_slew =
                max(pins[view][pin_index].rtran, pins[view][pin_index].ftran);
        }

        int swap = 0;
        while(pin_slew > maxTran) {
            unsigned curnet = pins[view][pin_index].net;
            unsigned fipin = nets[corner][curnet].inpin;
            unsigned curfi = pins[view][fipin].owner;

            if(curfi == UINT_MAX)
                break;
            if(r_type(cells[curfi]) == (numVt - 1) && isMax(cells[curfi]))
                break;
            swap = UpSizeCellGreedy(corr_pt, option, curfi, 0.0, 1);
            if(swap == 0)
                break;
            else
                swap_cell_cnt += swap;
        }
    }
    return swap_cell_cnt;
}
