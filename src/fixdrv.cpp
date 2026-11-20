#include "rsz/Resizer.hh"
#include "dpl/Opendp.h"

#include "cell_sizing.h"
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
#include "sizing_model.h"
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
#include "ord/Design.h"
#include "ord/Timing.h"
#include "ord/OpenRoad.hh"
#include "grt/GlobalRouter.h"

unsigned Sizer::FwdFixCapViolation(unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    unsigned change = 0;

    ecc::logInfo() << "Fwd fix cap violation .. for view " << view << " " << std::endl;

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
            // if(fabs(nets[corner][outnet].cap + loadCap -
            //         pins[view][cells[cur].outpins[k]].totcap) > 1e-3) {
            //     ecc::logPrintf(
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
                            LibCellInfo *old_cell =
                                getLibCellInfo(cell, corner);
                            LibCellInfo *lib_cell_info =
                                sizing_progression(cell, -1, 0, view);

                            if(lib_cell_info != NULL) {
                                delta_cap_size =
                                    lib_cell_info
                                        ->pins[pins[view][curinpin].lib_pin]
                                        .capacitance -
                                    pins[view][curinpin].cap;
                                double old_vio = max(
                                    0.0,
                                    pins[view][curinpin].totcap -
                                        old_cell
                                            ->pins[pins[view][curinpin].lib_pin]
                                            .maxCapacitance);
                                double new_vio = max(
                                    0.0,
                                    pins[view][curinpin].totcap -
                                        lib_cell_info
                                            ->pins[pins[view][curinpin].lib_pin]
                                            .maxCapacitance);
                                delta_cap_size += new_vio - old_vio;
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
                        if(FIXDRV_VT_SWAP && r_type(cells[curfo]) != 0) {
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

                change += targets.size();

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
                ecc::logInfo() << "load cap now is = "
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
                    ecc::logInfo() << "nothing more can be done.. ";
                    ecc::logInfo() << cells[cur].name << " " << lib_cell_info->name
                         << " FOs: " << cells[cur].fos.size() << " "
                         << pins[view][cells[cur].outpins[k]].totcap << "fF"
                         << endl;
                }
            }
#endif
        }
    }
    ofs.close();

    ecc::logPrintf("Fwd FixCap %d cells were changed\n", change);
    ecc::logInfo() << remains << " fF remains, origins " << origins << " fF . " << endl;
    if(fabs(cap_violation - origins) > 3) {
        ecc::logPrintf("cap violation %f, origins %f\n", cap_violation, origins);
        ecc::logPrintf("fwd fix cap not equal to remains\n");
        // exit(0);
    }
    cap_violation = remains;
    return change;
}

// increase the size of the cells to increase the maxCap, furthermore fix the
// cap violation.
unsigned Sizer::BwdFixCapViolation(unsigned view) {
    unsigned change = 0;
    ecc::logInfo() << "Bwd fix cap violation .. for view " << view << " ";
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
                else if(FIXDRV_VT_SWAP && r_type(cells[cur]) != (numVt - 1)) {
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
                ecc::logInfo() << endl
                     << "--cap violation at : " << cells[cur].name
                     << " max cap: " << maxCap
                     << " totcap: " << pins[view][cells[cur].outpins[k]].totcap
                     << endl;
            }
#endif
        }
    }
    ecc::logInfo() << remains << " fF out of " << origins << " fF remains. " << endl;
    ecc::logInfo() << "cells changed: " << change << endl;
    cap_violation = remains;
    return change;
}

// FIXME: need to update in the FWD fix slew violation
// FIXME: This function invoke calc_stats for each cell. This is not efficient.
// It should be invoked only once for all cells.
// FIXME: calcStats needs to be updated to handle one cell at a time.
unsigned Sizer::FwdFixSlewViolation(double maxTranRatio, unsigned view) {
    unsigned change = 0;
    unsigned thread_id = 0;

    ecc::logInfo() << "Fwd fix slew violation .. for view " << view << endl;
    unsigned corner = 0;  // mmmcViewList[view].corner;
    double prev_tns, cur_tns = 0.0;
    bool old_updatePinAcc = updatePinAcc;
    // updatePinAcc = true;
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

            if(!IsTranVio(pins[view][curpin], cur_max_tran)) {
                continue;
            }

            //            ecc::logInfo() << "MAX TRAN " <<
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
                // if(isff(cells[focell])) {
                //     continue;
                // }
                // CalcStats((unsigned)thread_id, false, "", view, false);
                prev_tns = viewTNS[view];
                double prev_tran = GetCellTran(cells[focell], view) +
                                   GetFICellTran(cells[focell], view) +
                                   GetFOCellTran(cells[focell], view) +
                                   GetNetFOTran(outnet, view) +
                                   GetCellCapVio(cells[focell], view);
                if(cell_resize(cells[focell], -1)) {
                    OneTimer(cells[focell], 1, true);
                    // CalcStats((unsigned)thread_id, false, "", view,
                    // false);
                    cur_tns = viewTNS[view];
                    double now_tran = GetCellTran(cells[focell], view) +
                                      GetFICellTran(cells[focell], view) +
                                      GetFOCellTran(cells[focell], view) +
                                      GetNetFOTran(outnet, view) +
                                      GetCellCapVio(cells[focell], view);
                    change++;
                    if(now_tran > prev_tran) {
                        cell_resize(cells[focell], 1);
                        cells[focell].isChanged -= 2;
                        change--;
                        OneTimer(cells[focell], 1, true);
                    }
                    else if(!IsTranVio(pins[view][curpin], cur_max_tran)) {
                        break;
                    }
                }
            }

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

                if(FIXDRV_VT_SWAP && r_type(cells[cur]) != (numVt - 1)) {
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
                    // ecc::logInfo() << "UPSIZED CELL " << cells[cur].name << " "
                    //    << prev_type << " --> " << cells[cur].type << endl;
                }
                else {
                    cell_retype(cells[cur], 1);
                    change++;
                    // ecc::logInfo() << "UPTYPED CELL " << cells[cur].name << " "
                    //    << prev_type << " --> " << cells[cur].type << endl;
                }

                OneTimer(cells[cur], STA_MARGIN, true);
            }
            //            ecc::logInfo() << "AFTER MAX TRAN " <<
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
            //            ecc::logInfo() << "MAX TRAN " <<
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

                if(FIXDRV_VT_SWAP && r_type(cells[ficell]) != (numVt - 1)) {
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
                    // ecc::logInfo() << "UPSIZED CELL " << cells[ficell].name << " "
                    //    << prev_type << " --> " << cells[ficell].type << endl;
                }
                else {
                    cell_retype(cells[ficell], 1);
                    change++;
                    // ecc::logInfo() << "UPTYPED CELL " << cells[ficell].name << " "
                    //    << prev_type << " --> " << cells[ficell].type << endl;
                }

                OneTimer(cells[ficell], STA_MARGIN, true);
            }
            // ecc::logInfo() << "AFTER MAX TRAN " << getFullPinName(pins[view][curpin])
            // << " "
            //    << max(pins[view][curpin].rtran, pins[view][curpin].ftran) <<
            //    "/" <<
            //    pins[view][curpin].max_tran << endl;
        }
    }
    updatePinAcc = old_updatePinAcc;
    ecc::logInfo() << "finished." << endl;
    ecc::logInfo() << "cells changed: " << change << endl;
    return change;
}

// FIXME: This function invoke calc_stats for each cell. This is not efficient.
// It should be invoked only once for all cells.
// FIXME: calcStats needs to be updated to handle one cell at a time.

// FIXME: This function invoke calc_stats for each cell. This is not efficient.
// It should be invoked only once for all cells.
// FIXME: calcStats needs to be updated to handle one cell at a time.
unsigned Sizer::FwdFixSlewViolationPost(double maxTranRatio, unsigned view) {
    unsigned change = 0;
    unsigned thread_id = 0;
    bool old_updatePinAcc = updatePinAcc;
    updatePinAcc = true;
    ecc::logInfo() << "Fwd fix slew violation .. for view " << view << endl;
    unsigned corner = 0;  // mmmcViewList[view].corner;
    double prev_tns, cur_tns = 0.0;

    for(unsigned i = 0; i < topolist.size(); i++) {
        unsigned cur = topolist[i];

        // if(cells[cur].isClockCell) {
        //     continue;
        // }
        if(getLibCellInfo(cells[cur], corner) == NULL) {
            continue;
        }
        if(post_slew_opt_output) {
            for(unsigned j = 0; j < cells[cur].outpins.size(); j++) {
                if(cells[cur].isDontTouch)
                    break;
                unsigned curpin = cells[cur].outpins[j];
                unsigned outnet = pins[view][curpin].net;
                // upsizing target cell
                while(IsTranVio(pins[view][curpin])) {
                    if(cells[cur].isDontTouch)
                        break;

                    double delta_impact_size = 0.0, delta_impact_type = 0.0;
                    double prev_tran = GetCellTran(cells[cur], view) +
                                       GetFICellTran(cells[cur], view) +
                                       GetCellTran(cells[cur], view) +
                                       GetCellCapVio(cells[cur], view);
                    string prev_type = cells[cur].type;
                    auto lib_cell_info = getLibCellInfo(cells[cur], corner);
                    double prev_width = lib_cell_info->width;
                    int size_num =
                        main_lib_cell_tables[corner]
                                            [cells[cur].main_lib_cell_id]
                                                ->lib_vt_size_table.size();
                    entry tmpEntry;
                    tmpEntry.id = cur;
                    for(int new_size = 0; new_size < size_num; new_size++) {
                        int step = new_size - cells[cur].c_size;
                        if(step == 0) {
                            continue;
                        }
                        bool change_size = cell_resize(cells[cur], step);

                        if(change_size) {
                            OneTimer(cells[cur], 0.1, true);
                        }

                        double cur_slack =
                            min(GetCellSlack(cells[cur], view),
                                GetFICellSlack(cells[cur], view));

                        cur_tns = prev_tns;

                        double now_tran = GetCellTran(cells[cur], view) +
                                          GetFICellTran(cells[cur], view) +
                                          GetCellTran(cells[cur], view) +
                                          GetCellCapVio(cells[cur], view);
                        double now_width =
                            getLibCellInfo(cells[cur], corner)->width;
                        //   GetcurTran(cells[cur], view);
                        double delta_tran =
                            now_tran - prev_tran;  // + new_tran - old_tran;
                        double delta_width = now_width - prev_width;
                        double benefit = 0;
                        if(delta_tran < 0) {
                            benefit = 0;
                        }
                        else {
                            if(delta_width <= 0) {
                                if(VERBOSE >= 4) {
                                    ecc::logPrintf("Error: not defined delta_width\n");
                                }
                                benefit = delta_tran;
                            }
                            else {
                                benefit = delta_tran / delta_width;
                            }
                        }
                        if(benefit < tmpEntry.delta_impact) {
                            tmpEntry.delta_impact = benefit;
                            tmpEntry.step = step;
                        }
                        if(change_size) {
                            cell_resize(cells[cur], -step);
                            cells[cur].isChanged -= 2;
                            OneTimer(cells[cur], 0.1, true);
                        }
                    }
                    if(tmpEntry.delta_impact < -0.001) {
                        ecc::logPrintf("cell %s size %d, step %d, delta_impact %f\n",
                               cells[cur].name.c_str(), cells[cur].c_size,
                               tmpEntry.step, tmpEntry.delta_impact);
                        bool change_size =
                            cell_resize(cells[tmpEntry.id], tmpEntry.step);
                        OneTimer(cells[cur], 0.1, true);
                        change++;
                        // targets.insert(tmpEntry);
                    }
                    else {
                        break;
                    }
                }
            }
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
            if(nets[corner][outnet].inpin == UINT_MAX) {
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
                    ecc::logPrintf("cell %s size %d, step %d, delta_impact %f\n",
                           cells[focell].name.c_str(), cells[focell].c_size,
                           tmpEntry.step, tmpEntry.delta_impact);
                    bool change_size =
                        cell_resize(cells[tmpEntry.id], tmpEntry.step);
                    OneTimer(cells[focell], 0.1, true);
                    change++;
                    // targets.insert(tmpEntry);
                }
            }

            //         if(change_size) {
            //             OneTimer(cells[focell], 1, true);
            //         }
            //         double cur_slack = min(GetCellSlack(cells[focell],
            //         view),
            //                                GetFICellSlack(cells[focell],
            //                                view));

            //         cur_tns = prev_tns;
            //         double now_tran = GetCellTran(cells[focell], view) +
            //                           GetFICellTran(cells[focell], view)
            //                           + GetFOCellTran(cells[focell],
            //                           view);

            //         double delta_tran = now_tran - prev_tran;

            //         double delta_slack = cur_slack - prev_slack;

        }
    }
    updatePinAcc = old_updatePinAcc;
    ecc::logInfo() << "Post fix slew " << change << " cells were changed." << endl;
    ecc::logInfo() << "finished." << endl;
    return change;
}

// FIXME: This function invoke calc_stats for each cell. This is not
// efficient. It should be invoked only once for all cells.
// FIXME: calcStats needs to be updated to handle one cell at a time.

int Sizer::FwdFixSlewViolationCell(bool corr_pt, unsigned option, unsigned cur,
                                   double maxTran, unsigned view) {
    ecc::logInfo() << "Fwd fix slew violation cell .. for view " << view << endl;
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

entry Sizer::calcCellBestEntry(CELL &cell, int cell_id, int sensFunc,
                               double gamma, double alpha, int view) {
    entry tmpEntry;
    int i = cell_id;
    tmpEntry.id = i;

    double npaths = 0.0;
    double totcaps = 0.0;

    for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
        npaths += pins[view][cells[i].outpins[j]].NPaths;
        totcaps += (double)pins[view][cells[i].outpins[j]].totcap;
    }
    auto &cap_vec = cap_lib_cell_vec[view][cell.type];
    if(GTR_METRIC2 == SF5) {
        if(NO_TOPO) {
            tmpEntry.tie_break = i;
        }
        else
            tmpEntry.tie_break = map2topoidx[i];
    }
    else if(GTR_METRIC2 == SF6) {
        tmpEntry.tie_break = (double)(r_size(cells[i]) + 1) / npaths;
    }
    else {
        tmpEntry.tie_break = (double)(r_size(cells[i]) + 1) / totcaps;
    }

    vector< double > delta_impact_size, delta_impact_type;
    vector< int > size_step, type_step;

    for(int k = 1; k <= MULTI_STEP; ++k) {
        // upsizing
        if(isSizable(cells[i], k) && !cells[i].touched) {
            double sf = 0.0;
            if(!mmmcOn) {
                sf = CalSens(cells[i], k, 0, sensFunc, gamma, alpha, view);
            }
            else {
                sf = CalSensMMMC(cells[i], k, 0, sensFunc, gamma, alpha, true);
            }
            // double delta_width =
            // ecc::logInfo() << "UPSIZING SF " << sf << endl;
            if(sf != 0.0) {
                delta_impact_size.push_back(1.0 / sf);
                size_step.push_back(k);
            }
        }
    }

    double max_delta_impact_size = 0.0;
    double max_delta_impact_type = 0.0;
    int max_step_size = 0;
    int max_step_type = 0;

    for(unsigned k = 0; k < delta_impact_size.size(); ++k) {
        if(max_delta_impact_size > delta_impact_size[k]) {
            max_step_size = size_step[k];
            max_delta_impact_size = delta_impact_size[k];
        }
    }

    tmpEntry.delta_impact = max_delta_impact_size;
    tmpEntry.step = max_step_size;
    tmpEntry.change = UPSIZE;

    return tmpEntry;
}

// slack/slew optimization
unsigned Sizer::AttackNew(unsigned iter, unsigned STAGE, double RATIO,
                          double leak_exponent, double alpha,
                          double break_ratio, unsigned thread_id, double toler,
                          unsigned view) {
    if(alpha == -1) {
        alpha = ALPHA;
    }
    unsigned swap_cnt = 0;
    if(STAGE == GLOBAL || STAGE == FINESWAP) {
        if(STAGE == GLOBAL) {
            ecc::logInfo() << endl
                 << "(" << thread_id << ") Start global opt.. round " << iter
                 << endl;
        }
        else {
            ecc::logInfo() << endl
                 << "(" << thread_id
                 << ") Start global opt (fine swap) .. round " << iter << endl;
        }
        // CallTimer(view);
        // CorrelatePT((unsigned)thread_id, view);
        // CalcStats((unsigned)thread_id, false, "Initial TIMING_RECOVERY",
        // view);

        for(unsigned view1 = 0; view1 < numViews; ++view1) {
            unsigned mode1 = mmmcViewList[view1].mode;
            if(viewTNS[view1] == 0.0) {
                viewWeightTime[view1] = 0.0;
            }
            else {
                if(NORM_SFT == 1) {
                    viewWeightTime[view1] = 1.0 / clk_period[mode1];
                }
                else if(NORM_SFT == 2) {
                    viewWeightTime[view1] = viewTNS[view1] / clk_period[mode1];
                }
                else if(NORM_SFT == 3) {
                    viewWeightTime[view1] = -viewWNS[view1] / clk_period[mode1];
                }
                else if(NORM_SFT == 4) {
                    viewWeightTime[view1] = viewWeight[view1];
                }
            }
        }
        // set< entry > targets;
        // if(!already_built) {
        //     buildTargets(iter, STAGE, RATIO, leak_exponent, alpha,
        //     break_ratio,
        //                  thread_id, toler, view);
        //     already_built = true;
        // }

        for(unsigned i = 0; i < numcells; i++) {
            if(cells[i].isClockCell)
                continue;
            if(cells[i].isDontTouch)
                continue;
            if(isff(cells[i])) {
                continue;
            }
            bool attack = false;
            // if(i % 100 == 0) {
            //     ecc::logPrintf("Attack %d/%d\n", i, numcells);
            // }
            for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
                if(pins[view][cells[i].outpins[j]].rslk < toler ||
                   pins[view][cells[i].outpins[j]].fslk < toler) {
                    attack = true;
                    break;
                }
            }

            if(attack) {
                if(r_type(cells[i]) == (numVt - 1) && isMax(cells[i]))
                    continue;
                // FIX : shouldn't it be checked first in loop_i ?

                entry tmpEntry = calcCellBestEntry(cells[i], i, sensFuncT,
                                                   leak_exponent, alpha, view);
                if(tmpEntry.delta_impact < 0.0)
                    targets.insert(tmpEntry);

                // ecc::logInfo() << "DELTA IMPACT " << tmpEntry.delta_impact << endl;
            }
        }
        ecc::logInfo() << "# priority targets  : " << targets.size() << endl;

        if(targets.size() == 0) {
            // it's stuck. let's release some of cells
            // Release(false, CRIT_LEGALIZE);
            return 0;
        }

        unsigned count = 0;
        unsigned update_cnt = 0;

        vector< bool > changed;
        changed.resize(numcells);
        for(unsigned i = 0; i < numcells; i++)  // FIX???
            changed[i] = false;
        CallTimer(view);
        CalcStats((unsigned)thread_id, false, "After Initial TIMING_RECOVERY",
                  view);
        double prev_tns, new_tns = 0.0;
        int num = targets.size();
        int iter = 0;
        while(targets.size() > 0) {
            entry tmp = *(targets.begin());
            targets.erase(tmp);
            entry *it = &tmp;
            if(iter % 100 == 0) {
                ecc::logPrintf("Attack targets %d/%d\n", iter, num);
            }
            iter++;
            if(update_cnt > (numcells * CORR_RATIO)) {
                CallTimer(view);
                CalcStats((unsigned)thread_id, false, "Corr TIMING_RECOVERY",
                          view);
                update_cnt = 0;
            }
            // if(update_cnt > (targets.size() * break_ratio)) {
            //     break;
            // }

            if(skew_violation_worst == 0.0 || worst_slack_worst >= toler) {
                break;
            }

            unsigned cur = it->id;
            if(changed[cur]) {
                // ecc::logInfo() << "Attack again " << cells[cur].name <<
                // std::endl;
            }

            double prev_slack = min(GetCellSlack(cells[cur], view),
                                    GetFICellSlack(cells[cur], view));
            double new_slack = prev_slack;

            if(prev_slack > toler)
                continue;

            double delta_delay = 0.0;

            bool change = false;
            if(it->change == UPSIZE) {
                if(STAGE == GLOBAL) {
                    if(VERBOSE >= 4)
                        ecc::logInfo() << "GLOBAL TARGET = UPSIZE (" << it->step
                             << "): " << cells[cur].name << " "
                             << cells[cur].type;
                }
                else {
                    if(VERBOSE >= 4)
                        ecc::logInfo() << "FINESWAP TARGET = UPSIZE (" << it->step
                             << "): " << cells[cur].name << " "
                             << cells[cur].type;
                }

                if(HOLD_CHECK) {
                    delta_delay = EstDeltaDelay(cells[cur], it->step, view);
                }

                change = cell_resize(cells[cur], it->step);
            }

            //// check timing after cell change

            bool restore = false;
            if(!CHECK_ALL_VIEW) {
                // ista for the new solution
                OneTimer(cells[cur], STA_MARGIN, true);

                new_slack = min(GetCellSlack(cells[cur], view),
                                GetFICellSlack(cells[cur], view));

                if(prev_slack > new_slack && new_slack < 0.0)
                    restore = true;

                if(!restore && HOLD_CHECK) {
                    if(EstHoldVio(cells[cur], delta_delay, view)) {
                        restore = true;
                    }
                }
            }

            if(restore && change) {  // restore
                if(it->change == UPSIZE) {
                    cell_resize(cells[cur], -it->step);
                }
                else {
                    cell_retype(cells[cur], -it->step);
                }

                // ista with the restored solution
                if(!CHECK_ALL_VIEW) {
                    OneTimer(cells[cur], STA_MARGIN, true);
                }
                else {
                    for(unsigned view1 = 0; view1 < numViews; ++view1) {
                        OneTimer(cells[cur], STA_MARGIN, true);
                    }
                }

                if(VERBOSE >= 4)
                    ecc::logInfo() << "-> " << cells[cur].type << " -- Restored" << endl;
                cells[cur].isChanged -= 2;
            }
            else {  // accept
                changed[cur] = true;
                if(VERBOSE >= 4)
                    ecc::logInfo() << "-> " << cells[cur].type << " -- Accepted" << endl;
                count++;
                update_cnt++;
                swap_cnt++;
                int i = it->id;
                if(!isMax(cells[i])) {
                    entry tmpEntry = calcCellBestEntry(
                        cells[i], i, sensFuncT, leak_exponent, alpha, view);
                    if(tmpEntry.delta_impact < 0.0)
                        targets.insert(tmpEntry);

                    // cells[cur].isChanged++;
                    // CalcStats((unsigned)thread_id, false, "After accept
                    // TIMING_RECOVERY", view);
                }
            }

            cells[cur].critical++;

            if(count > (double)numcells * RATIO * 0.01)
                break;
        }
        targets.clear();
    }
    ecc::logInfo() << "swap count = " << swap_cnt << endl;

    return swap_cnt;
}
