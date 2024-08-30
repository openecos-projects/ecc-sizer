///////////////////////////////////////////////////////////////////////////////
// Authors: Hyein Lee and Jiajia Li
//          (Ph.D. advisor: Andrew B. Kahng)
//
//          Many subsequent improvements were made by Minsoo Kim
//          leading up to the initial release.
//
// BSD 3-Clause License
//
// Copyright (c) 2018, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

#include <sys/param.h>
#include <algorithm>
#include <cassert>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include "sizer.h"
#include "db_sta/dbNetwork.hh"
double Sizer::CalcStats(unsigned thread_id, bool rpt_power, string stage,
                        unsigned view, bool log) {
    // skew_violation == positive means violations
    // slew_violation == positive means violations

    unsigned corner = 0;  // mmmcViewList[view].corner;
    slew_violation = CalcSlewViolation(view);
    double tran_tot, tran_max;
    int tran_num = 0;
    if(TEST_MODE == "ALL_TEST") {
        tran_tot = tran_max = 0.0;
        T[view]->getTranVio(tran_tot, tran_max, tran_num);
        printf("SLEW VIOLATION cnt %d, slew_violation_wst %f\n", tran_num,
               tran_max);
        printf("Our slew violation :%f, OpenSTA slew violation: %f\n",
               slew_violation, tran_tot);
    }
    if(!FIX_SLEW) {
        tran_tot = tran_max = 0.0;
        T[view]->getTranVio(tran_tot, tran_max, tran_num);
        slew_violation_cnt = tran_num;
        slew_violation_wst = tran_max;
        slew_violation = tran_tot;
    }

    skew_violation = CalcSlackViolation(view);
    viewTNS[view] = skew_violation;
    viewWNS[view] = min(max_neg_rslk, max_neg_fslk) + viewSlackMargin[view];
    viewVioCnt[view] = skew_violation_cnt;
    viewPower[view] = CalcPower(thread_id, rpt_power, view);
    viewWorstSlack[view] = worst_slack + viewSlackMargin[view];

    power = 0.0;
    skew_violation_worst = 0;
    worst_slack_worst = DBL_MAX;
    for(unsigned view1 = 0; view1 < numViews; ++view1) {
        power += viewPower[view1];
        if(skew_violation_worst < viewTNS[view1]) {
            skew_violation_worst = viewTNS[view1];
        }
        if(worst_slack_worst > viewWorstSlack[view1]) {
            worst_slack_worst = viewWorstSlack[view1];
        }
    }

    // TODO
    cap_violation = CalcCapViolation();
    if(TEST_MODE == "ALL_TEST") {
        double cap_tot, cap_max = 0;
        int cap_num = 0;
        T[view]->getCapVio(cap_tot, cap_max, cap_num);
        printf("CAP VIOLATION cnt %d, cap_violation_wst %f\n", cap_num,
               cap_max);
        printf("Our cap violation :%f, OpenSTA cap violation: %f\n",
               cap_violation, cap_tot);
    }
    // exit(0);
    l2_norm = 0.0;
    average_error = 0.0;
    score = calcScore(viewPower[view], skew_violation, slew_violation,
                      cap_violation);
    // max_pt_err = CalcPTErrors(average_error, l2_norm);
    tot_violations = slew_violation + skew_violation + cap_violation;
    if(VERBOSE >= 1) {
        cout << "--------------------------------------------" << endl;
        cout << "Total Slack violation  : " << (-1) * skew_violation << "ns"
             << endl;
        cout << "--Max negative rslack  : " << max_neg_rslk << endl;
        cout << "--Max negative fslack  : " << max_neg_fslk << endl;
        cout << "--Max positive rslack  : " << max_pos_rslk << endl;
        cout << "--Max positive rslack  : " << max_pos_rslk << endl;
        if(HOLD_CHECK) {
            cout << "--Min negative (hold) rslack  : " << min_neg_rslk << endl;
            cout << "--Min negative (hold) fslack  : " << min_neg_fslk << endl;
        }
        cout << "Total Cap violation    : " << cap_violation << "fF" << endl;
        cout << "Total Slew violation   : " << slew_violation << "ns ("
             << maxTran[corner] << "ns) " << endl;
        cout << "Total violations       : " << tot_violations << endl;
        cout << "Total power            : " << power << "uW" << endl;
        if(CORR_PT && pt_err)
            cout << "PT ERRORS  MAX : " << max_pt_err
                 << "\tAVG : " << average_error << "\tL2_NORM : " << l2_norm
                 << endl;
        cout << "--------------------------------------------" << endl;
    }

    if(log) {
        if(!rpt_power) {
            if(HOLD_CHECK)
                cout << stage << " THREAD" << thread_id << "[view" << view
                     << "] "
                        "Violations(total/TNS/WNS/WNSMin/slew/cap/leakage/"
                        "numNS/slk_gb): "
                     << tot_violations << " " << ((double)-1.0) * skew_violation
                     << " " << min(max_neg_rslk, max_neg_fslk) << "("
                     << viewWNS[view] << "/" << viewSlackMargin[view] << ") "
                     << slew_violation << " " << cap_violation << " "
                     << power * 0.000001 << " " << skew_violation_cnt << " "
                     << GetGB(view) << endl;
            else
                cout << stage << " THREAD" << thread_id << "[view" << view
                     << "] "
                        "Violations(total/TNS/WNS/slew/cap/leakage/numNS/"
                        "slk_gb): "
                     << tot_violations << " " << ((double)-1.0) * skew_violation
                     << " " << min(max_neg_rslk, max_neg_fslk) << "("
                     << viewWNS[view] << "/" << viewSlackMargin[view] << ") "
                     << slew_violation << " " << cap_violation << " "
                     << power * 0.000001 << " " << skew_violation_cnt << " "
                     << GetGB(view) << endl;
        }
        else {
            double int_leak_power = 0.0;
            int_leak_power = T[view]->getLeakPower();
            if(HOLD_CHECK)
                cout << stage << " THREAD" << thread_id
                     << " Violations(total/TNS/WNS/WNSMin/slew/cap/power/leak/"
                        "numNS/slk_gb): "
                     << tot_violations << " " << ((double)-1.0) * skew_violation
                     << " " << min(max_neg_rslk, max_neg_fslk) << "("
                     << viewWNS[view] << "/" << viewSlackMargin[view] << ") "
                     << slew_violation << " " << cap_violation << " "
                     << power * 0.000001 << " " << int_leak_power << " "
                     << skew_violation_cnt << " " << GetGB(view) << endl;
            else
                cout << stage << " THREAD" << thread_id << " VIEW" << view
                     << " Violations(total/TNS/WNS/slew/cap/power/leak/numNS/"
                        "slk_gb): "
                     << tot_violations << " " << ((double)-1.0) * skew_violation
                     << " " << min(max_neg_rslk, max_neg_fslk) << "("
                     << viewWNS[view] << "/" << viewSlackMargin[view] << ") "
                     << slew_violation << " " << cap_violation << " "
                     << power * 0.000001 << " " << int_leak_power << " "
                     << skew_violation_cnt << " " << GetGB(view) << endl;
        }
    }
    return tot_violations;
}

double Sizer::CalcSize() {
    double totSize = 0.;
    for(unsigned i = 0; i < numcells; i++) {
        LibCellInfo* lib_cell_info = getLibCellInfo(cells[i], 0);
        totSize += lib_cell_info->area;
    }
    return totSize;
}

double Sizer::CalcPTErrors(double& avg_err, double& l2_norm, unsigned view) {
    long double max_err = 0.0;
    long double total_err = 0.0;

    for(unsigned i = 0; i < numpins; ++i) {
        if(pins[view][i].rslk_ofs ==
               std::numeric_limits< double >::infinity() ||
           pins[view][i].fslk_ofs ==
               std::numeric_limits< double >::infinity()) {
            continue;
        }
        double rslk = pins[view][i].rslk_ofs;
        double fslk = pins[view][i].fslk_ofs;
        if(pins[view][i].rslk_ofs < 0) {
            rslk = -1 * pins[view][i].rslk_ofs;
        }

        if(pins[view][i].fslk_ofs < 0) {
            rslk = -1 * pins[view][i].fslk_ofs;
        }

        if(max_err < rslk || max_err < fslk) {
            max_err = max(rslk, fslk);
        }
        total_err += rslk + fslk;
        l2_norm += pow(rslk, 2) + pow(fslk, 2);
    }
    l2_norm = sqrt(l2_norm);
    avg_err = total_err / (2 * numpins);
    return max_err;
}
// FIXME: has bug
double Sizer::CalcSlewViolation(unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    double slew_viol = 0.;
    slew_violation_cnt = 0;
    slew_violation_wst = 0;
    ofstream ofs("our_tran_vio.txt");
    double slew_bound = 0.;
    for(unsigned i = 0; i < numcells; i++) {
        std::vector< unsigned > pin_id_list;
        for(unsigned j = 0; j < cells[i].inpins.size(); j++) {
            pin_id_list.push_back(cells[i].inpins[j]);
        }
        for(unsigned j = 0; j < cells[i].outpins.size(); j++) {
            pin_id_list.push_back(cells[i].outpins[j]);
        }
        for(unsigned curpin : pin_id_list) {
            // unsigned curpin = cells[i].inpins[j];

            if(curpin == UINT_MAX) {
                continue;
            }
            if(pins[view][curpin].name == "CLK" ||
               pins[view][curpin].name == "clk" ||
               pins[view][curpin].owner == UINT_MAX) {
                continue;
                // printf("Pin name %s\n", pins[view][curpin].name.c_str());
            }
            string name = cells[i].name;
            string pin_name = getFullPinName(pins[view][curpin]);
            auto pin_ =
                _ckt->_ord_design->getBlock()->findITerm(pin_name.c_str());
            assert(pin_->getNet() && pin_->getNet()->getSigType() != "POWER" &&
                   pin_->getNet()->getSigType() != "GROUND" &&
                   pin_->getNet()->getSigType() != "CLOCK");
            // if(pins[view][curpin].max_tran == 0) {
            //     printf("Pin name %s max_tran %f\n",
            //            pins[view][curpin].name.c_str(),
            //            pins[view][curpin].max_tran);
            // }
            double t_tran =
                max(pins[view][curpin].rtran, pins[view][curpin].ftran);

            if(t_tran > pins[view][curpin].max_tran) {
                slew_viol += t_tran - pins[view][curpin].max_tran;
                ofs << "Cell type: " << cells[i].type << " "
                    << getFullPinName(pins[view][curpin]) << " pin direction: "
                    << (pin_->isOutputSignal() ? "output" : "input")
                    << " max tran vio: " << t_tran << " "
                    << pins[view][curpin].max_tran << endl;
                if(!pin_->isOutputSignal()) {
                    unsigned net_id = pins[view][curpin].net;
                    if(net_id != UINT_MAX &&
                       nets[corner][net_id].inpin != UINT_MAX) {
                        unsigned cell_opin = nets[corner][net_id].inpin;
                        unsigned cell_id = pins[view][cell_opin].owner;
                        unsigned fopin = curpin;
                        timing_lookup wire_delay =
                            get_wire_delay(net_id, fopin, view);
                        double cur_max_tran = DBL_MAX;

                        cur_max_tran =
                            std::min(sqrt(pow(pins[view][fopin].max_tran -
                                                  pins[view][fopin].rtran_ofs,
                                              2) -
                                          pow(log(9) * wire_delay.rise, 2)),
                                     cur_max_tran);

                        cur_max_tran =
                            std::min(sqrt(pow(pins[view][fopin].max_tran -
                                                  pins[view][fopin].ftran_ofs,
                                              2) -
                                          pow(log(9) * wire_delay.fall, 2)),
                                     cur_max_tran);
                        if(log(9) * wire_delay.fall >
                           pins[view][curpin].max_tran) {
                            slew_bound += max(log(9) * wire_delay.fall +
                                                  pins[view][curpin].ftran_ofs -
                                                  pins[view][curpin].max_tran,
                                              0.0);
                        }
                        ofs << "now cell type: " << cells[i].type << " "
                            << "now cell name: " << cells[i].name << " "
                            << "now cell size " << cells[i].c_size << " "
                            << "pre Cell type: "
                            << (cell_id == UINT_MAX ? "isPI"
                                                    : cells[cell_id].type)
                            << " Cell name "
                            << (cell_id == UINT_MAX ? pins[view][cell_opin].name
                                                    : cells[cell_id].name)
                            << " " << "pre pin tran: "
                            << max(pins[view][cell_opin].rtran,
                                   pins[view][cell_opin].ftran)
                            << " " << "pre pin need max tran: " << cur_max_tran
                            << " " << "net cap: " << nets[corner][net_id].cap
                            << " " << "net delay "
                            << max(wire_delay.rise, wire_delay.fall) << " "
                            << endl;
                    }
                }
                slew_violation_cnt++;
            }
            slew_violation_wst =
                max(slew_violation_wst,
                    pins[view][curpin].rtran - pins[view][curpin].max_tran);
            slew_violation_wst =
                max(slew_violation_wst,
                    pins[view][curpin].ftran - pins[view][curpin].max_tran);
        }
    }
    ofs.close();
#if 0
    for(unsigned i = 0; i < POs.size(); i++) {
        unsigned curpin = POs[i];
        if(curpin == UINT_MAX) {
            continue;
        }
        double slew_max =
            std::max(pins[view][curpin].rtran, pins[view][curpin].ftran);
        if(slew_max > pins[view][curpin].max_tran) {
            slew_viol += slew_max - pins[view][curpin].max_tran;
            ofs << getFullPinName(pins[view][curpin])
                << " max tran vio: " << slew_max << " "
                << pins[view][curpin].max_tran << endl;
            slew_violation_cnt++;
        }
        slew_violation_wst =
            max(slew_violation_wst, slew_max - pins[view][curpin].max_tran);
    }
#endif
    printf("SLEW VIOLATION cnt %d, slew_violation_wst %f\n", slew_violation_cnt,
           slew_violation_wst);
    printf("Minimum slew_bound %f\n", slew_bound);
    return slew_viol;
}

double Sizer::CalcSlackViolation(unsigned view) {
    // worst_slack = worst timing slack; could be positive
    // max_neg_{r,f}slk = worst negative timing slack; could be only
    // negative
    unsigned corner = 0;  // mmmcViewList[view].corner;
    double slack_viol = 0.;
    worst_slack = DBL_MAX;
    max_neg_rslk = max_neg_fslk = 0.;
    min_neg_rslk = min_neg_fslk = 0.;
    max_pos_rslk = max_pos_fslk = 0.;
    skew_violation_cnt = 0;

    tot_pslack = 0.0;

    for(unsigned i = 0; i < FFs.size(); i++) {
        if(getLibCellInfo(cells[FFs[i]], corner) == NULL) {
            continue;
        }

        for(unsigned j = 0; j < cells[FFs[i]].inpins.size(); ++j) {
            unsigned curpin = cells[FFs[i]].inpins[j];
            // cout << "CALC SLACK -- " << cells[FFs[i]].name << endl;

            if(curpin == UINT_MAX) {
                continue;
            }

            if(libs[corner]
                   .find(cells[FFs[i]].type)
                   ->second.pins[pins[view][curpin].lib_pin]
                   .isClock) {
                continue;
            }

            if(DATA_PIN_ONLY) {
                if(!libs[corner]
                        .find(cells[FFs[i]].type)
                        ->second.pins[pins[view][curpin].lib_pin]
                        .isData) {
                    continue;
                }
            }

            double slack =
                min(pins[view][curpin].rslk, pins[view][curpin].fslk);

            if(slack < 0.0) {
                // cout << "TNS UPDATE " <<
                // getFullPinName(pins[view][curpin])
                // << " " << slack << endl;
                slack_viol += slack;
            }
            else {
                tot_pslack += slack;
            }

            if(pins[view][curpin].rslk < 0.0)
                skew_violation_cnt++;
            if(pins[view][curpin].fslk < 0.0)
                skew_violation_cnt++;
            worst_slack = min(worst_slack, slack);
            max_neg_rslk = min(max_neg_rslk, pins[view][curpin].rslk);
            max_neg_fslk = min(max_neg_fslk, pins[view][curpin].fslk);
            if(HOLD_CHECK) {
                min_neg_rslk = min(min_neg_rslk, pins[view][curpin].hold_rslk);
                min_neg_fslk = min(min_neg_fslk, pins[view][curpin].hold_fslk);
            }
            max_pos_rslk = max(max_pos_rslk, pins[view][curpin].rslk);
            max_pos_fslk = max(max_pos_fslk, pins[view][curpin].fslk);
            if(VERBOSE >= 4) {
                cout << "TIMING END POINT CHECK: "
                     << getFullPinName(pins[view][curpin]) << " "
                     << pins[view][curpin].rslk << "/"
                     << pins[view][curpin].fslk << endl;
            }
        }
    }

    for(unsigned i = 0; i < POs.size(); i++) {
        unsigned curpin = POs[i];
        // cout << pins[view][curpin].name << endl;
        if(curpin == UINT_MAX) {
            continue;
        }

        double slack = min(pins[view][curpin].rslk, pins[view][curpin].fslk);
        if(slack < 0.0) {
            // cout << "TNS UPDATE " << getFullPinName(pins[view][curpin])
            // << " " << slack << endl;
            slack_viol += slack;
        }
        else {
            tot_pslack += slack;
        }

        // if(pins[view][curpin].rslk < 0.0) {
        //    slack_viol+=pins[view][curpin].rslk;
        //} else {
        //    tot_pslack+=pins[view][curpin].rslk;
        //}
        // if(pins[view][curpin].fslk < 0.0) {
        //    slack_viol+=pins[view][curpin].fslk;
        //} else {
        //    tot_pslack+=pins[view][curpin].fslk;
        //}
        // slack_viol+=min(pins[view][curpin].rslk, 0.0);
        // slack_viol+=min(pins[view][curpin].fslk, 0.0);
        if(pins[view][curpin].rslk < 0.0)
            skew_violation_cnt++;
        if(pins[view][curpin].fslk < 0.0)
            skew_violation_cnt++;
        worst_slack = min(worst_slack, slack);
        max_neg_rslk = min(max_neg_rslk, pins[view][curpin].rslk);
        max_neg_fslk = min(max_neg_fslk, pins[view][curpin].fslk);

        if(HOLD_CHECK) {
            min_neg_rslk = min(min_neg_rslk, pins[view][curpin].hold_rslk);
            min_neg_fslk = min(min_neg_fslk, pins[view][curpin].hold_fslk);
        }
        max_pos_rslk = max(max_pos_rslk, pins[view][curpin].rslk);
        max_pos_fslk = max(max_pos_fslk, pins[view][curpin].fslk);
        if(VERBOSE >= 4) {
            cout << "TIMING END POINT CHECK: "
                 << getFullPinName(pins[view][curpin]) << " "
                 << pins[view][curpin].rslk << "/" << pins[view][curpin].fslk
                 << endl;
        }
    }
    if(ISO_TNS != 0 || ISO_TIME) {
        if(!mmmcOn) {
            slack_viol += ISO_TNS;
        }
        else {
            slack_viol += viewTNSMargin[view];
        }
    }
    if(slack_viol > -0.001)
        slack_viol = 0.0;
    if(worst_slack > -0.001 && worst_slack < 0.0)
        worst_slack = 0.0;
    return fabs(slack_viol);
}

void Sizer::UpdateCapsFromCells() {
    for(unsigned view = 0; view < numViews; ++view) {
        unsigned corner = 0;  // mmmcViewList[view].corner;
        for(unsigned i = 0; i < numcells; i++) {
            for(unsigned j = 0; j < cells[i].inpins.size(); j++) {
                LibCellInfo* lib_cell_info = getLibCellInfo(cells[i], corner);
                if(lib_cell_info) {
                    int input_j = cells[i].inpins[j];
                    pins[view][input_j].cap =
                        lib_cell_info->pins[pins[view][input_j].lib_pin]
                            .capacitance;
                    if(pins[view][input_j].cap == 0.0) {
                        string pin_name = getFullPinName(pins[view][input_j]);
                        auto pin_ = _ckt->_ord_design->getBlock()->findITerm(
                            pin_name.c_str());
                        sta::dbSta* sta = _ckt->_ord_timing->getSta();
                        sta::dbNetwork* network = sta->getDbNetwork();
                        sta::Port* port = network->dbToSta(pin_->getMTerm());
                        sta::LibertyPort* lib_port = network->libertyPort(port);
                        sta::LibertyLibrary* lib =
                            network->defaultLibertyLibrary();
                        pins[view][input_j].cap =
                            lib_port->capacitance() / cap_unit;
                        lib_cell_info->pins[pins[view][input_j].lib_pin]
                            .capacitance = pins[view][input_j].cap;
                    }
                    assert(pins[view][input_j].cap < 1e31);
                    // pin 作为输入引脚时的电容
                }
                // cout << "CAP CHECK: " << view << " " <<
                // getFullPinName(pins[view][cells[i].inpins[j]]) << " "  <<
                // pins[view][cells[i].inpins[j]].cap << endl;
            }
        }
        CalcCapViolation(view);
    }
}

double Sizer::CalcCapViolation(unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    unsigned mode = mmmcViewList[view].mode;
    double cap_viol = 0.;
    cap_violation_cnt = 0;
    cap_violation_wst = 0;
    ofstream ofs("our_cap_vio.txt");
    auto _corner = _ckt->_ord_timing->getCorners()[0];
    for(unsigned i = 0; i < numcells; i++) {
        LibCellInfo* lib_cell_info = getLibCellInfo(cells[i], corner);

        for(unsigned k = 0; k < cells[i].outpins.size(); ++k) {
            double maxCap = 0.0;
            if(lib_cell_info) {
                maxCap =
                    lib_cell_info->pins[pins[view][cells[i].outpins[k]].lib_pin]
                        .maxCapacitance;
                if(maxCap == std::numeric_limits< double >::max()) {
                    string pin_name =
                        getFullPinName(pins[view][cells[i].outpins[k]]);
                    auto pin_ = _ckt->_ord_design->getBlock()->findITerm(
                        pin_name.c_str());
                    double cap_limit =
                        _ckt->_ord_timing->getMaxCapLimit(pin_->getMTerm()) /
                        cap_unit;
                    lib_cell_info->pins[pins[view][cells[i].outpins[k]].lib_pin]
                        .maxCapacitance = cap_limit;
                    maxCap = cap_limit;
                }
            }
            if(use_margin) {
                maxCap *= cap_margin;
            }
            float loadCap = 0.;
            unsigned outnet = pins[view][cells[i].outpins[k]].net;
            if(nets[corner][outnet].is_clock) {
                continue;
            }
            for(unsigned j = 0; j < nets[corner][outnet].outpins.size(); j++) {
                loadCap += static_cast< float >(
                    pins[view][nets[corner][outnet].outpins[j]].cap);
                // string pin_name =
                //     getFullPinName(pins[view][nets[corner][outnet].outpins[j]]);
                // auto pin_ =
                //     _ckt->_ord_design->getBlock()->findITerm(pin_name.c_str());
                // pin_->getC
                assert(pins[view][nets[corner][outnet].outpins[j]].cap < 1e31);
                // if(loadCap > 2 * 1e31) {
                //     cout << "TOT CAP CHECK: " << view << " " << endl;
                // }
            }
            if(TEST_MODE == "ALL_TEST") {
                string pin_name =
                    getFullPinName(pins[view][cells[i].outpins[k]]);

                string netNameStr = nets[corner][outnet].name;
                auto ord_net =
                    _ckt->_ord_design->getBlock()->findNet(netNameStr.c_str());
                auto pin_ =
                    _ckt->_ord_design->getBlock()->findITerm(pin_name.c_str());
                sta::dbSta* sta = _ckt->_ord_timing->getSta();
                sta::Net* sta_net = sta->getDbNetwork()->dbToSta(ord_net);

                if(!(pin_->getNet() &&
                     pin_->getNet()->getSigType() != "POWER" &&
                     pin_->getNet()->getSigType() != "GROUND" &&
                     pin_->getNet()->getSigType() != "CLOCK")) {
                    printf("Error pin %s, Net %s, type %s, is vdd/clk/gnd\n",
                           pin_name.c_str(), pin_->getNet()->getName().c_str(),
                           pin_->getNet()->getSigType().getString());
                    continue;
                    // exit(0);
                }
                float pin_cap2;
                float wire_cap2;
                sta->connectedCap(sta_net, _corner, sta::MinMax::max(),
                                  pin_cap2, wire_cap2);
                wire_cap2 /= cap_unit;
                pin_cap2 /= cap_unit;
                double cap_limit =
                    _ckt->_ord_timing->getMaxCapLimit(pin_->getMTerm()) /
                    cap_unit;
                if(!isEqual(nets[corner][outnet].cap, wire_cap2)) {
                    // printf("Net %s, wire cap not equal %f, %f\n",
                    //        netNameStr.c_str(), nets[corner][outnet].cap,
                    //        wire_cap2);
                    // diff_cap++;
                }
                if(fabs(pin_cap2 - loadCap) > 0.05) {
                    // printf("Pin name %s, pin cap not equal %f, %f\n",
                    //        pin_name.c_str(), loadCap, pin_cap2);
                    // diff_cap++;
                }
                if(!isEqual(maxCap, cap_limit)) {
                    // printf("Pin name %s, CapLimit not equal %f, %f\n",
                    //        pin_name.c_str(), cap_limit, maxCap);
                    // diff_cap++;
                }
            }

            cap_viol += max(0.0, (nets[corner][outnet].cap + loadCap - maxCap));
            if(nets[corner][outnet].cap + loadCap > maxCap)
                cap_violation_cnt++;
            cap_violation_wst = max(
                cap_violation_wst, nets[corner][outnet].cap + loadCap - maxCap);
            if(cap_violation_wst > 2 * 1e31) {
                cout << "TOT CAP CHECK: " << view << " " << endl;
            }
            pins[view][cells[i].outpins[k]].totcap =
                nets[corner][outnet].cap + loadCap;
            if(nets[corner][outnet].cap + loadCap - maxCap > 0) {
                ofs << getFullPinName(pins[view][cells[i].outpins[k]])
                    << " max cap vio: "
                    << pins[view][cells[i].outpins[k]].totcap << " "
                    << nets[corner][outnet].cap << " "
                    << nets[corner][outnet].name << " " << maxCap << endl;
            }
        }
    }
    ofs.close();
#ifdef DRIVER_CELL
    for(unsigned i = 0; i < PIs.size(); i++) {
        LibCellInfo& driver =
            libs[corner][drivers[mode][PIs[i]]];  // drivers have a bug
        double maxCap =
            driver.pins[driver.lib_pin2id_map[driver.output]].maxCapacitance;
        unsigned outnet = pins[view][PIs[i]].net;
        double loadCap = 0.;
        for(unsigned j = 0; j < nets[corner][outnet].outpins.size(); j++)
            loadCap += pins[view][nets[corner][outnet].outpins[j]].cap;
        cap_viol += max(0.0, (nets[corner][outnet].cap + loadCap - maxCap));
        if(nets[corner][outnet].cap + loadCap > maxCap)
            cap_violation_cnt++;
        cap_violation_wst =
            max(cap_violation_wst, nets[corner][outnet].cap + loadCap - maxCap);
        pins[view][PIs[i]].totcap = nets[corner][outnet].cap + loadCap;
        if(cap_violation_wst > 2 * 1e31) {
            cout << "TOT CAP CHECK: " << view << " "
                 << getFullPinName(pins[view][PIs[i]]) << " "
                 << pins[view][PIs[i]].totcap << endl;
        }
        // cout << "TOT CAP CHECK: " << view << " " <<
        // getFullPinName(pins[view][PIs[i]]) << " "  <<
        // pins[view][PIs[i]].totcap << endl;
    }
#endif
    // has a bug cap_violation_wst
    printf("CAP VIOLATION cnt %d, cap_violation_wst %f\n", cap_violation_cnt,
           cap_violation_wst);
    return cap_viol;
}

double Sizer::CalcPower(unsigned thread, bool rpt_power, unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    double totPower = 0.;
    if(ALPHA == 0.0) {
        totPower = T[view]->getLeakPower();
    }
    else {
        totPower = T[view]->getTotPower();
    }
    return totPower;
}
