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

// ****************************************************************************
// analyzeTiming.cpp
//
// Make contact with PT/ETS/SOCE Tcl server, pass commands,
// get results and report
// ****************************************************************************

#include "analyze_timing.h"
#include <stdlib.h>
#include <sys/time.h>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include "MinMax.hh"
#include "ord/Timing.h"
#include "sta/Network.hh"
#include "sta/Sta.hh"
#include "utils.h"
#include "sizer.h"
#include <tcl8.6/tcl.h>
#include <tcl8.6/tclDecls.h>
#include "db_sta/dbNetwork.hh"
designTiming::designTiming() {
    program = PT;
    pt_time = 0.0;
}
designTiming::designTiming(ServerProg _program, Sizer *sizer) {
    _sizer = sizer;
    program = _program;
    pt_time = 0.0;
}
designTiming::~designTiming() {
}

void designTiming::testTCL() {
    _interpreter = Tcl_CreateInterp();
    _tclInputString = "puts \"TCL TEST";
    //_tclExpression = (char *)_tclInputString.c_str();
    Tcl_Eval(_interpreter, _tclExpression);
}

void designTiming::initializeServerContact(string clientName) {
    _interpreter = Tcl_CreateInterp();
    ifstream _clientTcl(clientName.c_str());
    if(!_clientTcl) {
        cerr << "Fatal error: cannot proceed without *client.tcl file" << endl;
        exit(1);
    }
    _tclInputString = "source " + clientName;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    Tcl_Eval(_interpreter, _tclExpression);

    _tclInputString = "InitClient";
    //_tclExpression = (char *)_tclInputString.c_str();
    cout << "InitClient" << endl;
    Tcl_Eval(_interpreter, _tclExpression);
    pt_time += cpuTime() - begin;
}

void designTiming::closeServerContact() {
    _tclInputString = "CloseClient";
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    Tcl_Eval(_interpreter, _tclExpression);
    Tcl_DeleteInterp(_interpreter);
    pt_time += cpuTime() - begin;
}

void designTiming::getCellDelay(double &rise_delay, double &fall_delay,
                                string cellInPin, string cellOutPin) {
    _tclInputString = "gate_delay " + cellInPin + " " + cellOutPin;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);

    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double temp1 = 0.0;
    double temp2 = 0.0;

    if(_tclAnswer != "") {
        // 方法1：使用更具体的格式匹配
        // char dummy1[20], dummy2[20];
        // sscanf(_tclAnswer.c_str(), "%s %f %s %f", dummy1, &temp2, dummy2,
        // &temp1);

        // // 或者方法2：查找并解析特定的键值对
        size_t fall_pos = _tclAnswer.find("fall_delay");
        size_t rise_pos = _tclAnswer.find("rise_delay");

        if(fall_pos != string::npos) {
            sscanf(_tclAnswer.c_str() + fall_pos, "fall_delay %f", &temp2);
        }
        if(rise_pos != string::npos) {
            sscanf(_tclAnswer.c_str() + rise_pos, "rise_delay %f", &temp1);
        }
    }
    rise_delay = temp1;
    fall_delay = temp2;

    // cout << delay << " " << riseFall << " " << endl;
}

// FIXME:
// void designTiming::getCellDelay(double &rise_delay, double &fall_delay,
//                                 string cellInPin, string cellOutPin) {
//     _tclInputString =
//         "report_dcalc -max -digits 3 -from " + cellInPin + " -to " +
//         cellOutPin;
//     // cout << _tclInputString << endl;
//     //_tclExpression = (char *)_tclInputString.c_str();
//     double begin = cpuTime();
//     _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
//     // report_dcalc -max -digits 3 -from $cellInPin -to $cellOutPin >
//     tmp3.rpt
//     // assert(0);
//     pt_time += cpuTime() - begin;
//     string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
//     float temp1 = 0;
//     float temp2 = 0;
//     std::cout << _tclAnswer << std::endl;
//     if(_tclAnswer == "") {
//         printf("Error: getCellDelay failed for %s %s\n", cellInPin.c_str(),
//                cellOutPin.c_str());
//         exit(0);
//     }
//     sscanf(_tclAnswer.c_str(), "%f %f", &temp1, &temp2);
// }

void designTiming::getFFDelay(double &rdelay, double &fdelay,
                              string cellOutPin) {
    _tclInputString = "PtGetFFDelay " + cellOutPin;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    printf("Error : don't have getFFDelay !\n");
    exit(0);
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    pt_time += cpuTime() - begin;
    float temp1;
    float temp2;
    sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
    rdelay = temp1;
    fdelay = temp2;
}

void designTiming::getNetDelay(double &delay, string sourcePinName,
                               string sinkPinName) {
    _tclInputString = "report_wire_delay " + sourcePinName + " " + sinkPinName;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    pt_time += cpuTime() - begin;
    float temp;
    sscanf(_tclAnswer.c_str(), "%f", &temp);
    delay = temp;
}

void designTiming::getInputSlew(double &riseSlew, double &fallSlew,
                                string pinName) {
    _tclInputString = "PtGetCellInputSlew " + pinName;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
    riseSlew = temp1;
    fallSlew = temp2;
}

string designTiming::_convertToString(double x) {
    std::ostringstream o;
    o << x;
    return o.str();
}

double designTiming::_convertToDouble(const string &s) {
    std::istringstream i(s);
    double x;
    i >> x;
    if(s == "INFINITY") {
        x = std::numeric_limits< double >::infinity();
    }
    return x;
}

int designTiming::_convertToInt(const string &s) {
    std::istringstream i(s);
    int x;
    i >> x;
    return x;
}

double designTiming::getWorstSlackHold(string _clkName) {
    if(program == PT) {
        _tclInputString = "PtWorstHoldSlack " + _clkName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWorstHoldSlack " + _clkName;
    }
    else if(program == OS) {
        double begin = cpuTime();
        _sizer->_ckt->_ord_design->evalTclString(
            "report_worst_slack -min > evaluation_temp.txt");
        ifstream in("evaluation_temp.txt");
        string t;
        double wns;
        in >> t >> wns;
        // wns /= _sizer->time_unit;
        pt_time += cpuTime() - begin;
        printf("Error: don't have OSWorstHoldSlack !\n");
        return wns;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _pathSlack = _convertToDouble(_answerStr);
    return (_pathSlack);
}

double designTiming::getWorstSlack(string _clkName) {
    if(program == PT) {
        _tclInputString = "PtWorstSlack " + _clkName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWorstSlack " + _clkName;
    }
    else if(program == OS) {
        double begin = cpuTime();
        _sizer->_ckt->_ord_design->evalTclString("report_wns > wns.txt");
        ifstream ifs("wns.txt");
        double wns;
        string _tclAnswer;
        string t2;
        ifs >> _tclAnswer >> t2 >> wns;
        ifs.close();
        wns = wns * 1e-9 / _sizer->time_unit;
        pt_time += cpuTime() - begin;
        return wns;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _pathSlack = _convertToDouble(_answerStr);
    _pathSlack = _pathSlack * 1e-12 / _sizer->time_unit;
    return (_pathSlack);
}

double designTiming::getTotPower() {
    if(program == PT) {
        _tclInputString = "PtTotalPower";
    }
    else if(program == ETS) {
        _tclInputString = "EtsTotalPower";
    }
    else {
        return getLeakPower();
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    // cout << _tclInputString << endl;
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    // cout << _tclAnswer << endl;
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _totPower = _convertToDouble(_answerStr);
    return (_totPower);
}

double designTiming::getCellPower(string cell_name) {
    if(program == PT) {
        _tclInputString = "PtTotalPower";
    }
    else if(program == ETS) {
        _tclInputString = "EtsTotalPower";
    }
    else {
        double begin = cpuTime();
        double totalLeakagePower = 0.0;
        auto inst =
            _sizer->_ckt->_ord_design->getBlock()->findInst(cell_name.c_str());
        auto corner = _sizer->_ckt->_ord_timing->getCorners()[0];
        totalLeakagePower +=
            _sizer->_ckt->_ord_timing->staticPower(inst, corner);
        totalLeakagePower /= _sizer->sw_adj;
        pt_time += cpuTime() - begin;
        return totalLeakagePower;
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    // cout << _tclInputString << endl;
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    // cout << _tclAnswer << endl;
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _totPower = _convertToDouble(_answerStr);
    return (_totPower);
}

double designTiming::getLeakPower() {
    if(program == PT) {
        _tclInputString = "PtLeakPower";
    }
    else if(program == ETS) {
        _tclInputString = "EtsLeakPower";
    }
    else {
        double begin = cpuTime();
        double totalLeakagePower = 0.0;
        auto corner = _sizer->_ckt->_ord_timing->getCorners()[0];
        // #progma omp parallel for
        // for(auto inst : _sizer->_ckt->_ord_design->getBlock()->getInsts()) {
        //     totalLeakagePower +=
        //         _sizer->_ckt->_ord_timing->staticPower(inst, corner);
        // }
        if(_sizer->cells == nullptr) {
            for(auto inst : _sizer->_ckt->_ord_design->getBlock()->getInsts()) {
                totalLeakagePower +=
                    _sizer->_ckt->_ord_timing->staticPower(inst, corner);
            }
        }
        else {
            for(int i = 0; i < _sizer->numcells; i++) {
                if(_sizer->cells[i].isStaticChanged) {
                    auto inst = _sizer->_ckt->_ord_design->getBlock()->findInst(
                        _sizer->cells[i].name.c_str());
                    _sizer->cells[i].static_power =
                        _sizer->_ckt->_ord_timing->staticPower(inst, corner);
                    _sizer->cells[i].isStaticChanged = false;
                }
                totalLeakagePower += _sizer->cells[i].static_power;
            }
        }
        totalLeakagePower /= _sizer->sw_adj;
        pt_time += cpuTime() - begin;
        return totalLeakagePower;
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    // cout << _tclInputString << endl;
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _leakPower = _convertToDouble(_answerStr);
    return (_leakPower);
}

double designTiming::getTNS(string _clkName) {
    if(program == PT) {
        _tclInputString = "PtGetTNS " + _clkName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetTNS " + _clkName;
    }
    else if(program == OS) {
        // _tclInputString = "OSGetTNS";
        double begin = cpuTime();
        _sizer->_ckt->_ord_design->evalTclString(
            "report_tns > evaluation_temp.txt");
        ifstream in("evaluation_temp.txt");
        string t;
        string t2;
        double tns;
        in >> t >> t2 >> tns;
        // tns /= _sizer->time_unit;
        pt_time += cpuTime() - begin;
        return tns;
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _pathSlack = _convertToDouble(_answerStr);
    _pathSlack = _pathSlack * 1e-12 / _sizer->time_unit;
    return (_pathSlack);
}

void designTiming::getTranVio(double &tot, double &max, int &num) {
    if(program == PT) {
        _tclInputString = "PtGetTranVio ";
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetTranVio ";
    }
    else if(program == OS) {
        _tclInputString = "OSGetTranVio ";
        double begin = cpuTime();
        auto design = _sizer->_ckt->_ord_design;
        ofstream ofs("opensta_tran_vio.txt");
        for(auto inst : design->getBlock()->getInsts()) {
            for(auto pin_ : inst->getITerms()) {
                if(pin_->getNet() && pin_->getNet()->getSigType() != "POWER" &&
                   pin_->getNet()->getSigType() != "GROUND" &&
                   pin_->getNet()->getSigType() != "CLOCK") {
                    auto m_term = pin_->getMTerm();
                    double r_slew = _sizer->_ckt->_ord_timing->getPinSlew(
                        pin_, ord::Timing::Rise);
                    double f_slew = _sizer->_ckt->_ord_timing->getPinSlew(
                        pin_, ord::Timing::Fall);
                    double now_slew = std::max(r_slew, f_slew);
                    double slew_limit =
                        _sizer->_ckt->_ord_timing->getMaxSlewLimit(m_term);
                    double slew_diff = std::max(
                        (now_slew - slew_limit) / _sizer->time_unit, 0.0);
                    tot += slew_diff;
                    if(slew_diff > 0) {
                        ofs << pin_->getName()
                            << " max tran vio: " << now_slew / _sizer->time_unit
                            << " " << slew_limit / _sizer->time_unit << endl;
                        num++;
                    }
                    max = std::max(max, slew_diff);
                }
            }
        }
        pt_time += cpuTime() - begin;
        ofs.close();
        return;
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);

    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    int temp3;
    sscanf(_tclAnswer.c_str(), "%f%f%d", &temp1, &temp2, &temp3);
    tot = temp1;
    max = temp2;
    num = temp3;
}

void designTiming::getCapVio(double &tot, double &max, int &num) {
    if(program == PT) {
        _tclInputString = "PtGetCapVio ";
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetCapVio ";
    }
    else if(program == OS) {
        _tclInputString = "OSGetCapVio ";
        double begin = cpuTime();
        auto design = _sizer->_ckt->_ord_design;
        ofstream ofs("opensta_cap_vio.txt");
        auto corner = _sizer->_ckt->_ord_timing->getCorners()[0];
        for(auto inst : design->getBlock()->getInsts()) {
            for(auto pin_ : inst->getITerms()) {
                if(pin_->getNet() && pin_->getNet()->getSigType() != "POWER" &&
                   pin_->getNet()->getSigType() != "GROUND" &&
                   pin_->getNet()->getSigType() != "CLOCK" &&
                   pin_->isOutputSignal()) {
                    auto m_term = pin_->getMTerm();
                    float pin_cap;
                    float wire_cap;
                    sta::dbSta *sta = _sizer->_ckt->_ord_timing->getSta();
                    sta::Net *sta_net =
                        sta->getDbNetwork()->dbToSta(pin_->getNet());
                    _sizer->_sta->connectedCap(
                        sta_net, corner, sta::MinMax::max(), pin_cap, wire_cap);
                    wire_cap /= _sizer->cap_unit;
                    pin_cap /= _sizer->cap_unit;
                    double now_cap = wire_cap + pin_cap;
                    double cap_limit =
                        _sizer->_ckt->_ord_timing->getMaxCapLimit(m_term) /
                        _sizer->cap_unit;
                    double cap_diff = std::max((now_cap - cap_limit), 0.0);
                    tot += cap_diff;
                    if(cap_diff > 0) {
                        ofs << pin_->getName() << " max cap vio: " << now_cap
                            << " " << wire_cap << " " << cap_limit << endl;
                        num++;
                    }
                    max = std::max(max, cap_diff);
                }
            }
        }
        pt_time += cpuTime() - begin;
        ofs.close();
        return;
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);

    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    int temp3;
    sscanf(_tclAnswer.c_str(), "%f%f%d", &temp1, &temp2, &temp3);
    tot = temp1;
    max = temp2;
    num = temp3;
}

double designTiming::getTNSHold(string _clkName) {
    if(program == PT) {
        _tclInputString = "PtGetTNSHold " + _clkName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetTNSHold " + _clkName;
    }
    else {
        printf("error: no getTNSHold !");
        exit(0);
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double _pathSlack = _convertToDouble(_answerStr);
    return (_pathSlack);
}

bool designTiming::sizeCell(string cellInstance, string cellMaster) {
    if(program == PT) {
        _tclInputString = "PtSizeCell " + cellInstance + " " + cellMaster;
    }
    else if(program == ETS) {
        _tclInputString = "EtsSizeCell " + cellInstance + " " + cellMaster;
    }
    else if(program == OS) {
        _tclInputString = "OSSizeCell " + cellInstance + " " + cellMaster;
        printf("Error: don't have OSSizeCell !\n");
        exit(0);
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    int _returnStatus = _convertToInt(_answerStr);
    if(_returnStatus == 0) {
        cerr << "Fatal error: size_cell failed; check the status on ptserver"
             << endl;
        return false;
    }
    return true;
}

bool designTiming::writeECOChange(string filename) {
    if(program == PT) {
        _tclInputString = "write_change -format ptsh -output  " + filename;
    }
    else if(program == ETS) {
        _tclInputString = "write_eco -format ets -output " + filename;
    }
    else {
        printf("error: no writeECOChange !");
        exit(0);
    }

    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

double designTiming::getCellSlack(string CellName) {
    if(CellName == "")
        return ((double)0);
    else {
        if(program == PT || program == ETS) {
            _tclInputString = "PtCellSlack " + CellName;
        }
        else {
            printf("error: no getCellSlack !");
            exit(0);
        }
        //_tclExpression = (char *)_tclInputString.c_str();
        double begin = cpuTime();
        _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
        pt_time += cpuTime() - begin;

        string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
        string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
        double _setupSlack = _convertToDouble(_answerStr);
        return (_setupSlack);
    }
}

bool designTiming::loadDesign(string benchname) {
    _tclInputString =
        "redirect pt.loadDesign.log {source pt." + benchname + ".tcl}";
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

// bool designTiming::updateSize(string filename) {
//     //_tclInputString = "redirect pt.updateSize.log {source " +
//     // filename+"}";

//     _sizer->_sta->networkChanged1();  // FIXME: This has a bug
//     _tclInputString = "source " + filename;
//     // cout << _tclInputString << endl;
//     //_tclExpression = (char *)_tclInputString.c_str();
//     double begin = cpuTime();
//     _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
//     pt_time += cpuTime() - begin;
//     string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
//     string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

//     if(_answerStr == "1")
//         return true;
//     else
//         return false;
// }

bool designTiming::checkSize(string filename) {
    if(program == PT) {
        _tclInputString = "PtGetCurSize " + filename;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetCurSize " + filename;
    }
    else if(program == OS) {
        _tclInputString = "OSGetCurSize " + filename;
        printf("Error: don't have OSGetCurSize !\n");
        exit(0);
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::checkServer() {
    _tclInputString = "checkServer";
    // cout << "checkServer\" " << this << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

void designTiming::getPinSlack(double &riseSlack, double &fallSlack,
                               string pinName) {
    if(program == PT) {
        _tclInputString = "PtGetPinSlack " + pinName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetPinSlack " + pinName;
    }
    else if(program == OS) {
        double begin = cpuTime();
        auto pin_ =
            _sizer->_ckt->_ord_design->getBlock()->findITerm2(pinName.c_str());
        sta::dbSta *sta = _sizer->_ckt->_ord_timing->getSta();
        sta::dbNetwork *network = sta->getDbNetwork();
        // sta::Port *port = network->dbToSta(pin_->getMTerm());
        double r_att =
            _sizer->_ckt->_ord_timing->getPinSlack(pin_, ord::Timing::Rise);
        double f_att =
            _sizer->_ckt->_ord_timing->getPinSlack(pin_, ord::Timing::Fall);
        pt_time += cpuTime() - begin;

        riseSlack = r_att / _sizer->time_unit;
        fallSlack = f_att / _sizer->time_unit;

        pt_time += cpuTime() - begin;
        cout << "Slack " << pinName << " " << riseSlack << " " << fallSlack
             << endl;
        return;
    }
    //_tclExpression = (char *)_tclInputString.c_str();

    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
    if(temp1 >= 100000000000.0 - 1e-5) {
        temp1 = std::numeric_limits< float >::infinity();
    }
    if(temp2 >= 100000000000.0 - 1e-5) {
        temp2 = std::numeric_limits< float >::infinity();
    }
    riseSlack = temp1;
    fallSlack = temp2;
}

void designTiming::getPinMinSlack(double &riseSlack, double &fallSlack,
                                  string pinName) {
    if(program == PT) {
        _tclInputString = "PtGetPinMinSlack " + pinName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetPinMinSlack " + pinName;
    }
    else {
        printf("error: no getPinMinSlack !");
        exit(0);
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();

    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp1;
    float temp2;
    sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
    riseSlack = temp1;
    fallSlack = temp2;
    // cout << pinName << " " << riseSlack << " " << fallSlack << endl;
}

void designTiming::getPinTran(double &riseTran, double &fallTran,
                              string pinName) {
    _tclInputString = "OSGetPinTran " + pinName;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    if(VERBOSE > 3) {
        printf("Error: not OSGetPinTran !, pin name %s\n", pinName.c_str());
    }
    auto pin_ =
        _sizer->_ckt->_ord_design->getBlock()->findITerm2(pinName.c_str());
    sta::dbSta *sta = _sizer->_ckt->_ord_timing->getSta();
    sta::dbNetwork *network = sta->getDbNetwork();
    // sta::Port *port = network->dbToSta(pin_->getMTerm());
    double r_slew =
        _sizer->_ckt->_ord_timing->getPinSlew(pin_, ord::Timing::Rise);
    double f_slew =
        _sizer->_ckt->_ord_timing->getPinSlew(pin_, ord::Timing::Fall);
    pt_time += cpuTime() - begin;

    riseTran = r_slew / _sizer->time_unit;
    fallTran = f_slew / _sizer->time_unit;
    cout << pinName << " " << riseTran << " " << fallTran << endl;
}

// void designTiming::getPinCap(double &riseTran, double &fallTran,
//                               string pinName) {
//     _tclInputString = "OSGetPinTran " + pinName;
//     // cout << _tclInputString << endl;
//     //_tclExpression = (char *)_tclInputString.c_str();
//     double begin = cpuTime();
//     _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
//     pt_time += cpuTime() - begin;

//     string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
//     float temp1;
//     float temp2;
//     sscanf(_tclAnswer.c_str(), "%f%f", &temp1, &temp2);
//     riseTran = temp1;
//     fallTran = temp2;
//     // cout << pinName << " " << riseSlack << " " << fallSlack << endl;
// }

bool designTiming::writePinSlack(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinSlack " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinSlack " + infile + " " + outfile;
    }
    else if(program == OS) {
        _tclInputString = "OSWritePinSlack " + infile + " " + outfile;
        printf("Error: don't have OSWritePinSlack !\n");
        exit(0);
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::writePinMinSlack(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinMinSlack " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinMinSlack " + infile + " " + outfile;
    }
    else {
        printf("error: no writePinMinSlack !");
        // exit(0);
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::writeMaxTranConst(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinMaxTranConst " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinMaxTranConst " + infile + " " + outfile;
    }
    else {
        _tclInputString = "OSWritePinMaxTranConst " + infile + " " + outfile;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

// JLPWR
bool designTiming::writePinToggleRate(string infile, string outfile,
                                      string clock_period) {
    if(program == PT) {
        _tclInputString = "PtWritePinToggleRate " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinToggleRateWithClock " + infile + " " +
                          outfile + " " + clock_period;
    }
    else if(program == OS) {
        _tclInputString = "OSWritePinToggleRate " + infile + " " + outfile;
        printf("Error: don't have OSWritePinToggleRate !\n");
        // exit(0);
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::writePinToggleRate(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinToggleRate " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinToggleRate " + infile + " " + outfile;
    }
    else if(program == OS) {
        _tclInputString = "OSWritePinToggleRate " + infile + " " + outfile;
        printf("Error: don't have OSWritePinToggleRate !\n");
        // exit(0);
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

// JLPWR
void designTiming::getPinToggleRate(double &toggleRate, string pinName) {
    if(program == PT || program == ETS) {
        _tclInputString = "PtGetPinToggleRate " + pinName;
    }
    else {
        _tclInputString = "OSGetPinToggleRate " + pinName;
        toggleRate = 0;
        return;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();

    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    float temp;
    sscanf(_tclAnswer.c_str(), "%f", &temp);
    toggleRate = temp;
}

bool designTiming::writePinTran(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinTran " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "PtWritePinTran " + infile + " " + outfile;
    }
    else {
        printf("error: no writePinTran !");
        // exit(0);
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::writePinAll(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinAll " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinAll " + infile + " " + outfile;
    }
    else if(program == OS) {
        _tclInputString = "OSWritePinAll " + infile + " " + outfile;
        printf("Error: don't have OSWritePinAll !\n");
        // exit(0);
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    if(_answerStr == "1")
        return true;
    else
        return false;
}

// FIXEME:
void designTiming::getPinArrival(double &riseArrival, double &fallArrival,
                                 string pinName) {
    _tclInputString = "OSGetPinArrival " + pinName;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    printf("Error: not OSGetPinArrival !, pin name %s\n", pinName.c_str());
    double begin = cpuTime();
    auto pin_ =
        _sizer->_ckt->_ord_design->getBlock()->findITerm2(pinName.c_str());
    sta::dbSta *sta = _sizer->_ckt->_ord_timing->getSta();
    sta::dbNetwork *network = sta->getDbNetwork();
    // assert(0);
    // sta::Port *port = network->dbToSta(pin_->getMTerm());
    double r_att =
        _sizer->_ckt->_ord_timing->getPinArrival(pin_, ord::Timing::Rise);
    double f_att =
        _sizer->_ckt->_ord_timing->getPinArrival(pin_, ord::Timing::Fall);
    pt_time += cpuTime() - begin;

    riseArrival = r_att / _sizer->time_unit;
    fallArrival = f_att / _sizer->time_unit;

    pt_time += cpuTime() - begin;
    cout << "Arrival " << pinName << " " << riseArrival << " " << fallArrival
         << endl;
}

double designTiming::getRiseSlack(string PinName) {
    if(program == PT || program == ETS) {
        _tclInputString = "PtGetRiseSlack " + PinName;
    }
    else {
        _tclInputString = "OSGetRiseSlack " + PinName;
    }
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double slack = _convertToDouble(_answerStr);
    return slack;
}

double designTiming::getFallSlack(string PinName) {
    if(program == PT || program == ETS) {
        _tclInputString = "PtGetFallSlack " + PinName;
    }
    else {
        _tclInputString = "OSGetFallSlack " + PinName;
    }

    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double slack = _convertToDouble(_answerStr);
    return slack;
}

double designTiming::getRiseTran(string PinName) {
    if(program == PT) {
        _tclInputString = "PtGetRiseTran " + PinName;
    }
    else {
        _tclInputString = "OSGetRiseTran " + PinName;
    }
    // printf("Error: not OSGetRiseTran !, pin name %s\n", PinName.c_str());
    // exit(0);
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double tran = _convertToDouble(_answerStr);
    return tran;
}

double designTiming::getFallTran(string PinName) {
    if(program == PT) {
        _tclInputString = "PtGetFallTran " + PinName;
    }
    else {
        _tclInputString = "OSGetFallTran " + PinName;
    }

    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double tran = _convertToDouble(_answerStr);
    return tran;
}

double designTiming::getRiseArrival(string PinName) {
    _tclInputString = "PtGetRiseArrival " + PinName;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double val = _convertToDouble(_answerStr);
    return val;
}

double designTiming::getFallArrival(string PinName) {
    _tclInputString = "PtGetFallArrival " + PinName;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double val = _convertToDouble(_answerStr);
    return val;
}

double designTiming::getCeff(string PinName) {
    if(program == PT) {
        _tclInputString = "PtGetCeff " + PinName;
    }
    else if(program == ETS) {
        _tclInputString = "EtsGetCeff " + PinName;
    }
    else {
        auto design = _sizer->_ckt->_ord_design;
        // ofstream ofs("opensta_cap_vio.txt");
        auto corner = _sizer->_ckt->_ord_timing->getCorners()[0];
        auto pin_ = design->getBlock()->findITerm2(PinName.c_str());
        if(!pin_) {
            printf("error: no getCeff !\n");
            return 0;
        }
        auto m_term = pin_->getMTerm();
        float pin_cap;
        float wire_cap;
        sta::dbSta *sta = _sizer->_ckt->_ord_timing->getSta();
        sta::Net *sta_net = sta->getDbNetwork()->dbToSta(pin_->getNet());
        _sizer->_sta->connectedCap(sta_net, corner, sta::MinMax::max(), pin_cap,
                                   wire_cap);
        wire_cap /= _sizer->cap_unit;
        pin_cap /= _sizer->cap_unit;
        double now_cap = wire_cap + pin_cap;
        return now_cap;
        printf("error: no getCeff !");
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    double val = _convertToDouble(_answerStr);
    return val;
}

bool designTiming::writePinCeff(string infile, string outfile) {
    if(program == PT) {
        _tclInputString = "PtWritePinCeff " + infile + " " + outfile;
    }
    else if(program == ETS) {
        _tclInputString = "EtsWritePinCeff " + infile + " " + outfile;
    }
    else {
        printf("error: no writePinCeff !");
        exit(0);
    }
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

bool designTiming::runECO(unsigned mode) {
    if(program != PT)
        return false;

    string cmd = "";
    if(mode == 0) {
        cmd = "fix_eco_timing -type setup -methods size_cell";
    }
    else if(mode == 1) {
        cmd = "fix_eco_drc -type max_transition -methods size_cell";
    }
    else if(mode == 2) {
        cmd = "fix_eco_drc -type max_capacitance -methods size_cell";
    }

    _tclInputString = "" + cmd;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;
    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));

    if(_answerStr == "1")
        return true;
    else
        return false;
}

string designTiming::getLibCell(string CellName) {
    _tclInputString = "PtGetLibCell " + CellName;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    return (_answerStr);
}

void designTiming::Exit() {
    _tclInputString = "exitServer";
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
}

string designTiming::doOneCmd(string command) {
    _tclInputString = "" + command;
    // cout << _tclInputString << endl;
    //_tclExpression = (char *)_tclInputString.c_str();
    double begin = cpuTime();
    _sizer->_ckt->_ord_design->evalTclString(_tclInputString);
    pt_time += cpuTime() - begin;

    string _tclAnswer(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    string _answerStr(Tcl_GetStringResult(sta::Sta::sta()->tclInterp()));
    return _answerStr;
}
