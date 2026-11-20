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
// ****************************************************************************
// analyzeTiming.h
// ****************************************************************************
// ****************************************************************************

#ifndef __DESIGN_TIMING__
#define __DESIGN_TIMING__

#include <array>
#include <iostream>
#include <string>
#include <vector>
#include "tcl.h"
#include "sta/ArcDelayCalc.hh"
#include "sta/Bfs.hh"
#include "sta/FuncExpr.hh"
#include "sta/Fuzzy.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/InputDrive.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/Parasitics.hh"
#include "sta/PortDirection.hh"
#include "sta/Sdc.hh"
#include "sta/Scene.hh"
#include "sta/Search.hh"
#include "sta/StaMain.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingModel.hh"
#include "sta/Units.hh"
class Sizer;

using namespace std;

namespace sta { class dbSta; }
namespace ecc {
enum class PinTimingMetric { Slack, Slew, Arrival };
// Values are in STA SI units. Lookup accepts both instance pins and ports.
const sta::Pin* findTimingPin(sta::dbSta* sta, const std::string& name);
std::array<double, 2> pinTiming(sta::dbSta* sta, const sta::Pin* pin,
                               PinTimingMetric metric, const sta::MinMax* min_max);
std::array<double, 2> timingArcDelays(sta::dbSta* sta, const sta::Pin* from,
                                     const sta::Pin* to, bool wire);
}

class designTiming {
   public:
    explicit designTiming(Sizer* sizer);
    double pt_time = 0;
    double getWorstSlack(string clockName);
    double getWorstSlackHold(string clockName);
    double getTNS(string clockName = "");
    double getTNSHold(string clockName = "");
    void getTranVio(double &tot, double &max, int &num);
    bool sizeCell(string cellInstance, string cellMaster);
    double getRiseSlack(string PinName);
    double getFallSlack(string PinName);
    double getRiseTran(string PinName);
    double getFallTran(string PinName);
    double getRiseArrival(string PinName);
    double getFallArrival(string PinName);
    void getNetDelay(double &delay, string sourcePinName, string sinkPinName);
    void getCellDelay(double &rise_delay, double &fall_delay, string cellInPin,
                      string cellOutPin);
    void getPinSlack(double &riseSlack, double &fallSlack, string pinName);
    void getPinMinSlack(double &riseSlack, double &fallSlack, string pinName);
    void getPinTran(double &riseTran, double &fallTran, string pinName);
    void getPinArrival(double &riseArrival, double &fallArrival,
                       string pinName);
    double getCellPower(string cell_name);
    double getCeff(string PinName);
    string getLibCell(string CellName);
    bool writePinSlack(string infile, string outfile);
    bool writePinTran(string infile, string outfile);
    bool writePinAll(string infile, string outfile);

    double getLeakPower();
    double getTotPower();
    void getCapVio(double &tot, double &max, int &num);

   private:
    enum class PinReport { Slack, Slew, All };
    bool writePins(const string& input, const string& output, PinReport report);
    void pinValues(double& rise, double& fall, const string& name,
                   ecc::PinTimingMetric metric, const sta::MinMax* min_max);
    Sizer* _sizer;
};

#endif
