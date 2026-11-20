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

#ifndef __CKT_H__
#define __CKT_H__

#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include "db_sta/dbSta.hh"
#include "sta/Sta.hh"
// #include "cell_sizing.h"
#include "tcl.h"
#include "utils.h"
#include <queue>
#include <pthread.h>
#include <signal.h>
#include <sys/param.h>
#include <sys/time.h>
#include <sys/utsname.h>
#include <unistd.h>
#include <cassert>
#include <cctype>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <set>
#include <stack>
#include <string>
#include <utility>
#include <vector>
#include "ord/Timing.h"
#include "ord/Tech.h"
#include "ord/Design.h"

// Parasitic refresh is supplied by the application session.
#define NUM_VTS 3


// Swig uses C linkage for init functions.
extern "C" {
extern int Sta_Init(Tcl_Interp* interp);
}

// to import TCL functions
namespace sta {
extern const char* tcl_inits[];

}

// typedef unsigned cell_vtypes;
using namespace std;

struct LibCellInfo;
struct entry;
// Outer dimension is size; inner dimension is Vt.
struct LibCellTable {
    string name;
    vector< vector< LibCellInfo* > > lib_vt_size_table;
};

struct CellSol {
    int c_vtype;
    int c_size;

    CellSol() : c_vtype(-1), c_size(-1) {
    }
    CellSol(const CellSol& orig) : c_vtype(orig.c_vtype), c_size(orig.c_size) {
    }

    const CellSol& operator=(const CellSol& assign) {
        c_vtype = assign.c_vtype;
        c_size = assign.c_size;
        return *this;
    }
};

struct SUB_NODE {
    string pin_name;
    unsigned id = 0;
    bool isSink : 1 = false;
    bool visited : 1 = false;
    bool is_branch : 1 = true;
    double cap = 0;  // Wire capacitance; the timer adds input capacitance.
    int pinId = -1;
    double totres = 0;
    vector< unsigned > adj;
    unsigned fanin = 0;
    vector< unsigned > fanouts;
    vector< double > res;  // Resistance to each adjacent node.
    double delay = 0;
    double m1 = 0;  // Elmore delay moment.
    double m2 = 0;
};

struct NET {
    string name;
    unsigned inpin = UINT_MAX;
    vector< unsigned > outpins;
    double cap = 0;
    bool is_clock = false;
    // Driver at index zero, then sinks and internal parasitic nodes.
    vector< SUB_NODE > subNodeVec;
    vector< vector< double > > subNodeResVec;
};

struct PIN {
    unsigned id;
    unsigned net;
    unsigned lib_pin;
    string name;
    bool waiveTran = false;
    bool waiveSlack;
    double cap, totcap, ceff;
    double max_tran;
    double rtran, ftran;
    double rslk, fslk;
    double hold_rslk, hold_fslk;
    double rAAT, fAAT;
    double rRAT, fRAT;
    vector< double > rdelay, fdelay;
    double rtran_ofs, ftran_ofs;
    double rslk_ofs, fslk_ofs;
    double slk_gb;
    double rAAT_ofs, fAAT_ofs;
    double rRAT_ofs, fRAT_ofs;
    ulong NPaths;
    ulong NPathsLessThanSlack;
    ulong NCellPaths;
    unsigned owner;
    unsigned spef_pin;
    bool isPI;
    bool isPO;
    bool bb_checked_tran;
    bool bb_checked_rat;
    bool bb_checked_aat;
    vector< bool > bb_checked_delay;

    double sw_coef;
    double toggle_rate;

    double m1, m2;

    PIN()
        : id(UINT_MAX),
          net(UINT_MAX),
          lib_pin(UINT_MAX),
          name(""),
          cap(0.0),
          sw_coef(0.0),
          toggle_rate(0.0),
          totcap(0.0),
          ceff(0.0),
          max_tran(0.0),
          rtran(0.0),
          ftran(0.0),
          rslk(0.0),
          fslk(0.0),
          hold_rslk(0.0),
          hold_fslk(0.0),
          rAAT(0.0),
          fAAT(0.0),
          rRAT(9999.99),
          fRAT(9999.99),
          rdelay(vector< double >()),
          fdelay(vector< double >()),
          rtran_ofs(0.0),
          ftran_ofs(0.0),
          rslk_ofs(0.0),
          fslk_ofs(0.0),
          slk_gb(0.0),
          rAAT_ofs(0.0),
          fAAT_ofs(0.0),
          rRAT_ofs(0.0),
          fRAT_ofs(0.0),
          NPaths(1),
          NPathsLessThanSlack(1),
          NCellPaths(1),
          owner(UINT_MAX),
          spef_pin(UINT_MAX),
          isPI(false),
          isPO(false),
          bb_checked_tran(false),
          bb_checked_rat(false),
          bb_checked_aat(false),
          bb_checked_delay(vector< bool >()),
          m1(0.0),
          m2(0.0) {
    }

    PIN(const PIN& orig)
        : id(orig.id),
          net(orig.net),
          lib_pin(orig.lib_pin),
          name(orig.name),
          cap(orig.cap),
          totcap(orig.totcap),
          ceff(orig.ceff),
          sw_coef(orig.sw_coef),
          toggle_rate(orig.toggle_rate),
          max_tran(orig.max_tran),
          rtran(orig.rtran),
          ftran(orig.ftran),
          rslk(orig.rslk),
          fslk(orig.fslk),
          hold_rslk(orig.hold_rslk),
          hold_fslk(orig.hold_fslk),
          rAAT(orig.rAAT),
          fAAT(orig.fAAT),
          rRAT(orig.rRAT),
          fRAT(orig.rRAT),
          rdelay(orig.rdelay),
          fdelay(orig.fdelay),
          rtran_ofs(orig.rtran_ofs),
          ftran_ofs(orig.ftran_ofs),
          rslk_ofs(orig.rslk_ofs),
          fslk_ofs(orig.fslk_ofs),
          slk_gb(orig.slk_gb),
          rAAT_ofs(orig.rAAT_ofs),
          fAAT_ofs(orig.fAAT_ofs),
          rRAT_ofs(orig.rRAT_ofs),
          fRAT_ofs(orig.fRAT_ofs),
          NPaths(orig.NPaths),
          NPathsLessThanSlack(orig.NPathsLessThanSlack),
          NCellPaths(orig.NCellPaths),
          owner(orig.owner),
          spef_pin(orig.spef_pin),
          isPI(orig.isPI),
          isPO(orig.isPO),
          bb_checked_tran(orig.bb_checked_tran),
          bb_checked_rat(orig.bb_checked_rat),
          bb_checked_aat(orig.bb_checked_aat),
          bb_checked_delay(orig.bb_checked_delay),
          m1(orig.m1),
          m2(orig.m2) {
    }
    const PIN& operator=(const PIN& assign) {
        id = assign.id;
        net = assign.net;
        lib_pin = assign.lib_pin;
        name = assign.name;
        cap = assign.cap;
        totcap = assign.totcap;
        ceff = assign.ceff;
        sw_coef = assign.sw_coef;
        toggle_rate = assign.toggle_rate;
        max_tran = assign.max_tran;
        rtran = assign.rtran;
        ftran = assign.ftran;
        rslk = assign.rslk;
        fslk = assign.fslk;
        hold_rslk = assign.hold_rslk;
        hold_fslk = assign.hold_fslk;
        rAAT = assign.rAAT;
        fAAT = assign.fAAT;
        rRAT = assign.rRAT;
        fRAT = assign.rRAT;
        rdelay = assign.rdelay;
        fdelay = assign.fdelay;
        NPaths = assign.NPaths;
        NPathsLessThanSlack = assign.NPathsLessThanSlack;
        NCellPaths = assign.NCellPaths;
        owner = assign.owner;
        rslk_ofs = assign.rslk_ofs;
        fslk_ofs = assign.fslk_ofs;
        slk_gb = assign.slk_gb;
        rtran_ofs = assign.rtran_ofs;
        ftran_ofs = assign.ftran_ofs;
        rAAT_ofs = assign.rAAT_ofs;
        fAAT_ofs = assign.fAAT_ofs;
        rRAT_ofs = assign.rRAT_ofs;
        fRAT_ofs = assign.fRAT_ofs;
        spef_pin = assign.spef_pin;
        isPI = assign.isPI;
        isPO = assign.isPO;
        bb_checked_tran = assign.bb_checked_tran;
        bb_checked_rat = assign.bb_checked_rat;
        bb_checked_aat = assign.bb_checked_aat;
        bb_checked_delay = assign.bb_checked_delay;
        m1 = assign.m1;
        m2 = assign.m2;
        return *this;
    }
};

struct CELL {
    string name;
    string type;
    bool isFF : 1;
    bool isClockCell : 1;
    bool isDontTouch : 1;
    int isChanged = 1;
    bool isStaticChanged = true;
    unsigned depth;
    vector< double > max_tran;
    bool touched;
    bool downsized;
    double static_power;

    int main_lib_cell_id;
    cell_vtypes c_vtype;
    int c_size;
    unsigned critical;
    map< string, unsigned > pinchar;
    vector< unsigned > inpins;
    vector< unsigned > outpins;
    vector< unsigned > FOtoPO;
    vector< unsigned > FIfromPI;
    unsigned outpin;

    unsigned clock_pin;
    unsigned data_pin;

    vector< unsigned > fis;
    vector< unsigned > fos;

    list< unsigned > tabu;

    CELL()
        : name(""),
          type(""),
          isFF(false),
          isClockCell(false),
          isDontTouch(false),
          isChanged(1),
          main_lib_cell_id(-1),
          static_power(0.0),
          depth(1),
          max_tran(vector< double >()),
          touched(false),
          downsized(false),
          c_vtype(default_vtype),
          c_size(0),
          critical(0),
          inpins(vector< unsigned >()),
          outpins(vector< unsigned >()),
          outpin(UINT_MAX),
          clock_pin(UINT_MAX),
          data_pin(UINT_MAX),
          fis(vector< unsigned >()),
          tabu(list< unsigned >()),
          fos(vector< unsigned >()),
          FOtoPO(vector< unsigned >()),
          FIfromPI(vector< unsigned >()) {
    }

    CELL(const CELL& orig)
        : name(orig.name),
          type(orig.type),
          isFF(orig.isFF),
          isClockCell(orig.isClockCell),
          isDontTouch(orig.isDontTouch),
          isChanged(orig.isChanged),
          depth(orig.depth),
          max_tran(orig.max_tran),
          touched(orig.touched),
          downsized(orig.downsized),
          main_lib_cell_id(orig.main_lib_cell_id),
          c_vtype(orig.c_vtype),
          c_size(orig.c_size),
          pinchar(orig.pinchar),
          inpins(orig.inpins),
          outpins(orig.outpins),
          outpin(orig.outpin),
          clock_pin(orig.clock_pin),
          data_pin(orig.data_pin),
          static_power(orig.static_power),
          isStaticChanged(orig.isStaticChanged),
          fis(orig.fis),
          fos(orig.fos),
          tabu(orig.tabu),
          FOtoPO(orig.FOtoPO),
          FIfromPI(orig.FIfromPI) {
    }
    const CELL& operator=(const CELL& assign) {
        name = assign.name;
        type = assign.type;
        isFF = assign.isFF;
        depth = assign.depth;
        max_tran = assign.max_tran;
        touched = assign.touched;
        static_power = assign.static_power;
        downsized = assign.downsized;
        isClockCell = assign.isClockCell;
        isDontTouch = assign.isDontTouch;
        isChanged = assign.isChanged;
        isStaticChanged = assign.isStaticChanged,
        main_lib_cell_id = assign.main_lib_cell_id;
        c_vtype = assign.c_vtype;
        c_size = assign.c_size;
        pinchar = assign.pinchar;
        inpins = assign.inpins;
        outpins = assign.outpins;
        outpin = assign.outpin;
        clock_pin = assign.clock_pin;
        data_pin = assign.data_pin;
        fis = assign.fis;
        fos = assign.fos;
        FOtoPO = assign.FOtoPO;
        FIfromPI = assign.FIfromPI;
        tabu = assign.tabu;
        return *this;
    }
};

struct LibLUT {
    // Look up table is indexed by the output load and the input transition
    // values
    // Example:
    //   Let L = loadIndices[i]
    //       T = transitionIndices[j]
    //   Then, the table value corresponding to L and T will be:
    //       table[i][j]
    //
    // Always: index1 = load; index2 = tran
    string templ;
    vector< double > loadIndices;
    vector< double > transitionIndices;
    vector< vector< double > > tableVals;
};

ostream& operator<<(ostream& os, const LibLUT& lut);

struct LibTimingInfo {
    bool isFunc;
    unsigned cnt0;  // fall delay
    unsigned cnt1;  // rise delay
    unsigned cnt2;  // fall tran
    unsigned cnt3;  // rise tran
    string fromPin;
    string toPin;
    char timingSense;  // "non_unate" or "negative_unate" or "positive_unate".
    // Note that ISPD-12 library will have only negative-unate combinational
    // cells. The clock arcs
    // for sequentials will be non_unate (which can be ignored because of the
    // simplified sequential
    // timing model for ISPD-12).

    LibLUT fallDelay;
    LibLUT riseDelay;
    LibLUT fallTransition;
    LibLUT riseTransition;
    LibTimingInfo() : fromPin(""), toPin(""), timingSense('-') {
    }
};

struct LibPowerInfo {
    string toPin;
    string relatedPin;
    vector< string > pgPins;
    bool isFunc;
    int cnt;

    LibLUT fallPower;
    LibLUT risePower;
};

ostream& operator<<(ostream& os, const LibTimingInfo& timing);
ostream& operator<<(ostream& os, const LibPowerInfo& power);

struct LibPinInfo {
    string name;            // pin name
    double capacitance;     // input pin cap (not defined for output pins)
    double maxCapacitance;  // the max load this pin can drive
    double maxTran;         // the max transition this pin can drive
    bool isInput : 1;       // whether the pin is input pin
    bool isOutput : 1;      // whether the pin is output pin
    bool isClock : 1;       // whether the pin is a clock pin or not
    bool isData : 1;        // whether the pin is a data pin or not
    int lib_pin_id : 14;
    bool IQN : 1;

    LibPinInfo()
        : capacitance(0.0),
          maxCapacitance(std::numeric_limits< double >::max()),
          maxTran(DBL_MAX),
          isInput(true),
          isOutput(false),
          isClock(false),
          isData(false),
          lib_pin_id(-1),
          IQN(false) {
    }

    LibPinInfo(const LibPinInfo& orig)
        : name(orig.name),
          capacitance(orig.capacitance),
          maxCapacitance(orig.maxCapacitance),
          maxTran(orig.maxTran),
          IQN(orig.IQN),
          isInput(orig.isInput),
          isOutput(orig.isOutput),
          isClock(orig.isClock),
          isData(orig.isData),
          lib_pin_id(orig.lib_pin_id) {
    }

    const LibPinInfo& operator=(const LibPinInfo& assign) {
        name = assign.name;
        capacitance = assign.capacitance;
        maxCapacitance = assign.maxCapacitance;
        maxTran = assign.maxTran;
        isInput = assign.isInput;
        isOutput = assign.isOutput;
        isClock = assign.isClock;
        isData = assign.isData;
        lib_pin_id = assign.lib_pin_id;
        IQN = assign.IQN;
        return *this;
    }
};

ostream& operator<<(ostream& os, const LibPinInfo& pin);

struct LibCellInfo {
    string name;       // cell name
    string libname;    // library name Jiajia added 1/9/2014
    string footprint;  // only the cells with the same footprint are swappable
    cell_vtypes c_vtype;    // vt
    cell_sizes c_size;      // size
    int cap_size;           // capacitance size
    double leakagePower;    // cell leakage power
    double area;            // cell area (will not be a metric for ISPD-12)
    double width;           // cell width for minIA
    bool isSequential : 1;  // if true then sequential cell, else combinational
    bool dontTouch : 1;     // is the sizer allowed to size this cell?
    bool dontUse : 1;       // is the sizer allowed to use this cell?
    bool hasQN : 1;         // has QN pin
    double le_fall;
    double le_rise;
    double Ron;
    double max_tran;
    double partial_order;
    string output;

    map< string, unsigned > lib_pin2id_map;

    map< unsigned, LibPinInfo > pins;  // pin name->LibPinInfo
    map< unsigned, LibTimingInfo > timingArcs;
    map< unsigned, LibPowerInfo > powerTables;

    LibCellInfo()
        : c_vtype(s),
          leakagePower(0.0),
          area(0.0),
          width(0.0),
          isSequential(false),
          c_size(0),
          cap_size(0),
          hasQN(false),
          dontTouch(false),
          dontUse(false),
          le_fall(0.0),
          le_rise(0.0),
          Ron(0.0),
          max_tran(DBL_MAX),
          partial_order(0.0) {
    }

    LibCellInfo(const LibCellInfo& orig)
        : name(orig.name),
          footprint(orig.footprint),
          c_vtype(orig.c_vtype),
          leakagePower(orig.leakagePower),
          area(orig.area),
          max_tran(orig.max_tran),
          partial_order(orig.partial_order),
          width(orig.width),
          c_size(orig.c_size),
          cap_size(orig.cap_size),
          hasQN(orig.hasQN),
          isSequential(orig.isSequential),
          dontTouch(orig.dontTouch),
          dontUse(orig.dontUse),
          le_fall(orig.le_fall),
          le_rise(orig.le_rise),
          Ron(orig.Ron),
          lib_pin2id_map(orig.lib_pin2id_map),
          pins(orig.pins),
          timingArcs(orig.timingArcs),
          powerTables(orig.powerTables),
          libname(orig.libname) {
    }
};

// Table template information
struct LibTableTempl {
    string name;
    vector< double > transitionIndices;
    vector< double > loadIndices;
    // Indicates whether index1 is load cap or transition time
    bool loadFirst;
    bool tranFirst;
};

struct LibInfo {
    string name;
    double time_unit;
    double voltage_unit;
    double current_unit;
    double cap_unit;
    double leak_power_unit;
    double int_power_unit;
    double sw_power_unit;
    map< string, LibTableTempl > templs;
    double max_transition;
    double volt;
};

struct SOL {
    string cell_name;
    string inst_name;
    unsigned x;
    double delta_x;
    int width;
    cell_vtypes vt;
    double cost_leak;
    double cost_move;
    double best_leak_prev;
    double best_move_prev;

    SOL()
        : cell_name(""),
          inst_name(""),
          x(0.0),
          delta_x(0.0),
          width(0),
          vt(s),
          cost_leak(0.0),
          cost_move(0.0),
          best_leak_prev(0.0),
          best_move_prev(0.0) {
    }

    SOL(const SOL& orig)
        : cell_name(orig.cell_name),
          inst_name(orig.inst_name),
          x(orig.x),
          delta_x(orig.delta_x),
          width(orig.width),
          vt(orig.vt),
          cost_leak(orig.cost_leak),
          cost_move(orig.cost_move),
          best_leak_prev(orig.best_leak_prev),
          best_move_prev(orig.best_move_prev) {
    }

    const SOL& operator=(const SOL& assign) {
        cell_name = assign.cell_name;
        inst_name = assign.inst_name;
        x = assign.x;
        delta_x = assign.delta_x;
        width = assign.width;
        vt = assign.vt;
        cost_leak = assign.cost_leak;
        cost_move = assign.cost_move;
        best_leak_prev = assign.best_leak_prev;
        best_move_prev = assign.best_move_prev;
        return *this;
    }
};

ostream& operator<<(ostream& os, SOL& sol);

class Sizer;
class Sta;

class Circuit {
   public:
    Circuit();
    Circuit(Sizer* sizer) : _sizer(sizer) {
    }
    ~Circuit() {
        delete _ord_timing;
    }

    friend class Sizer;
    map< string, unsigned > check_map;
    string getFullPinName(PIN& pin);
    void buildSizingModel();
    void assertNoIllegalExternalPadBoundaryChain();
    void Print_Stats();

    void merge_timingArcs(LibTimingInfo& timing1, LibTimingInfo& timing2);

    void InitData();
    bool isDontUse(string master);
    ord::Design* _ord_design = nullptr;
    ord::Timing* _ord_timing = nullptr;
    ord::Tech* _ord_tech = nullptr;

   protected:

    void importLiberty(sta::LibertyLibrary* library, unsigned corner = 0);
    // OpenSTA
    void importDesign(sta::dbSta* _sta);

    void importParasitics(sta::dbSta* _sta);
    sta::dbSta* _sta = nullptr;
    Tcl_Interp* sta_interp = nullptr;
    std::vector< string > old_master_map;

   private:
    Sizer* _sizer;
    /////////////////////////////////
    //
    // circuit information
    //
   public:
    vector< double > inrtran, inftran;

    unsigned numpins, numcells, numnets;

    vector< CELL > g_cells;
    vector< CELL > g_phy_cells;
    vector< PIN > g_pins;
    vector< vector< NET > > g_nets;

    queue< pair< double, double > > search_queue;

    vector< unsigned > PIs;
    vector< unsigned > POs;
    vector< unsigned > FFs;

    map< string, double > indelays;
    map< string, double > outdelays;

    map< string, unsigned > cell2id;
    map< string, unsigned > pcell2id;
    map< string, unsigned > net2id;
    map< string, unsigned > pin2id;  // from 0 to numpins-1

    vector< unsigned > topolist;
    vector< unsigned > rtopolist;
    vector< unsigned > map2topoidx;

    ////////////////////////////////
    // Added geometry information
    //
    unsigned rowW;
    unsigned rowH;
    unsigned numW;
    unsigned numH;
    int siteLeft;
    int siteRight;
    int siteBottom;
    int siteTop;
    int chipLeft;
    int chipRight;
    int chipBottom;
    int chipTop;

    unsigned factor;
    vector< set< entry > > rowCellList;

   private:
    // verilog

    // sdc

    // spec


    // timing

    // lib


    // functions for lib parser

    void importCell(sta::LibertyLibrary* sta_lib, sta::LibertyCell* sta_cell,
                    LibCellInfo& out_cell);
    void importPin(sta::LibertyLibrary* sta_lib, sta::LibertyPort* sta_port,
                   LibCellInfo& out_cell, LibPinInfo& out_pin);
    void importTimingArcs(sta::LibertyLibrary* sta_lib,
                      sta::TimingArcSet* sta_arcset, LibTimingInfo& out_timing);
    void update_lut(LibLUT& toLut, LibLUT fromLut);
    void average_lut(LibLUT& lut, int num);
    void report_cell(LibCellInfo cell);

    // etc
    void generate_sizelookup();
    void runGR(int gr_overflow_iterations, bool fast, int slack_max_iter);

    map< string, unsigned > generateLibCellTable();
    void assignLibCellTables(map< string, unsigned > check_map);
    void assignMaxTrans();
    void assignLibPinId();
    void createLibCellTable(LibCellTable& lib_cell_table, unsigned corner = 0);
};

ostream& operator<<(ostream& os, const LibCellInfo& cell);

#endif
