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

#include "rsz/Resizer.hh"
#include "dpl/Opendp.h"
#include "est/EstimateParasitics.h"
#include "repair_parasitics.h"

#include "cell_sizing.h"
#include <iomanip>

#include <omp.h>
#include <stdlib.h>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include "sizing_model.h"
#include "odb/db.h"
#include "ord/ordMain.hh"
#include "utils.h"
#include <iostream>
#include <string>
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
#include "design_edit.h"
#include "repair_drv.h"
#include "repair_vg.h"
#include "repair_vg_cuda.h"
#include "repair_gobi.h"
#include "repair_gobi_cuda.h"
#include "repair_timing.h"
#include <stdexcept>
#include "special_net.h"

// global variables
// Profiling runtime switch and output file name.
bool ISO_TIME = false;
bool MINIMUM = false;
bool MAXIMUM = false;
bool CORR_AAT = false;
double SLEW_GB = 10.0;
unsigned MAX_TRIALS = 3000;
unsigned MAX_TARGETS = 40;
unsigned PWR_WORST_VIEW = 0;
double MAX_CELL_SLACK = 1.5;
bool USE_FULL_STA = false;
bool USE_FULL_STA2 = false;
unsigned MAX_VISIT = 30;
bool CRIT_CELL_NO_TOUCH = false;
bool INIT_WORST_PATH = false;
unsigned tot_accept = 0;
bool TRIAL_MOVE = false;
int TRIAL_MOVE_NUM = true;
bool COMPARE_TNS = false;
bool TABU = false;
bool CHECK_ALL_VIEW = false;
string debug_net = "";
bool NO_FOOTPRINT = false;
bool STM28 = false;
bool C40 = false;
bool ASAP7 = true;
bool TIMING_RECOVERY = false;
bool OLD_SIZER = false;
bool ALL_MOVE = false;
bool PEEPHOLE = false;
int PEEPHOLE_NUMCELL = 1;
int PEEPHOLE_ITER = 3;
bool NO_SEQ_OPT = false;
// Keep the input placement fixed for sizing-only A/B experiments.  The
// historical flow normally calls runOrdTO() before export, which can move
// cells during legalization and makes a placement-controlled comparison
// impossible.
bool PRESERVE_PLACEMENT = false;
bool NO_CLKBUF_OPT = true;
double ISO_TNS = 0.0;
bool VAR_GB = false;
double VAR_GB_TH = 1.0;
double VAR_GB_RATE = 0.01;
bool HOLD_CHECK = false;
bool SAFE_LEAK_OPT = true;
double SAFE_LEAK_OPT_GB = 0.0;
double CRIT_PATH_RATIO = 0.99;
int RELEASE_MODE = 3;
bool RELEASE = false;
double POWER_OPT_GB = 0.0;
double ATTACK_NEW_RATIO = 80;
double TIMING_OPT_GB = 0.0;
int SAME_SS_LIMIT = 5;
double TRIAL_RATE = 0.3;
bool NO_TOPO = false;
double COMMON_THRESH = 0.6;
double TIMEOUT = 864000;
double SLK_TH = 0.02;
int ALPHA_STUCK = 2;
double STUCK_THRES = 0.005;
bool MIN_VT = false;
bool FINAL_PWR_OPT = false;
bool MIN_SIZE = false;
bool UPDATE_LIST = false;
int MULTI_STEP = 3;
int MULTI_STEP_KICK = 1;
int MULTI_STEP_PWR = 1;
double KICK_RATIO = 0.01;
double MAX_KICK_RATIO = 0.01;
bool KICK_MOVE_CHANGE = false;
int KICK_METHOD = 4;
int KICK_SFT = 16;  // SF 6 is TNS only, originally 16
int NORM_SFT = 2;
double KICK_SLACK = 0.02;
int KICK_MAX = 1;
int KICK_STEP = 1;
bool VT_ONLY = false;
bool SIZE_ONLY = false;
// FIX_SLEW selects the legacy slew-statistics source in CalcStats. The
// command-file repair controls are kept separate so disabling repair does not
// silently change the timing objective.
bool FIX_SLEW = true;
bool FIX_CAP_REPAIR = true;
bool FIX_SLEW_REPAIR = true;
bool FIX_SLEW_POST = false;
bool FIX_GLOBAL = true;
bool FIXDRV_VT_SWAP = false;  // allow VT-swap (retype) moves in fix cap/slew phases
int GWTW_MAX = 1;
int GWTW_DIV = 4;
int GWTW_NUM_START = 4;

bool DATA_PIN_ONLY = false;
bool NO_LOG = false;
bool CORR_DYN = false;
int MAX_THREAD = 1;
int PTPORT = 7474;
int PNRPORT = 8020;
int TOLER_NUM = 5;
double TOLERANCE = 0.0;
double TOLER_STOP = 0.0;
bool TOLER_STEP = false;
double ALPHA = 0.0;
double NORMALIZE_FACTOR = 10.0;
double GAMMA = 0.5;
double SLEW_TH = 0.2;
unsigned OFFSET = 50;
double CRISLACK = 0.0;
double BETA = 1.0;
unsigned THRESHOLD = 50;
double ratio_sweep = 5;
int PTNUM = 1;
int PRFT_PTNUM = 1;
int VERBOSE = 1;
bool USE_PT = true;
// bool CORR_PT = true;
bool CORR_PT = false;
bool CORR_PT_FILE = true;
// bool     CORR_PT_FILE = false;
bool PT_FULL_UPDATE = false;
bool FAST = false;
bool PRFT_ONLY = true;
bool GTR_IN = false;
unsigned ML_WIRETRAN_MODEL = 2;
unsigned ML_WIREDELAY_MODEL = 1;
unsigned ML_CELLTRAN_MODEL = 1;
double STA_MARGIN = 0.0005;
double GUARD_BAND = 0.0;
double GUARD_BAND_GTR = 10.0;
double GUARD_BAND_2ND_GTR = 0.0;
string PRFT_FI = "";
string UD_FI = "";
string GTR_FI = "";
bool LEAKOPT = true;
int DIFFICULTY = 0;
int MAX_TRAN_CONST = 1;
double CORR_RATIO = 0.1;
double PWR_CORR_RATIO = 0.01;
bool GTRWPT_ONLY = false;
double X_IN = 0.0;
double EXP_IN = 0.0;
unsigned MIN_IA_MODE = 0;
unsigned tabuNum = 0;
int RATIO2 = 30;
int ATTACK_RATIO = 30;
#ifdef ALPHA_BIN
bool BACKGROUND = true;
#else
bool BACKGROUND = true;
#endif
int LEAKOPT_OPTION = 0;

DelayMetric WIRE_METRIC = EM;
SlewMetric SLEW_METRIC = PERI;
CapMetric CAP_METRIC = CTOT;
string TEST_MODE = "NO_TEST";
GTRMetric GTR_METRIC1 = SF8;
GTRMetric GTR_METRIC2 = SF8;
CorrPTMetric CORR_PT_METRIC = SLK;

// static declaration of thread-local variables //
// double Sizer::max_pt_err;
// double Sizer::average_error;
// double Sizer::l2_norm;
// int Sizer::leak_iter;

// double Sizer::skew_violation_worst;
// double Sizer::worst_slack_worst;
// double Sizer::worst_slack;
// double Sizer::max_neg_rslk;
// double Sizer::max_neg_fslk;
// double Sizer::min_neg_rslk;
// double Sizer::min_neg_fslk;
// double Sizer::max_pos_rslk;
// double Sizer::max_pos_fslk;

// double Sizer::tot_pslack;
// double Sizer::tot_violations;
// double Sizer::slew_violation;
// double Sizer::skew_violation;
// double Sizer::cap_violation;
// unsigned Sizer::slew_violation_cnt;
// unsigned Sizer::skew_violation_cnt;
// unsigned Sizer::cap_violation_cnt;
// double Sizer::slew_violation_wst;
// double Sizer::cap_violation_wst;
// double Sizer::power;
// double Sizer::best_score_local;
// double Sizer::best_alpha_local;
// double Sizer::local_alpha;
// double Sizer::toler;
// double Sizer::best_failed_wns_local;

// PIN **Sizer::pins;
// NET **Sizer::nets;

// CELL * Sizer::best_cells_local;
// CELL * Sizer::best_failed_cells_local;
// designTiming **Sizer::T;

extern int test_timer(string root, string benchmark, string sizes);
extern double r_entry(const LibLUT &liblut, const double tran,
                      const double cap);

struct THREAD_ARGS {
    Sizer *this_instance;
    unsigned thread_id;
};

//////////////////////////////////////////////////

bool sort_func(double a, double b) {
    return (a > b);
}
bool sort_func_inverse(double a, double b) {
    return (a < b);
}

bool sort_func_str(string a, string b) {
    return (a.compare(b) < 0);
}

class sortBy {
    const vector< unsigned > &_quantity;

   public:
    sortBy(const vector< unsigned > &quantity) : _quantity(quantity) {
    }
    bool operator()(const unsigned &a, const unsigned &b) const {
        return _quantity[a] < _quantity[b];
    }
};

DelayMetric str2DelayMetric(const string &str) {
    DelayMetric out = DM0;

    if(str == "ND") {
        out = ND;
    }
    else if(str == "EM") {
        out = EM;
    }
    else if(str == "DM0") {
        out = DM0;
    }
    else if(str == "DM1") {
        out = DM1;
    }
    else if(str == "DM2") {
        out = DM2;
    }
    else if(str == "DM3") {
        out = DM3;
    }
    else if(str == "DM4") {
        out = DM4;
    }
    else if(str == "ML") {
        out = ML;
    }
    else if(str == "DEBUG") {
        out = DEBUG;
    }
    return out;
}

CapMetric str2CapMetric(const string &str) {
    CapMetric out = CTOT;

    if(str == "CTOT") {
        out = CTOT;
    }
    else if(str == "CEFFMC") {
        out = CEFFMC;
    }
    else if(str == "CEFFKM") {
        out = CEFFKM;
    }
    else if(str == "CEFFPT") {
        out = CEFFPT;
    }
    return out;
}

SlewMetric str2SlewMetric(const string &str) {
    SlewMetric out = PERI;

    if(str == "PERI") {
        out = PERI;
    }
    else if(str == "S2M") {
        out = S2M;
    }
    else if(str == "PERI_S2M") {
        out = PERI_S2M;
    }
    else if(str == "MLS") {
        out = MLS;
    }
    return out;
}

TestMode str2TestMode(const string &str) {
    TestMode out = NO_TEST;

    if(str == "TEST1") {
        out = TEST1;
    }
    else if(str == "TEST2") {
        out = TEST2;
    }
    else if(str == "TEST3") {
        out = TEST3;
    }
    else if(str == "TEST4") {
        out = TEST4;
    }
    else if(str == "TEST5") {
        out = TEST5;
    }
    else if(str == "TEST6") {
        out = TEST6;
    }
    else if(str == "HL_TEST") {
        out = HL_TEST;
    }
    else if(str == "SK_TEST") {
        out = SK_TEST;
    }
    else if(str == "STA_TEST") {
        out = STA_TEST;
    }
    else if(str == "SLK_CORR_TEST") {
        out = SLK_CORR_TEST;
    }
    return out;
}

GTRMetric str2GTRMetric(const string &str) {
    GTRMetric out = SF8;

    if(str == "SF1") {
        out = SF1;
    }
    else if(str == "SF2") {
        out = SF2;
    }
    else if(str == "SF3") {
        out = SF3;
    }
    else if(str == "SF4") {
        out = SF4;
    }
    else if(str == "SF5") {
        out = SF5;
    }
    else if(str == "SF6") {
        out = SF6;
    }
    else if(str == "SF7") {
        out = SF7;
    }
    else if(str == "SF8") {
        out = SF8;
    }
    else if(str == "SF9") {
        out = SF9;
    }
    return out;
}

CorrPTMetric str2CorrPTMetric(const string &str) {
    CorrPTMetric out = SLK;

    if(str == "ALL") {
        out = ALL;
    }
    else if(str == "SLK") {
        out = SLK;
    }
    else if(str == "TRAN") {
        out = TRAN;
    }
    return out;
}

int get_available_port(int start_port) {
    int port = start_port;
    int sockfd;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    while(port <= 65535) {
        // Create socket.
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if(sockfd < 0) {
            ecc::logError() << "Socket creation failed" << std::endl;
            return -1;
        }

        // Set address and port.
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        // Bind port.
        if(bind(sockfd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            // If the port is already in use, try the next one.
            close(sockfd);
            ++port;
            continue;
        }

        ecc::logInfo() << "Available port found: " << port << std::endl;
        close(sockfd);
        return port;
    }

    return -1;  // No available port found
}

int getTokenI(string line, string option) {
    int token;
    size_t sizeStr, startPos, endPos;
    string arg;
    sizeStr = option.size();
    startPos = line.find_first_not_of(" ", line.find(option) + sizeStr);
    endPos = line.find_first_of(" ", startPos);
    arg = line.substr(startPos, endPos - startPos);
    sscanf(arg.c_str(), "%d", &token);
    return token;
}

float getTokenF(string line, string option) {
    float token;
    size_t sizeStr, startPos, endPos;
    string arg;
    sizeStr = option.size();
    startPos = line.find_first_not_of(" ", line.find(option) + sizeStr);
    endPos = line.find_first_of(" ", startPos);
    arg = line.substr(startPos, endPos - startPos);
    sscanf(arg.c_str(), "%f", &token);
    return token;
}

string getTokenS(string line, string option) {
    size_t sizeStr, startPos, endPos;
    string arg;
    sizeStr = option.size();
    startPos = line.find_first_not_of(" ", line.find(option) + sizeStr);
    endPos = line.find_first_of(" ", startPos);
    arg = line.substr(startPos, endPos - startPos);
    return arg;
}

char *string_to_char(string str) {
    char *writable = new char[str.size() + 1];
    std::copy(str.begin(), str.end(), writable);
    writable[str.size()] = '\0';
    return writable;
}

vector< string > split(string input, string delim) {
    vector< string > result;
    char *charArray = string_to_char(input);
    char *p = strtok(charArray, delim.c_str());
    while(p) {
        result.push_back(string(p));
        p = strtok(NULL, delim.c_str());
    }
    delete[] charArray;
    return result;
}

void Sizer::setEquivCellSortMode(const string& mode) {
    if(mode == "leakage") {
        sortEquivCellsByLeakage = true;
        return;
    }
    if(mode == "drive_resistance" || mode == "drive_res" ||
       mode == "drive") {
        sortEquivCellsByLeakage = false;
        return;
    }

    ecc::logInfo() << "Error: unsupported -equiv_cell_sort mode '" << mode
         << "'. Use drive_resistance or leakage." << endl;
    exit(1);
}

string Sizer::equivCellSortModeName() const {
    return sortEquivCellsByLeakage ? "leakage" : "drive_resistance";
}

int Sizer::equivCellDriveOrder(const string& cell_name) const {
    auto iter = cellName2EquivOrder.find(cell_name);
    if(iter == cellName2EquivOrder.end()) {
        return std::numeric_limits< int >::max();
    }
    return iter->second;
}

bool Sizer::compareEquivCellsForSizing(const LibCellInfo* lhs,
                                       const LibCellInfo* rhs) const {
    if(sortEquivCellsByLeakage &&
       lhs->leakagePower != rhs->leakagePower) {
        return lhs->leakagePower < rhs->leakagePower;
    }

    int lhs_order = equivCellDriveOrder(lhs->name);
    int rhs_order = equivCellDriveOrder(rhs->name);
    if(lhs_order != rhs_order) {
        return lhs_order < rhs_order;
    }

    if(!sortEquivCellsByLeakage &&
       lhs->leakagePower != rhs->leakagePower) {
        return lhs->leakagePower < rhs->leakagePower;
    }

    if(lhs->partial_order != rhs->partial_order) {
        return lhs->partial_order < rhs->partial_order;
    }

    return lhs->name < rhs->name;
}

LibCellInfo *Sizer::getLibCellInfo(int main_lib_cell_id, cell_sizes size,
                                   cell_vtypes vtype, unsigned corner) {
    LibCellTable *lib_cell_table = NULL;

    if(main_lib_cell_id != -1 &&
       main_lib_cell_id < main_lib_cell_tables[corner].size()) {
        lib_cell_table = main_lib_cell_tables[corner][main_lib_cell_id];
    }
    else {
        if(VERBOSE >= 4) {
            ecc::logPrintf("Error: main_lib_cell_id can't find %d\n", main_lib_cell_id);
        }
    }

    if(lib_cell_table == NULL) {
        return NULL;
    }

    if(size < main_lib_cell_tables[corner][main_lib_cell_id]
                  ->lib_vt_size_table.size()) {
        if(vtype < main_lib_cell_tables[corner][main_lib_cell_id]
                       ->lib_vt_size_table[size]
                       .size()) {
            return main_lib_cell_tables[corner][main_lib_cell_id]
                ->lib_vt_size_table[size][vtype];
        }
    }
    else {
        ecc::logPrintf(
            "Error: Cell size %d /vtype %d can't find, cell main_lib_cell_id "
            "%d\n",
            size, vtype, main_lib_cell_id);
    }
    return NULL;
}

LibCellInfo *Sizer::getLibCellInfo(CELL &cell, unsigned corner) {
    // assert(cell.type != "");
    // unordered_map< string, LibCellInfo >::iterator temp_iter =
    //     libs[corner].find(cell.type);
    if(cell.main_lib_cell_id < 0) {
        return nullptr;
    }
    if(isff(cell) && cell.clock_pin == UINT_MAX) {
        return nullptr;
    }
    assert(main_lib_cell_tables.size());
    return getLibCellInfo(cell.main_lib_cell_id, cell.c_size,
                          static_cast< cell_vtypes >(cell.c_vtype), corner);
}

LibCellInfo *Sizer::getLibCellInfo(string type, unsigned corner) {
    unordered_map< string, LibCellInfo >::iterator temp_iter =
        libs[corner].find(type);
    assert(type != "");
    if(temp_iter != libs[corner].end()) {
        return &(temp_iter->second);
    }
    else {
        return NULL;
    }
}

bool Sizer::cell_change(CELL &cell, CellSol cell_sol, bool update_cap) {
    if(cell_sol.c_size < 0 || cell_sol.c_vtype < 0) {
        return false;
    }

    LibCellInfo *new_lib_cell_info =
        getLibCellInfo(cell.main_lib_cell_id, cell_sol.c_size,
                       static_cast< cell_vtypes >(cell_sol.c_vtype));

    if(new_lib_cell_info != NULL) {
        cell.isChanged++;
        cell.type = new_lib_cell_info->name;
        cell.c_size = cell_sol.c_size;
        cell.c_vtype = static_cast< cell_vtypes >(cell_sol.c_vtype);
        if(update_cap) {
            for(unsigned i = 0; i < cell.inpins.size(); i++) {
                for(unsigned j = 0; j < numViews; ++j) {
                    pins[j][cell.inpins[i]].cap =
                        new_lib_cell_info->pins[pins[j][cell.inpins[i]].lib_pin]
                            .capacitance;
                }
            }
        }

        return true;
    }

    return false;
}

bool Sizer::cell_resize(CELL &cell, int steps, bool pt_corr, bool update_cap) {
    if(VT_ONLY)
        return false;

    bool min = false;
    bool max = false;

    if(cell.c_size == 0) {
        min = true;
    }
    max = isMax(cell);

    if(steps == 0)
        return max;

    if(steps > 0 && max) {
        return false;
    }

    if(steps < 0 && min) {
        return false;
    }

    unsigned corner = 0;
    cell_sizes size = cell.c_size;

    LibCellInfo *lib_cell_info = getLibCellInfo(cell, corner);
    if(lib_cell_info == NULL) {
        return false;
    }

    LibCellTable *lib_cell_table = NULL;

    if(cell.main_lib_cell_id != -1 &&
       cell.main_lib_cell_id < main_lib_cell_tables[corner].size()) {
        lib_cell_table = main_lib_cell_tables[corner][cell.main_lib_cell_id];
    }

    if(lib_cell_table == NULL) {
        return false;
    }

    int new_size = size + steps;

    if(new_size >= lib_cell_table->lib_vt_size_table.size()) {
        new_size = lib_cell_table->lib_vt_size_table.size() - 1;
    }
    else if(new_size < 0) {
        new_size = 0;
    }

    if(new_size == size) {
        return false;
    }

    LibCellInfo *new_lib_cell_info =
        getLibCellInfo(cell.main_lib_cell_id, new_size, cell.c_vtype, corner);

    if(new_lib_cell_info != NULL) {
        cell.isChanged++;
        cell.type = new_lib_cell_info->name;
        cell.c_size = new_size;

        if(update_cap) {
            for(unsigned j = 0; j < numViews; ++j) {
                for(unsigned i = 0; i < cell.inpins.size(); i++)
                    pins[j][cell.inpins[i]].cap =
                        new_lib_cell_info->pins[pins[j][cell.inpins[i]].lib_pin]
                            .capacitance;
            }
            // for(unsigned j = 0; j < numViews; ++j) {
        }

        if(pt_corr) {
            for(unsigned view = 0; view < numViews; ++view) {
                T[view]->sizeCell(cell.name, cell.type);
            }
            cell.isChanged = 0;
        }

        return true;
    }

    return false;
}

// FIXME:
bool Sizer::cell_resize_cap(CELL &cell, int steps, bool pt_corr,
                            bool update_cap) {
    if(VT_ONLY)
        return false;

    bool min = false;
    bool max = false;

    unsigned corner = 0;
    LibCellInfo *lib_cell_info = getLibCellInfo(cell, corner);
    if(lib_cell_info == NULL) {
        return false;
    }
    int cap_size = lib_cell_info->cap_size;
    if(cell.c_size == 0) {
        min = true;
    }
    max = isMax(cell);

    if(steps == 0)
        return max;

    if(steps > 0 && max) {
        return false;
    }

    if(steps < 0 && min) {
        return false;
    }

    cell_sizes size = cell.c_size;

    LibCellTable *lib_cell_table = NULL;

    if(cell.main_lib_cell_id != -1 &&
       cell.main_lib_cell_id < main_lib_cell_tables[corner].size()) {
        lib_cell_table = main_lib_cell_tables[corner][cell.main_lib_cell_id];
    }

    if(lib_cell_table == NULL) {
        return false;
    }

    int new_size = size + steps;

    if(new_size >= lib_cell_table->lib_vt_size_table.size()) {
        new_size = lib_cell_table->lib_vt_size_table.size() - 1;
    }
    else if(new_size < 0) {
        new_size = 0;
    }

    if(new_size == size) {
        return false;
    }

    LibCellInfo *new_lib_cell_info =
        getLibCellInfo(cell.main_lib_cell_id, new_size, cell.c_vtype, corner);

    if(new_lib_cell_info != NULL) {
        cell.isChanged++;
        cell.type = new_lib_cell_info->name;
        cell.c_size = new_size;

        if(update_cap) {
            for(unsigned j = 0; j < numViews; ++j) {
                for(unsigned i = 0; i < cell.inpins.size(); i++)
                    pins[j][cell.inpins[i]].cap =
                        new_lib_cell_info->pins[pins[j][cell.inpins[i]].lib_pin]
                            .capacitance;
            }
            // for(unsigned j = 0; j < numViews; ++j) {
        }

        if(pt_corr) {
            for(unsigned view = 0; view < numViews; ++view) {
                T[view]->sizeCell(cell.name, cell.type);
            }
            cell.isChanged = 0;
        }

        return true;
    }

    return false;
}

bool Sizer::cell_retype(CELL &cell, int dir, bool pt_corr, bool update_cap) {
    if(SIZE_ONLY)
        return false;

    cell_vtypes vtype = cell.c_vtype;
    // ecc::logPrintf("retype %s\n", cell.name.c_str());
    if(dir == 0)
        return (cell.c_vtype == (numVt - 1));

    if(dir > 0 && vtype == (numVt - 1)) {
        return false;
    }

    if(dir < 0 && vtype == 0) {
        return false;
    }

    unsigned corner = 0;
    LibCellInfo *lib_cell_info = getLibCellInfo(cell, corner);
    if(lib_cell_info == NULL) {
        return false;
    }

    LibCellTable *lib_cell_table = NULL;

    if(cell.main_lib_cell_id != -1 &&
       cell.main_lib_cell_id < main_lib_cell_tables[corner].size()) {
        lib_cell_table = main_lib_cell_tables[corner][cell.main_lib_cell_id];
    }

    if(lib_cell_table == NULL) {
        return false;
    }

    int new_vt = vtype + dir;
    int numVt_cell = 0;

    if(lib_cell_table->lib_vt_size_table.size() > 0) {
        numVt_cell = lib_cell_table->lib_vt_size_table[0].size();
    }
    else {
        return false;
    }

    if(new_vt >= numVt_cell || new_vt < 0) {
        return false;
    }
    else {
        LibCellInfo *new_lib_cell_info =
            getLibCellInfo(cell.main_lib_cell_id, cell.c_size,
                           static_cast< cell_vtypes >(new_vt), corner);
        if(new_lib_cell_info != NULL) {
            cell.isChanged++;
            cell.type = new_lib_cell_info->name;
            cell.c_vtype = static_cast< cell_vtypes >(new_vt);

            if(update_cap) {
                for(unsigned j = 0; j < numViews; ++j) {
                    for(unsigned i = 0; i < cell.inpins.size(); i++)
                        pins[j][cell.inpins[i]].cap =
                            new_lib_cell_info
                                ->pins[pins[j][cell.inpins[i]].lib_pin]
                                .capacitance;
                }
            }

            if(pt_corr) {
                for(unsigned view = 0; view < numViews; ++view) {
                    T[view]->sizeCell(cell.name, cell.type);
                }
                cell.isChanged = 0;
            }
            return true;
        }
    }
    return false;
}

bool Sizer::cell_move(CELL &cell, cell_sizes org_size, cell_vtypes org_vt,
                      int move) {
    cell_change(cell, org_size, org_vt);
    bool changed = true;

    switch(move) {
        case 0:
            changed = false;
            break;

        case 1:
            // 1: size up
            cell_resize(cell, 1);
            if((int)org_size == (int)cell.c_size &&
               (int)org_vt == (int)cell.c_vtype)
                changed = false;
            break;

        case 2:
            // 2: size down
            cell_resize(cell, -1);
            if((int)org_size == (int)cell.c_size &&
               (int)org_vt == (int)cell.c_vtype)
                changed = false;
            break;

        case 3:
            // 3: vth up
            cell_retype(cell, 1);
            if((int)org_size == (int)cell.c_size &&
               (int)org_vt == (int)cell.c_vtype)
                changed = false;
            break;

        case 4:
            // 4: vth down
            cell_retype(cell, -1);
            if((int)org_size == (int)cell.c_size &&
               (int)org_vt == (int)cell.c_vtype)
                changed = false;
            break;

        case 5:
            // 5: size up vth up
            cell_resize(cell, 1);
            cell_retype(cell, 1);
            if((int)org_size == (int)cell.c_size &&
               (int)org_vt == (int)cell.c_vtype)
                changed = false;
            break;

        case 6:
            // 6: size up vth down
            cell_resize(cell, 1);
            cell_retype(cell, -1);
            if((int)org_size == (int)cell.c_size &&
               (int)org_vt == (int)cell.c_vtype)
                changed = false;
            break;

        case 7:
            // 7: size down vth up
            cell_resize(cell, -1);
            cell_retype(cell, 1);
            if((int)org_size == (int)cell.c_size &&
               (int)org_vt == (int)cell.c_vtype)
                changed = false;
            break;

        case 8:
            // 8: size down vth down
            cell_resize(cell, -1);
            cell_retype(cell, -1);
            if((int)org_size == (int)cell.c_size &&
               (int)org_vt == (int)cell.c_vtype)
                changed = false;
            break;

        default:
            break;
    }
    //    ecc::logInfo() <<cell.type<<")";
    //    if ( changed )
    //        ecc::logInfo() << " -- cell changed" << endl;
    //    else
    //        ecc::logInfo() << " -- cell not changed" << endl;
    return changed;
}

bool Sizer::cell_change(CELL &cell, cell_sizes size, cell_vtypes vt,
                        bool update_cap) {
    CellSol cell_sol;

    cell_sol.c_vtype = vt;
    cell_sol.c_size = size;

    if(cell_sol.c_size < 0 || cell_sol.c_vtype < 0) {
        return false;
    }

    LibCellInfo *new_lib_cell_info =
        getLibCellInfo(cell.main_lib_cell_id, cell_sol.c_size,
                       static_cast< cell_vtypes >(cell_sol.c_vtype));

    if(new_lib_cell_info != NULL) {
        cell.isChanged++;
        cell.type = new_lib_cell_info->name;
        cell.c_size = cell_sol.c_size;
        cell.c_vtype = static_cast< cell_vtypes >(cell_sol.c_vtype);
        if(update_cap) {
            for(unsigned j = 0; j < numViews; ++j) {
                for(unsigned i = 0; i < cell.inpins.size(); i++)
                    pins[j][cell.inpins[i]].cap =
                        new_lib_cell_info->pins[pins[j][cell.inpins[i]].lib_pin]
                            .capacitance;
            }
        }

        // ecc::logInfo() << "CHANGED -- " << cell.type << endl;

        return true;
    }

    return false;
}

void Sizer::ClearSwapFlag() {
    for(unsigned i = 0; i < numcells; i++) {
        cells[i].isChanged++;
    }
}

void Sizer::Clean() {
    char Commands[250];
    sprintf(Commands, "\\rm -f launch_pt.*.cmd");
    system(Commands);
    sprintf(Commands, "\\rm -f ptclient.*.tcl");
    system(Commands);
    sprintf(Commands, "\\rm -f ptserver.*.tcl");
    system(Commands);
    sprintf(Commands, "\\rm -f socket*");
    system(Commands);
}
double Sizer::calcScore(double leakage, double tns, double slew, double cap) {
    double t_score = leakage - init_tot[0] + tnsPenalty * fabs(tns) +
                     slewPenalty * fabs(slew) + capPenalty * fabs(cap);
    return t_score;
}

void Sizer::CleanIntFiles() {
    char Commands[250];
    sprintf(Commands,
            "\\rm -f *_sizes.tcl *.pin_list *.pt_all.timing ex_change.tcl "
            "pt*.log command.log parasitics_command.log *.pt_slack.timing "
            "*.pt_tran.timing *.pt.ceff");
    system(Commands);
}

void Sizer::Parser(ord::Design& design) {
    double begin = cpuTime();
    directory = "./";
    _ckt = new Circuit(this);
    _ckt->_ord_design = &design;
    _ckt->_ord_tech = design.getTech();
    _ckt->_ord_timing = new ord::Timing(&design);
    _ckt->_sta = design.getTech()->getSta();
    _ckt->Parser(directory + benchname);
    _sta = _ckt->_sta;
    ecc::logInfo() << endl
         << "Hard Runtime Limit   : " << RuntimeLimit / 3600 << " hours"
         << endl;
    _ckt->Print_Stats();

    _ckt->numpins = _ckt->g_pins.size();
    _ckt->numcells = _ckt->g_cells.size();
    _ckt->numnets = _ckt->g_nets[0].size();
    this->numcells = _ckt->numcells;
    this->numpins = _ckt->numpins;
    this->numnets = _ckt->numnets;

    for(unsigned i = 0; i < _ckt->numcells; i++) {
        CELL cell = _ckt->g_cells[i];
        g_cells.push_back(cell);
    }

    for(unsigned j = 0; j < numViews; j++) {
        g_pins.push_back(vector< PIN >());
        for(unsigned i = 0; i < _ckt->numpins; i++) {
            PIN pin = _ckt->g_pins[i];
            g_pins[j].push_back(pin);
            if(mmmcOn) {
                for(unsigned k = 0; k < mmmcWaiveTranLists[j].size(); ++k) {
                    if(getFullPinName(pin) == mmmcWaiveTranLists[j][k]) {
                        g_pins[j][i].waiveTran = true;
                        ecc::logInfo() << "WAIVE TRAN " << j << " " << getFullPinName(pin)
                             << endl;
                    }
                }
            }
        }
    }

    for(unsigned j = 0; j < numCorners; j++) {
        g_nets.push_back(vector< NET >());
        for(unsigned i = 0; i < _ckt->numnets; i++) {
            NET net = _ckt->g_nets[j][i];
            g_nets[j].push_back(net);
        }
    }

    /*
    for(unsigned i=0 ; i<_ckt->numnets ; i++) {

      ecc::logInfo() << "--------- all sub node list (" << g_nets[0][i].name <<
    ")--------" << endl;

        vector <SUB_NODE> *subNodeVecPtr = &g_nets[0][i].subNodeVec;
            std::vector<SUB_NODE>::iterator subNodeIter;
        for (subNodeIter = subNodeVecPtr->begin(); subNodeIter !=
    subNodeVecPtr->end(); ++subNodeIter) {
                ecc::logInfo() << "Id: " << subNodeIter->id << ", Cap: " <<
    subNodeIter->cap << " " << subNodeIter->isSink << endl;

                for (unsigned int i=0; i < subNodeIter->adj.size(); ++i) {
                    iout << "Id: " << subNodeIter->id << ", Res: " <<
    subNodeIter->res[i] << " Adj :" << subNodeIter->adj[i] << endl;
                }
            }

        ecc::logInfo() << endl ;
    }
    */

    for(unsigned i = 0; i < _ckt->FFs.size(); ++i) {
        FFs.push_back(_ckt->FFs[i]);
    }
    for(unsigned i = 0; i < _ckt->POs.size(); ++i) {
        POs.push_back(_ckt->POs[i]);
        _pos_set.insert(_ckt->POs[i]);
        if(VERBOSE > 0)
            ecc::logInfo() << "PO -- " << getFullPinName(g_pins[0][POs[i]]) << endl;
        if(g_pins[0][POs[i]].owner != UINT_MAX) {
            ecc::logPrintf("PO has cell %s \n",
                   g_cells[g_pins[0][POs[i]].owner].name.c_str());
        }
    }
    for(unsigned i = 0; i < _ckt->PIs.size(); ++i) {
        PIs.push_back(_ckt->PIs[i]);
        if(VERBOSE > 0)
            ecc::logInfo() << "PI -- " << getFullPinName(g_pins[0][PIs[i]]) << endl;
    }
    for(unsigned mode = 0; mode < this->numModes; ++mode) {
        for(unsigned i = 0; i < PIs.size(); ++i) {
            int pin_id = PIs[i];
            this->g_pins[mode][pin_id].rAAT = this->inrdelays[mode][pin_id];
            this->g_pins[mode][pin_id].fAAT = this->infdelays[mode][pin_id];
        }
    }
    for(unsigned j = 0; j < numViews; j++) {
        for(unsigned i = 0; i < _ckt->numpins; i++) {
            PIN pin = _ckt->g_pins[i];
            int net_id = pin.net;
            if(pin.spef_pin == UINT_MAX && net_id != UINT_MAX &&
               infdelays[0].count(pin.id) == 0 && _pos_set.count(pin.id) == 0) {
                // outdelays has bug
                NET net = _ckt->g_nets[j][net_id];
                if(!net.is_clock && spefFile != "" && spefFile != "None") {
                    ecc::logPrintf(
                        "Error: pin %s has no spef pin, pin's net name is %s\n",
                        getFullPinName(pin).c_str(), net.name.c_str());
                }
            }
        }
    }
    cell2id = _ckt->cell2id;
    net2id = _ckt->net2id;
    pin2id = _ckt->pin2id;

    numpins = g_pins[0].size();
    numcells = g_cells.size();
    numnets = g_nets[0].size();

    if(numcells > 200000)
        MAX_THREAD = min(MAX_THREAD, 4);
    else if(numcells > 100000)
        MAX_THREAD = min(MAX_THREAD, 8);

    SortTopo();
    CountPaths();

    // for(unsigned j=1 ; j<numViews ; j++) {
    //    for(unsigned i=0 ; i<_ckt->numpins ; i++) {
    //        g_pins[j][i] = g_pins[0][i];
    //    }
    //}

    InitNets();  // calculate network cap for net delay

    // write pin list file for PT correlation
    if(TEST_MODE != "" && TEST_MODE != "NO_TEST") {
        string pin_file = benchname + ".pin_list";
        ofstream ofp(pin_file.c_str());
        for(unsigned i = 0; i < numpins; i++) {
            string full_pin_name;
            if(g_pins[0][i].owner != UINT_MAX) {
                full_pin_name =
                    g_cells[g_pins[0][i].owner].name + "/" + g_pins[0][i].name;
            }
            else {
                full_pin_name = g_pins[0][i].name;
            }
            ofp << full_pin_name << endl;
        }
        ofp.close();
    }

    time_IO += cpuTime() - begin;
}

void Sizer::ExitPTimer() {
    for(unsigned i = 0; i < MAX_THREAD; i++) {
        for(int view = 0; view < numViews; view++) {
            tot_timer_time += PTimer[i][view]->pt_time;
            // PTimer[i][view]->Exit();
            ecc::logInfo() << "Exit PrimeTime number " << i << " " << view << endl;
            // delete PTimer[i][view];
        }
        // delete[] PTimer[i];
    }
    // delete[] PTimer;
}

designTiming *Sizer::LaunchPTimer(unsigned thread_id, unsigned view) {
    char Commands[250];
    char serverNameChar[100];
    int port = PTPORT + thread_id * 5;
    std::ostringstream ostr;

    if(useOpenSTA) {
        if(util_tcl_file != "") {
            string exe_cmd = "source " + util_tcl_file;
            _ckt->_ord_design->evalTclString(exe_cmd);
        }
        designTiming *PT = new designTiming(OS, this);
        return PT;
        // exeOSServer(serverName, port, view);
    }
    else {
        ecc::logPrintf("Error: OpenSTA is not enabled\n");
        exit(0);
    }
}

void Sizer::InitPowerBeforeUpdate(vector< CELL > &c) {
    auto block = _ckt->_ord_design->getBlock();
    auto db = ord::OpenRoad::openRoad()->getDb();
    auto corner = this->_ckt->_ord_timing->getCorners()[0];
    for(unsigned i = 0; i < c.size(); i++) {
        if(getLibCellInfo(c[i]) == NULL)
            continue;
        auto inst = block->findInst(c[i].name.c_str());
        c[i].static_power = this->_ckt->_ord_timing->staticPower(inst, corner);
        c[i].isStaticChanged = false;
    }
}

void Sizer::refreshOpenStaParasitics(bool) {
    if(!refresh_design) throw std::logic_error("missing sizing refresh service");
    refresh_design();
}

void Sizer::applyOpenStaDbChanges(
    const std::function< void() > &apply_changes) {
    auto design = _ckt->_ord_design;
    auto sta = _ckt->_ord_timing->getSta();

    if(!noSPEF && !spefFile.empty()) {
        auto resizer = design->getResizer();
        auto estimate_parasitics = resizer->getEstimateParasitics();
        if(estimate_parasitics->isIncrementalParasiticsEnabled()) {
            apply_changes();
            resizer->updateParasiticsAndTiming();
            return;
        }
        est::IncrementalParasiticsGuard parasitics_guard(estimate_parasitics);
        apply_changes();
        resizer->updateParasiticsAndTiming();
        return;
    }

    if(!use_gr_rc) {
        apply_changes();
        design->evalTclString("estimate_parasitics -placement");
        sta->findRequireds();
        return;
    }

    auto grt = design->getGlobalRouter();
    auto resizer = design->getResizer();
    auto estimate_parasitics = resizer->getEstimateParasitics();

    // The incremental API requires a routed GR parasitics seed. This fallback
    // is only for callers that reach a size update before the normal GR setup.
    if(estimate_parasitics->getParasiticsSrc() !=
       est::ParasiticsSrc::kGlobalRouting) {
        refreshOpenStaParasitics(false);
    }

    if(estimate_parasitics->isIncrementalParasiticsEnabled()) {
        apply_changes();
        resizer->updateParasiticsAndTiming();
        return;
    }

    est::IncrementalParasiticsGuard parasitics_guard(estimate_parasitics);
    grt->startIncremental();
    apply_changes();
    resizer->updateParasiticsAndTiming();
    grt->endIncremental();
}

ecc::BufferInsertionResult Sizer::insertBufferAfterDriver(
    odb::dbNet *net, odb::dbMaster *buffer_master,
    const ecc::BufferInsertionOptions &options) {
    ecc::BufferInsertionResult result;
    applyOpenStaDbChanges([&]() {
        result = ecc::OpenDbBufferInserter::insertBufferAfterDriver(
            net, buffer_master, options);
        if(result) {
            if(use_gr_rc) {
                _ckt->_ord_design->getOpendp()->legalCellPos(result.buffer);
            }
            ecc::connectMissingWildcardSpecialNetPins(net->getBlock());
        }
    });
    return result;
}

ecc::BufferInsertionResult Sizer::insertBufferBeforeLoad(
    odb::dbObject *load, odb::dbMaster *buffer_master,
    const ecc::BufferInsertionOptions &options) {
    ecc::BufferInsertionResult result;
    applyOpenStaDbChanges([&]() {
        result = ecc::OpenDbBufferInserter::insertBufferBeforeLoad(
            load, buffer_master, options);
        if(result) {
            if(use_gr_rc) {
                _ckt->_ord_design->getOpendp()->legalCellPos(result.buffer);
            }
            ecc::connectMissingWildcardSpecialNetPins(result.buffer->getBlock());
        }
    });
    return result;
}

ecc::BufferInsertionResult Sizer::insertBufferBeforeLoads(
    odb::dbNet *net, const std::vector< odb::dbObject * > &loads,
    odb::dbMaster *buffer_master,
    const ecc::BufferInsertionOptions &options) {
    ecc::BufferInsertionResult result;
    applyOpenStaDbChanges([&]() {
        result = ecc::OpenDbBufferInserter::insertBufferBeforeLoads(
            net, loads, buffer_master, options);
        if(result) {
            if(use_gr_rc) {
                _ckt->_ord_design->getOpendp()->legalCellPos(result.buffer);
            }
            ecc::connectMissingWildcardSpecialNetPins(net->getBlock());
        }
    });
    return result;
}

ecc::FanoutRepairResult Sizer::repairFanout(
    odb::dbMaster *buffer_master,
    const ecc::FanoutRepairOptions &options) {
    ecc::FanoutRepairResult result;
    odb::dbBlock *block = _ckt->_ord_design->getBlock();
    applyOpenStaDbChanges([&]() {
        result = ecc::OpenDbFanoutRepairer::repair(block, buffer_master,
                                                   options);
        if(result) {
            if(use_gr_rc) {
                auto *opendp = _ckt->_ord_design->getOpendp();
                for(odb::dbInst *buffer : result.inserted_buffers) {
                    opendp->legalCellPos(buffer);
                }
            }
            ecc::connectMissingWildcardSpecialNetPins(block);
        }
    });
    return result;
}

void Sizer::UpdatePTSizes(vector< CELL > &cells, unsigned option) {
    ecc::logError() << "Update PT sizes... " << std::endl;
    ecc::logError() << "numcell = " << cells.size() << std::endl;
    auto db = ord::OpenRoad::openRoad()->getDb();
    auto block = _ckt->_ord_design->getBlock();
    int count = 0;
    for(unsigned i = 0; i < cells.size(); i++) {
        LibCellInfo *lib_cell_info = getLibCellInfo(cells[i]);
        if(lib_cell_info == NULL || cells[i].isDontTouch)
            continue;
        if(!PT_FULL_UPDATE && !cells[i].isChanged)
            continue;
        count++;
    }
    ecc::logPrintf("Update PT sizes changed count %d\n", count);
    if(count > 0) {
        double begin = cpuTime();
        applyOpenStaDbChanges([&]() {
            for(unsigned i = 0; i < cells.size(); i++) {
                LibCellInfo *lib_cell_info = getLibCellInfo(cells[i]);
                if(lib_cell_info == NULL || cells[i].isDontTouch)
                    continue;
                if(!PT_FULL_UPDATE && !cells[i].isChanged)
                    continue;
                auto inst = block->findInst(cells[i].name.c_str());
                assert(inst);
                auto new_master = db->findMaster(cells[i].type.c_str());
                if(new_master) {
                    inst->swapMaster(new_master);
                    cells[i].isChanged = 0;
                    cells[i].isStaticChanged = true;
                }
                else {
                    string type = inst->getMaster()->getName();
                    ecc::logError() << "Error: cannot find master " << cells[i].type
                              << " for cell " << cells[i].name
                              << ", current master is " << type << std::endl;
                    cells[i].type = type;
                    cells[i].isChanged = 0;
                    cells[i].isStaticChanged = true;
                }
            }
        });
        T[0]->pt_time += cpuTime() - begin;
    }

    // else ecc::logInfo() << "No cell has been changed." << endl;
}

void Sizer::CheckTriSizes(string opt_str) {
    string filename = benchname + "." + opt_str + ".sizes";
    vector< pair< string, string > > sizes = readSizes(filename);

    vector< bool > checked;
    checked.resize(numcells);

    for(unsigned i = 0; i < numcells; ++i) {
        checked[i] = false;
    }
    for(unsigned i = 0; i < sizes.size(); ++i) {
        string cellName = sizes[i].first;
        string cellType = sizes[i].second;
        checked[cell2id[cellName]] = true;
        if(cells[cell2id[cellName]].type != cellType) {
            ecc::logInfo() << "Error -- cell size mismatch: " << cellName << " "
                 << cells[cell2id[cellName]].type << "(File: " << cellType
                 << ")" << endl;
        };
    }
    for(unsigned i = 0; i < numcells; ++i) {
        if(!checked[i]) {
            ecc::logInfo() << "Error -- no cell size info from the file: "
                 << cells[i].name << " " << cells[i].type << endl;
        }
    }
}

void Sizer::CheckPTSizes(unsigned option) {
    T = PTimer[option];

    string filename = "pt.sizes";
    T[0]->checkSize(filename);

    vector< pair< string, string > > sizes = readSizes(filename);

    vector< bool > checked;
    checked.resize(numcells);

    for(unsigned i = 0; i < numcells; ++i) {
        checked[i] = false;
    }
    for(unsigned i = 0; i < sizes.size(); ++i) {
        string cellName = sizes[i].first;
        string cellType = sizes[i].second;
        checked[cell2id[cellName]] = true;
        if(cells[cell2id[cellName]].type != cellType) {
            ecc::logInfo() << "Error -- cell size mismatch: " << cellName << " "
                 << cells[cell2id[cellName]].type << "(Timer: " << cellType
                 << ")" << endl;
        };
    }
    for(unsigned i = 0; i < numcells; ++i) {
        if(!checked[i]) {
            ecc::logInfo() << "Error -- no cell size info from timer: " << cells[i].name
                 << " " << cells[i].type << endl;
        }
    }
}

// void Sizer::InitPTSizes() {
//     string filename = benchname + "_init_sizes.tcl";
//     ofstream outsz(filename.c_str());
//     int count = 0;
//     for(unsigned i = 0; i < numcells; i++) {
//         if(getLibCellInfo(g_cells[i]) == NULL)
//             continue;
//         if(!PT_FULL_UPDATE && !g_cells[i].isChanged)
//             continue;
//         LibCellInfo *lib_cell_info = getLibCellInfo(g_cells[i]);
//         if(useOpenSTA) {
//             outsz << "OSSizeCell " << g_cells[i].name << " "
//                   << lib_cell_info->name << endl;
//         }
//         else if(!useETS) {
//             outsz << "PtSizeCell " << g_cells[i].name << " "
//                   << lib_cell_info->name << endl;
//         }
//         else {
//             outsz << "EtsSizeCell " << g_cells[i].name << " "
//                   << lib_cell_info->name << endl;
//         }
//         g_cells[i].isChanged = false;
//         count++;
//     }
//     outsz.close();
//     if(count > 0) {
//         for(unsigned view = 0; view < numViews; ++view) {
//             T[view]->updateSize(filename);
//         }
//     }
// }
bool Sizer::replaceCell(odb::dbInst *dinst, odb::dbMaster *new_master,
                        std::unordered_set< odb::dbNet * > &parasitics_invalid_) {
    using odb::dbInst;
    using odb::dbMaster;
    dbMaster *replacement_master = new_master;
    auto sta_ = _ckt->_ord_timing->getSta();
    auto db_network_ = sta_->getDbNetwork();
    auto inst = db_network_->dbToSta(dinst);
    auto opendp_ = ord::OpenRoad::openRoad()->getOpendp();
    dbMaster *master = dinst->getMaster();
    if(replacement_master) {
        // dbInst *dinst = db_network_->staToDb(inst);

        dinst->swapMaster(replacement_master);
        // sta_->replaceCell(inst, replacement_cell1);
        // designAreaIncr(area(replacement_master));

        // Legalize the position of the instance in case it leaves the die
        // opendp_->legalCellPos(dinst);
        // if(parasitics_src_ == ParasiticsSrc::global_routing) {
        // }

        // auto *pin_iter = db_network_->pinIterator(inst);
        for(auto item : dinst->getITerms()) {
            auto item_net = item->getNet();
            if(!item_net || item_net->getSigType() == odb::dbSigType::POWER ||
               item_net->getSigType() == odb::dbSigType::GROUND) {
                continue;
            }
            parasitics_invalid_.insert(item_net);
        }
        return true;
    }
    return false;
}

void Sizer::UpdatePTSizes(unsigned option, int &count) {
    //    double begin=cpuTime();
    //    for(unsigned i=0 ; i<numcells ; i++) {
    //        if(isff(cells[i])) continue;
    //        //ecc::logInfo() << "Update sizing: " << cells[i].name << "->" <<
    //        cells[i].type << endl;
    //        T[view]->sizeCell(cells[i].name, cells[i].type);
    //    }
    // std::ostringstream ostr;
    // ostr.str("");
    // ostr << option;

    ecc::logError() << "Update PT sizes... " << std::endl;
    ecc::logError() << "numcell = " << numcells << std::endl;
    auto db = ord::OpenRoad::openRoad()->getDb();
    auto block = _ckt->_ord_design->getBlock();
    count = 0;
    for(unsigned i = 0; i < numcells; i++) {
        LibCellInfo *lib_cell_info = getLibCellInfo(cells[i]);
        if(lib_cell_info == NULL || cells[i].isDontTouch)
            continue;
        if(!PT_FULL_UPDATE && !cells[i].isChanged)
            continue;
        count++;
    }
    auto sta_ = _ckt->_ord_timing->getSta();
    ecc::logPrintf("Update PT sizes changed count %d\n", count);
    if(count > 0) {
        double begin = cpuTime();
        applyOpenStaDbChanges([&]() {
            for(unsigned i = 0; i < numcells; i++) {
                LibCellInfo *lib_cell_info = getLibCellInfo(cells[i]);
                if(lib_cell_info == NULL || cells[i].isDontTouch)
                    continue;
                if(!PT_FULL_UPDATE && !cells[i].isChanged)
                    continue;
                auto inst = block->findInst(cells[i].name.c_str());
                assert(inst);
                auto new_master = db->findMaster(cells[i].type.c_str());
                if(new_master == NULL) {
                    string type = inst->getMaster()->getName();
                    ecc::logError() << "Error: cannot find master " << cells[i].type
                              << " for cell " << cells[i].name
                              << ", current master is " << type << std::endl;
                    cells[i].type = type;
                    cells[i].isChanged = 0;
                    cells[i].isStaticChanged = true;
                }
                else {
                    inst->swapMaster(new_master);
                    cells[i].isChanged = 0;
                    cells[i].isStaticChanged = true;
                }
            }
        });
        T[0]->pt_time += cpuTime() - begin;
    }
    // else ecc::logInfo() << "No cell has been changed." << endl;
}

void Sizer::wait(int seconds) {
    clock_t endwait;
    endwait = clock() + seconds * CLOCKS_PER_SEC;
    while(clock() < endwait) {
    }
}

void Sizer::exePTServerOne(int port, unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    unsigned mode = mmmcViewList[view].mode;

    if(numViews > 1) {
        if(mmmcScrList[view] != "") {
            ptScriptFile = mmmcScrList[view];
        }
    }
    bool ptsiOn = false;
    bool ptpxOff = true;
    string ptOption = "";
    std::ostringstream ostr;
    ostr.str("");
    ostr << port;
    string serverTcl = "ptserver." + ostr.str() + ".tcl";
    std::ostringstream ostr2;
    ostr2.str("");
    ostr2 << view;
    string ptDesignTcl = "pt." + benchname + ostr2.str() + ".tcl";
    string serverCmdTcl = "sizer.tbc";
    ofstream fout;

    string ptsiStr = ptsiOn ? "-keep_capacitive_coupling " : "";

    // make ptserver tcl
    fout.open(serverTcl.c_str());

    fout << "package ifneeded tbcload 1.7 [list load [file join . "
            "libtbcload1.7.so]]"
         << endl;
    fout << "source " << serverCmdTcl << endl;
    fout << "source " << ptDesignTcl << endl;
    fout << "set svcPort " << port << endl;
    fout << "proc doService {sock msg} {" << endl;
    fout << "    puts \"got command $msg on ptserver\"" << endl;
    fout << "    puts $sock [eval \"$msg\"]" << endl;
    fout << "    flush $sock" << endl;
    fout << "}" << endl;
    fout << "proc  svcHandler {sock} {" << endl;
    fout << "  set l [gets $sock]" << endl;
    fout << "  if {[eof $sock]} {" << endl;
    fout << "     close $sock " << endl;
    fout << "  } else {" << endl;
    fout << "    doService $sock $l" << endl;
    fout << "  }" << endl;
    fout << "}" << endl;
    fout << "proc accept {sock addr port} {" << endl;
    fout << "  fileevent $sock readable [list svcHandler $sock]" << endl;
    fout << "  fconfigure $sock -buffering line -blocking 0" << endl;
    fout << "  puts $sock \"$addr:$port, You are connected to the pt server.\""
         << endl;
    fout << "  puts $sock \"It is now [exec date]\"" << endl;
    // fout << "  puts! \"Accepted connection from $addr at [exec date]\"" <<
    // endl;
    fout << "}" << endl;
    fout << "set hostname [info hostname]" << endl;
    fout << "set outFp [open hostname-${svcPort}.tmp \"w\"]" << endl;
    fout << "puts $outFp $hostname" << endl;
    fout << "close $outFp" << endl;
    fout << "catch { socket -server accept $svcPort } test" << endl;
    fout << "set outFp [open socket-${svcPort}.tmp \"w\"]" << endl;
    fout << "puts $outFp $test" << endl;
    fout << "close $outFp" << endl;
    fout << "if { [regexp {couldn't} $test] || [regexp {^Error} $test] } {"
         << endl;
    fout << "    [exit]" << endl;
    fout << "}" << endl;
    fout << "vwait events" << endl;

    fout.close();

    fout.open(ptDesignTcl.c_str());

    ifstream fin;
    if(ptScriptFile != "") {
        fin.open(ptScriptFile.c_str());
    }

    if(ptScriptFile == "" || !fin.is_open()) {
        fout << "package ifneeded tbcload 1.7 [list load [file join . "
                "libtbcload1.7.so]]"
             << endl;
        fout << "source ./sizer.tbc" << endl;
        fout << "set design  \"" << benchname << "\"" << endl;
        fout << "set lib_path \". " << dbLibPath << "\"" << endl;
        fout << "set lib_file_list \[\]" << endl;
        string lib_file_name;
        vector< string > lib_name_list;
        for(int i = 0; i < envlibLibs.size(); i++) {
            if(libLibPath == "") {
                lib_file_name = envlibLibs[i];
            }
            else {
                lib_file_name = libLibPath + "/" + envlibLibs[i];
            }

            fout << "lappend lib_file_list " << lib_file_name << endl;
            ifstream temp_is(lib_file_name.c_str());
            string lib_name = _ckt->read_lib_name(temp_is);
            temp_is.close();
            lib_name_list.push_back(lib_name);
        }
        if(!mmmcOn) {
            for(int i = 0; i < libLibs.size(); i++) {
                if(libLibPath == "") {
                    lib_file_name = libLibs[i];
                }
                else {
                    lib_file_name = libLibPath + "/" + libLibs[i];
                }
                fout << "lappend lib_file_list " << lib_file_name << endl;
                ifstream temp_is(lib_file_name.c_str());
                string lib_name = _ckt->read_lib_name(temp_is);
                temp_is.close();
                lib_name_list.push_back(lib_name);
            }
        }
        else {
            for(unsigned j = 0; j < mmmcLibLists[corner].size(); ++j) {
                if(libLibPath != "") {
                    lib_file_name = libLibPath + "/" + mmmcLibLists[corner][j];
                }
                else {
                    lib_file_name = mmmcLibLists[corner][j];
                }
                fout << "lappend lib_file_list " << lib_file_name << endl;
                ifstream temp_is(lib_file_name.c_str());
                string lib_name = _ckt->read_lib_name(temp_is);
                temp_is.close();
                lib_name_list.push_back(lib_name);
            }
        }

        fout << "set lib_list [list *";
        for(unsigned i = 0; i < lib_name_list.size(); ++i) {
            fout << " " << lib_name_list[i];
        }
        fout << "]" << endl;
        fout << "set verilog_input " << verilogFile << endl;

        if(timerSdcFile == "") {
            timerSdcFile = sdcFile;
        }

        if(!mmmcOn)
            fout << "set sdc " << timerSdcFile << endl;
        else
            fout << "set sdc " << mmmcSdcList[mode] << endl;

        if(!noSPEF) {
            if(!mmmcOn)
                fout << "set spef " << ptsiStr << spefFile << endl;
            else
                fout << "set spef " << ptsiStr << mmmcSpefList[corner] << endl;
        }
        fout << "PtLoadDesign" << endl;
    }
    else {
        string temp;
        while(std::getline(fin, temp)) {
            fout << temp << endl;
        }
        fin.close();
    }
    fout.close();

    string var = "PRIMETIME_EXECUTABLE";
    const char *val = ::getenv(var.c_str());
    if(val != 0)
        ecc::logInfo() << var << ": " << val << endl;
    else
        val = "pt_shell";

    string launchFile = "launch_pt." + ostr.str() + ".cmd";
    fout.open(launchFile.c_str());

    if(ptLaunchScriptFile != "") {
        fin.open(ptLaunchScriptFile.c_str());
    }

    if(ptLaunchScriptFile == "" || !fin.is_open()) {
        if(ptLogSave) {
            if(BACKGROUND)
                fout << val << " " << ptOption << " -f " << serverTcl << " > pt"
                     << port << ".log" << endl;
            else
                fout << val << " " << ptOption << " -f " << serverTcl
                     << " |& tee pt" << port << ".log" << endl;
        }
        else
            fout << val << " " << ptOption << " -f " << serverTcl << endl;
    }
    else {
        string temp;
        while(std::getline(fin, temp)) {
            fout << temp << endl;
        }
        fin.close();
    }
    fout.close();

    // Execute PrimeTime
    char Commands[250];
    sprintf(Commands, "chmod +x launch_pt.%d.cmd", port);
    system(Commands);

    if(!BACKGROUND) {
        sprintf(Commands,
                "xterm -T SizerPT%d -iconic -e csh -f launch_pt.%d.cmd &", port,
                port);
    }
    else {
        sprintf(Commands, "./launch_pt.%d.cmd &", port);
    }
    system(Commands);
}

void Sizer::exeETSServerOne(int port, unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    unsigned mode = mmmcViewList[view].mode;
    std::ostringstream ostr;
    ostr.str("");
    ostr << port;
    string serverTcl = "etsserver." + ostr.str() + ".tcl";
    std::ostringstream ostr2;
    ostr2.str("");
    ostr2 << view;
    string ptDesignTcl = "ets." + benchname + ostr2.str() + ".tcl";
    // string serverCmdTcl = "ptserver_cmds.tcl";
    string serverCmdTcl = "sizer.tbc";
    ofstream fout;

    // make soceserver tcl
    fout.open(serverTcl.c_str());

    fout << "package ifneeded tbcload 1.7 [list load [file join . "
            "libtbcload1.7.so]]"
         << endl;
    fout << "source " << serverCmdTcl << endl;
    fout << "source " << ptDesignTcl << endl;
    fout << "set svcPort " << port << endl;
    fout << "proc doService {sock msg} {" << endl;
    fout << "    puts \"got command $msg on ptserver\"" << endl;
    fout << "    puts $sock [eval \"$msg\"]" << endl;
    fout << "    flush $sock" << endl;
    fout << "}" << endl;
    fout << "proc  svcHandler {sock} {" << endl;
    fout << "  set l [gets $sock]" << endl;
    fout << "  if {[eof $sock]} {" << endl;
    fout << "     close $sock " << endl;
    fout << "  } else {" << endl;
    fout << "    doService $sock $l" << endl;
    fout << "  }" << endl;
    fout << "}" << endl;
    fout << "proc accept {sock addr port} {" << endl;
    fout << "  fileevent $sock readable [list svcHandler $sock]" << endl;
    fout << "  fconfigure $sock -buffering line -blocking 0" << endl;
    fout << "  puts $sock \"$addr:$port, You are connected to the pt server.\""
         << endl;
    fout << "  puts $sock \"It is now [exec date]\"" << endl;
    // fout << "  puts! \"Accepted connection from $addr at [exec date]\"" <<
    // endl;
    fout << "}" << endl;
    fout << "set hostname [info hostname]" << endl;
    fout << "set outFp [open hostname-${svcPort}.tmp \"w\"]" << endl;
    fout << "puts $outFp $hostname" << endl;

    fout << "close $outFp" << endl;
    fout << "catch { socket -server accept $svcPort } test" << endl;
    fout << "set outFp [open socket-${svcPort}.tmp \"w\"]" << endl;
    fout << "puts $outFp $test" << endl;
    fout << "close $outFp" << endl;
    fout << "if { [regexp {couldn't} $test] || [regexp {^Error} $test] } {"
         << endl;
    fout << "    [exit]" << endl;
    fout << "}" << endl;
    fout << "vwait events" << endl;
    fout.close();

    fout.open(ptDesignTcl.c_str());

    ifstream fin;
    if(numViews > 1) {
        if(mmmcScrList[view] != "") {
            etsScriptFile = mmmcScrList[view];
        }
    }
    if(etsScriptFile != "") {
        fin.open(etsScriptFile.c_str());
    }

    if(etsScriptFile == "" || !fin.is_open()) {
        string lib_file_name;
        fout << "set design  \"" << benchname << "\"" << endl;
        fout << "set lib_file_list \[\]" << endl;
        for(int i = 0; i < envlibLibs.size(); i++) {
            if(libLibPath == "") {
                lib_file_name = envlibLibs[i];
            }
            else {
                lib_file_name = libLibPath + "/" + envlibLibs[i];
            }
            fout << "lappend lib_file_list " << lib_file_name << endl;
        }
        if(!mmmcOn) {
            for(int i = 0; i < libLibs.size(); i++) {
                if(libLibPath == "") {
                    lib_file_name = libLibs[i];
                }
                else {
                    lib_file_name = libLibPath + "/" + libLibs[i];
                }
                fout << "lappend lib_file_list " << lib_file_name << endl;
            }
        }
        else {
            for(unsigned j = 0; j < mmmcLibLists[corner].size(); ++j) {
                if(libLibPath != "") {
                    lib_file_name = libLibPath + "/" + mmmcLibLists[corner][j];
                }
                else {
                    lib_file_name = mmmcLibLists[corner][j];
                }
                fout << "lappend lib_file_list " << lib_file_name << endl;
            }
        }
        fout << "set verilog_input " << verilogFile << endl;

        if(timerSdcFile == "") {
            timerSdcFile = sdcFile;
        }

        if(!mmmcOn) {
            fout << "set sdc " << timerSdcFile << endl;
        }
        else {
            fout << "set sdc " << mmmcSdcList[mode] << endl;
        }

        if(!noSPEF) {
            if(!mmmcOn) {
                fout << "set spef " << spefFile << endl;
            }
            else {
                fout << "set spef " << mmmcSpefList[corner] << endl;
            }
        }
        if(tcfFile != "") {
            fout << "set tcf " << tcfFile << endl;
        }
        if(falsePathFile != "") {
            fout << "set false_path_file " << falsePathFile << endl;
        }
        fout << "EtsLoadDesign" << endl;
    }
    else {
        string temp;
        while(std::getline(fin, temp)) {
            fout << temp << endl;
        }
        fin.close();
    }
    fout.close();

    // ofstream fout;
    string launchFile = "launch_ets." + ostr.str() + ".cmd";
    string var = "ETS_EXECUTABLE";
    const char *etsCmd = ::getenv(var.c_str());
    if(etsCmd != 0)
        ecc::logInfo() << var << ": " << etsCmd << endl;
    else
        etsCmd = "ets";

    if(useTempus)
        etsCmd = "tempus";

    ofstream fout2;
    fout2.open(launchFile.c_str());
    if(ptLogSave)
        fout2 << etsCmd << " " << ptOption << " -nowin -init " << serverTcl
              << " -log ets.log" << endl;
    else
        fout2 << etsCmd << " " << ptOption << " -nowin -init " << serverTcl
              << endl;
    if(VERBOSE >= 1)
        ecc::logInfo() << "File write done" << endl;
    fout2.close();
    if(VERBOSE >= 1)
        ecc::logInfo() << "Launch file generated" << endl;

    //    if ( VERBOSE == 100 ) {
    //        ecc::logInfo() << "Waiting" ;
    //        while ( i++ < 30 ) {
    //            ecc::logInfo() << ".";
    //                wait (1);
    //        }
    //        ecc::logInfo() << endl;
    //    }
    //
    if(system(NULL)) {
        if(VERBOSE >= 1)
            ecc::logInfo() << "Processor is OK" << endl;
    }
    else {
        if(VERBOSE >= 1)
            ecc::logInfo() << "Processor is not responding!!" << endl;
    }

    if(VERBOSE >= 1)
        ecc::logInfo() << "System commands..." << endl;

    // Execute ETS
    char Commands[250];

    sprintf(Commands, "chmod +x ./%s", launchFile.c_str());

    if(VERBOSE >= 1)
        ecc::logInfo() << "RUN COMMAND....chmod +x " << launchFile.c_str() << endl;

    int ret;
    if(!(ret = system(Commands))) {
        if(VERBOSE >= 1)
            ecc::logInfo() << "Command : " << Commands << " is done sucessfully" << endl;
    }
    else {
        if(VERBOSE >= 1)
            ecc::logInfo() << "Command : " << Commands << " exited with code " << ret
                 << endl;
    }

    if(!BACKGROUND) {
        sprintf(Commands, "xterm -T SizerETS%d -iconic -e csh -f ./%s &", port,
                launchFile.c_str());
    }
    else {
        sprintf(Commands, "./%s &", launchFile.c_str());
    }

    if(VERBOSE >= 1)
        ecc::logInfo() << "RUN COMMAND.... xterm -T Sizer...." << endl;

    if(!(ret = system(Commands))) {
        if(VERBOSE >= 1)
            ecc::logInfo() << "Command : " << Commands << " is done sucessfully" << endl;
    }
    else {
        if(VERBOSE >= 1)
            ecc::logInfo() << "Command : " << Commands << " exited with code " << ret
                 << endl;
    }

    ecc::logInfo() << "Running ETS done" << endl;
}

void Sizer::exeOSServerOne(int port, unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    unsigned mode = mmmcViewList[view].mode;

    if(numViews > 1) {
        if(mmmcScrList[view] != "") {
            ptScriptFile = mmmcScrList[view];
        }
    }
    bool ptsiOn = false;
    bool ptpxOff = true;
    string ptOption = "";
    std::ostringstream ostr;
    ostr.str("");
    ostr << port;
    string serverTcl = "osserver." + ostr.str() + ".tcl";
    std::ostringstream ostr2;
    ostr2.str("");
    ostr2 << view;
    string ptDesignTcl = "os." + benchname + ostr2.str() + ".tcl";
    // string serverCmdTcl = "sizer.tbc";
    string serverCmdTcl = "sizer_os.tcl";
    ofstream fout;

    string ptsiStr = ptsiOn ? "-keep_capacitive_coupling " : "";

    // make ptserver tcl
    fout.open(serverTcl.c_str());

    // fout << "set ::env(TCL_INIT_DIR) /home/tool/tcl/tcl8.4.20/lib/tcl8.4" <<
    // endl;

    // fout << "package ifneeded tbcload 1.7 [list load [file join .
    // libtbcload1.7.so]]" << endl;
    fout << "source " << serverCmdTcl << endl;
    fout << "source " << ptDesignTcl << endl;
    fout << "set svcPort " << port << endl;
    fout << "proc doService {sock msg} {" << endl;
    fout << "    puts \"got command $msg on osserver\"" << endl;
    fout << "    puts $sock [eval \"$msg\"]" << endl;
    fout << "    flush $sock" << endl;
    fout << "}" << endl;
    fout << "proc  svcHandler {sock} {" << endl;
    fout << "  set l [gets $sock]" << endl;
    fout << "  if {[eof $sock]} {" << endl;
    fout << "     close $sock " << endl;
    fout << "  } else {" << endl;
    fout << "    doService $sock $l" << endl;
    fout << "  }" << endl;
    fout << "}" << endl;
    fout << "proc accept {sock addr port} {" << endl;
    fout << "  fileevent $sock readable [list svcHandler $sock]" << endl;
    fout << "  fconfigure $sock -buffering line -blocking 0" << endl;
    fout << "  puts $sock \"$addr:$port, You are connected to the os server.\""
         << endl;
    fout << "  puts $sock \"It is now [exec date]\"" << endl;
    // fout << "  puts! \"Accepted connection from $addr at [exec date]\"" <<
    // endl;
    fout << "}" << endl;
    fout << "set hostname [info hostname]" << endl;
    fout << "set outFp [open hostname-${svcPort}.tmp \"w\"]" << endl;
    fout << "puts $outFp $hostname" << endl;
    fout << "close $outFp" << endl;
    fout << "catch { socket -server accept $svcPort } test" << endl;
    fout << "set outFp [open socket-${svcPort}.tmp \"w\"]" << endl;
    fout << "puts $outFp $test" << endl;
    fout << "close $outFp" << endl;
    fout << "if { [regexp {couldn't} $test] || [regexp {^Error} $test] } {"
         << endl;
    fout << "    [exit]" << endl;
    fout << "}" << endl;
    fout << "vwait events" << endl;

    fout.close();

    fout.open(ptDesignTcl.c_str());

    ifstream fin;
    if(ptScriptFile != "") {
        fin.open(ptScriptFile.c_str());
    }

    if(ptScriptFile == "" || !fin.is_open()) {
        // fout << "package ifneeded tbcload 1.7 [list load [file join . "
        //         "libtbcload1.7.so]]"
        //      << endl;
        // fout << "source ./sizer.tbc" << endl;
        fout << "source ./sizer_os.tcl" << endl;
        fout << "set design  \"" << benchname << "\"" << endl;
        fout << "set lib_path \". " << dbLibPath << "\"" << endl;
        fout << "set lib_file_list \[\]" << endl;
        string lib_file_name;
        vector< string > lib_name_list;
        for(int i = 0; i < envlibLibs.size(); i++) {
            if(libLibPath == "") {
                lib_file_name = envlibLibs[i];
            }
            else {
                lib_file_name = libLibPath + "/" + envlibLibs[i];
            }

            fout << "lappend lib_file_list " << lib_file_name << endl;
            ifstream temp_is(lib_file_name.c_str());
            string lib_name = _ckt->read_lib_name(temp_is);
            temp_is.close();
            lib_name_list.push_back(lib_name);
        }
        if(!mmmcOn) {
            for(int i = 0; i < libLibs.size(); i++) {
                if(libLibPath == "") {
                    lib_file_name = libLibs[i];
                }
                else {
                    lib_file_name = libLibPath + "/" + libLibs[i];
                }
                fout << "lappend lib_file_list " << lib_file_name << endl;
                ifstream temp_is(lib_file_name.c_str());
                string lib_name = _ckt->read_lib_name(temp_is);
                temp_is.close();
                lib_name_list.push_back(lib_name);
            }
        }
        else {
            for(unsigned j = 0; j < mmmcLibLists[corner].size(); ++j) {
                if(libLibPath != "") {
                    lib_file_name = libLibPath + "/" + mmmcLibLists[corner][j];
                }
                else {
                    lib_file_name = mmmcLibLists[corner][j];
                }
                fout << "lappend lib_file_list " << lib_file_name << endl;
                ifstream temp_is(lib_file_name.c_str());
                string lib_name = _ckt->read_lib_name(temp_is);
                temp_is.close();
                lib_name_list.push_back(lib_name);
            }
        }

        fout << "set lib_list [list *";
        for(unsigned i = 0; i < lib_name_list.size(); ++i) {
            fout << " " << lib_name_list[i];
        }
        fout << "]" << endl;
        fout << "set verilog_input " << verilogFile << endl;

        if(timerSdcFile == "") {
            timerSdcFile = sdcFile;
        }

        if(!mmmcOn)
            fout << "set sdc " << timerSdcFile << endl;
        else
            fout << "set sdc " << mmmcSdcList[mode] << endl;

        if(!noSPEF) {
            if(!mmmcOn)
                fout << "set spef " << ptsiStr << spefFile << endl;
            else
                fout << "set spef " << ptsiStr << mmmcSpefList[corner] << endl;
        }
        fout << "OSLoadDesign" << endl;
    }
    else {
        string temp;
        while(std::getline(fin, temp)) {
            fout << temp << endl;
        }
        fin.close();
    }
    fout.close();

    string var = "OPENSTA_EXECUTABLE";
    const char *val = ::getenv(var.c_str());

    string launchFile = "launch_os." + ostr.str() + ".cmd";
    fout.open(launchFile.c_str());
    if(val != 0)
        ecc::logInfo() << var << ": " << val << endl;
    else {
        fout << "OPENSTA_EXECUTABLE=$(pwd)/sta" << endl;
        val = "${OPENSTA_EXECUTABLE}";
    }
    if(ptLaunchScriptFile != "") {
        fin.open(ptLaunchScriptFile.c_str());
    }
    if(ptLaunchScriptFile == "" || !fin.is_open()) {
        if(ptLogSave) {
            if(BACKGROUND)
                fout << val << " " << ptOption << " -f " << serverTcl << " > os"
                     << port << ".log" << endl;
            else
                fout << val << " " << ptOption << " -f " << serverTcl
                     << " |& tee os" << port << ".log" << endl;
        }
        else
            fout << val << " " << ptOption << " -f " << serverTcl << endl;
    }
    else {
        string temp;
        while(std::getline(fin, temp)) {
            fout << temp << endl;
        }
        fin.close();
    }
    fout.close();

    // Execute PrimeTime
    char Commands[250];
    // sprintf(Commands, "chmod +x launch_os.%d.cmd", port);
    // system(Commands);

    if(!BACKGROUND) {
        sprintf(Commands,
                "xterm -T SizerOpenSTA%d -iconic -e csh -f launch_os.%d.cmd &",
                port, port);
    }
    else {
        sprintf(Commands, "./launch_os.%d.cmd &", port);
    }
    // system(Commands);
}

void Sizer::exePTServer(string &serverName, int &port, unsigned view) {
    char socketFile[250];
    char hostnameFile[250];
    char Commands[250];
    bool flag;
    ifstream file;
    string line;

    while(true) {
        flag = false;
        if(port > 9999) {
            ecc::logInfo() << "Error: cannot open PT socket" << endl;
            exit(0);
        }
        std::ostringstream ostr;
        ostr.str("");
        ostr << port;
        exePTServerOne(port, view);
        bool flag2 = true;
        sprintf(socketFile, "socket-%d.tmp", port);
        sprintf(hostnameFile, "hostname-%d.tmp", port);
        int tmp_cnt = 0;
        ecc::logInfo() << "waiting for socket file ";
        while(flag2) {
            ifstream fp;
            wait(1);
            fp.open(socketFile);
            if(fp)
                flag2 = false;
            fp.close();
            ecc::logInfo() << ".";
            if(tmp_cnt++ > max(200, (int)numcells))
                break;
        }
        ecc::logInfo() << endl;
        file.open(socketFile);
        ecc::logInfo() << "open socket file: " << socketFile << endl;
        if(!file) {
            ecc::logInfo() << "Error: socket file not found" << endl;
        }
        while(std::getline(file, line)) {
            if(line.find("Error") != string::npos)
                flag = true;
        }
        file.close();
        if(!flag)
            break;
        port++;
    }
    line = "";

    file.open(hostnameFile);
    if(file) {
        while(std::getline(file, line)) {
            if(line != "") {
                serverName = line;
            }
        }
    }
    file.close();
    ecc::logInfo() << "PT socket is opened at " << serverName << ":" << port << endl;
    // sprintf(Commands, "\\rm socket-*.tmp" );
    sprintf(Commands, "\\rm -f %s", socketFile);
    system(Commands);
}

void Sizer::exeETSServer(string &serverName, int &port, unsigned view) {
    char socketFile[250];
    char hostnameFile[250];
    char Commands[250];
    bool flag;
    ifstream file;
    string line;

    while(true) {
        flag = false;
        if(port > 9999) {
            ecc::logInfo() << "Error: cannot open ETS socket" << endl;
            exit(0);
        }
        exeETSServerOne(port, view);
        bool flag2 = true;
        sprintf(socketFile, "socket-%d.tmp", port);
        sprintf(hostnameFile, "hostname-%d.tmp", port);
        int tmp_cnt = 0;
        ecc::logInfo() << "waiting for socket file " << endl;
        while(flag2) {
            ifstream fp;
            wait(1);
            fp.open(socketFile);
            if(fp)
                flag2 = false;
            fp.close();
            ecc::logInfo() << ".";
            if(tmp_cnt++ > max(200, (int)numcells))
                break;
        }
        ecc::logInfo() << endl;
        file.open(socketFile);
        ecc::logInfo() << "open socket file: " << socketFile << endl;
        if(!file) {
            ecc::logInfo() << "Error: socket file not found" << endl;
            flag = true;
        }
        else {
            while(std::getline(file, line)) {
                if(line.find("Error") != string::npos)
                    flag = true;
                if(line.find("couldn") != string::npos)
                    flag = true;
            }
        }
        file.close();
        if(!flag)
            break;
        port++;
    }
    line = "";

    file.open(hostnameFile);
    if(file) {
        while(std::getline(file, line)) {
            if(line != "") {
                serverName = line;
            }
        }
    }
    file.close();
    ecc::logInfo() << "ETS socket is opened at " << serverName << ":" << port << endl;
    // sprintf(Commands, "\\rm socket-*.tmp" );
    if(NO_LOG) {
        sprintf(Commands, "\\rm -f %s", socketFile);
        system(Commands);
    }
}

void Sizer::exeOSServer(string &serverName, int &port, unsigned view) {
    char socketFile[250];
    char hostnameFile[250];
    char Commands[250];
    bool flag;
    ifstream file;
    string line;

    while(true) {
        flag = false;
        if(port > 9999) {
            ecc::logInfo() << "Error: cannot open OpenSTA socket" << endl;
            exit(0);
        }
        std::ostringstream ostr;
        ostr.str("");
        ostr << port;
        exeOSServerOne(port, view);
        bool flag2 = true;
        sprintf(socketFile, "socket-%d.tmp", port);
        sprintf(hostnameFile, "hostname-%d.tmp", port);
        int tmp_cnt = 0;
        ecc::logInfo() << "waiting for socket file ";
        while(flag2) {
            ifstream fp;
            wait(1);
            fp.open(socketFile);
            if(fp)
                flag2 = false;
            fp.close();
            ecc::logInfo() << ".";
            if(tmp_cnt++ > max(200, (int)numcells))
                break;
        }
        ecc::logInfo() << endl;
        file.open(socketFile);
        ecc::logInfo() << "open socket file: " << socketFile << endl;
        if(!file) {
            ecc::logInfo() << "Error: socket file not found" << endl;
        }
        while(std::getline(file, line)) {
            if(line.find("Error") != string::npos)
                flag = true;
        }
        file.close();
        if(!flag)
            break;
        port++;
    }
    line = "";

    file.open(hostnameFile);
    if(file) {
        while(std::getline(file, line)) {
            if(line != "") {
                serverName = line;
            }
        }
    }
    file.close();
    ecc::logInfo() << "OpenSTA socket is opened at " << serverName << ":" << port << endl;
    // sprintf(Commands, "\\rm socket-*.tmp" );
    sprintf(Commands, "\\rm -f %s", socketFile);
    system(Commands);
}

void Sizer::SizeOut(bool success) {
    double begin = cpuTime();
    ecc::logInfo() << endl << "Saving final sizes... " << benchname + ".size" << endl;
    string filename = benchname + ".size";
    if(!success)
        filename = benchname + ".int.size";
    ofstream outsz(filename.c_str());
    int changed = 0;
    for(unsigned i = 0; i < numcells; i++) {
        LibCellInfo *lib_cell_info = getLibCellInfo(cells[i]);
        if(lib_cell_info != NULL && lib_cell_info->name != init_sizes[i]) {
            changed++;
        }
        if(lib_cell_info != NULL) {  //&& lib_cell_info->name != init_sizes[i]
            outsz << cells[i].name << " " << lib_cell_info->name << endl;
            // ecc::logInfo() << cells[i].name << " "<<lib_cell_info->name<<endl;
        }
    }
    ecc::logPrintf("Save out----------- %d cells were changed------------\n", changed);
    outsz.close();
    time_SizeOut = cpuTime() - begin;
}

void Sizer::SizeOut(string path, vector< CELL > &c) {
    string filename = path + "/" + benchname + ".size";
    ecc::logInfo() << endl << "Saving final sizes... " << filename << endl;
    ofstream outsz(filename.c_str());
    int changed = 0;
    for(unsigned i = 0; i < c.size(); i++) {
        LibCellInfo *lib_cell_info = getLibCellInfo(c[i]);
        if(lib_cell_info != NULL && lib_cell_info->name != init_sizes[i]) {
            changed++;
        }
        if(lib_cell_info != NULL) {
            outsz << c[i].name << " " << lib_cell_info->name << endl;
            // ecc::logInfo() << c[i].name << " "<<lib_cell_info->name<<endl;
        }
    }
    ecc::logPrintf("Save out----------- %d cells were changed------------\n", changed);
    outsz.close();
}

void Sizer::SizeChangeOut(string path, vector< CELL > &c) {
    string filename = path + "/" + benchname + ".size";
    ecc::logInfo() << endl << "Saving final sizes... " << filename << endl;
    ofstream outsz(filename.c_str());
    int changed = 0;
    for(unsigned i = 0; i < c.size(); i++) {
        LibCellInfo *lib_cell_info = getLibCellInfo(c[i]);
        if(lib_cell_info != NULL && lib_cell_info->name != init_sizes[i]) {
            changed++;
        }
        if(lib_cell_info != NULL && lib_cell_info->name != init_sizes[i]) {
            outsz << c[i].name << " " << lib_cell_info->name << endl;
            // ecc::logInfo() << c[i].name << " "<<lib_cell_info->name<<endl;
        }
    }
    ecc::logPrintf("Save out----------- %d cells were changed------------\n", changed);
    outsz.close();
}

void Sizer::SizeOut(string path) {
    string filename = path + "/" + benchname + ".size";
    ecc::logInfo() << endl << "Saving final sizes... " << filename << endl;
    ofstream outsz(filename.c_str());
    int changed = 0;
    for(unsigned i = 0; i < numcells; i++) {
        LibCellInfo *lib_cell_info = getLibCellInfo(cells[i]);
        if(lib_cell_info != NULL && lib_cell_info->name != init_sizes[i]) {
            changed++;
        }
        if(lib_cell_info != NULL) {
            outsz << cells[i].name << " " << lib_cell_info->name << endl;
            // ecc::logInfo() << c[i].name << " "<<lib_cell_info->name<<endl;
        }
    }
    ecc::logPrintf("Save out----------- %d cells were changed------------\n", changed);
    outsz.close();
}

void Sizer::SizeTempOut(string option) {
    ecc::logInfo() << endl
         << "Saving final sizes... " << benchname + "." + option + ".sizes"
         << endl;
    string filename = benchname + "." + option + ".sizes";
    ofstream outsz(filename.c_str());
    int changed = 0;
    for(unsigned i = 0; i < numcells; i++) {
        LibCellInfo *lib_cell_info = getLibCellInfo(cells[i]);
        if(lib_cell_info != NULL && lib_cell_info->name != init_sizes[i]) {
            changed++;
        }
        if(lib_cell_info != NULL) {
            outsz << cells[i].name << " " << lib_cell_info->name << endl;
            // ecc::logInfo() << c[i].name << " "<<lib_cell_info->name<<endl;
        }
    }
    ecc::logPrintf("Save out----------- %d cells were changed------------\n", changed);
    outsz.close();
}

bool Sizer::SizeIn(string path) {
    string filename = path + "/" + benchname + ".size";
    ecc::logInfo() << endl << "Size in sizes... " << filename << endl;
    vector< pair< string, string > > sizes = readSizes(filename);
    if(sizes.size() == 0)
        return false;
    for(unsigned i = 0; i < sizes.size(); ++i) {
        string cellName = sizes[i].first;
        string cellType = sizes[i].second;
        cells[cell2id[cellName]].type = cellType;
        unordered_map< string, LibCellInfo >::iterator temp_iter =
            libs[0].find(cellType);
        LibCellInfo *lib_cell_info = &temp_iter->second;
        if(lib_cell_info != NULL) {
            cells[cell2id[cellName]].c_vtype = lib_cell_info->c_vtype;
            cells[cell2id[cellName]].c_size = lib_cell_info->c_size;
        }
        cells[cell2id[cellName]].isChanged = 1;
    }
    return true;
}

void Sizer::UDSizeIn(string filename) {
    ecc::logInfo() << "Reading sizes... " << filename << endl;
    vector< pair< string, string > > sizes = readSizes(filename);
    if(sizes.size() == 0)
        return;
    for(unsigned i = 0; i < sizes.size(); ++i) {
        string cellName = sizes[i].first;
        string cellType = sizes[i].second;
        cells[cell2id[cellName]].type = cellType;
        LibCellInfo *lib_cell_info = getLibCellInfo(cells[cell2id[cellName]]);
        cells[cell2id[cellName]].c_vtype = lib_cell_info->c_vtype;
        cells[cell2id[cellName]].c_size = lib_cell_info->c_size;
        cells[cell2id[cellName]].isChanged++;
    }
}

void Sizer::UDSizeIn(vector< CELL > &int_cells, string filename) {
    ecc::logInfo() << "Reading sizes... " << filename << endl;
    vector< pair< string, string > > sizes = readSizes(filename);
    if(sizes.size() == 0)
        return;
    for(unsigned i = 0; i < sizes.size(); ++i) {
        string cellName = sizes[i].first;
        string cellType = sizes[i].second;
        cells[cell2id[cellName]].type = cellType;
        LibCellInfo *lib_cell_info = getLibCellInfo(cells[cell2id[cellName]]);
        int_cells[cell2id[cellName]].c_vtype = lib_cell_info->c_vtype;
        int_cells[cell2id[cellName]].c_size = lib_cell_info->c_size;
        int_cells[cell2id[cellName]].isChanged++;
    }
}

void Sizer::SizeInit(string option) {
    ecc::logInfo() << "Reading sizes... " << option << endl;
    string filename = option;
    vector< pair< string, string > > sizes = readSizes(filename);
    if(sizes.size() == 0)
        return;
    for(unsigned i = 0; i < sizes.size(); ++i) {
        string cellName = sizes[i].first;
        string cellType = sizes[i].second;
        g_cells[cell2id[cellName]].type = cellType;
        LibCellInfo *lib_cell_info = getLibCellInfo(g_cells[cell2id[cellName]]);
        if(lib_cell_info != NULL) {
            g_cells[cell2id[cellName]].c_vtype = lib_cell_info->c_vtype;
            g_cells[cell2id[cellName]].c_size = lib_cell_info->c_size;
        }
        // ecc::logInfo() << "SIZE INIT " << g_cells[cell2id[cellName]].name  << " " <<
        // g_cells[cell2id[cellName]].type << endl;
        g_cells[cell2id[cellName]].isChanged++;
    }
}

void Sizer::SizeInfromPT(string filename) {
    ecc::logInfo() << "Reading sizes... " << filename << endl;
    vector< pair< string, string > > sizes = readSizesPT(filename);
    if(sizes.size() == 0)
        return;
    for(unsigned i = 0; i < sizes.size(); ++i) {
        string cellName = sizes[i].first;
        string cellType = sizes[i].second;

        if(cells[cell2id[cellName]].type != cellType) {
            cells[cell2id[cellName]].type = cellType;
            LibCellInfo *lib_cell_info =
                getLibCellInfo(cells[cell2id[cellName]]);
            if(lib_cell_info != NULL) {
                cells[cell2id[cellName]].c_vtype = lib_cell_info->c_vtype;
                cells[cell2id[cellName]].c_size = lib_cell_info->c_size;
            }
        }
    }
}

vector< pair< string, string > > Sizer::readSizes(const string &filename) {
    std::vector< std::pair< std::string, std::string > > sizes;
    // Open the file in read mode, exit if could not open
    ifstream infile(filename.c_str());
    if(!infile) {
        ecc::logInfo() << "-E-: readSizes: Could not read sizes from '" << filename << "'"
             << endl;
        return sizes;
        //    exit(1);
    }
    // Read the entire file
    string inst, size;
    while(!infile.eof()) {
        infile >> inst;
        if(infile.eof())
            break;
        infile >> size;
        if(infile.eof())
            break;
        sizes.push_back(make_pair(inst, size));
    }
    // Close the file
    infile.close();
    // Return the sizes vector
    return sizes;
}

vector< pair< string, string > > Sizer::readSizesPT(const string &filename) {
    std::vector< std::pair< std::string, std::string > > sizes;
    // Open the file in read mode, exit if could not open
    ifstream infile(filename.c_str());
    if(!infile) {
        ecc::logInfo() << "-E-: readSizes: Could not read sizes from '" << filename << "'"
             << endl;
        return sizes;
        //    exit(1);
    }
    // Read the entire file
    string tmp, cmd, inst, size;
    while(!infile.eof()) {
        infile >> cmd;
        if(cmd != "size_cell")
            continue;
        infile >> tmp;
        inst = tmp.substr(1, tmp.length() - 2);
        if(infile.eof())
            break;
        infile >> tmp;
        size = tmp.substr(1, tmp.length() - 2);
        if(infile.eof())
            break;
        // ecc::logInfo() << inst << " " << size << endl;
        sizes.push_back(make_pair(inst, size));
    }
    // Close the file
    infile.close();
    // Return the sizes vector
    return sizes;
}

double Sizer::ReportWithPT(vector< CELL > &c, string sizeout, double &wns_input,
                           double &power_input, unsigned view) {
    unsigned swap_count = 0;
    unsigned hvt_init_count = 0;
    unsigned hvt_count = 0;
    for(unsigned i = 0; i < numcells; ++i) {
        if(g_cells[i].c_vtype == s) {
            ++hvt_init_count;
        }
    }
    for(unsigned i = 0; i < numcells; ++i) {
        if(g_cells[i].type != c[i].type) {
            ++swap_count;
        }
        if(c[i].c_vtype == s) {
            ++hvt_count;
        }
    }

    double hvt_ratio = (double)hvt_count / (double)numcells;
    double hvt_init_ratio = (double)hvt_init_count / (double)numcells;

    // double current_leak_int = leakage_power;

    // InitPowerBeforeUpdate(c);
    UpdatePTSizes(c, 0);

    double current_leak = 0.0;
    double current_tot = 0.0;
    if(useOpenSTA) {
        // string find_timing = T[view]->doOneCmd("find_timing");
        current_leak = T[view]->getLeakPower();
        current_tot = current_leak;
    }
    else {
        current_leak = T[view]->getLeakPower();
        current_tot = T[view]->getTotPower();
    }

    double skew_violation = T[view]->getTNS(clk_name[worst_corner]);
    double wns = T[view]->getWorstSlack(clk_name[worst_corner]);
    double tran_tot, tran_max;
    tran_tot = tran_max = 0.0;
    int tran_num = 0;
    T[view]->getTranVio(tran_tot, tran_max, tran_num);

    double hold_wns = 0.0;
    double hold_tns = 0.0;

    if(HOLD_CHECK) {
        hold_wns = T[view]->getWorstSlackHold(clk_name[worst_corner]);
        hold_tns = T[view]->getTNSHold(clk_name[worst_corner]);
    }

    ecc::logInfo() << sizeout << "[" << view << "] WNS from Timer     : " << wns << endl;
    ecc::logInfo() << sizeout << "[" << view
         << "] TNS                : " << skew_violation << " ps" << endl;
    ecc::logInfo() << sizeout << "[" << view << "] Leakage Power      : " << current_leak
         << " (" << (current_leak / init_leak[view] - 1) * 100.0 << " % )"
         << endl;
    ecc::logInfo() << sizeout << "[" << view << "] Total Power        : " << current_tot
         << " (" << (current_tot / init_tot[view] - 1) * 100.0 << " % ) "
         << endl;
    ecc::logInfo() << sizeout << "[" << view << "] # Swap             : " << swap_count
         << endl;
    ecc::logInfo() << sizeout << "[" << view
         << "] % HVT cells        : " << hvt_ratio * 100.0 << endl;
    ecc::logInfo() << sizeout << "[" << view
         << "] % HVT cells (init) : " << hvt_init_ratio * 100.0 << endl;

    if(HOLD_CHECK) {
        ecc::logInfo() << sizeout << "[" << view << "] WNS hold           : " << hold_wns
             << endl;
        ecc::logInfo() << sizeout << "[" << view << "] TNS hold           : " << hold_tns
             << endl;
    }

    ecc::logInfo() << sizeout << " Cap. Vio.      : " << cap_violation
         << " fF,  #: " << cap_violation_cnt << " max: " << cap_violation_wst
         << endl;
    ecc::logInfo() << sizeout << " Slew. Vio.     : " << tran_tot
         << " ps,  #: " << tran_num << ", max: " << tran_max << endl;

    SizeOut("./", c);
    // SizeChangeOut(c, sizeout);
    wns_input = wns;
    if(ALPHA == 0.0) {
        power_input = current_leak;
    }
    else {
        power_input = current_tot;
    }

    return wns;
}

inline void Sizer::buildTargets(unsigned iter, unsigned STAGE, double RATIO,
                                double leak_exponent, double alpha,
                                double break_ratio, unsigned thread_id,
                                double toler, unsigned view) {
    for(unsigned i = 0; i < numcells; i++) {
        if(cells[i].isClockCell)
            continue;
        if(cells[i].isDontTouch)
            continue;
        if(isff(cells[i])) {
            continue;
        }
        bool attack = false;
        if(i % 100 == 0) {
            ecc::logPrintf("Attack %d/%d\n", i, numcells);
        }
        for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
            if(pins[view][cells[i].outpins[j]].rslk < toler ||
               pins[view][cells[i].outpins[j]].fslk < toler) {
                attack = true;
                break;
            }
        }

        if(attack) {
            if(r_type(cells[i]) == (numVt - 1) && isMax(cells[i]))
                continue;  // FIX : shouldn't it be checked first in loop_i?

            entry tmpEntry;
            tmpEntry.id = i;

            double npaths = 0.0;
            double totcaps = 0.0;

            for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
                npaths += pins[view][cells[i].outpins[j]].NPaths;
                totcaps += (double)pins[view][cells[i].outpins[j]].totcap;
            }

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
                        sf = CalSens(cells[i], k, 0, sensFuncT, leak_exponent,
                                     alpha, view);
                    }
                    else {
                        sf = CalSensMMMC(cells[i], k, 0, sensFuncT,
                                         leak_exponent, alpha, true);
                    }
                    // ecc::logInfo() << "UPSIZING SF " << sf << endl;
                    if(sf != 0.0) {
                        delta_impact_size.push_back(1.0 / sf);
                        size_step.push_back(k);
                    }
                }

                // upgrading
                if(isSwappable(cells[i], k)) {
                    double sf = 0.0;
                    if(!mmmcOn) {
                        sf = CalSens(cells[i], 0, k, sensFuncT, leak_exponent,
                                     alpha, view);
                    }
                    else {
                        sf = CalSensMMMC(cells[i], k, 0, sensFuncT,
                                         leak_exponent, alpha, true);
                    }
                    // ecc::logInfo() << "UPGRADING SF " << sf << endl;
                    if(sf != 0.0) {
                        delta_impact_type.push_back(1.0 / sf);
                        type_step.push_back(k);
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

            for(unsigned k = 0; k < delta_impact_type.size(); ++k) {
                if(max_delta_impact_type > delta_impact_type[k]) {
                    max_step_type = type_step[k];
                    max_delta_impact_type = delta_impact_type[k];
                }
            }

            tmpEntry.delta_impact = max_delta_impact_size;
            tmpEntry.step = max_step_size;
            tmpEntry.change = UPSIZE;

            if(tmpEntry.delta_impact < 0.0)
                targets.insert(tmpEntry);

            tmpEntry.delta_impact = max_delta_impact_type;
            tmpEntry.step = max_step_type;
            tmpEntry.change = UPTYPE;

            if(tmpEntry.delta_impact < 0.0)
                targets.insert(tmpEntry);

            // ecc::logInfo() << "DELTA IMPACT " << tmpEntry.delta_impact << endl;
        }
    }
    ecc::logInfo() << "# priority targets  : " << targets.size() << endl;

    // if no cells with a positive gain, simply insert top x% negative slack
    // cells.
    bool uniform_flag = false;
    if(targets.size() == 0 && STAGE == GLOBAL) {
        uniform_flag = true;

        for(unsigned i = 0; i < numcells; i++) {
            bool attack = false;
            if(cells[i].isClockCell)
                continue;
            if(cells[i].isDontTouch)
                continue;
            if(isff(cells[i])) {
                continue;
            }
            for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
                if(pins[view][cells[i].outpins[j]].rslk <
                       max_neg_rslk * RATIO * 0.01 ||
                   pins[view][cells[i].outpins[j]].fslk <
                       max_neg_fslk * RATIO * 0.01) {
                    if(pins[view][cells[i].outpins[j]].rslk >= toler &&
                       pins[view][cells[i].outpins[j]].fslk >= toler) {
                        continue;
                    }

                    attack = true;
                    break;
                }
            }

            if(attack) {
                entry tmpEntry;
                tmpEntry.delta_impact = GetCellSlack(cells[i], view);
                tmpEntry.id = i;

                double npaths = 0.0;
                double totcaps = 0.0;

                for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
                    npaths += pins[view][cells[i].outpins[j]].NPaths;
                    totcaps += (double)pins[view][cells[i].outpins[j]].totcap;
                }

                if(r_type(cells[i]) == (numVt - 1) && isMax(cells[i]))
                    continue;
                if(GTR_METRIC2 == SF5) {
                    if(NO_TOPO) {
                        tmpEntry.tie_break = i;
                    }
                    else
                        tmpEntry.tie_break = map2topoidx[i];
                }
                else if(GTR_METRIC2 == SF6) {
                    tmpEntry.tie_break = (double)r_size(cells[i]) / npaths;
                }
                else {
                    tmpEntry.tie_break = (double)r_size(cells[i]) / totcaps;
                }
                if(r_type(cells[i]) != (numVt - 1)) {
                    tmpEntry.change = UPTYPE;
                }
                else if(!isMax(cells[i])) {
                    tmpEntry.change = UPSIZE;
                }
                targets.insert(tmpEntry);
            }
        }
        ecc::logInfo() << "# uniform targets  : " << targets.size() << endl;
    }

    ecc::logError() << "Targets Building Finished. Target.size = " << targets.size()
              << std::endl;
    // if(targets.size() == 0) {
    //     // it's stuck. let's release some of cells
    //     // Release(false, CRIT_LEGALIZE);
    //     return 0;
    // }
}

// slack/slew optimization
unsigned Sizer::Attack(unsigned iter, unsigned STAGE, double RATIO,
                       double leak_exponent, double alpha, double break_ratio,
                       unsigned thread_id, double toler, unsigned view) {
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

                entry tmpEntry;
                tmpEntry.id = i;

                double npaths = 0.0;
                double totcaps = 0.0;

                for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
                    npaths += pins[view][cells[i].outpins[j]].NPaths;
                    totcaps += (double)pins[view][cells[i].outpins[j]].totcap;
                }

                if(GTR_METRIC2 == SF5) {
                    if(NO_TOPO) {
                        tmpEntry.tie_break = i;
                    }
                    else
                        tmpEntry.tie_break = map2topoidx[i];
                }
                else if(GTR_METRIC2 == SF6) {
                    tmpEntry.tie_break =
                        (double)(r_size(cells[i]) + 1) / npaths;
                }
                else {
                    tmpEntry.tie_break =
                        (double)(r_size(cells[i]) + 1) / totcaps;
                }

                vector< double > delta_impact_size, delta_impact_type;
                vector< int > size_step, type_step;

                for(int k = 1; k <= MULTI_STEP; ++k) {
                    // upsizing
                    if(isSizable(cells[i], k) && !cells[i].touched) {
                        double sf = 0.0;
                        if(!mmmcOn) {
                            sf = CalSens(cells[i], k, 0, sensFuncT,
                                         leak_exponent, alpha, view);
                        }
                        else {
                            sf = CalSensMMMC(cells[i], k, 0, sensFuncT,
                                             leak_exponent, alpha, true);
                        }
                        // ecc::logInfo() << "UPSIZING SF " << sf << endl;
                        if(sf != 0.0) {
                            delta_impact_size.push_back(1.0 / sf);
                            size_step.push_back(k);
                        }
                    }

                    // upgrading
                    if(isSwappable(cells[i], k)) {
                        double sf = 0.0;
                        if(!mmmcOn) {
                            sf = CalSens(cells[i], 0, k, sensFuncT,
                                         leak_exponent, alpha, view);
                        }
                        else {
                            sf = CalSensMMMC(cells[i], k, 0, sensFuncT,
                                             leak_exponent, alpha, true);
                        }
                        // ecc::logInfo() << "UPGRADING SF " << sf << endl;
                        if(sf != 0.0) {
                            delta_impact_type.push_back(1.0 / sf);
                            type_step.push_back(k);
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

                for(unsigned k = 0; k < delta_impact_type.size(); ++k) {
                    if(max_delta_impact_type > delta_impact_type[k]) {
                        max_step_type = type_step[k];
                        max_delta_impact_type = delta_impact_type[k];
                    }
                }

                tmpEntry.delta_impact = max_delta_impact_size;
                tmpEntry.step = max_step_size;
                tmpEntry.change = UPSIZE;

                if(tmpEntry.delta_impact < 0.0)
                    targets.insert(tmpEntry);

                tmpEntry.delta_impact = max_delta_impact_type;
                tmpEntry.step = max_step_type;
                tmpEntry.change = UPTYPE;

                if(tmpEntry.delta_impact < 0.0)
                    targets.insert(tmpEntry);

                // ecc::logInfo() << "DELTA IMPACT " << tmpEntry.delta_impact << endl;
            }
        }
        ecc::logInfo() << "# priority targets  : " << targets.size() << endl;

        // if no cells with a positive gain, simply insert top x% negative slack
        // cells.
        bool uniform_flag = false;
        if(false) {
            uniform_flag = true;

            for(unsigned i = 0; i < numcells; i++) {
                bool attack = false;
                if(cells[i].isClockCell)
                    continue;
                if(cells[i].isDontTouch)
                    continue;
                if(isff(cells[i])) {
                    continue;
                }
                for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
                    if(pins[view][cells[i].outpins[j]].rslk <
                           max_neg_rslk * RATIO * 0.01 ||
                       pins[view][cells[i].outpins[j]].fslk <
                           max_neg_fslk * RATIO * 0.01) {
                        if(pins[view][cells[i].outpins[j]].rslk >= toler &&
                           pins[view][cells[i].outpins[j]].fslk >= toler) {
                            continue;
                        }

                        attack = true;
                        break;
                    }
                }

                if(attack) {
                    entry tmpEntry;
                    tmpEntry.delta_impact = GetCellSlack(cells[i], view);
                    tmpEntry.id = i;

                    double npaths = 0.0;
                    double totcaps = 0.0;

                    for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
                        npaths += pins[view][cells[i].outpins[j]].NPaths;
                        totcaps +=
                            (double)pins[view][cells[i].outpins[j]].totcap;
                    }

                    if(r_type(cells[i]) == (numVt - 1) && isMax(cells[i]))
                        continue;
                    if(GTR_METRIC2 == SF5) {
                        if(NO_TOPO) {
                            tmpEntry.tie_break = i;
                        }
                        else
                            tmpEntry.tie_break = map2topoidx[i];
                    }
                    else if(GTR_METRIC2 == SF6) {
                        tmpEntry.tie_break = (double)r_size(cells[i]) / npaths;
                    }
                    else {
                        tmpEntry.tie_break = (double)r_size(cells[i]) / totcaps;
                    }
                    if(r_type(cells[i]) != (numVt - 1)) {
                        tmpEntry.change = UPTYPE;
                    }
                    else if(!isMax(cells[i])) {
                        tmpEntry.change = UPSIZE;
                    }
                    targets.insert(tmpEntry);
                }
            }
            ecc::logInfo() << "# uniform targets  : " << targets.size() << endl;
        }

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

        // CallTimer(view);
        // CalcStats((unsigned)thread_id, false, "After Initial
        // TIMING_RECOVERY",
        //           view);
        double prev_tns, new_tns = 0.0;
        int num = targets.size();
        int iter = 0;
        for(set< entry >::iterator it = targets.begin(); it != targets.end();
            it++) {
            if(iter % 100 == 0) {
                ecc::logPrintf("Attack targets %d/%d\n", iter, num);
            }
            iter++;
            if(update_cnt > (numcells * CORR_RATIO) && CORR_PT) {
                CallTimer(view);
                CorrelatePT((unsigned)thread_id, view);
                CalcStats((unsigned)thread_id, false, "Corr TIMING_RECOVERY",
                          view);
                update_cnt = 0;
            }
            if(update_cnt > (targets.size() * break_ratio)) {
                break;
            }

            if(skew_violation_worst == 0.0 || worst_slack_worst >= toler) {
                break;
            }

            unsigned cur = it->id;
            if(changed[cur])
                continue;

            double prev_slack = min(GetCellSlack(cells[cur], view),
                                    GetFICellSlack(cells[cur], view));
            double new_slack = prev_slack;

            if(COMPARE_TNS) {
                CalcStats((unsigned)thread_id, false, "", view, false);
                prev_tns = viewTNS[view];
                new_tns = prev_tns;
            }

            if(prev_slack > toler)
                continue;

            if(tabuNum > 0) {
                bool redundant = false;

                unsigned pre_sol = cells[cur].c_size * 10 + cells[cur].c_vtype;

                list< unsigned >::iterator iter;
                for(iter = cells[cur].tabu.begin();
                    iter != cells[cur].tabu.end(); iter++) {
                    if(*iter == pre_sol) {
                        redundant = true;
                        break;
                    }
                }

                if(redundant = false) {
                    if(cells[cur].tabu.size() < tabuNum) {
                        cells[cur].tabu.push_back(pre_sol);
                    }
                    else {
                        cells[cur].tabu.pop_front();
                        cells[cur].tabu.push_back(pre_sol);
                    }
                }
            }

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
            else {
                if(STAGE == GLOBAL) {
                    if(VERBOSE >= 4)
                        ecc::logInfo() << "GLOBAL TARGET = UPTYPE (" << it->step
                             << "): " << cells[cur].name << " "
                             << cells[cur].type;
                }
                else {
                    if(VERBOSE >= 4)
                        ecc::logInfo() << "FINESWAP TARGET = UPTYPE (" << it->step
                             << "): " << cells[cur].name << " "
                             << cells[cur].type;
                }

                if(HOLD_CHECK) {
                    delta_delay = EstDeltaDelay(cells[cur], 0, it->step, view);
                }

                change = cell_retype(cells[cur], it->step);
            }

            //// check timing after cell change

            bool restore = false;
            if(!CHECK_ALL_VIEW) {
                // ista for the new solution
                OneTimer(cells[cur], STA_MARGIN, true);

                new_slack = min(GetCellSlack(cells[cur], view),
                                GetFICellSlack(cells[cur], view));

                if(COMPARE_TNS) {
                    CalcStats((unsigned)thread_id, false, "", view, false);
                    new_tns = viewTNS[view];
                }

                if(prev_slack > new_slack && new_slack < 0.0)
                    restore = true;

                if(COMPARE_TNS && new_tns > prev_tns)
                    restore = true;

                if(!restore && HOLD_CHECK) {
                    if(EstHoldVio(cells[cur], delta_delay, view)) {
                        restore = true;
                    }
                }
            }
            else {
                for(unsigned view1 = 0; view1 < numViews; ++view1) {
                    if(restore)
                        break;
                    prev_slack = min(GetCellSlack(cells[cur], view1),
                                     GetFICellSlack(cells[cur], view1));

                    if(COMPARE_TNS) {
                        CalcStats((unsigned)thread_id, false, "", view1, false);
                        prev_tns = viewTNS[view1];
                    }

                    // ista for the new solution
                    OneTimer(cells[cur], STA_MARGIN, true);

                    new_slack = min(GetCellSlack(cells[cur], view1),
                                    GetFICellSlack(cells[cur], view1));
                    if(COMPARE_TNS) {
                        CalcStats((unsigned)thread_id, false, "", view1, false);
                        new_tns = viewTNS[view1];

                        // ecc::logInfo() << "TNS comparison " << view << " " << prev_tns
                        // << " " << new_tns << endl;
                    }

                    if(new_slack < prev_slack && new_slack < 0.0)
                        restore = true;

                    if(COMPARE_TNS && new_tns > 0.0 && new_tns > prev_tns)
                        restore = true;

                    if(!restore && HOLD_CHECK) {
                        if(EstHoldVio(cells[cur], delta_delay, view1)) {
                            restore = true;
                        }
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
                // cells[cur].isChanged++;
                // CalcStats((unsigned)thread_id, false, "After accept
                // TIMING_RECOVERY", view);
            }

            cells[cur].critical++;

            double stuck_ratio = 1.0;
            if(uniform_flag)
                stuck_ratio = STUCK_RATIO;
            else
                stuck_ratio = 1.0;

            if(count > (double)targets.size() * RATIO * 0.01 * stuck_ratio)
                break;
        }
        targets.clear();
    }
    ecc::logInfo() << "swap count = " << swap_cnt << endl;

    return swap_cnt;
}

unsigned Sizer::OptWNSPath(unsigned STAGE, unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    set< entry > targets;
    unsigned swap_cnt = 0;
    for(unsigned i = 0; i < numcells; i++) {
        if(cells[i].isClockCell)
            continue;
        if(cells[i].isDontTouch)
            continue;
        if(getLibCellInfo(cells[i]) == NULL)
            continue;

        for(unsigned k = 0; k < cells[i].outpins.size(); ++k) {
            if(isEqual(pins[view][cells[i].outpins[k]].rslk,
                       min(max_neg_rslk, max_neg_fslk)) ||
               isEqual(pins[view][cells[i].outpins[k]].fslk,
                       min(max_neg_rslk, max_neg_fslk))) {
                if(STAGE == DNSIZE) {
                    unsigned curnet = pins[view][cells[i].outpins[k]].net;

                    for(unsigned j = 0; j < nets[corner][curnet].outpins.size();
                        j++) {
                        unsigned curpin = nets[corner][curnet].outpins[j];
                        unsigned curfo = pins[view][curpin].owner;

                        if(curfo == UINT_MAX)
                            continue;  // PO
                        if(getLibCellInfo(cells[curfo]) == NULL)
                            continue;
                        if(isMin(cells[curfo]))
                            continue;  // min size
                        if(cells[curfo].isClockCell)
                            continue;
                        if(cells[curfo].isDontTouch)
                            continue;
                        entry tmpEntry;
                        tmpEntry.id = curfo;
                        tmpEntry.change = DNSIZE;
                        tmpEntry.delta_impact =
                            -min(pins[view][cells[curfo].outpins[k]].rslk,
                                 pins[view][cells[curfo].outpins[k]].fslk);

                        if(tmpEntry.delta_impact < 0)
                            targets.insert(tmpEntry);
                    }
                }
                else if(STAGE == UPSIZE) {
                    entry tmpEntry;
                    tmpEntry.id = i;
                    tmpEntry.change = UPSIZE;
                    tmpEntry.delta_impact =
                        (double)r_size(cells[i]) /
                        (double)pins[view][cells[i].outpins[k]].totcap;
                    targets.insert(tmpEntry);
                }
                else if(STAGE == OPT_SLEW) {
                    double cell_delay = 0.0;
                    double input_slew = 0.0;
                    for(unsigned j = 0; j < cells[i].inpins.size(); j++) {
                        double pin_slew =
                            max(pins[view][cells[i].inpins[j]].rtran,
                                pins[view][cells[i].inpins[j]].ftran);
                        if(input_slew < pin_slew) {
                            input_slew = pin_slew;
                        }
                        if(isEqual(pins[view][cells[i].inpins[j]].rslk,
                                   min(max_neg_rslk, max_neg_fslk))) {
                            for(unsigned k = 0;
                                k <
                                pins[view][cells[i].inpins[j]].rdelay.size();
                                ++k) {
                                if(cell_delay <
                                   pins[view][cells[i].inpins[j]].rdelay[k])
                                    cell_delay = pins[view][cells[i].inpins[j]]
                                                     .rdelay[k];
                            }
                            break;
                        }
                        else if(isEqual(pins[view][cells[i].inpins[j]].fslk,
                                        min(max_neg_rslk, max_neg_fslk))) {
                            for(unsigned k = 0;
                                k <
                                pins[view][cells[i].inpins[j]].fdelay.size();
                                ++k) {
                                if(cell_delay <
                                   pins[view][cells[i].inpins[j]].fdelay[k])
                                    cell_delay = pins[view][cells[i].inpins[j]]
                                                     .fdelay[k];
                            }
                            break;
                        }
                    }
                    entry tmpEntry;
                    tmpEntry.id = i;
                    tmpEntry.tie_break = input_slew;
                    tmpEntry.change = UPSIZE;
                    tmpEntry.delta_impact = (double)cell_delay;
                    targets.insert(tmpEntry);
                }
            }
        }
    }

    unsigned restore = 0;
    for(set< entry >::iterator it = targets.begin(); it != targets.end();
        it++) {
        unsigned cur = it->id;
        bool change_size = false;

        // try sizing according to sensitivity
        //
        if(STAGE == DNSIZE)
            change_size = cell_resize(cells[cur], -1);
        else if(STAGE == UPSIZE)
            change_size = cell_resize(cells[cur], 1);
        else if(STAGE == OPT_SLEW) {
            for(unsigned view = 0; view < mmmcViewList.size(); ++view)
                FwdFixSlewViolationCell(false, 0, cur, it->tie_break * 0.8,
                                        view);
        }

        for(unsigned view1 = 0; view1 < mmmcViewList.size(); ++view1)
            OneTimer(cells[cur], STA_MARGIN, true);

        // check slack, max cap constraints
        bool restore_flag = false;
        LibCellInfo *lib_cell_info = getLibCellInfo(cells[cur]);

        if(lib_cell_info == NULL) {
            continue;
        }

        double maxCap, curCap;
        for(unsigned k = 0; k < cells[cur].outpins.size(); ++k) {
            if(pins[view][cells[cur].outpins[k]].rslk < 0) {
                restore_flag = true;
                break;
            }
            if(pins[view][cells[cur].outpins[k]].fslk < 0) {
                restore_flag = true;
                break;
            }
            maxCap =
                lib_cell_info->pins[pins[view][cells[cur].outpins[k]].lib_pin]
                    .maxCapacitance;
            if(use_margin) {
                maxCap *= cap_margin;
            }
            curCap = pins[view][cells[cur].outpins[k]].totcap;
            if(curCap > maxCap) {
                restore_flag = true;
                break;
            }

            double max_tran = max(pins[view][cells[cur].outpins[k]].rtran,
                                  pins[view][cells[cur].outpins[k]].ftran);
            if(max_tran > cells[cur].max_tran[corner]) {
                restore_flag = true;
                break;
            }
        }

        // restore
        if(restore_flag && change_size) {
            if(STAGE == DNSIZE)
                cell_resize(cells[cur], 1);
            else
                cell_resize(cells[cur], -1);
            cells[cur].isChanged -= 2;
            OneTimer(cells[cur], STA_MARGIN);
            restore++;
        }
        else {
            swap_cnt++;
        }
    }
    ecc::logInfo() << "OptWNS: swap: " << swap_cnt << " target: " << targets.size()
         << endl;
    if(swap_cnt != 0)
        ecc::logInfo() << "OptWNS: Accept rate: "
             << (double)swap_cnt / (double)(swap_cnt + restore) << endl;
    targets.clear();
    return swap_cnt;
}

int Sizer::DownSizeFOCellsGreedy(bool corr_pt, unsigned option,
                                 unsigned cell_index, double gb,
                                 unsigned dont_touch, unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    int swap_cnt = 0;
    double WNS = 0.0;
    double TNS = 0.0;
    if(corr_pt)
        T = PTimer[option];
    if(corr_pt)
        TNS = T[view]->getTNS(clk_name[worst_corner]);
    else
        TNS = CalcSlackViolation();
    if(corr_pt)
        WNS = T[view]->getWorstSlack(clk_name[worst_corner]);
    else
        WNS = min(max_neg_rslk, max_neg_fslk);
    bool revert = false;

    set< entry > targets;

    unsigned curnet = pins[view][cells[cell_index].outpin].net;
    // ecc::logInfo() << "net - " << nets[corner][curnet].name << endl;
    // ecc::logInfo() << "net inpin - " <<
    // getFullPinName(pins[nets[corner][curnet].inpin]) << endl;

    // bool ignore = false;

    for(unsigned j = 0; j < nets[corner][curnet].outpins.size(); j++) {
        // ignore = false;
        unsigned curpin = nets[corner][curnet].outpins[j];
        unsigned curfo = pins[view][curpin].owner;

        if(curfo == UINT_MAX)
            continue;  // PO
        // if (isff(cells[curfo])) continue;           // Flip-flop
        if(isMin(cells[curfo]))
            continue;  // min size
        // if (curfo == dont_touch) continue;    // dont_touch

        /*
        for (unsigned k = 0; k <
        nets[corner][pins[view][cells[curfo].outpin].net].outpins.size(); k ++ )
        {
            if (
        pins[view][nets[pins[view][cells[curfo].outpin].net].outpins[k]].owner =
        dont_touch ) {
                ignore = true;
                break;
            }
        }

        if ( ignore ) continue;
        */

        entry tmpEntry2;
        tmpEntry2.id = curfo;
        tmpEntry2.change = DNSIZE;
        tmpEntry2.tie_break = 0;

        // SF = slack * input_cap
        tmpEntry2.delta_impact = -min(pins[view][cells[curfo].outpin].rslk,
                                      pins[view][cells[curfo].outpin].fslk) *
                                 pins[view][curpin].cap;
        // ecc::logInfo() << "fo cells ----"  << cells[curfo].name << "-" <<
        // cells[curfo].type << "delta impact: " << tmpEntry2.delta_impact <<
        // endl;
        // ecc::logInfo() << "pin name -- " << getFullPinName(pins[view][curpin]) << endl;
        targets.insert(tmpEntry2);
    }

    // ecc::logInfo() << "Target cell # " << targets.size() << endl;
    if(targets.size() == 0)
        return 0;

    for(set< entry >::iterator it = targets.begin(); it != targets.end();
        it++) {
        revert = false;
        unsigned cur = it->id;

        bool change_size = false;
        if(isMin(cells[cur]))
            continue;
        change_size = cell_resize(cells[cur], -1, corr_pt);
        OneTimer(cells[cur], STA_MARGIN);
        double new_tns = 0.0;
        if(corr_pt)
            new_tns = T[view]->getTNS(clk_name[worst_corner]);
        else
            new_tns = CalcSlackViolation();
        if(TNS < new_tns)
            revert = true;

        double pin_slack = 0.0;

        if(corr_pt)
            pin_slack = min(T[view]->getRiseSlack(
                                getFullPinName(pins[view][cells[cur].outpin])),
                            T[view]->getFallSlack(
                                getFullPinName(pins[view][cells[cur].outpin])));
        else
            pin_slack = min(pins[view][cells[cur].outpin].rslk,
                            pins[view][cells[cur].outpin].fslk);

        LibCellInfo *lib_cell_info = getLibCellInfo(cells[cur]);

        if(lib_cell_info == NULL) {
            continue;
        }

        double maxCap =
            lib_cell_info->pins[pins[view][cells[cur].outpin].lib_pin]
                .maxCapacitance;
        if(use_margin) {
            maxCap *= cap_margin;
        }
        double curCap = pins[view][cells[cur].outpin].totcap;

        if(pin_slack < WNS + gb || maxCap < curCap || revert) {  // restore
            if(change_size) {
                cell_resize(cells[cur], 1, corr_pt);
                OneTimer(cells[cur], STA_MARGIN);
            }
        }
        else {
            swap_cnt++;
            ecc::logInfo() << "down cells ----" << cells[cur].name << "-"
                 << cells[cur].type << endl;
        }
    }
    targets.clear();
    return swap_cnt;
}

int Sizer::UpSizeCellGreedy(bool corr_pt, unsigned option, unsigned cell_index,
                            double gb, int upsize, unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    if(cell_index == UINT_MAX)
        return 0;
    int swap_cnt = 0;
    double WNS = 0.0;
    double TNS = 0.0;
    if(corr_pt)
        T = PTimer[option];
    if(corr_pt)
        WNS = T[view]->getWorstSlack(clk_name[worst_corner]);
    else
        WNS = min(max_neg_rslk, max_neg_fslk);
    if(corr_pt)
        TNS = T[view]->getTNS(clk_name[worst_corner]);
    else
        TNS = CalcSlackViolation();
    double org_slk = 0.0;
    double new_slk = 0.0;
    bool revert = false;

    if(corr_pt) {
        org_slk = min(T[view]->getRiseSlack(
                          getFullPinName(pins[view][cells[cell_index].outpin])),
                      T[view]->getFallSlack(getFullPinName(
                          pins[view][cells[cell_index].outpin])));
    }
    else {
        org_slk = min(pins[view][cells[cell_index].outpin].rslk,
                      pins[view][cells[cell_index].outpin].fslk);
    }

    ecc::logInfo() << "WNS is -- " << WNS << " TNS is -- " << TNS << " org slk -- "
         << org_slk << " guardband[worst_corner] -- " << gb << endl;
    //    org_slk += gb;
    //    WNS += gb;

    if(org_slk > 0)
        return 0;

    bool change = false;
    if(r_type(cells[cell_index]) != (numVt - 1)) {
        revert = false;
        change = cell_retype(cells[cell_index], 1, corr_pt);
        OneTimer(cells[cell_index], STA_MARGIN);
        double new_tns = 0.0;
        // TNS check
        if(corr_pt)
            new_tns = T[view]->getTNS(clk_name[worst_corner]);
        else
            new_tns = CalcSlackViolation();
        if(TNS < new_tns)
            revert = true;

        if(corr_pt) {
            new_slk =
                min(T[view]->getRiseSlack(
                        getFullPinName(pins[view][cells[cell_index].outpin])),
                    T[view]->getFallSlack(
                        getFullPinName(pins[view][cells[cell_index].outpin])));
        }
        else {
            new_slk = min(pins[view][cells[cell_index].outpin].rslk,
                          pins[view][cells[cell_index].outpin].fslk);
        }

        ecc::logInfo() << "new slk (uptype) -- " << new_slk << endl;
        if(new_slk < org_slk + gb) {
            // revert
            if(change) {
                cell_retype(cells[cell_index], -1, corr_pt);
                OneTimer(cells[cell_index], STA_MARGIN);
            }
        }
        else {
            for(unsigned i = 0; i < cells[cell_index].inpins.size(); i++) {
                unsigned input_net =
                    pins[view][cells[cell_index].inpins[i]].net;
                for(unsigned j = 0; j < nets[corner][input_net].outpins.size();
                    j++) {
                    unsigned pin_index = nets[corner][input_net].outpins[j];
                    double pin_slack = 0.0;
                    if(corr_pt) {
                        pin_slack =
                            min(T[view]->getRiseSlack(
                                    getFullPinName(pins[view][pin_index])),
                                T[view]->getFallSlack(
                                    getFullPinName(pins[view][pin_index])));
                    }
                    else {
                        pin_slack = min(pins[view][pin_index].rslk,
                                        pins[view][pin_index].fslk);
                    }
                    if(pin_slack < WNS) {
                        revert = true;
                        break;
                    }
                }
                if(revert)
                    break;
            }

            if(!revert) {
                // accept and go futher
                ecc::logInfo() << "Cell uptyped --- " << cells[cell_index].name << " (->"
                     << cells[cell_index].type << ") slack: " << org_slk << "->"
                     << new_slk << endl;
                org_slk = new_slk;
                swap_cnt++;
                return swap_cnt;
            }
            else {
                // revert
                if(change) {
                    cell_retype(cells[cell_index], -1, corr_pt);
                    OneTimer(cells[cell_index], STA_MARGIN);
                }
            }
        }
    }

    if((!isMax(cells[cell_index]) && upsize == 1) ||
       (isLessThan(cells[cell_index], 1) && upsize == 2)) {
        revert = false;
        bool change = cell_resize(cells[cell_index], upsize, corr_pt);
        OneTimer(cells[cell_index], STA_MARGIN);

        double new_tns = 0.0;
        if(corr_pt)
            new_tns = T[view]->getTNS(clk_name[worst_corner]);
        else
            new_tns = CalcSlackViolation();
        if(TNS < new_tns) {
            ecc::logInfo() << "TNS worse " << TNS << " " << new_tns << endl;
            revert = true;
        }

        if(corr_pt) {
            new_slk =
                min(T[view]->getRiseSlack(
                        getFullPinName(pins[view][cells[cell_index].outpin])),
                    T[view]->getFallSlack(
                        getFullPinName(pins[view][cells[cell_index].outpin])));
        }
        else {
            new_slk = min(pins[view][cells[cell_index].outpin].rslk,
                          pins[view][cells[cell_index].outpin].fslk);
        }
        ecc::logInfo() << "Try -- new slk (upsize/" << upsize << "): " << new_slk << "/"
             << new_tns << " cell type : " << cells[cell_index].type << endl;
        // ecc::logInfo() << "Try -- new slk (upsize/" << upsize << "): " << new_slk << "
        // cell type : " << cells[cell_index].type << endl;

        if(new_slk < org_slk + gb) {
            ecc::logInfo() << "slk worse " << new_slk << " " << org_slk + gb << endl;

            // revert
            if(change)
                cell_resize(cells[cell_index], -upsize, corr_pt);
            OneTimer(cells[cell_index], STA_MARGIN);
        }
        else {
            for(unsigned i = 0; i < cells[cell_index].inpins.size(); i++) {
                unsigned input_net =
                    pins[view][cells[cell_index].inpins[i]].net;
                for(unsigned j = 0; j < nets[corner][input_net].outpins.size();
                    j++) {
                    unsigned pin_index = nets[corner][input_net].outpins[j];
                    double pin_slack = 0.0;
                    if(corr_pt) {
                        pin_slack =
                            min(T[view]->getRiseSlack(
                                    getFullPinName(pins[view][pin_index])),
                                T[view]->getFallSlack(
                                    getFullPinName(pins[view][pin_index])));
                    }
                    else {
                        pin_slack = min(pins[view][pin_index].rslk,
                                        pins[view][pin_index].fslk);
                    }
                    if(pin_slack < WNS) {
                        ecc::logInfo() << "pin slk of sibling (upsize) -- " << pin_slack
                             << "/" << WNS << endl;
                        revert = true;
                        break;
                    }
                }
                if(revert)
                    break;
            }

            if(!revert) {
                // accept and go futher
                // ecc::logInfo() << "Cell upsized --- " << cells[cell_index].name << "
                // (->" << cells[cell_index].type << ") slack: " << org_slk
                // <<"->" << new_slk<< endl;
                org_slk = new_slk;
                swap_cnt++;
            }
            else {
                // revert
                // ecc::logInfo() << "Cell revert ......" << endl;
                if(change)
                    cell_resize(cells[cell_index], -upsize, corr_pt);
                OneTimer(cells[cell_index], STA_MARGIN);
            }
        }
    }
    return swap_cnt;
}

unsigned Sizer::OptWNSPathBalance(bool corr_pt, unsigned option,
                                  unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    ecc::logInfo() << "OptWNSPathBalance() start..." << endl;
    if(corr_pt)
        T = PTimer[option];
    // double GB = -5.0;
    double WNS = 0;
    // int count = 0;
    int swap_cnt = 0;
    int stopCnt = 0;
    double prevWNS = 0;

    if(corr_pt)
        WNS = T[view]->getWorstSlack(clk_name[worst_corner]);
    else
        WNS = min(max_neg_rslk, max_neg_fslk);

    while(true) {
        if(corr_pt) {
            CorrelatePT(option);
            // to add guardband
            // CalcSlack();
        }
        prevWNS = WNS;
        ecc::logInfo() << "[OptWNSPathBalance" << option << "] ";
        CalcStats();
        if(corr_pt)
            WNS = T[view]->getWorstSlack(clk_name[worst_corner]);
        else
            WNS = min(max_neg_rslk, max_neg_fslk);

        if(WNS == prevWNS)
            stopCnt++;
        else
            stopCnt = 0;

        ecc::logInfo() << "WNS is -- " << WNS << endl;

        if(WNS >= 0.0)
            break;
        if(stopCnt > 5)
            break;

        vector< vector< unsigned > > path_cell_list;
        vector< unsigned > PO_list;

        for(unsigned i = 0; i < POs.size(); i++) {
            double rslack = pins[view][POs[i]].rslk;
            double fslack = pins[view][POs[i]].fslk;
            if(!isEqual(rslack, min(max_neg_rslk, max_neg_fslk)) &&
               !isEqual(fslack, min(max_neg_rslk, max_neg_fslk)))
                continue;
            unsigned pinid = nets[corner][pins[view][POs[i]].net].inpin;
            vector< unsigned > path;
            GetCellsWorstPath(path, pins[view][pinid]);

            if(path.size() > 1) {
                path_cell_list.push_back(path);
                PO_list.push_back(POs[i]);
            }
        }

        for(unsigned i = 0; i < numcells; i++) {
            unsigned poid;
            if(!isff(cells[i]))
                continue;
            poid = cells[i].data_pin;
            unsigned pinid = nets[corner][pins[view][poid].net].inpin;

            double rslack = pins[view][poid].rslk;
            double fslack = pins[view][poid].fslk;
            if(!isEqual(rslack, min(max_neg_rslk, max_neg_fslk)) &&
               !isEqual(fslack, min(max_neg_rslk, max_neg_fslk)))
                continue;
            vector< unsigned > path;
            GetCellsWorstPath(path, pins[view][pinid]);

            if(path.size() > 1) {
                path_cell_list.push_back(path);
                PO_list.push_back(poid);
            }
        }

        ecc::logInfo() << "Critical path # : " << path_cell_list.size() << endl;

        // optimize critical paths

        if(path_cell_list.size() == 0)
            continue;

        swap_cnt = 0;
        for(unsigned path_i = 0; path_i < path_cell_list.size(); path_i++) {
            vector< unsigned > path = path_cell_list[path_i];
            // double AAT = 0.0;
            // double delay = 0.0;
            // double tran = 0.0;

            for(unsigned i = path.size() - 1; i > 1; i--) {
                if(r_size(cells[path[i - 1]]) - r_size(cells[path[i]]) > 2) {
                    ecc::logInfo() << "Unbalanced path: " << cells[path[i]].type << "----"
                         << cells[path[i - 1]].type << endl;
                    swap_cnt +=
                        UpSizeCellGreedy(corr_pt, option, path[i], 0.0, 1);
                }
            }
        }
    }

    // string size_out = "balance_opt";
    SizeOut(outputDir);
    return swap_cnt;
}

vector< unsigned > Sizer::GetCellList(unsigned index, unsigned num_cell) {
    vector< unsigned > cell_list;

    cell_list.push_back(index);

    int num_ficell = (int)(num_cell - 1) / 2;
    int num_focell = (int)num_cell - 1 - num_ficell;

    unsigned iter = 0;

    for(unsigned i = 0; i < cells[index].fis.size(); i++) {
        if(cells[index].fis[i] != UINT_MAX) {
            iter++;
            cell_list.push_back(cells[index].fis[i]);
        }
        if(iter == num_ficell)
            break;
    }

    iter = 0;
    for(unsigned i = 0; i < cells[index].fos.size(); i++) {
        if(cells[index].fos[i] != UINT_MAX) {
            iter++;
            cell_list.push_back(cells[index].fos[i]);
        }
        if(iter == num_focell)
            break;
    }

    if(VERBOSE >= 300) {
        for(unsigned i = 0; i < cell_list.size(); i++) {
            ecc::logInfo() << cells[cell_list[i]].name << " ";
        }
        ecc::logInfo() << endl;
    }

    return cell_list;
}

vector< vector< int > > Sizer::GenSequence(unsigned option, unsigned num_cell) {
    // gray code sequence generate

    // gray code 27 cases

    vector< vector< int > > gray_code_ary;

    ecc::logInfo() << option << endl;
    vector< int > sequence;
    for(unsigned j = 0; j < option; j++) {
        sequence.push_back(j);
    }
    for(int j = option - 1; j >= 0; j--) {
        sequence.push_back(j);
    }

    vector< int > bin;

    if(num_cell == 3) {
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);
        unsigned cnt_j = 0, cnt_k = 0, cnt_l = 0;
        for(unsigned j = 0; j < option; j++) {
            bin[0] = sequence[cnt_j % sequence.size()];
            cnt_j++;
            for(unsigned k = 0; k < option; k++) {
                bin[1] = sequence[cnt_k % sequence.size()];
                cnt_k++;

                for(unsigned l = 0; l < option; l++) {
                    bin[2] = sequence[cnt_l % sequence.size()];
                    cnt_l++;
                    vector< int > code;
                    for(unsigned m = 0; m < bin.size(); m++) {
                        code.push_back(bin[m]);
                    }
                    gray_code_ary.push_back(code);
                }
            }
        }
    }
    else if(num_cell == 4) {
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);

        unsigned cnt_j = 0, cnt_k = 0, cnt_l = 0;
        unsigned cnt_m = 0;
        for(unsigned j = 0; j < option; j++) {
            bin[0] = sequence[cnt_j % sequence.size()];
            cnt_j++;
            for(unsigned k = 0; k < option; k++) {
                bin[1] = sequence[cnt_k % sequence.size()];
                cnt_k++;

                for(unsigned l = 0; l < option; l++) {
                    bin[2] = sequence[cnt_l % sequence.size()];
                    cnt_l++;

                    for(unsigned m = 0; m < option; m++) {
                        bin[3] = sequence[cnt_m % sequence.size()];
                        cnt_m++;

                        vector< int > code;
                        for(unsigned o = 0; o < bin.size(); o++) {
                            code.push_back(bin[o]);
                        }
                        gray_code_ary.push_back(code);
                    }
                }
            }
        }
    }
    else if(num_cell == 5) {
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);

        unsigned cnt_j = 0, cnt_k = 0, cnt_l = 0;
        unsigned cnt_m = 0, cnt_n = 0;
        for(unsigned j = 0; j < option; j++) {
            bin[0] = sequence[cnt_j % sequence.size()];
            cnt_j++;
            for(unsigned k = 0; k < option; k++) {
                bin[1] = sequence[cnt_k % sequence.size()];
                cnt_k++;

                for(unsigned l = 0; l < option; l++) {
                    bin[2] = sequence[cnt_l % sequence.size()];
                    cnt_l++;

                    for(unsigned m = 0; m < option; m++) {
                        bin[3] = sequence[cnt_m % sequence.size()];
                        cnt_m++;

                        for(unsigned n = 0; n < option; n++) {
                            bin[4] = sequence[cnt_n % sequence.size()];
                            cnt_n++;
                            vector< int > code;
                            for(unsigned o = 0; o < bin.size(); o++) {
                                code.push_back(bin[o]);
                            }
                            gray_code_ary.push_back(code);
                        }
                    }
                }
            }
        }
    }

    if(VERBOSE >= 300) {
        for(unsigned i = 0; i < gray_code_ary.size(); ++i) {
            for(unsigned j = 0; j < gray_code_ary[i].size(); ++j) {
                ecc::logInfo() << gray_code_ary[i][j] << " ";
            }
            ecc::logInfo() << endl;
        }
    }
    return gray_code_ary;
}

unsigned Sizer::InitWNSPath(unsigned view, unsigned numPath) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    ecc::logInfo() << "InitWNSPath() starts....... " << view << endl;

    vector< vector< unsigned > > path_cell_list;

    ecc::logInfo() << "INIT WNS PATH WORST SLACK " << worst_slack << endl;

    for(unsigned i = 0; i < POs.size(); i++) {
        double slack = min(pins[view][POs[i]].rslk, pins[view][POs[i]].fslk);
        ecc::logInfo() << "INIT WNS PATH PO " << getFullPinName(pins[view][POs[i]]) << " "
             << slack << " " << worst_slack << endl;
        if(slack > worst_slack)
            continue;
        unsigned pinid = nets[corner][pins[view][POs[i]].net].inpin;
        vector< unsigned > path;
        GetCellsWorstPath(path, pins[view][pinid]);

        if(path.size() > 1) {
            path_cell_list.push_back(path);
        }
    }

    for(unsigned i = 0; i < FFs.size(); i++) {
        unsigned curpin = UINT_MAX;

        for(unsigned j = 0; j < cells[FFs[i]].inpins.size(); ++j) {
            curpin = cells[FFs[i]].inpins[j];

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

            curpin = cells[FFs[i]].inpins[j];

            double slack =
                min(pins[view][curpin].rslk, pins[view][curpin].fslk);

            ecc::logInfo() << "INIT WNS PATH FF " << getFullPinName(pins[view][curpin])
                 << " " << slack << " " << worst_slack << endl;

            if(slack > worst_slack)
                continue;

            unsigned curnet = pins[view][curpin].net;

            if(curnet == UINT_MAX)
                continue;

            unsigned pinid = nets[corner][curnet].inpin;

            vector< unsigned > path;
            GetCellsWorstPath(path, pins[view][pinid]);

            if(path.size() > 1) {
                path_cell_list.push_back(path);
            }
        }
    }

    // optimize one critical path

    if(path_cell_list.size() == 0)
        return 0;
    unsigned cnt = 0;

    // for ( unsigned i = 0 ; i < path_cell_list.size(); i ++ )
    for(unsigned i = 0; i < numPath; i++) {
        vector< unsigned > path = path_cell_list[i];

        for(unsigned j = 0; j < path.size(); ++j) {
            ecc::logInfo() << "init size -- " << path[j] << endl;

            if(InitSize(path[j]))
                cnt++;

            if(CRIT_CELL_NO_TOUCH)
                cells[path[j]].isDontTouch = true;

        }  // End one path optimization
    }

    return cnt;
}

bool Sizer::InitSize(unsigned index) {
    if(cells[index].type != init_sizes[index]) {
        cells[index].type = init_sizes[index];
        LibCellInfo *lib_cell_info = getLibCellInfo(init_sizes[index]);
        if(lib_cell_info != NULL) {
            cells[index].c_vtype = lib_cell_info->c_vtype;
            cells[index].c_size = lib_cell_info->c_size;
        }
        cells[index].isChanged++;
        return true;
    }
    else {
        return false;
    }
}

unsigned Sizer::OptWNSPathGray(bool corr_pt, unsigned thread_id,
                               unsigned option, int num_cell, unsigned STEP,
                               unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    ecc::logInfo() << "OptWNSPathGray() starts.......for view " << view << endl;
    ecc::logInfo() << "option " << option;
    ecc::logInfo() << " num_cell " << num_cell;
    ecc::logInfo() << " STEP " << STEP;

    if(corr_pt)
        T = PTimer[thread_id];

    // gray code sequence generate

    // gray code 27 cases

    unsigned num_cell_set = num_cell;
    unsigned gray_code_num = (unsigned)pow(option, (double)num_cell);

    vector< vector< int > > gray_code_ary;

    ecc::logInfo() << option << endl;
    vector< int > sequence;
    for(unsigned j = 0; j < option; j++) {
        sequence.push_back(j);
    }
    for(int j = option - 1; j >= 0; j--) {
        sequence.push_back(j);
    }

    vector< int > bin;

    if(num_cell == 3) {
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);
        unsigned cnt_j = 0, cnt_k = 0, cnt_l = 0;
        for(unsigned j = 0; j < option; j++) {
            bin[0] = sequence[cnt_j % sequence.size()];
            cnt_j++;
            for(unsigned k = 0; k < option; k++) {
                bin[1] = sequence[cnt_k % sequence.size()];
                cnt_k++;

                for(unsigned l = 0; l < option; l++) {
                    bin[2] = sequence[cnt_l % sequence.size()];
                    cnt_l++;
                    vector< int > code;
                    for(unsigned m = 0; m < bin.size(); m++) {
                        code.push_back(bin[m]);
                    }
                    gray_code_ary.push_back(code);
                }
            }
        }
    }
    else if(num_cell == 4) {
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);

        unsigned cnt_j = 0, cnt_k = 0, cnt_l = 0;
        unsigned cnt_m = 0;
        for(unsigned j = 0; j < option; j++) {
            bin[0] = sequence[cnt_j % sequence.size()];
            cnt_j++;
            for(unsigned k = 0; k < option; k++) {
                bin[1] = sequence[cnt_k % sequence.size()];
                cnt_k++;

                for(unsigned l = 0; l < option; l++) {
                    bin[2] = sequence[cnt_l % sequence.size()];
                    cnt_l++;

                    for(unsigned m = 0; m < option; m++) {
                        bin[3] = sequence[cnt_m % sequence.size()];
                        cnt_m++;

                        vector< int > code;
                        for(unsigned o = 0; o < bin.size(); o++) {
                            code.push_back(bin[o]);
                        }
                        gray_code_ary.push_back(code);
                    }
                }
            }
        }
    }
    else if(num_cell == 5) {
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);
        bin.push_back(0);

        unsigned cnt_j = 0, cnt_k = 0, cnt_l = 0;
        unsigned cnt_m = 0, cnt_n = 0;
        for(unsigned j = 0; j < option; j++) {
            bin[0] = sequence[cnt_j % sequence.size()];
            cnt_j++;
            for(unsigned k = 0; k < option; k++) {
                bin[1] = sequence[cnt_k % sequence.size()];
                cnt_k++;

                for(unsigned l = 0; l < option; l++) {
                    bin[2] = sequence[cnt_l % sequence.size()];
                    cnt_l++;

                    for(unsigned m = 0; m < option; m++) {
                        bin[3] = sequence[cnt_m % sequence.size()];
                        cnt_m++;

                        for(unsigned n = 0; n < option; n++) {
                            bin[4] = sequence[cnt_n % sequence.size()];
                            cnt_n++;
                            vector< int > code;
                            for(unsigned o = 0; o < bin.size(); o++) {
                                code.push_back(bin[o]);
                            }
                            gray_code_ary.push_back(code);
                        }
                    }
                }
            }
        }
    }

    int count = 0;
    if(corr_pt) {
        CorrelatePT();
        // to add guardband
        // CalcSlack();
    }
    CalcStats();
    double WNS = 0.0;
    if(corr_pt)
        WNS = T[view]->getWorstSlack(clk_name[worst_corner]);
    else
        WNS = min(max_neg_rslk, max_neg_fslk);
    if(WNS >= 0)
        return 0;

    double prev_TNS, TNS;
    prev_TNS = TNS = 0.0;

    double GB = -5.0;
    double min_tns = skew_violation;
    vector< CELL > cur_best;
    cur_best.resize(numcells);

    for(unsigned k = 0; k < numcells; k++) {
        cur_best[k] = cells[k];
    }

    int stuck_count = 0;
    bool found_local_min = false;

    double begin = cpuTime();
    int trial_count = 0;

    bool stuck = false;
    while((count < 20 && WNS < 0.0) || (count >= 20 && WNS < GB)) {
        if(WNS >= 0.0)
            break;
        if(stuck)
            break;

        vector< vector< unsigned > > path_cell_list;
        vector< unsigned > PO_list;

        for(unsigned i = 0; i < POs.size(); i++) {
            double rslack = pins[view][POs[i]].rslk;
            double fslack = pins[view][POs[i]].fslk;
            if(!isEqual(rslack, min(max_neg_rslk, max_neg_fslk)) &&
               !isEqual(fslack, min(max_neg_rslk, max_neg_fslk)))
                continue;
            unsigned pinid = nets[corner][pins[view][POs[i]].net].inpin;
            vector< unsigned > path;
            GetCellsWorstPath(path, pins[view][pinid]);

            if(path.size() > 1) {
                path_cell_list.push_back(path);
                PO_list.push_back(POs[i]);
            }
        }

        for(unsigned i = 0; i < numcells; i++) {
            unsigned poid;
            // if (!isff(cells[i])) continue;
            if(pins[view][cells[i].inpins[0]].name == "ck")
                poid = cells[i].inpins[1];
            else
                poid = cells[i].inpins[0];
            unsigned pinid = nets[corner][pins[view][poid].net].inpin;

            double rslack = pins[view][poid].rslk;
            double fslack = pins[view][poid].fslk;
            if(!isEqual(rslack, min(max_neg_rslk, max_neg_fslk)) &&
               !isEqual(fslack, min(max_neg_rslk, max_neg_fslk)))
                continue;
            vector< unsigned > path;
            GetCellsWorstPath(path, pins[view][pinid]);

            if(path.size() > 1) {
                path_cell_list.push_back(path);
                PO_list.push_back(poid);
            }
        }

        // optimize one critical path

        if(path_cell_list.size() == 0)
            continue;

        // for ( unsigned i = 0 ; i < path_cell_list.size(); i ++ )
        for(unsigned i = 0; i < 1; i++) {
            vector< unsigned > path = path_cell_list[i];

            if(path.size() < num_cell_set)
                continue;

            // for (unsigned j = 0 ; j < path.size()-num_cell_set; j = j + STEP
            // )
            for(unsigned j = 0; j < path.size(); j = j + STEP) {
                if(prev_TNS <= TNS)
                    stuck_count++;
                if(stuck_count > 3) {
                    stuck = true;
                    break;
                }

                prev_TNS = TNS;

                int start = 0;
                if(j + num_cell_set >= path.size())
                    start = path.size() - num_cell_set - 1;
                else
                    start = j;
                unsigned best_sol = UINT_MAX;
                double max_neg_slk = 0.0;

                //                double wslk = 0.0;
                //                for (unsigned l = 0; l < num_cell_set ; l ++ )
                //                {
                //                    for ( unsigned m = 0 ; m <
                //                    cells[path[start+l]].inpins.size(); m++ )
                //                    {
                //                        double slk =
                //                        min(pins[view][cells[path[start+l]].inpins[m]].rslk,
                //                        pins[view][cells[path[start+l]].inpins[m]].fslk);
                //                        if ( wslk > slk )
                //                            wslk = slk;
                //                    }
                //                    if (wslk >
                //                    min(pins[view][cells[path[start+l]].outpin].rslk,pins[view][cells[path[start+l]].outpin].fslk))
                //                        wslk =
                //                        min(pins[view][cells[path[start+l]].outpin].rslk,pins[view][cells[path[start+l]].outpin].fslk);
                //                }
                //                max_neg_slk = wslk;
                max_neg_slk =
                    min(pins[view][cells[path[path.size() - 1]].outpin].rslk,
                        pins[view][cells[path[path.size() - 1]].outpin].fslk);
                //                ecc::logInfo() << "Original slack - " << max_neg_slk <<
                //                endl;
                vector< cell_sizes > org_size_list;
                vector< cell_vtypes > org_vt_list;

                for(unsigned l = 0; l < num_cell_set; l++) {
                    org_size_list.push_back(cells[path[start + l]].c_size);
                    org_vt_list.push_back(cells[path[start + l]].c_vtype);
                }

                for(unsigned k = 0; k < gray_code_num; k++) {
                    trial_count++;

                    //                    double one_rslk, one_fslk;
                    //                    one_rslk = one_fslk = -100000;
                    bool max_cap_vio = false;

                    //                    for (unsigned l = 0; l < num_cell_set
                    //                    ; l ++ ) {
                    //                        ecc::logInfo() << gray_code_ary[k][l] ;
                    //                    }
                    //                    ecc::logInfo() << endl;

                    // try
                    for(unsigned l = 0; l < num_cell_set; l++) {
                        // ecc::logInfo() << "change set[" << k << "]: " <<
                        // cells[path[start+l]].name;

                        if(k == 0) {
                            //                              cout
                            //                              <<cells[path[start+l]].type<<"
                            //                              ";
                            //                            if (
                            //                            gray_code_ary[k][l] !=
                            //                            0 ) {
                            //                                cout
                            //                                <<"*("<<cells[path[start+l]].type<<"->";
                            //
                            //                                cell_move(cells[path[start+l]],
                            //                                gray_code_ary[k][l]);
                            //                                OneTimer(cells[path[start+l]],STA_MARGIN);
                            //
                            //                                ecc::logInfo() <<
                            //                                cells[path[start+l]].type<<"-";
                            //                                cout
                            //                                <<gray_code_ary[k][l]<<")";
                            //                            }
                        }
                        else {
                            if(gray_code_ary[k][l] != gray_code_ary[k - 1][l]) {
                                //                               ecc::logInfo() <<
                                //                               gray_code_ary[k][l]
                                //                               << "--" ;

                                bool changed = cell_move(
                                    cells[path[start + l]], org_size_list[l],
                                    org_vt_list[l], gray_code_ary[k][l]);
                                if(changed)
                                    OneTimer(cells[path[start + l]],
                                             STA_MARGIN);
                            }
                        }
                    }
                    // ecc::logInfo() << endl;

                    for(unsigned l = 0; l < num_cell_set; l++) {
                        // max cap check
                        LibCellInfo *lib_cell_info =
                            getLibCellInfo(cells[path[start + l]]);

                        if(lib_cell_info == NULL) {
                            continue;
                        }

                        double maxCap = 0.0;
                        if(lib_cell_info) {
                            maxCap =
                                lib_cell_info
                                    ->pins[pins[view]
                                               [cells[path[start + l]].outpin]
                                                   .lib_pin]
                                    .maxCapacitance;
                            if(use_margin) {
                                maxCap *= cap_margin;
                            }
                        }
                        if(pins[view][cells[path[start + l]].outpin].totcap +
                               nets[corner]
                                   [pins[view][cells[path[start + l]].outpin]
                                        .net]
                                       .cap >
                           maxCap)
                            max_cap_vio = true;

                        // input check
                        for(unsigned m = 0;
                            m < cells[path[start + l]].fis.size(); m++) {
                            unsigned fanin_cell = cells[path[start + l]].fis[m];

                            // max cap check
                            // output check
                            LibCellInfo *lib_cell_info =
                                getLibCellInfo(cells[fanin_cell]);

                            if(lib_cell_info == NULL) {
                                continue;
                            }

                            maxCap = 0.0;
                            if(lib_cell_info) {
                                maxCap =
                                    lib_cell_info
                                        ->pins[pins[view]
                                                   [cells[fanin_cell].outpin]
                                                       .lib_pin]
                                        .maxCapacitance;
                                if(use_margin) {
                                    maxCap *= cap_margin;
                                }
                            }
                            if(pins[view][cells[fanin_cell].outpin].totcap +
                                   nets[corner]
                                       [pins[view][cells[fanin_cell].outpin]
                                            .net]
                                           .cap >
                               maxCap)
                                max_cap_vio = true;
                        }
                    }

                    double wslk = 0.0;
                    for(unsigned l = 0; l < num_cell_set; l++) {
                        for(unsigned m = 0;
                            m < cells[path[start + l]].inpins.size(); m++) {
                            double slk =
                                min(pins[view][cells[path[start + l]].inpins[m]]
                                        .rslk,
                                    pins[view][cells[path[start + l]].inpins[m]]
                                        .fslk);
                            if(wslk > slk)
                                wslk = slk;
                        }
                        if(wslk >
                           min(pins[view][cells[path[start + l]].outpin].rslk,
                               pins[view][cells[path[start + l]].outpin].fslk))
                            wslk = min(
                                pins[view][cells[path[start + l]].outpin].rslk,
                                pins[view][cells[path[start + l]].outpin].fslk);
                    }

                    if(wslk >
                       min(pins[view][cells[path[path.size() - 1]].outpin].rslk,
                           pins[view][cells[path[path.size() - 1]].outpin]
                               .fslk))
                        wslk =
                            min(pins[view][cells[path[path.size() - 1]].outpin]
                                    .rslk,
                                pins[view][cells[path[path.size() - 1]].outpin]
                                    .fslk);

                    // ecc::logInfo() << "slack with gray code [" << k << "] :" <<
                    // one_rslk << " " << one_fslk << endl;

                    // save the best solution

                    if(!max_cap_vio && max_neg_slk < wslk) {
                        // ecc::logInfo() << "max slack " << max_neg_slk ;
                        max_neg_slk = wslk;
                        best_sol = k;
                        // ecc::logInfo() << "->" << max_neg_slk ;
                        // ecc::logInfo() << " *** best slack change with gray code [" <<
                        // k << "] :" << wslk << endl;
                    }
                }

                //    ecc::logInfo() << "Restored slack - " <<
                //    pins[view][cells[path[path.size()-1]].outpin].rslk << " "
                //    << pins[view][cells[path[j]].outpin].fslk << endl;

                // apply best solution
                if(best_sol != UINT_MAX) {
                    for(unsigned l = 0; l < num_cell_set; l++) {
                        bool changed = cell_move(
                            cells[path[start + l]], org_size_list[l],
                            org_vt_list[l], gray_code_ary[best_sol][l]);
                        if(changed)
                            OneTimer(cells[path[start + l]], STA_MARGIN);
                    }
                    //                    ecc::logInfo() << "Improved slack [" << best_sol
                    //                    << "] - "<< "(" ;
                    //                    ecc::logInfo() <<
                    //                    pins[view][cells[path[path.size()-1]].outpin].rslk
                    //                        << ", " <<
                    //                        pins[view][cells[path[path.size()-1]].outpin].fslk
                    //                        << ")" << endl;
                }
                else {
                    // restore
                    for(unsigned l = 0; l < num_cell_set; l++) {
                        bool changed =
                            cell_move(cells[path[start + l]], org_size_list[l],
                                      org_vt_list[l], 0);
                        if(changed)
                            OneTimer(cells[path[start + l]], STA_MARGIN);
                    }
                }
            }  // End one path optimization

            CallTimer();
            if(corr_pt) {
                CorrelatePT();
                // to add guardband
                // CalcSlack();
            }
            CalcStats();

            TNS = skew_violation;

            if(corr_pt)
                WNS = T[view]->getWorstSlack(clk_name[worst_corner]) +
                      guardband[worst_corner];
            else
                WNS = min(max_neg_rslk, max_neg_fslk);
            count++;

            // ecc::logInfo() << count << " Trial -- current WNS :" << WNS << " TNS : " <<
            // TNS << endl;

            // current best solution
            if(min_tns > TNS) {
                ecc::logInfo() << " Current best -- WNS : " << WNS << " TNS : " << TNS
                     << endl;
                min_tns = TNS;
                found_local_min = true;
                for(unsigned k = 0; k < numcells; k++) {
                    cur_best[k] = cells[k];
                }
            }
        }

        if(count > 50)
            break;
    }

    if(TNS > min_tns && found_local_min || !found_local_min) {
        ecc::logInfo() << "Revert to the best solution.." << endl;
        // revert to the best solution
        for(unsigned i = 0; i < numcells; i++) {
            cells[i] = cur_best[i];
        }
    }

    double cpu_time = cpuTime() - begin;

    ecc::logInfo() << "total tried paths num : " << count
         << " / avg trial count per path : " << trial_count / count << endl;
    ecc::logInfo() << "RunTime (total/path/trial): " << cpu_time << " "
         << cpu_time / count << " " << cpu_time / count / trial_count << endl;

    return 0;
}

// further cap optimization
void Sizer::Release(bool success, unsigned STAGE, unsigned view) {
    unsigned corner = 0;  // mmmcViewList[view].corner;
    if(STAGE == GLOBAL) {
        ecc::logInfo() << "RELEASE/GLOBAL - Start global releasing .. " << endl;
        for(unsigned i = 0; i < numcells; i++) {
            // if(isff(cells[i]))
            //    continue;
            if(cells[i].isClockCell)
                continue;
            if(cells[i].isDontTouch)
                continue;

            bool attack = false;

            for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
                if(pins[view][cells[i].outpins[j]].rslk >
                       max_pos_rslk * RATIO2 * 0.01 &&
                   pins[view][cells[i].outpins[j]].fslk <
                       max_pos_fslk * RATIO2 * 0.1) {
                    attack = true;
                    break;
                }
            }

            if(attack) {
                if(r_type(cells[i]) != 0) {
                    bool change = cell_retype(cells[i], -1);
                    LibCellInfo *lib_cell_info = getLibCellInfo(cells[i]);
                    if(lib_cell_info == NULL) {
                        continue;
                    }

                    double maxCap =
                        lib_cell_info->pins[pins[view][cells[i].outpin].lib_pin]
                            .maxCapacitance;
                    if(use_margin) {
                        maxCap *= cap_margin;
                    }
                    if(lib_cell_info &&
                       pins[view][cells[i].outpin].totcap > maxCap && change)
                        cell_retype(cells[i], 1);
                }
                else if(!isMin(cells[i])) {
                    bool change = cell_resize(cells[i], -1);
                    LibCellInfo *lib_cell_info = getLibCellInfo(cells[i]);
                    if(lib_cell_info == NULL) {
                        continue;
                    }
                    double maxCap =
                        lib_cell_info->pins[pins[view][cells[i].outpin].lib_pin]
                            .maxCapacitance;
                    if(use_margin) {
                        maxCap *= cap_margin;
                    }
                    if(lib_cell_info &&
                       pins[view][cells[i].outpin].totcap > maxCap && change)
                        cell_resize(cells[i], 1);
                }
            }
        }
    }
    else if(STAGE == LEGALIZE) {
        ecc::logInfo() << "RELEASE/LEGALIZE - Start releasing .. with factor 0.5" << endl;
        for(unsigned i = 0; i < numcells; i++) {
            // if(isff(cells[i]))
            //    continue;
            if(cells[i].isClockCell)
                continue;
            if(cells[i].isDontTouch)
                continue;
            if(!isMin(cells[i]))
                cell_resize(cells[i], -1);
            else if(r_type(cells[i]) != 0)
                cell_retype(cells[i], -1);
        }
    }

    else if(STAGE == CRIT_LEGALIZE) {
        ecc::logInfo() << "RELEASE/CRIT_LEGALIZE - Start releasing .. " << endl;
        for(unsigned i = 0; i < numcells; i++) {
            // if ( isff(cells[i])) continue;

            bool attack = true;

            for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
                double rslack = pins[view][cells[i].outpins[j]].rslk;
                double fslack = pins[view][cells[i].outpins[j]].fslk;
                if(rslack > CRIT_PATH_RATIO * min(max_neg_rslk, max_neg_fslk) &&
                   fslack > CRIT_PATH_RATIO * min(max_neg_rslk, max_neg_fslk)) {
                    attack = false;
                    break;
                }
            }

            if(!attack)
                continue;

            if(!isMin(cells[i]))
                cell_resize(cells[i], -2);

            for(unsigned j = 0;
                j <
                nets[corner][pins[view][cells[i].outpin].net].outpins.size();
                j++) {
                unsigned index =
                    nets[corner][pins[view][cells[i].outpin].net].outpins[j];
                if(pins[view][index].owner == UINT_MAX)
                    continue;
                // if ( isff(cells[pins[index].owner])) continue;

                // ecc::logInfo() << "legalizing..target cell -- " <<
                // cells[pins[view][index].owner].type <<" " <<
                // r_size(cells[pins[view][index].owner]) << endl;
                if(!isMin(cells[pins[view][index].owner]))
                    cell_resize(cells[pins[view][index].owner], -2);
            }
        }
    }
    else if(STAGE == CRIT_RESET) {
        ecc::logInfo() << "RELEASE/CRIT_RESET - Start reset .. " << endl;
        for(unsigned i = 0; i < numcells; i++) {
            // if ( isff(cells[i])) continue;

            bool attack = true;

            for(unsigned j = 0; j < cells[i].outpins.size(); ++j) {
                double rslack = pins[view][cells[i].outpins[j]].rslk;
                double fslack = pins[view][cells[i].outpins[j]].fslk;
                if(rslack > CRIT_PATH_RATIO * min(max_neg_rslk, max_neg_fslk) &&
                   fslack > CRIT_PATH_RATIO * min(max_neg_rslk, max_neg_fslk)) {
                    attack = false;
                    break;
                }
            }

            if(attack) {
                cells[i].type = g_cells[i].type;
                LibCellInfo *lib_cell_info = getLibCellInfo(cells[i]);
                if(lib_cell_info != NULL) {
                    cells[i].c_vtype = lib_cell_info->c_vtype;
                    cells[i].c_size = lib_cell_info->c_size;
                }
                cells[i].isChanged++;
            }
        }
    }
}

void *static_sizer_driver(void *void_thread_args) {
    struct THREAD_ARGS *thread_args;
    thread_args = (struct THREAD_ARGS *)void_thread_args;

    Sizer *_this = thread_args->this_instance;
    _this->main(thread_args->thread_id, false);
    return void_thread_args;
}

void *static_pt_driver(void *void_thread_args) {
    struct THREAD_ARGS *thread_args;
    thread_args = (struct THREAD_ARGS *)void_thread_args;

    Sizer *_this = thread_args->this_instance;
    _this->main(thread_args->thread_id, true);
    return void_thread_args;
}

void *timer_thread(void *void_thread_args) {
    struct THREAD_ARGS *thread_args;
    thread_args = (struct THREAD_ARGS *)void_thread_args;
    Sizer *_this = thread_args->this_instance;
    int count = 0;

    while(TIMEOUT > count) {
        sleep(1);
        count++;
    }
    if((pthread_cancel(_this->threads[thread_args->thread_id])) != 0)
        perror("pthread_cancel");
    ecc::logInfo() << "(timeout) Killing main thread " << thread_args->thread_id << endl;
    return nullptr;
}

void *static_poweropt_driver(void *void_thread_args) {
    struct THREAD_ARGS *thread_args;
    thread_args = (struct THREAD_ARGS *)void_thread_args;
    // pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    // for timeout
    //
    // pthread_t t_timer;
    // if ( -1 == pthread_create(&t_timer, NULL, timer_thread,
    // void_thread_args)) {
    //    perror("pthread_create");
    //    return NULL;
    //}

    Sizer *_this = thread_args->this_instance;
    _this->Post_PowerOpt(thread_args->thread_id);
    return void_thread_args;
}

#include "legalize_core.h"

// DreamPlace greedy+abacus legalization (vendored ops). Greedy distributes
// cells across rows by bin capacity, then abacus legalizes in-row with
// fixed cells as first-class obstacles. Movable core cells are indexed
// [0, M), fixed core cells [M, N) per the DREAMPlace convention.
// Returns movable cells placed, -1 when inapplicable (caller falls back).
int Sizer::legalizeDreamplace() {
    auto block = _ckt->_ord_design->getBlock();
    std::vector<odb::dbRow*> rows;
    for(auto* row : block->getRows()) {
        rows.push_back(row);
    }
    if(rows.empty()) {
        ecc::logPrintf("dp-legalize: no rows, fall back to detailed_placement\n");
        return -1;
    }
    const int site_w = rows[0]->getSite()->getWidth();
    const int site_h = rows[0]->getSite()->getHeight();
    std::vector<ecc::LegalizeRowSeg> segs;
    std::map<int, odb::dbOrientType> orient_of_y;
    for(auto* row : rows) {
        if(row->getSite()->getWidth() != site_w ||
           row->getSite()->getHeight() != site_h) {
            ecc::logPrintf("dp-legalize: non-uniform rows, fall back\n");
            return -1;
        }
        segs.push_back({row->getOrigin().x(), row->getOrigin().y(),
                        row->getSiteCount()});
        int ry = row->getOrigin().y();
        if(orient_of_y.count(ry) && orient_of_y[ry] != row->getOrient()) {
            ecc::logPrintf("dp-legalize: inconsistent orient at y=%d, fall back\n",
                   ry);
            return -1;
        }
        orient_of_y[ry] = row->getOrient();
    }

    std::vector<ecc::LegalizeNode> nodes;
    std::vector<odb::dbInst*> insts;
    for(auto* inst : block->getInsts()) {
        auto* master = inst->getMaster();
        if(!master->isCore() || inst->isFixed()) {
            continue;
        }
        if((int)master->getHeight() != site_h) {
            ecc::logPrintf("dp-legalize: multi-row movable cell %s, fall back\n",
                   master->getName().c_str());
            return -1;
        }
        int ix = 0, iy = 0;
        inst->getLocation(ix, iy);
        insts.push_back(inst);
        nodes.push_back({double(ix), double(iy), double(master->getWidth()),
                         double(master->getHeight()), false});
    }
    const int num_movable = (int)insts.size();
    for(auto* inst : block->getInsts()) {
        // Any fixed instance (macros included) is an obstacle for the ops.
        if(!inst->isFixed()) {
            continue;
        }
        auto* bbox = inst->getBBox();
        int x_min = bbox->xMin();
        int y_min = bbox->yMin();
        int x_max = bbox->xMax();
        int y_max = bbox->yMax();
        if(inst->getHalo() != nullptr) {
            odb::Rect halo = inst->getTransformedHalo();
            x_min -= halo.xMin();
            y_min -= halo.yMin();
            x_max += halo.xMax();
            y_max += halo.yMax();
        }
        nodes.push_back({double(x_min), double(y_min),
                         double(x_max - x_min),
                         double(y_max - y_min), true});
    }
    if(num_movable == 0) {
        return 0;
    }

    std::vector<double> out_x;
    std::vector<int> out_row;
    int base_x = 0, base_y = 0, num_rows = 0;
    if(ecc::dpLegalizeCore(segs, site_w, site_h, nodes, num_movable,
                           double(dp_padding), out_x, out_row, base_x, base_y,
                           num_rows) != 0) {
        return -1;
    }

    double max_disp = 0.0;
    for(int i = 0; i < num_movable; ++i) {
        const int ry = base_y + out_row[i] * site_h;
        insts[i]->setOrient(orient_of_y[ry]);
        insts[i]->setLocation((int)llround(out_x[i]), ry);
        insts[i]->setPlacementStatus(odb::dbPlacementStatus::PLACED);
        max_disp = std::max(max_disp, fabs(out_x[i] - nodes[i].x) +
                                          fabs(ry - nodes[i].y));
    }
    ecc::logPrintf("dp-legalize: placed %d cells, max displacement %.0f dbu\n",
           num_movable, max_disp);
    return num_movable;
}

void Sizer::legalizePlacement() {
    if(use_native_abacus && legalizeDreamplace() >= 0) {
        return;
    }
    _ckt->_ord_design->evalTclString(
        "detailed_placement -use_negotiation -abacus");
}

void Sizer::runOrdTO() {
    _ckt->assertNoIllegalExternalPadBoundaryChain();
    int view = 0;
    rsz::Resizer *resizer = ord::OpenRoad::openRoad()->getResizer();
    resizer->setBlockageAwareSteiner(true);
    int repaired_net_count, slew_violations, cap_violations, fanout_violations,
        length_violations;
    // _ckt->_ord_design->evalTclString("buffer_ports");
    _ckt->_ord_design->evalTclString("report_worst_slack -min -digits 3");
    _ckt->_ord_design->evalTclString("report_worst_slack -max -digits 3");
    _ckt->_ord_design->evalTclString("report_tns -digits 3");
    // Keep the detailed report_checks table out of the production log; the
    // repair commands below compute and report their own summary metrics.
    // _ckt->_ord_design->evalTclString("set_max_fanout 32 [current_design]");
    _ckt->_ord_design->evalTclString(
        "repair_design -slew_margin 10 -cap_margin 10 -verbose");
    // Repair hold in the setup corner before setup optimization consumes slack.
    _ckt->_ord_design->evalTclString(
        "repair_timing -hold -hold_margin " + to_string(hold_margin) +
        " -skip_pin_swap -verbose");
    _ckt->_ord_design->evalTclString("repair_timing -setup -setup_margin " +
                                     to_string(setup_margin) +
                                     " -skip_pin_swap -verbose");
    const int connected_special_pins
        = ecc::connectMissingWildcardSpecialNetPins(
            _ckt->_ord_design->getBlock());
    if(connected_special_pins > 0) {
        ecc::logInfo() << "Connected " << connected_special_pins
             << " new instance pins to wildcard special nets." << endl;
    }
    const char *pre_final_legalization_def =
        std::getenv("SIZER_PRE_FINAL_LEGALIZATION_DEF");
    if(pre_final_legalization_def != nullptr &&
       pre_final_legalization_def[0] != '\0') {
        ecc::logInfo() << "Write DEF before final legalization: "
             << pre_final_legalization_def << endl;
        _ckt->_ord_design->writeDef(pre_final_legalization_def);
    }
    legalizePlacement();
    // Legalization moves degrade the repaired timing state. Rebuild placement
    // parasitics, run a margin-free recovery pass, then re-legalize any cells
    // inserted by the recovery.
    // _ckt->_ord_design->evalTclString("estimate_parasitics -placement");
    // _ckt->_ord_design->evalTclString(
    //     "repair_design -slew_margin 20 -cap_margin 20 -verbose");
    // _ckt->_ord_design->evalTclString("repair_timing -setup -verbose");
    // legalizePlacement();
    // Rebuild parasitics after the final detailed placement so the reported
    // final slacks match the exported design state.
    if(use_gr_rc && (noSPEF || spefFile.empty())) {
        ecc::logPrintf("Run final global-routing RC refresh after runOrdTO...\n");
        refreshOpenStaParasitics(true);
    }
    else {
        _ckt->_ord_design->evalTclString("estimate_parasitics -placement");
        _ckt->_ord_timing->getSta()->findRequireds();
    }
    _ckt->_ord_design->evalTclString("puts \"=== OpenSTA Final Timing Summary ===\"");
    _ckt->_ord_design->evalTclString("report_worst_slack -max -digits 3");
    _ckt->_ord_design->evalTclString("report_tns -max -digits 3");
    _ckt->_ord_design->evalTclString("report_worst_slack -min -digits 3");
    _ckt->_ord_design->evalTclString("report_tns -min -digits 3");
    _ckt->_ord_design->evalTclString(
        "puts \"max_slew_violations [sta::max_slew_violation_count] "
        "worst_slew_slack [format %.3f [sta::max_slew_check_slack]]\"");
    _ckt->_ord_design->evalTclString(
        "puts \"max_cap_violations [sta::max_capacitance_violation_count] "
        "worst_cap_slack [format %.3f [sta::max_capacitance_check_slack]]\"");
    _ckt->assertNoIllegalExternalPadBoundaryChain();
    double wns = T[view]->getWorstSlack(clk_name[worst_corner]);
    double tns = T[view]->getTNS(clk_name[worst_corner]);

    double leak = 0.0;
    double tot = 0.0;

    tot = leak;

    double tran_tot, tran_max;
    tran_tot = tran_max = 0.0;
    int tran_num = 0;
    T[view]->getTranVio(tran_tot, tran_max, tran_num);

    double cap_tot, cap_max;
    cap_tot = cap_max = 0.0;
    int cap_num = 0;
    T[view]->getCapVio(cap_tot, cap_max, cap_num);
    ecc::logInfo() << "[view " << view << "] Final WNS after runOrdTO  : " << wns << " ns"
         << endl;
    ecc::logInfo() << "[view " << view << "] Final TNS after runOrdTO  : " << tns << " ns"
         << endl;
    // ecc::logInfo() << "[view " << view << "] Initial Leakage Power    : " << leak
    //      << endl;
    // ecc::logInfo() << "[view " << view << "] Initial Total Power    : " << tot
    //      << endl;
    ecc::logInfo() << "[view " << view << "] Final Tran after runOrdTO : " << tran_tot
         << " ps " << tran_num << " " << tran_max << " ps" << endl;
}
void Sizer::Parallel_Sizer_Launcher() {
    double begin = cpuTime();
    PRFT_PTNUM = 1;
    use_slew_margin = true;
    slew_margin = 1;
    input_slew_margin = 1.0;
    max_time_recovery_iter = 4;
    // attack new
    // use_attack_new = false;
    ATTACK_NEW_RATIO = 20;
    use_margin = true;
    cap_margin = 1;
    //
    // ATTACK_RATIO = 20;
    // MULTI_STEP = 2;
    STA_MARGIN = 0.001;
    // slack_margin = 0.001;
    SetGB(0.001);
    PTimer = new designTiming **[MAX_THREAD];

    for(int i = 0; i < MAX_THREAD; i++) {
        PTimer[i] = new designTiming *[numViews];
        for(int view = 0; view < numViews; view++) {
            ecc::logInfo() << "Launch " << i * numViews + view << "th PT" << endl;
            PTimer[i][view] = LaunchPTimer(i * numViews + view, view);
        }
    }

    for(int i = 0; i < MAX_THREAD; ++i) {
        char filename[100];
        sprintf(filename, "%s%s%s_%d", directory.c_str(), benchname.c_str(),
                "_gtr_no_pt", i);
        ifstream fin(filename);
        if(fin) {
            remove(filename);
        }
        sprintf(filename, "%s%s%s_%d", directory.c_str(), benchname.c_str(),
                "gtr_pt", i);
        ifstream fin2(filename);
        if(fin2) {
            remove(filename);
        }
    }

    char filename[100];
    sprintf(filename, "%s%s_%s", directory.c_str(), benchname.c_str(),
            "GTR_NO_PT");
    ifstream fin(filename);
    if(fin) {
        remove(filename);
    }

    T = PTimer[0];
    // InitPowerBeforeUpdate(g_cells);
    if(MINIMUM || GTR_IN || MAXIMUM) {
        UpdatePTSizes(g_cells);
    }
    auto corner = this->_ckt->_ord_timing->getCorners()[0];
    double totalLeakagePower = 0;
    for(int i = 0; i < numcells; i++) {
        auto inst =
            _ckt->_ord_design->getBlock()->findInst(g_cells[i].name.c_str());
        g_cells[i].static_power = _ckt->_ord_timing->staticPower(inst, corner);
        g_cells[i].isChanged = 0;
        g_cells[i].isStaticChanged = false;
        totalLeakagePower += g_cells[i].static_power;
    }
    totalLeakagePower /= sw_adj;
    init_tot[0] = totalLeakagePower;
    init_leak[0] = totalLeakagePower;
    if(false) {
        runOrdTO();
    }

    double temp_best_tns = DBL_MAX;
    ecc::logPrintf("Start get initial values\n");
    for(unsigned view = 0; view < numViews; ++view) {
        GetSwitchPowerCoef(view);
        GetMaxTranConst(view);

        // if(useOpenSTA) {
        //     string find_timing = T[view]->doOneCmd("find_timing");
        // }
        // UpdatePTSizes(view);
#if 1
        double wns = T[view]->getWorstSlack(clk_name[worst_corner]);
        double tns = T[view]->getTNS(clk_name[worst_corner]);

        double leak = 0.0;
        double tot = 0.0;

        tot = leak;

        double tran_tot, tran_max;
        tran_tot = tran_max = 0.0;
        int tran_num = 0;
        T[view]->getTranVio(tran_tot, tran_max, tran_num);

        double cap_tot, cap_max;
        cap_tot = cap_max = 0.0;
        int cap_num = 0;
        T[view]->getCapVio(cap_tot, cap_max, cap_num);
        ecc::logInfo() << "[view " << view << "] Initial WNS from Timer    : " << wns
             << " ps" << endl;
        ecc::logInfo() << "[view " << view << "] Initial TNS            : " << tns
             << " ps" << endl;
        ecc::logInfo() << "[view " << view << "] Initial Leakage Power    : " << leak
             << endl;
        ecc::logInfo() << "[view " << view << "] Initial Total Power    : " << tot
             << endl;
        ecc::logInfo() << "[view " << view << "] Initial Tran           : " << tran_tot
             << " ps " << tran_num << " " << tran_max << " ps" << endl;
        ecc::logInfo() << "[view " << view << "] Initial cap           : " << cap_tot
             << " ps " << cap_num << " " << cap_max << " fF" << endl;
        if(ISO_TIME) {
            if(viewSlackMargin[view] == 0.0) {
                if(wns < 0) {
                    viewSlackMargin[view] = -wns;
                }
                else {
                    viewSlackMargin[view] = 0.0;
                }
            }

            if(viewTNSMargin[view] == 0.0) {
                viewTNSMargin[view] = tns;
            }
        }

        if(temp_best_tns > tns) {
            temp_best_tns = tns;
        }

        init_wns[view] = wns;
        init_tns[view] = tns;
        init_leak[view] = leak;
        init_tot[view] = tot;
        init_score[view] = calcScore(leak, tns, tran_tot, cap_tot);
        if(view == 0)
            best_tns = tns;
        if(init_wns_worst > init_wns[view]) {
            init_wns_worst = init_wns[view];
        }
#endif
    }

    if(ISO_TIME && ISO_TNS == 0) {
        ISO_TNS = best_tns;
    }

    // double clk_period_org = clk_period[worst_corner];

    if(VAR_GB_TH > 0.0)
        VAR_GB_TH = -clk_period[worst_corner] * 0.1;

    best_score = DBL_MAX;
    second_best_power = DBL_MAX;
    // SetGB();

    if(LEAKOPT) {
        SFlist.resize(PRFT_PTNUM);
        localSFlist.resize(PRFT_PTNUM);
        localSollist.resize(PRFT_PTNUM);
        localAlphalist.resize(PRFT_PTNUM);
        alphalist.resize(PRFT_PTNUM);
        for(unsigned i = 0; i < PRFT_PTNUM; ++i) {
            SFlist[i] = i;
            alphalist[i] = ALPHA;
            nextSFlist.push_back(i);
            nextAlphalist.push_back(ALPHA);
            localSollist[i] = 1000000000;
            localSFlist[i] = PRFT_PTNUM;
        }
        if(PRFT_PTNUM == 1) {
            SFlist[0] = sensFunc;
            nextSFlist[0] = sensFunc;
            localSFlist[0] = sensFunc;
        }
        // Assign initial solution
        best_cells_poweropt.resize(numcells);
        for(unsigned i = 0; i < numcells; i++)
            best_cells_poweropt[i] = g_cells[i];

        // best_failed_cells.resize(numcells);
        // for(unsigned i = 0; i < numcells; i++)
        //     best_failed_cells[i] = g_cells[i];

        // best_failed_cells_poweropt.resize(numcells);
        // for(unsigned i = 0; i < numcells; i++)
        //     best_failed_cells_poweropt[i] = g_cells[i];

        second_best_cells_poweropt.resize(numcells);
        for(unsigned i = 0; i < numcells; i++)
            second_best_cells_poweropt[i] = g_cells[i];

        multi_start_cells_poweropt_1.resize(numcells);
        // multi_start_cells_poweropt_2.resize(numcells);
        // multi_start_cells_poweropt_3.resize(numcells);

        for(unsigned i = 0; i < numcells; i++) {
            multi_start_cells_poweropt_1[i] = g_cells[i];
            // multi_start_cells_poweropt_2[i] = g_cells[i];
            // multi_start_cells_poweropt_3[i] = g_cells[i];
        }

        ecc::logInfo() << "MAXIMUM ITERATION = " << GWTW_MAX << endl;
        for(unsigned GWTWIter = 0; GWTWIter < GWTW_MAX; ++GWTWIter) {
            ecc::logInfo() << GWTWIter << "TH ITERATION !!!" << endl;

            // update sorted SF list and corresponding alpha list
            unsigned idx = 0;
            for(unsigned j = 0; j < nextSFlist.size(); ++j) {
                bool redundant = false;
                for(unsigned k = 0; k < idx; ++k) {
                    if(SFlist[k] == nextSFlist[j]) {
                        redundant = true;
                        break;
                    }
                }
                if(!redundant) {
                    SFlist[idx] = nextSFlist[j];
                    alphalist[idx] = nextAlphalist[j];
                }
                ++idx;
            }
            nextSFlist.clear();
            nextAlphalist.clear();

            ecc::logInfo() << "Current SF list: ";
            for(unsigned j = 0; j < SFlist.size(); ++j) {
                ecc::logInfo() << SFlist[j] << " ";
            }
            ecc::logInfo() << endl;

            ecc::logInfo() << "Current Alpha list: ";
            for(unsigned j = 0; j < alphalist.size(); ++j) {
                ecc::logInfo() << alphalist[j] << " ";
            }
            ecc::logInfo() << endl;

            // if(feasible || PRFT_ONLY)
            ecc::logInfo() << "Start PRFT..." << GWTWIter << endl;

            vector< THREAD_ARGS > thread_args;

            for(int j = 0; j < min(MAX_THREAD, PRFT_PTNUM); j++) {
                ecc::logInfo() << "PREPARING THREAD " << j << endl;
                // j == thread_id == SF option

                THREAD_ARGS tmp_thread_args;
                tmp_thread_args.this_instance = this;
                tmp_thread_args.thread_id = j;
                thread_args.push_back(tmp_thread_args);
            }

            // poweropt + kickopt
            for(int j = 0; j < PRFT_PTNUM; j++) {
                // thread_args[0].thread_id = j;
                static_poweropt_driver((void *)&thread_args[0]);

                if(j != PRFT_PTNUM - 1) {
                    //
                    // use_margin = true;
                    use_slew_margin = true;
                    auto corner_ = this->_ckt->_ord_timing->getCorners()[0];
                    auto _ord_design = _ckt->_ord_design;
                    auto block = _ord_design->getBlock();
                    auto _sta = ord::OpenRoad::openRoad()->getSta();
                    _sta->networkChanged();
                    for(unsigned i = 0; i < numcells; i++) {
                        LibCellInfo *lib_cell_info =
                            getLibCellInfo(best_cells_poweropt[i]);
                        if(lib_cell_info == NULL ||
                           best_cells_poweropt[i].isDontTouch)
                            continue;
                        auto inst = block->findInst(
                            best_cells_poweropt[i].name.c_str());
                        auto new_master =
                            _ord_design->getTech()->getDB()->findMaster(
                                best_cells_poweropt[i].type.c_str());
                        if(inst->getMaster()->getName() !=
                           best_cells_poweropt[i].type) {
                            ecc::logPrintf("Change %s %s\n",
                                   inst->getMaster()->getName().c_str(),
                                   best_cells_poweropt[i].type.c_str());
                            inst->swapMaster(new_master);
                            best_cells_poweropt[i].isStaticChanged = true;
                        }
                        else {
                            best_cells_poweropt[i].isStaticChanged = false;
                        }
                        best_cells_poweropt[i].isChanged = 0;
                    }
                    if(spefFile == "") {
                        _ckt->runGR(50, false, 6);
                    }
                    _ckt->readSpef_opensta(_sta);
                    int corner = 0;
                    for(unsigned i = 0; i < g_nets[corner].size(); ++i) {
                        g_nets[corner][i].cap =
                            _ckt->g_nets[corner][i]
                                .cap;  //* 1e-12 / _sizer->cap_unit

                        if(VERBOSE > 4)
                            ecc::logInfo() << "CORNER " << corner << " NETS --- " << i
                                 << " " << g_nets[corner][i].name << endl;
                        int sink_num = 0;
                        for(auto &sn : _ckt->g_nets[corner][i].subNodeVec) {
                            if(sn.isSink) {
                                sn.pinId = pin2id[sn.pin_name];
                                g_pins[corner][sn.pinId].spef_pin = sn.id;
                            }
                            sink_num += sn.isSink;
                        }
                        g_nets[corner][i].subNodeVec =
                            _ckt->g_nets[corner][i].subNodeVec;
                        // g_nets[corner][i].subNodeResVec;
                    }
                    InitNets();
                    max_time_recovery_iter = 2;
                    ATTACK_RATIO = 30;
                    FIX_SLEW_POST = true;
                    post_slew_opt_output = false;
                    // ATTACK_RATIO = std::max(ATTACK_RATIO, 10);
                }
                // pthread_create(&threads[j], NULL, (),
                //                ;
                ecc::logInfo() << "THREAD CREATE DONE!! " << j << endl;
            }
            for(int j = 0; j < min(MAX_THREAD, PRFT_PTNUM); j++) {
                // pthread_join(threads[j], NULL);
                ecc::logInfo() << "THREAD JOIN DONE!! " << j << endl;
            }
            thread_args.clear();

            if(PRFT_ONLY) {
                ecc::logInfo() << "Best feasible score after POWEROPT on TOP = "
                     << best_score << endl;
            }
            else {
                ecc::logInfo() << "Best feasible score after POWEROPT on TOP = "
                     << best_score << endl;
            }

            // double wns, power;
            // ReportWithPT(best_cells_poweropt, "prft", wns, power);
            time_LeakOpt = cpuTime() - begin;

            if(VERBOSE >= 1) {
                ecc::logInfo() << "BEST score " << best_score << " " << second_best_power
                     << endl;
            }

            if(best_score != DBL_MAX) {
                vector< CellSol > current_cells;
                current_cells.resize(numcells);
                for(unsigned i = 0; i < numcells; i++) {
                    current_cells[i].c_vtype =
                        (int)best_cells_poweropt[i].c_vtype;
                    current_cells[i].c_size =
                        (int)best_cells_poweropt[i].c_size;
                }
                all_cells.push_back(current_cells);
                powerlist.push_back(best_score);
                if(VERBOSE >= 1) {
                    ecc::logInfo() << "COPY SOL FROM BEST DONE " << endl;
                }
            }

            if(second_best_power != DBL_MAX) {
                vector< CellSol > current_cells2;
                current_cells2.resize(numcells);
                for(unsigned i = 0; i < numcells; i++) {
                    current_cells2[i].c_vtype =
                        (int)second_best_cells_poweropt[i].c_vtype;
                    current_cells2[i].c_size =
                        (int)second_best_cells_poweropt[i].c_size;
                }
                all_cells.push_back(current_cells2);
                powerlist.push_back(second_best_power);
                if(VERBOSE >= 1) {
                    ecc::logInfo() << "COPY SOL FROM SECOND BEST DONE " << endl;
                }
            }
            if(VERBOSE >= 1) {
                ecc::logInfo() << "GET NEXT START END" << endl;
            }
        }
        SizeOut(outputDir, best_cells_poweropt);
        // SizeChangeOut(outputDir);
        // double vio1, power1 = 0.0;
        // double vio = 0.0;
        // FinalReport();
    }

    ExitPTimer();
}
void Sizer::FinalReport() {
    //
    SetGB(0);
    use_margin = false;
    use_slew_margin = false;
    auto corner_ = this->_ckt->_ord_timing->getCorners()[0];
    auto _ord_design = _ckt->_ord_design;
    auto block = _ord_design->getBlock();
    auto _sta = ord::OpenRoad::openRoad()->getSta();
    applyOpenStaDbChanges([&]() {
        for(unsigned i = 0; i < numcells; i++) {
            LibCellInfo *lib_cell_info = getLibCellInfo(best_cells_poweropt[i]);
            if(lib_cell_info == NULL || best_cells_poweropt[i].isDontTouch)
                continue;
            auto inst = block->findInst(best_cells_poweropt[i].name.c_str());
            auto new_master = _ord_design->getTech()->getDB()->findMaster(
                best_cells_poweropt[i].type.c_str());
            if(inst->getMaster()->getName() != best_cells_poweropt[i].type) {
                if(new_master) {
                    ecc::logPrintf("Change %s %s\n",
                           inst->getMaster()->getName().c_str(),
                           best_cells_poweropt[i].type.c_str());
                    inst->swapMaster(new_master);
                }
                else {
                    ecc::logPrintf("Change %s %s not found\n",
                           inst->getMaster()->getName().c_str(),
                           best_cells_poweropt[i].type.c_str());
                }
                best_cells_poweropt[i].isStaticChanged = true;
            }
            else {
                best_cells_poweropt[i].isStaticChanged = false;
            }
            best_cells_poweropt[i].isChanged = 1;
        }

        if(!noSPEF && !spefFile.empty()) {
            return;
        }
        if(!PRESERVE_PLACEMENT) {
            _ord_design->evalTclString(
                "set_placement_padding -global -left " + std::to_string(dp_padding) +
                " -right " + std::to_string(dp_padding));
            legalizePlacement();
        }
    });

    if(!noSPEF && spefFile != "") {
        return;
    }

    double begin = cpuTime();
    refreshOpenStaParasitics(true);
    ecc::logPrintf("Run Global Routing Time %f\n", cpuTime() - begin);
    _ckt->readSpef_opensta(_sta);
    int corner = 0;
    ofstream ofs("net_changed.log");
    for(unsigned i = 0; i < g_nets[corner].size(); ++i) {
        double old_cap = g_nets[corner][i].cap;
        g_nets[corner][i].cap =
            _ckt->g_nets[corner][i].cap;  //* 1e-12 / _sizer->cap_unit
        if(fabs(g_nets[corner][i].cap - old_cap) > 1e-6) {
            ofs << "CAP CHANGE " << g_nets[corner][i].name << ","
                << g_nets[corner][i].cap << "," << old_cap << ","
                << g_nets[corner][i].outpins.size() << ",";
            if(_ckt->g_nets[corner][i].inpin != UINT_MAX) {
                ofs << getFullPinName(g_pins[0][_ckt->g_nets[corner][i].inpin]);
            }
            ofs << endl;
        }
        if(VERBOSE > 4)
            ecc::logInfo() << "CORNER " << corner << " NETS --- " << i << " "
                 << g_nets[corner][i].name << endl;
        int sink_num = 0;
        for(auto &sn : _ckt->g_nets[corner][i].subNodeVec) {
            if(sn.isSink) {
                sn.pinId = pin2id[sn.pin_name];
                g_pins[corner][sn.pinId].spef_pin = sn.id;
            }
            sink_num += sn.isSink;
        }
        g_nets[corner][i].subNodeVec = _ckt->g_nets[corner][i].subNodeVec;
    }
    ofs.close();
    InitNets();
    max_time_recovery_iter -= 4;
    max_time_recovery_iter = std::max(max_time_recovery_iter, 1);
    // ATTACK_RATIO -= 20;
    ATTACK_RATIO = std::max(ATTACK_RATIO, 10);
    int view = 0;
    GetSwitchPowerCoef(view);
    GetMaxTranConst(view);
    cells = new CELL[numcells];
    for(unsigned i = 0; i < numcells; i++)
        cells[i] = best_cells_poweropt[i];
    pins = new PIN *[numViews];
    for(unsigned i = 0; i < numViews; ++i) {
        pins[i] = new PIN[numpins];
        for(unsigned j = 0; j < numpins; j++)
            pins[i][j] = g_pins[i][j];
    }

    nets = new NET *[numCorners];
    for(unsigned i = 0; i < numCorners; ++i) {
        nets[i] = new NET[numnets];
        for(unsigned j = 0; j < numnets; j++) {
            nets[i][j] = g_nets[i][j];
        }
    }
    int count = 0;
    UpdatePTSizes(view, count);
    UpdateCapsFromCells();
    CallTimer(view);
    CorrelatePT(view);
    CalcStats(0);
    showAllSlew(0, "after_tran.csv");
}

void Sizer::PostWNSOpt(string input, unsigned view) {
    cells = new CELL[numcells];
    for(unsigned i = 0; i < numcells; i++)
        cells[i] = g_cells[i];

    pins = new PIN *[numViews];

    for(unsigned i = 0; i < numViews; ++i) {
        pins[i] = new PIN[numpins];
        for(unsigned j = 0; j < numpins; j++)
            pins[i][j] = g_pins[i][j];
    }

    nets = new NET *[numCorners];

    for(unsigned i = 0; i < numCorners; ++i) {
        nets[i] = new NET[numnets];
        for(unsigned j = 0; j < numnets; j++)
            nets[i][j] = g_nets[i][j];
    }

    SizeIn(outputDir);

    UpdateCapsFromCells();
    int count = 0;
    UpdatePTSizes(view, count);
    CallTimer();
    CorrelatePT();
    CalcStats();

    double oneTMR = cpuTime();

    int optCnt = 0;
    int stopCnt = 0;
    double skew_violation_prev;
    while(skew_violation > clk_period[worst_corner] * 0.1) {
        skew_violation_prev = skew_violation;
        ecc::logInfo() << "[PostWNSOpt] without correlation # " << optCnt++ << endl;
        if(FIX_CAP_REPAIR) {
            FwdFixCapViolation();
            BwdFixCapViolation();
        }
        if(FIX_SLEW_REPAIR) {
            FwdFixSlewViolation(1.0);
        }
        CallTimer();
        CalcStats();
        if(skew_violation > 0) {
            ecc::logInfo() << "[PostWNSOpt] OptWNSPathCell # " << optCnt << endl;
            OptWNSPath(DNSIZE);
        }
        CallTimer();
        CalcStats();
        if(skew_violation_prev <= skew_violation)
            stopCnt++;
        else
            stopCnt = 0;
        if(stopCnt > 2) {
            break;
        }
    }
    oneTMR = (cpuTime() - oneTMR);
    ecc::logInfo() << "[PostWNSOpt] without correlation runtime ---" << oneTMR << endl;
    string size_str = input + "_aft_wns_opt";
    SizeOut(outputDir);

    oneTMR = cpuTime();
    UpdateCapsFromCells();
    CallTimer();
    CorrelatePT();
    CalcStats();

    optCnt = 0;
    while(skew_violation > 0.) {
        ecc::logInfo() << "[PostWNSOpt] with correlation # " << optCnt++ << endl;
        skew_violation_prev = skew_violation;
        if(FIX_CAP_REPAIR) {
            FwdFixCapViolation();
            BwdFixCapViolation();
        }
        if(FIX_SLEW_REPAIR) {
            FwdFixSlewViolation(1.0);
        }
        CallTimer();
        CorrelatePT();
        CalcStats();
        if(skew_violation > 0) {
            ecc::logInfo() << "[PostWNSOptCorr] OptWNSPath Upsize # " << optCnt << endl;
            OptWNSPath(UPSIZE);
        }
        // if ( skew_violation > 0 ) {
        //    ecc::logInfo() << "[PostWNSOptCorr] OptWNSPathGray # " << optCnt << endl;
        //    OptWNSPathGray(true,0, 3,3,2);
        //}
        CallTimer();
        CorrelatePT();
        CalcStats();
        if(skew_violation > 0 && skew_violation_prev <= skew_violation) {
            ecc::logInfo() << "Fail to find feasible solution :(" << endl;
            break;
        }
    }

    feasible = true;
    best_cells.resize(numcells);
    for(unsigned i = 0; i < numcells; i++)
        best_cells[i] = cells[i];

    oneTMR = (cpuTime() - oneTMR);
    ecc::logInfo() << "[PostWNSOpt] with correlation runtime: " << oneTMR << endl;

    // size_str = input + "_wns_opt_final";
    SizeOut(outputDir);
    double wns, power;
    // ReportWithPT(best_cells, "wns_opt_final", wns, power);
}

// JL
void Sizer::Post_PowerOpt(int thread_id) {
    cells = new CELL[numcells];
    vector< CELL > best_cells_local;
    // vector< CELL > best_failed_cells_local;
    best_cells_local.resize(numcells);
    // best_failed_cells_local.resize(numcells);

    // pthread_mutex_lock(&mutex1);

    T = PTimer[thread_id];

    pins = new PIN *[numViews];

    for(unsigned i = 0; i < numViews; ++i) {
        pins[i] = new PIN[numpins];
        for(unsigned j = 0; j < numpins; j++) {
            pins[i][j] = g_pins[i][j];
            assert(pins[i][j].cap < 1e31);
        }
    }

    nets = new NET *[numCorners];

    for(unsigned i = 0; i < numCorners; ++i) {
        nets[i] = new NET[numnets];
        for(unsigned j = 0; j < numnets; j++)
            nets[i][j] = g_nets[i][j];
    }

    int start_index = (int)thread_id / GWTW_DIV;
    ecc::logInfo() << "THREAD " << thread_id << " -- START INDEX " << start_index << endl;

    if(start_index >= GWTW_NUM_START) {
        ecc::logInfo() << "WARNING: start index " << start_index << " is larger than "
             << GWTW_NUM_START << endl;
    }

    for(unsigned i = 0; i < numcells; i++) {
        cells[i] = best_cells_poweropt[i];
        best_cells_local[i] = best_cells_poweropt[i];
        // best_failed_cells_local[i] = best_cells_poweropt[i];
    }

    if(start_index == 0) {
        ecc::logInfo() << "(" << thread_id << ") COPY BEST SO FAR CELLS DONE" << endl;
    }
    else if(start_index == 1) {
        ecc::logInfo() << "(" << thread_id << ") COPY MULTI START 1 CELLS DONE" << endl;
    }
    else if(start_index == 2) {
        ecc::logInfo() << "(" << thread_id << ") COPY MULTI START 2 CELLS DONE" << endl;
    }
    else if(start_index == 3) {
        ecc::logInfo() << "(" << thread_id << ") COPY MULTI START 3 CELLS DONE" << endl;
    }
    else {
        ecc::logInfo() << "(" << thread_id << ") COPY BEST SO FAR CELLS DONE" << endl;
    }

    // pthread_mutex_unlock(&mutex1);

    for(unsigned view = 0; view < numViews; ++view) {
        // ecc::logInfo() << "WORST SLACK (" << thread_id << "-" << view << ") "
        //      << T[view]->getWorstSlack(clk_name[worst_corner]) << endl;
        // bool old_att = CORR_AAT;
        // CORR_AAT = true;
        UpdateCapsFromCells();
        CallTimer(view);
        CorrelatePT((unsigned)thread_id, view);
        CalcStats((unsigned)thread_id, true, "Initial after corr", view);
        // CORR_AAT = old_att;
    }
    if(init_score[0] == DBL_MAX) {
        int view = 0;
        init_wns[view] = worst_slack_worst;
        init_tns[view] = skew_violation;
        init_leak[view] = power;
        // init_tot[view] = power;
        init_score[view] = score;

        ecc::logInfo() << "[view " << view
             << "] Initial WNS from Timer    : " << init_wns[view] << " ps"
             << endl;
        ecc::logInfo() << "[view " << view
             << "] Initial TNS            : " << init_tns[view] << " ps"
             << endl;
        ecc::logInfo() << "[view " << view
             << "] Initial Leakage Power    : " << init_leak[view] << endl;
        ecc::logInfo() << "[view " << view
             << "] Initial Tran           : " << slew_violation << endl;
        ecc::logInfo() << "[view " << view
             << "] Initial Cap           : " << cap_violation << endl;
    }

    ecc::logInfo() << "THREAD " << thread_id << " BEFORE KICKOPT" << endl;
    // CalcStats((unsigned)thread_id, true, "OPT");
    if(VERBOSE >= 1) {
        ecc::logInfo() << "Initial TNS from Sizer     : " << skew_violation << " ns"
             << endl;
    }
    // bool flag = false;
    double TMR;
    TMR = cpuTime();

    char opt_str[250];
    sprintf(opt_str, "%d", thread_id);

    double kick_ratio = KICK_RATIO;

    ecc::logInfo() << "Worst slack among views : " << worst_slack_worst << endl;

    best_score_local =
        calcScore(power, skew_violation, slew_violation, cap_violation);
    best_score = best_score_local;
    SizeOut(outputDir);
    ecc::logPrintf("Initial Score: %f\n", best_score_local);
    // pthread_mutex_lock(&mutex1);
    if(localSFlist[thread_id] == PRFT_PTNUM) {
        localSFlist[thread_id] = SFlist[thread_id];
        localAlphalist[thread_id] = alphalist[thread_id];
    }
    else {
        int div_index = (int)thread_id % GWTW_DIV;
        localSFlist[thread_id] = SFlist[div_index];
        localAlphalist[thread_id] = alphalist[div_index];
    }
    // pthread_mutex_unlock(&mutex1);

    TMR = cpuTime();

    int kick_max_iteration = KICK_MAX;

    if(FAST)
        kick_max_iteration = 1;

    double kick_slack = KICK_SLACK;
    double kick_leak_exponent = 1.0;
    best_alpha_local = localAlphalist[thread_id];
    local_alpha = localAlphalist[thread_id];

    vector< CELL > int_best_cells;
    int_best_cells.resize(numcells);
    for(unsigned i = 0; i < numcells; i++)
        int_best_cells[i] = cells[i];
    bool updated_int_best_cells = false;
    double cur_best_tns = skew_violation;

    int stuck_count = 0;
    int kick_stuck_count = 0;
    bool peephole_opt = false;

    if(TRIAL_MOVE) {
        kick_max_iteration = 1;
    }
    ecc::logInfo() << "KICK MAX ITER: " << kick_max_iteration << endl;
    double prev_best_score = best_score;
    bool g_updated_local = false;
    bool g_updated_failed_local = false;
    for(unsigned i = 0; i < kick_max_iteration; i++) {
        if(kick_stuck_count >= PEEPHOLE_ITER && PEEPHOLE) {
            peephole_opt = true;
        }

        // for(unsigned view = 0; view < numViews; ++view) {
        //     CallTimer(view);
        //     CorrelatePT((unsigned)thread_id, view);
        //     CalcStats((unsigned)thread_id, true, "KICK_MOVE", view);
        // }

        if(stuck_count == ALPHA_STUCK) {
            local_alpha = local_alpha * 1.1;
            if(local_alpha > 1.0) {
                local_alpha = ALPHA;
            }
            stuck_count = 0;
        }

        if(kick_ratio > MAX_KICK_RATIO) {
            kick_ratio = MAX_KICK_RATIO;
        }
        if(i != 0 && leak_iter != 0) {
            ecc::logInfo() << "(" << thread_id << ") KICK iteration " << i + 1
                 << " with ratio = " << kick_ratio
                 << "  alpha = " << local_alpha << endl;
            unsigned kick_cnt = 0;
            if(KICK_METHOD == 1) {
                kick_cnt = IncrSlack(kick_leak_exponent, local_alpha);
            }
            else if(KICK_METHOD == 2) {
                kick_cnt = IncrSlackMore(kick_ratio, local_alpha);
            }
            else if(KICK_METHOD == 3) {
                kick_cnt = IncrSlackRandom(kick_ratio, kick_slack);
            }
            else if(KICK_METHOD == 4) {
                kick_cnt = IncrTNS(kick_ratio, kick_slack);
            }
            else {
                kick_cnt = IncrSlackRandom(kick_ratio, kick_slack);
            }
            KICK_STEP = max(KICK_STEP - 1, 1);
            if(kick_cnt == 0) {
                if(KICK_METHOD != 3) {
                    kick_cnt = IncrSlackRandom(kick_ratio, kick_slack);
                }
                else {
                    kick_cnt = IncrSlackRandom(kick_ratio, kick_slack * 2);
                }
            }
            if(kick_cnt > 0) {
                for(unsigned view = 0; view < numViews; ++view) {
                    CallTimer(view);
                    CorrelatePT((unsigned)thread_id, view);
                    CalcStats((unsigned)thread_id, true, "AFTER_KICK", view);
                }
            }
        }

        // int leak_iter = 0;
        int max_iter = optEffort;
        if(FAST)
            max_iter = 1;
        bool updated_local = false;
        bool updated_failed_local = false;
        toler = .0;

        int max_trial = max_iter;
        int iter = 0;
        if(TRIAL_MOVE) {
            max_trial = 1;
        }

        bool all_feasible = false;
        // timing x 3 + legal power reduction x1 each loop, controlled by
        // max_trial
        while(iter < max_trial) {
            iter++;

            // Smart guardband
            if(TOLER_STEP) {
                toler = init_wns[0] * TOLERANCE;
            }
            else {
                toler =
                    init_wns[0] * TOLERANCE * (1 - iter / (double)TOLER_NUM);
            }

            if(toler < init_wns[0] * TOLER_STOP) {
                toler = init_wns[0] * TOLER_STOP;
            }

            if(POWER_OPT_GB != 0.0) {
                toler = POWER_OPT_GB;
            }
            else {
                if(i == kick_max_iteration - 1) {
                    toler = -0.005;
                }
            }

            // for(unsigned view = 0; view < numViews; ++view) {
            //     CalcStats((unsigned)thread_id, true, "BEFORE_PWR_OPT", view);
            // }

            ecc::logInfo() << i << "-" << iter << "-" << leak_iter
                 << "th iteration, tolerance = " << toler << "ns"
                 << "/" << worst_slack << "ns"
                 << " " << worst_slack_worst << endl;

            unsigned accept = 0;

            ecc::logInfo() << "TEST " << ISO_TNS << " " << ISO_TIME << " "
                 << skew_violation_worst << endl;

            all_feasible = false;

            // unsigned max_time_recovery_iter = 7;
            // Timing recovery
            for(unsigned time_recovery_iter = 0;
                time_recovery_iter < max_time_recovery_iter;
                time_recovery_iter++) {
                if(all_feasible) {
                    break;
                }
                if(time_recovery_iter >= 4 && updatePinFast) {
                    updatePinFast = true;
                }
                for(unsigned j = 0; j < numcells; j++)
                    cells[j].touched = false;

                int view = 0;
                // CallTimer(view);
                // CorrelatePT((unsigned)thread_id, view);
                // CalcStats((unsigned)thread_id, true, "AFTER_PWROPT",
                // view);

                ///// check fanout cells of critical cells
                if(TABU) {
                    for(unsigned j = 0; j < numcells; j++) {
                        if(GetCellSlack(cells[j]) < 0.0) {
                            for(unsigned k = 0; k < cells[j].fos.size(); ++k) {
                                if(cells[j].fos[k] != UINT_MAX) {
                                    cells[cells[j].fos[k]].touched = true;
                                }
                            }
                        }
                    }
                }
                // TGR loop
                if(slew_violation != 0.0 || (skew_violation != 0.0) ||
                   cap_violation != 0.0) {
                    double begin = cpuTime();
                    unsigned same_ss_count = 0;
                    unsigned swap_cnt = 0;
                    double prev_ss = skew_violation;
                    double curr_ss = prev_ss;
                    unsigned change = 0;
                    unsigned all_change = 0;

                    change = 0;
                    all_change = 0;
                    double prev_best_score = best_score_local;
                    if(INIT_WORST_PATH && init_wns[view] > 0) {
                        if(viewVioCnt[view] < 30 && viewVioCnt[view] > 0) {
                            change += InitWNSPath(view, 1);
                        }
                        if(change > 0) {
                            CallTimer(view);
                            CorrelatePT((unsigned)thread_id, view);
                            CalcStats((unsigned)thread_id, true,
                                      "AFTER_INIT_PATH", view);
                        }
                    }

                    if(FIX_CAP_REPAIR && cap_violation != 0) {
                        // CalcStats((unsigned)thread_id, true,
                        //           "AFTER_FIX_CAP", view);
                        change += FwdFixCapViolation(view);
                        change += BwdFixCapViolation(view);
                        CallTimer(view);
                        if(change > 1000) {
                            CorrelatePT((unsigned)thread_id, view);
                        }
                        CalcStats((unsigned)thread_id, true, "AFTER_FIX_CAP",
                                  view);
                    }
                    ecc::logPrintf(
                        "CURRENT TNS: %f, slew: %f, cap: %f, power: %f, "
                        "score: %f\n",
                        skew_violation, slew_violation, cap_violation, power,
                        score);
                    if(score < best_score_local) {
                        ecc::logInfo() << "(" << thread_id
                             << ") Local best score is updated "
                                "(inside of power opt loop) "
                             << best_score_local << "/" << score << endl;
                        best_score_local = score;
                        best_alpha_local = local_alpha;
                        string temp = (string)opt_str + "_best_infeasible";
                        // pthread_mutex_lock(&mutex1);
                        SizeOut(outputDir);
                        // pthread_mutex_unlock(&mutex1);
                        for(unsigned j = 0; j < numcells; ++j) {
                            best_cells_local[j] = cells[j];
                        }
                        updated_local = true;
                    }

                    all_change += change;
                    change = 0;
                    if(FIX_SLEW_REPAIR && slew_violation != 0.0) {
                        change += FwdFixSlewViolation(1.0, view);
                        // change += BwdFixSlewViolation(1.0, view);
                        if(FIX_SLEW_POST) {
                            change += FwdFixSlewViolationPost(1.0, view);
                        }
                        if(iter % 2 == 0) {
                        }
                        CallTimer(view);
                        if(change > 1000) {
                            CorrelatePT((unsigned)thread_id, view);
                        }
                        CalcStats((unsigned)thread_id, true, "AFTER_FIX_SLEW",
                                  view);
                    }

                    ecc::logPrintf(
                        "CURRENT TNS: %f, slew: %f, cap: %f, power: %f, "
                        "score: %f\n",
                        skew_violation, slew_violation, cap_violation, power,
                        score);
                    if(score < best_score_local) {
                        ecc::logInfo() << "(" << thread_id
                             << ") Local best score is updated "
                                "(inside of power opt loop) "
                             << best_score_local << "/" << score << endl;
                        best_score_local = score;
                        best_alpha_local = local_alpha;
                        string temp = (string)opt_str + "_best_infeasible";
                        // pthread_mutex_lock(&mutex1);
                        SizeOut(outputDir);
                        // pthread_mutex_unlock(&mutex1);
                        for(unsigned j = 0; j < numcells; ++j) {
                            best_cells_local[j] = cells[j];
                        }
                        updated_local = true;
                    }

                    if(use_count_npath) {
                        if(sensFuncT == 5 || sensFuncT == 8 || sensFuncT == 9) {
                            CountNPaths(view);
                        }
                    }
                    if(time_recovery_iter == 0 && use_attack_new) {
                        change += AttackNew(i + 1, GLOBAL, ATTACK_NEW_RATIO,
                                            1.0, local_alpha, 1.0, thread_id,
                                            TIMING_OPT_GB, view);
                        if(change > 0) {
                            // for(int i = 0; i < numcells; i++) {
                            //     assert(change[i] == cells[i].isChanged);
                            // }
                            // ecc::logPrintf();
                            CallTimer(view);
                            CorrelatePT((unsigned)thread_id, view);
                            CalcStats((unsigned)thread_id, true,
                                      "AFTER_TIM_REC", view);
                        }
                        use_attack_new = false;
                    }
                    else {
                        if(FIX_GLOBAL) {
                            change += Attack(i + 1, GLOBAL, ATTACK_RATIO, 1.0,
                                             local_alpha, 1.0, thread_id,
                                             TIMING_OPT_GB, view);
                            // change += Attack(i + 1, GLOBAL,
                            // ATTACK_RATIO, 1.0,
                            //                  local_alpha, 1.0, thread_id,
                            //                  TIMING_OPT_GB, view);
                        }
                        if(change > 0) {
                            // for(int i = 0; i < numcells; i++) {
                            //     assert(change[i] == cells[i].isChanged);
                            // }
                            // ecc::logPrintf();
                            CallTimer(view);
                            CorrelatePT((unsigned)thread_id, view);
                            CalcStats((unsigned)thread_id, true,
                                      "AFTER_TIM_REC", view);
                        }
                    }

                    all_change += change;
                    // change +=
                    //     Attack(i + 1, FINESWAP, 30, 1.0, local_alpha,
                    //            thread_id, TIMING_OPT_GB, view);

                    change = 0;
                    if(time_recovery_iter == max_time_recovery_iter - 1) {
                        if(FIX_CAP_REPAIR) {
                            change += FwdFixCapViolation(view);
                            change += BwdFixCapViolation(view);
                            CallTimer(view);
                            if(change > 0 && time_recovery_iter != 0) {
                                CorrelatePT((unsigned)thread_id, view);
                            }
                            CalcStats((unsigned)thread_id, true,
                                      "AFTER_FIX_CAP", view);
                        }
                        if(score < best_score_local) {
                            ecc::logInfo() << "(" << thread_id
                                 << ") Local best score is updated "
                                    "(inside of power opt loop) "
                                 << best_score_local << "/" << score << endl;
                            best_score_local = score;
                            best_alpha_local = local_alpha;
                            string temp = (string)opt_str + "_best_infeasible";
                            // pthread_mutex_lock(&mutex1);
                            SizeOut(outputDir);
                            // pthread_mutex_unlock(&mutex1);
                            for(unsigned j = 0; j < numcells; ++j) {
                                best_cells_local[j] = cells[j];
                            }
                            updated_local = true;
                        }
                        if(FIX_SLEW_REPAIR) {
                            change += FwdFixSlewViolation(1.0, view);
                            // change += BwdFixSlewViolation(1.0, view);
                            if(FIX_SLEW_POST) {
                                change += FwdFixSlewViolationPost(1.0, view);
                            }
                            CallTimer(view);
                            if(change > 0 && time_recovery_iter != 0) {
                                CorrelatePT((unsigned)thread_id, view);
                            }
                            CalcStats((unsigned)thread_id, true,
                                      "AFTER_FIX_SLEW", view);
                            all_change += change;
                            change = 0;
                        }
                    }
                    ecc::logPrintf(
                        "CURRENT TNS: %f, slew: %f, cap: %f, power: %f, "
                        "score: %f\n",
                        skew_violation, slew_violation, cap_violation, power,
                        score);

                    if(score < best_score_local) {
                        ecc::logInfo() << "(" << thread_id
                             << ") Local best score is updated "
                                "(inside of power opt loop) "
                             << best_score_local << "/" << score << endl;
                        best_score_local = score;
                        best_alpha_local = local_alpha;
                        string temp = (string)opt_str + "_best_infeasible";
                        // pthread_mutex_lock(&mutex1);
                        SizeOut(outputDir);
                        // pthread_mutex_unlock(&mutex1);
                        for(unsigned j = 0; j < numcells; ++j) {
                            best_cells_local[j] = cells[j];
                        }
                        updated_local = true;
                    }
                    double wns;
                    wns = min(max_neg_rslk, max_neg_fslk);

                    ecc::logInfo() << "TIM_REC_LOOP " << thread_id
                         << " view/itr/wns/TNS/PWR/swap : " << view << " " << i
                         << " " << wns << " " << skew_violation << " " << power
                         << " " << all_change << " " << same_ss_count << endl;

                    prev_ss = curr_ss;
                    curr_ss = skew_violation;

                    same_ss_count =
                        (prev_ss > curr_ss && all_change > 0)
                            ? 0
                            : same_ss_count + 1;  // slack degradation

                    ecc::logInfo() << "CURRENT BEST TNS " << cur_best_tns << " "
                         << curr_ss << endl;
                    if(curr_ss != 0 && cur_best_tns > curr_ss) {
                        ecc::logInfo() << "UPDATE BEST TNS..." << endl;
                        cur_best_tns = curr_ss;
                        for(unsigned j = 0; j < numcells; j++)
                            int_best_cells[j] = cells[j];
                        updated_int_best_cells = true;
                    }

                    if(all_change == 0 ||
                       (prev_best_score < 0.97 * best_score_local)) {
                        break;
                    }

                    viewRuntime[view] += cpuTime() - begin;
                    ecc::logInfo() << "Runtime TimingRecovery so far for view " << view
                         << " : " << viewRuntime[view] << " sec. ("
                         << viewRuntime[view] / 60 << " min. )" << endl;
                }
                if(skew_violation < 0.03 && slew_violation < 0.03 &&
                   cap_violation < 0.03) {
                    all_feasible = true;
                }
                Profile();
            }
            for(unsigned j = 0; j < numcells; j++)
                cells[j].touched = false;
            // POWER reduce loop
            if(false) {
                ecc::logInfo() << "REDUCE LEAK ITER " << init_wns_worst << " " << toler
                     << " " << TOLERANCE << " " << TOLER_STOP << " "
                     << leak_iter << " " << worst_slack_worst << " "
                     << skew_violation_worst << endl;

                worst_slack_worst = DBL_MAX;
                skew_violation_worst = DBL_MAX;
                // for(unsigned view = 0; view < numViews; ++view) {
                //     CallTimer(view);
                //     CorrelatePT((unsigned)thread_id, view);
                //     CalcStats((unsigned)thread_id, true, "BEFORE_PWR_OPT",
                //               view);
                // }
                // CalcStats((unsigned)thread_id, true, "BEFORE_PWR_OPT");

                accept = ReducePowerLegal(
                    thread_id, localSFlist[thread_id], leak_iter, local_alpha,
                    toler, peephole_opt, updated_local, best_cells_local);
                tot_accept += accept;

                leak_iter++;
                if(accept != 0) {
                    for(unsigned view = 0; view < numViews; ++view) {
                        CallTimer(view);
                        CorrelatePT((unsigned)thread_id, view);
                        CalcStats((unsigned)thread_id, true, "IN_PWROPT_LOOP",
                                  view);
                    }
                }

                if(score < best_score_local) {
                    ecc::logInfo() << "(" << thread_id
                         << ") Local best power is updated "
                            "(inside of power opt loop) "
                         << score << "/" << best_score_local << endl;
                    best_score_local = score;
                    best_alpha_local = local_alpha;
                    string temp = (string)opt_str + "_feasible";
                    SizeOut(outputDir);
                    // report results

                    for(unsigned view = 0; view < numViews; ++view) {
                        // if(useOpenSTA) {
                        //     string find_timing =
                        //     T[view]->doOneCmd("find_timing");
                        // }
                        unsigned swap_count = 0;
                        unsigned uptype_swap_count = 0;
                        unsigned downtype_swap_count = 0;
                        unsigned upsize_swap_count = 0;
                        unsigned downsize_swap_count = 0;
                        unsigned hvt_init_count = 0;
                        unsigned hvt_count = 0;
                        for(unsigned i = 0; i < numcells; ++i) {
                            if(g_cells[i].c_vtype == s) {
                                ++hvt_init_count;
                            }
                        }

                        for(unsigned i = 0; i < numcells; ++i) {
                            if(g_cells[i].type != cells[i].type) {
                                ++swap_count;
                                if(cells[i].c_vtype > g_cells[i].c_vtype) {
                                    uptype_swap_count++;
                                }
                                else if(cells[i].c_vtype < g_cells[i].c_vtype) {
                                    downtype_swap_count++;
                                }

                                if(cells[i].c_size > g_cells[i].c_size) {
                                    upsize_swap_count++;
                                }
                                else if(cells[i].c_size < g_cells[i].c_size) {
                                    downsize_swap_count++;
                                }
                            }
                            if(cells[i].c_vtype == s) {
                                ++hvt_count;
                            }
                        }

                        double hvt_ratio = (double)hvt_count / (double)numcells;
                        double hvt_init_ratio =
                            (double)hvt_init_count / (double)numcells;

                        ecc::logInfo() << "[view " << view
                             << "] # Swap             : " << swap_count << "("
                             << uptype_swap_count << "/" << downtype_swap_count
                             << "/" << upsize_swap_count << "/"
                             << downsize_swap_count << ")" << endl;
                        ecc::logInfo() << "[view " << view
                             << "] % HVT cells        : " << hvt_ratio * 100.0
                             << endl;
                        ecc::logInfo() << "[view " << view << "] % HVT cells (init) : "
                             << hvt_init_ratio * 100.0 << endl;
                    }
                    Profile();

                    // msk (it was ++j)
                    // FIXME:
                    // for(unsigned j = 0; j < numcells; ++j) {
                    //     best_cells_local[j] = cells[j];
                    // }
                    updated_local = true;
                }
                ecc::logInfo() << "(" << thread_id
                     << ") Power after power reduction iteration "
                     << leak_iter + 1 << " : " << power << endl;
            }

            if(!updated_local) {
                break;
            }
        }

        if(FINAL_PWR_OPT) {
            FinalPowerOpt(SLK_TH, thread_id);
            for(unsigned view = 0; view < numViews; ++view) {
                CallTimer(view);
                if(CORR_PT) {
                    CorrelatePT((unsigned)thread_id, view);
                }
            }
            CalcStats((unsigned)thread_id, true, "FINAL_PWR_OPT");
        }
        if(best_score_local < best_score) {
            ecc::logInfo() << "(" << thread_id
                 << ") Local best score is updated -- final power opt. "
                 << best_score << "/" << best_score_local << endl;
            best_score = best_score_local;
            string temp = (string)opt_str + "_best_infeasible";
            // pthread_mutex_lock(&mutex1);
            SizeOut(outputDir);
            // pthread_mutex_unlock(&mutex1);
            for(unsigned j = 0; j < numcells; ++j) {
                best_cells_poweropt[j] = best_cells_local[j];
            }
            updated_local = true;
        }
        ecc::logInfo() << "(" << thread_id << ") Power after KICK iteration " << i + 1
             << " kick ratio " << kick_ratio << " : " << power << endl;
        // local parameters update
        if(updated_local) {
            if(all_feasible) {
                ecc::logInfo() << "(" << thread_id
                     << ") Local best score is updated (in the kick loop) "
                     << power << "/" << best_score_local << endl;
            }
            else
                ecc::logInfo() << "(" << thread_id
                     << ") Local best failed power is "
                        "updated (in the kick loop) "
                     << power << "/" << best_score_local << endl;
            // degrade_count = 0;
            kick_ratio = kick_ratio * 0.9;
            kick_slack = kick_slack * 0.9;
            kick_leak_exponent = kick_leak_exponent * 1.1;
        }
        else {
            kick_ratio = kick_ratio * 2;
            kick_slack = kick_slack * 3;
            kick_leak_exponent = kick_leak_exponent * 0.1;
            stuck_count++;
            ecc::logInfo() << "(" << thread_id
                 << ") current best_score_local = " << best_score_local << endl;
        }

        // if (cpuTime()-global_begin > 0.9 * RuntimeLimit) break;

        if(prev_best_score <= best_score) {
            kick_stuck_count++;
            ecc::logInfo() << "KICK STUCK " << prev_best_score << " " << best_score << " "
                 << kick_stuck_count << endl;
        }
        else {
            ecc::logInfo() << "KICK NO STUCK " << prev_best_score << " " << best_score
                 << " " << kick_stuck_count << endl;
        }

        if(updated_local)
            g_updated_local = true;
        if(updated_failed_local)
            g_updated_failed_local = true;
    }

    TMR = (cpuTime() - TMR);
    ecc::logInfo() << "(" << thread_id << ") Power after KICK: " << power
         << " kick: " << kick_ratio << " cpu time: " << TMR
         << " total time: " << cpuTime() - global_begin << endl;

    // if(skew_violation == 0. || WIRE_METRIC != ND)
    // WIRE_METRIC != ND
    // FIXME:
    if(false) {
        // pthread_mutex_lock(&mutex1);

        ecc::logInfo() << "(" << thread_id << ") uses SF" << localSFlist[thread_id]
             << endl;

        // localSollist stores best leakage power for SF, used for sorting SFs
        // (idx = SF, value = best leakage)
        localSollist[localSFlist[thread_id]] = best_score_local;

        ecc::logInfo() << "best_score/best_score_local: " << best_score << " "
             << best_score_local << endl;

        // insert SF into sorted list for next iteration
        if(PRFT_PTNUM != 1) {
            vector< unsigned >::iterator it;
            vector< double >::iterator it2;
            it2 = nextAlphalist.begin();
            bool inserted = false;
            for(it = nextSFlist.begin(); it != nextSFlist.end(); ++it, ++it2) {
                ecc::logInfo() << "(" << thread_id << ") nextSFitem " << *it << endl;
                ecc::logInfo() << "(" << thread_id << ") solution " << localSollist[*it]
                     << endl;
                if(best_score_local < localSollist[*it]) {
                    nextSFlist.insert(it, localSFlist[thread_id]);
                    nextAlphalist.insert(it2, best_alpha_local);
                    inserted = true;
                    break;
                }
            }
            if(!inserted) {
                nextSFlist.push_back(localSFlist[thread_id]);
                nextAlphalist.push_back(best_alpha_local);
            }
        }
        else {
            if(best_score_local >= best_score) {
                ecc::logInfo() << "Optimization got stuck with the current SF " << endl;
                nextSFlist.push_back(sensFunc2);
                nextAlphalist.push_back(best_alpha_local);
            }
            else {
                nextSFlist.push_back(localSFlist[thread_id]);
                nextAlphalist.push_back(best_alpha_local);
            }
        }

        ecc::logInfo() << "(" << thread_id << ") Current SF next list: ";
        for(unsigned i = 0; i < nextSFlist.size(); ++i) {
            ecc::logInfo() << nextSFlist[i] << " ";
        }
        ecc::logInfo() << endl;

        ecc::logInfo() << "(" << thread_id << ") Current Alpha next list: ";
        for(unsigned i = 0; i < nextAlphalist.size(); ++i) {
            ecc::logInfo() << nextAlphalist[i] << " ";
        }
        ecc::logInfo() << endl;

        ecc::logInfo() << "Current local value list: ";
        for(unsigned i = 0; i < localSollist.size(); ++i) {
            ecc::logInfo() << localSollist[i] << " ";
        }
        ecc::logInfo() << endl;
        if(score < best_score) {
            ecc::logInfo() << "From (" << thread_id << ") BEST FAILED POWER "
                 << best_score << endl;
            best_score = score;
            best_cells_poweropt.resize(numcells);
            for(unsigned i = 0; i < numcells; i++) {
                best_cells_poweropt[i] = cells[i];
            }
            double wns, power;
            // ReportWithPT(best_cells_poweropt, "failed_power_opt", wns,
            // power);
        }

        double ini_score = init_score[0];

        ecc::logInfo() << "best_score/init_score/best_tns/cur_best_tns : " << best_score
             << "/" << ini_score << "/" << best_tns << "/" << cur_best_tns
             << endl;

        ecc::logInfo() << "done" << endl;
        // pthread_mutex_unlock(&mutex1);
    }
    showAllSlew(0, "before_tran.csv");
    ecc::logInfo() << "start delete" << endl;
    delete[] cells;
    ecc::logInfo() << "delete cells done" << endl;
    // best_cells_local.clear();
    // ecc::logInfo() << "delete best_cells_local done" << endl;
    // best_failed_cells_local.clear();
    // ecc::logInfo() << "delete best_failed_cells_local done" << endl;
    // if ( updated_int_best_cells )
    // int_best_cells.clear();
    // ecc::logInfo() << "delete int_best_cells done" << endl;
    for(unsigned i = 0; i < numViews; ++i) {
        delete[] pins[i];
    }
    delete[] pins;
    ecc::logInfo() << "delete pins done" << endl;
    for(unsigned i = 0; i < numCorners; ++i) {
        delete[] nets[i];
    }
    delete[] nets;
    ecc::logInfo() << "delete nets done" << endl;

#ifdef TIME_MON
    time_Fineswap += cpuTime() - begin;
#endif
    TMR = (cpuTime() - TMR);
    ecc::logInfo() << "LEAKOPT_time : " << TMR << endl;
    ecc::logInfo() << "(" << thread_id << ") Score after PowerOpt: " << best_score_local
         << " option: " << thread_id << " cpu time: " << TMR << endl;
}

CellSol Sizer::GetCommonCell(unsigned cell_index) {
    CellSol solution;

    if(getLibCellInfo(g_cells[cell_index]) == NULL) {
        return solution;
    }

    vector< vector< double > > score_list;
    vector< vector< double > > prob_list;

    LibCellTable *lib_cell_table = NULL;
    if(g_cells[cell_index].main_lib_cell_id != -1 &&
       g_cells[cell_index].main_lib_cell_id < main_lib_cell_tables[0].size()) {
        lib_cell_table =
            main_lib_cell_tables[0][g_cells[cell_index].main_lib_cell_id];
    }

    if(lib_cell_table == NULL) {
        return solution;
    }
    if(lib_cell_table->lib_vt_size_table.size() == 0) {
        return solution;
    }

    //    ecc::logInfo() << lib_cell_table->name << " #VT = " <<
    //    lib_cell_table->lib_vt_size_table[0].size() << " #SIZE = " <<
    //    lib_cell_table->lib_vt_size_table.size() << endl;

    // [size] [vt]
    for(unsigned i = 0; i < lib_cell_table->lib_vt_size_table.size(); ++i) {
        vector< double > tmp;

        for(unsigned j = 0; j < lib_cell_table->lib_vt_size_table[0].size();
            ++j) {
            tmp.push_back(0.0);
        }

        score_list.push_back(tmp);

        vector< double > tmp2;

        for(unsigned j = 0; j < lib_cell_table->lib_vt_size_table[0].size();
            ++j) {
            tmp2.push_back(0.0);
        }

        prob_list.push_back(tmp2);
    }

    // ecc::logInfo() << "GET COMMON CELL " << cell_index << " " <<
    // g_cells[cell_index].name << " " << g_cells[cell_index].type << "  #VT = "
    // << prob_list[0].size() << " #SIZES = " << prob_list.size() << endl;

    int size_all = all_cells.size();

    for(unsigned i = 0; i < all_cells.size(); ++i) {
        unsigned size_index = (unsigned)all_cells[i][cell_index].c_size;
        unsigned vt_index = (unsigned)all_cells[i][cell_index].c_vtype;
        if(size_index > lib_cell_table->lib_vt_size_table.size()) {
            continue;
        }
        if(lib_cell_table->lib_vt_size_table.size() > 0) {
            if(vt_index > lib_cell_table->lib_vt_size_table[0].size()) {
                continue;
            }
        }
        // ecc::logInfo() << i << "th solution -- (vt/size) " << vt_index << "/" <<
        // size_index << endl;

        prob_list[size_index][vt_index] += 1.0 / (double)size_all;
        score_list[size_index][vt_index] +=
            (1.0 / (double)size_all) * powerlist[i];
    }

    double max_prob = 0.0;
    double max_score = 0.0;
    int max_size = 0;
    int max_vt = 0;

    for(unsigned i = 0; i < prob_list.size(); ++i) {
        for(unsigned j = 0; j < prob_list[i].size(); ++j) {
            if(max_prob < prob_list[i][j]) {
                max_prob = prob_list[i][j];
            }
            if(max_score < score_list[i][j]) {
                max_score = score_list[i][j];
                max_size = i;
                max_vt = j;
            }
        }
    }

    if(max_prob > COMMON_THRESH) {
        solution.c_vtype = max_vt;
        solution.c_size = max_size;
    }

    // ecc::logInfo() << "GET COMMON CELL " << cell_index << " " <<
    // g_cells[cell_index].name << " " << g_cells[cell_index].type << " : " <<
    // prob_list[max_size][max_vt] << "/" << score_list[max_size][max_vt] <<
    // "(size/vt)" << max_size <<"/" << max_vt<<  endl;

    return solution;
}

void Sizer::FinalPowerOpt(double slk_th, unsigned thread_id) {
    if(CORR_PT)
        CorrelatePT((unsigned)thread_id);

    CalcStats((unsigned)thread_id);
    // T = PTimer[thread_id];

    ecc::logInfo() << endl << "Final power optimization .." << thread_id << endl;

    set< entry > targets;
    for(unsigned i = 0; i < numcells; i++) {
        double prev_slack = GetCellSlack(cells[i]);

        if(prev_slack < slk_th)
            continue;
        if(cells[i].isClockCell)
            continue;
        if(isMin(cells[i]) && r_type(cells[i]) == 0)
            continue;
        if(cells[i].isDontTouch)
            continue;

        entry tmpEntry;
        tmpEntry.id = i;
        if(NO_TOPO) {
            tmpEntry.tie_break = i;
        }
        else
            tmpEntry.tie_break = map2topoidx[i];

        // dngrading
        double sf = 0.0;
        if(!mmmcOn) {
            sf = CalSens(cells[i], 0, -1, 0, 1.0, 0.5);
        }
        else {
            sf = CalSensMMMC(cells[i], 0, -1, 0, 1.0, 0.5);
        }
        tmpEntry.delta_impact = sf;

        tmpEntry.change = DNTYPE;

        tmpEntry.step = -1;

        if(tmpEntry.step != 0)
            targets.insert(tmpEntry);
    }

    ecc::logInfo() << "# priority targets  : " << targets.size() << endl;
    unsigned count = 0;

    for(set< entry >::iterator it = targets.begin(); it != targets.end();
        it++) {
        unsigned cur = it->id;
        double prv_slk = GetCellSlack(cells[cur]);

        if(prv_slk < slk_th) {
            continue;
        }

        ecc::logInfo() << "FINAL PWR OPT TARGET = UPTYPE (" << it->step
             << "): " << cells[cur].name << " " << cells[cur].type;

        bool change = cell_retype(cells[cur], it->step, true);

        ecc::logInfo() << "( TRY " << cells[cur].type << ")";

        bool restore = false;

        for(unsigned view = 0; view < numViews; ++view) {
            if(T[view]->getWorstSlack(clk_name[worst_corner]) < 0.0) {
                restore = true;
            }
        }

        if(restore && change) {  // restore

            cell_retype(cells[cur], -it->step, true);
            ecc::logInfo() << "-> " << cells[cur].type << " -- Restored" << endl;
        }
        else {  // accept

            ecc::logInfo() << "-> " << cells[cur].type << " -- Accepted" << endl;
            OneTimer(cells[cur], STA_MARGIN);
            count++;
        }
    }
    targets.clear();
}

unsigned Sizer::IncrTNS(double kick_ratio, double kick_slack) {
    ecc::logInfo() << endl << "Increase total slack ... " << endl;
    for(unsigned view = 0; view < numViews; ++view) {
        CountPathsLesserThanSlack(view, kick_slack);
    }

    double prv_slk = 0.0, cur_slk = 0.0;
    set< entry > targets;
    for(unsigned i = 0; i < numcells; i++) {
        if(isff(cells[i]))
            continue;
        if(cells[i].isClockCell)
            continue;
        if(isMax(cells[i]) && r_type(cells[i]) == (numVt - 1))
            continue;
        if(cells[i].isDontTouch)
            continue;
        if(cells[i].downsized)
            continue;

        entry tmpEntry;
        tmpEntry.id = i;
        if(NO_TOPO) {
            tmpEntry.tie_break = i;
        }
        else
            tmpEntry.tie_break = map2topoidx[i];

        bool skip_type = false;
        bool skip_size = false;

        if(tabuNum > 0) {
            unsigned pre_sol = cells[i].c_size * 10 + cells[i].c_vtype;

            unsigned cur_sol1 = pre_sol + 1;
            unsigned cur_sol2 = pre_sol + 10;

            list< unsigned >::iterator iter;
            for(iter = cells[i].tabu.begin(); iter != cells[i].tabu.end();
                iter++) {
                if(*iter == cur_sol1) {
                    skip_type = true;
                }
                if(*iter == cur_sol2) {
                    skip_size = true;
                }
            }
        }
        vector< double > delta_impact_size, delta_impact_type;
        vector< int > size_step, type_step;

        for(int k = 1; k <= MULTI_STEP_KICK; ++k) {
            // upsizing
            if(isSizable(cells[i], k) && !skip_size) {
                double sf = 0.0;
                if(!mmmcOn) {
                    sf = CalSens(cells[i], k, 0, KICK_SFT, 1.0, 1);
                }
                else {
                    sf = CalSensMMMC(cells[i], k, 0, KICK_SFT, 1.0, 1);
                }
                if(sf != 0.0) {
                    delta_impact_size.push_back(1.0 / sf);
                    size_step.push_back(k);
                }
            }

            // upgrading
            if(isSwappable(cells[i], k) && !skip_type) {
                double sf = 0.0;
                if(!mmmcOn) {
                    sf = CalSens(cells[i], 0, k, KICK_SFT, 1.0, 1);
                }
                else {
                    sf = CalSensMMMC(cells[i], 0, k, KICK_SFT, 1.0, 1);
                }
                if(sf != 0.0) {
                    delta_impact_type.push_back(1.0 / sf);
                    type_step.push_back(k);
                }
            }
        }

        double max_delta_impact = DBL_MAX;
        int step = 0;
        bool isUpSize = true;

        for(unsigned k = 0; k < delta_impact_size.size(); ++k) {
            if(max_delta_impact > delta_impact_size[k]) {
                step = size_step[k];
                max_delta_impact = delta_impact_size[k];
            }
        }

        for(unsigned k = 0; k < delta_impact_type.size(); ++k) {
            if(max_delta_impact > delta_impact_type[k]) {
                isUpSize = false;
                step = type_step[k];
                max_delta_impact = delta_impact_type[k];
            }
        }

        if(isUpSize) {
            tmpEntry.change = UPSIZE;
        }
        else {
            tmpEntry.change = UPTYPE;
        }

        tmpEntry.delta_impact = max_delta_impact;
        tmpEntry.step = step;

        if(tmpEntry.step != 0)
            targets.insert(tmpEntry);
    }

    ecc::logInfo() << "# priority targets  : " << targets.size() << endl;
    unsigned count = 0;

    for(set< entry >::iterator it = targets.begin(); it != targets.end();
        it++) {
        unsigned cur = it->id;
        prv_slk = GetCellSlack(cells[cur]);

        if(prv_slk > kick_slack) {
            continue;
        }

        double delta_delay = 0.0;
        bool change = false;

        if(it->change == UPSIZE) {
            if(VERBOSE >= 1)
                ecc::logInfo() << "KICK TARGET = UPSIZE (" << it->step
                     << "): " << cells[cur].name << " " << cells[cur].type;

            if(HOLD_CHECK) {
                delta_delay = EstDeltaDelay(cells[cur], it->step, 0);
            }
            change = cell_resize(cells[cur], it->step);
        }
        else {
            if(VERBOSE >= 1)
                ecc::logInfo() << "KICK TARGET = UPTYPE (" << it->step
                     << "): " << cells[cur].name << " " << cells[cur].type;
            if(HOLD_CHECK) {
                delta_delay = EstDeltaDelay(cells[cur], 0, it->step);
            }
            change = cell_retype(cells[cur], it->step);
        }

        OneTimer(cells[cur], STA_MARGIN);

        cur_slk = GetCellSlack(cells[cur]);

        bool restore = false;

        // double fi_nslack = FiNSlackSUM(cells[cur]);

        if(cur_slk - prv_slk < 0.0) {
            restore = true;
        }

        if(!restore && HOLD_CHECK) {
            if(EstHoldVio(cells[cur], delta_delay)) {
                restore = true;
            }
        }

        if(restore && change) {  // restore
            if(it->change == UPSIZE) {
                cell_resize(cells[cur], -it->step);
            }
            else {
                cell_retype(cells[cur], -it->step);
            }
            OneTimer(cells[cur], STA_MARGIN);
            if(VERBOSE >= 1)
                ecc::logInfo() << "-> " << cells[cur].type << " -- Restored" << endl;
            count++;
        }
        else {  // accept
            if(VERBOSE >= 1)
                ecc::logInfo() << "-> " << cells[cur].type << " -- Accepted" << endl;
            count++;
        }
        if(count >= numcells * kick_ratio)
            break;
    }
    return count;
}

unsigned Sizer::IncrSlack(double leak_exponent, double alpha) {
    ecc::logInfo() << endl << "Increase slack .." << leak_exponent << endl;
    if(alpha == -1) {
        alpha = ALPHA;
    }

    double prv_slk = 0.0, cur_slk = 0.0;
    set< entry > targets;
    for(unsigned i = 0; i < numcells; i++) {
        if(isff(cells[i]))
            continue;
        if(cells[i].isClockCell)
            continue;
        if(isMax(cells[i]) && r_type(cells[i]) == (numVt - 1))
            continue;
        if(cells[i].isDontTouch)
            continue;
        if(cells[i].downsized)
            continue;
        // if(cells[i].depth > THRESHOLD)
        //    continue;

        entry tmpEntry;
        tmpEntry.id = i;
        if(NO_TOPO) {
            tmpEntry.tie_break = i;
        }
        else
            tmpEntry.tie_break = map2topoidx[i];

        bool skip_type = false;
        bool skip_size = false;

        if(tabuNum > 0) {
            unsigned pre_sol = cells[i].c_size * 10 + cells[i].c_vtype;

            unsigned cur_sol1 = pre_sol + 1;
            unsigned cur_sol2 = pre_sol + 10;

            list< unsigned >::iterator iter;
            for(iter = cells[i].tabu.begin(); iter != cells[i].tabu.end();
                iter++) {
                if(*iter == cur_sol1) {
                    skip_type = true;
                }
                if(*iter == cur_sol2) {
                    skip_size = true;
                }
            }
        }
        vector< double > delta_impact_size, delta_impact_type;
        vector< int > size_step, type_step;

        for(int k = 1; k <= MULTI_STEP; ++k) {
            // upsizing
            if(isSizable(cells[i], k) && !skip_size) {
                double sf = 0.0;
                if(!mmmcOn) {
                    sf =
                        CalSens(cells[i], k, 0, KICK_SFT, leak_exponent, alpha);
                }
                else {
                    sf = CalSensMMMC(cells[i], k, 0, KICK_SFT, leak_exponent,
                                     alpha);
                }
                if(sf != 0.0) {
                    delta_impact_size.push_back(1.0 / sf);
                    size_step.push_back(k);
                }
            }

            // upgrading
            if(isSwappable(cells[i], k) && !skip_type) {
                double sf = 0.0;
                if(!mmmcOn) {
                    sf =
                        CalSens(cells[i], 0, k, KICK_SFT, leak_exponent, alpha);
                }
                else {
                    sf = CalSensMMMC(cells[i], 0, k, KICK_SFT, leak_exponent,
                                     alpha);
                }
                if(sf != 0.0) {
                    delta_impact_type.push_back(1.0 / sf);
                    type_step.push_back(k);
                }
            }
        }

        double max_delta_impact = DBL_MAX;
        int step = 0;
        bool isUpSize = true;

        for(unsigned k = 0; k < delta_impact_size.size(); ++k) {
            if(max_delta_impact > delta_impact_size[k]) {
                step = size_step[k];
                max_delta_impact = delta_impact_size[k];
            }
        }

        for(unsigned k = 0; k < delta_impact_type.size(); ++k) {
            if(max_delta_impact > delta_impact_type[k]) {
                isUpSize = false;
                step = type_step[k];
                max_delta_impact = delta_impact_type[k];
            }
        }

        if(isUpSize) {
            tmpEntry.change = UPSIZE;
        }
        else {
            tmpEntry.change = UPTYPE;
        }

        tmpEntry.delta_impact = max_delta_impact;
        tmpEntry.step = step;

        if(tmpEntry.step != 0)
            targets.insert(tmpEntry);
    }

    ecc::logInfo() << "# priority targets  : " << targets.size() << endl;
    unsigned count = 0;

    for(set< entry >::iterator it = targets.begin(); it != targets.end();
        it++) {
        unsigned cur = it->id;
        prv_slk = GetCellSlack(cells[cur]);

        double delta_delay = 0.0;
        bool change = false;

        if(it->change == UPSIZE) {
            if(VERBOSE >= 1)
                ecc::logInfo() << "KICK TARGET = UPSIZE (" << it->step
                     << "): " << cells[cur].name << " " << cells[cur].type;

            if(HOLD_CHECK) {
                delta_delay = EstDeltaDelay(cells[cur], it->step, 0);
            }
            change = cell_resize(cells[cur], it->step);
        }
        else {
            if(VERBOSE >= 1)
                ecc::logInfo() << "KICK TARGET = UPTYPE (" << it->step
                     << "): " << cells[cur].name << " " << cells[cur].type;
            if(HOLD_CHECK) {
                delta_delay = EstDeltaDelay(cells[cur], 0, it->step);
            }
            change = cell_retype(cells[cur], it->step);
        }

        OneTimer(cells[cur], STA_MARGIN);

        cur_slk = GetCellSlack(cells[cur]);

        bool restore = false;

        // double fi_nslack = FiNSlackSUM(cells[cur]);

        if(cur_slk - prv_slk < KICK_SLACK) {
            restore = true;
        }

        if(!restore && HOLD_CHECK) {
            if(EstHoldVio(cells[cur], delta_delay)) {
                restore = true;
            }
        }

        if(restore && change) {  // restore
            if(it->change == UPSIZE) {
                cell_resize(cells[cur], -it->step);
            }
            else {
                cell_retype(cells[cur], -it->step);
            }
            OneTimer(cells[cur], STA_MARGIN);
            if(VERBOSE >= 1)
                ecc::logInfo() << "-> " << cells[cur].type << " -- Restored" << endl;
            count++;
        }
        else {  // accept
            if(VERBOSE >= 1)
                ecc::logInfo() << "-> " << cells[cur].type << " -- Accepted" << endl;
            count++;
        }
        if(count >= (double)targets.size() * KICK_RATIO)
            break;
    }
    ecc::logInfo() << "#Swaps to increase slack: " << count << endl;
    targets.clear();
    return count;
}

unsigned Sizer::IncrSlackMore(double kick_ratio, double alpha) {
    if(alpha == -1) {
        alpha = ALPHA;
    }
    ecc::logInfo() << endl << "Increase slack more .." << endl;

    double prv_slk = 0.0, cur_slk = 0.0;
    set< entry > targets;
    for(unsigned i = 0; i < numcells; i++) {
        // if(isff(cells[i]))
        //    continue;
        if(cells[i].isClockCell)
            continue;
        if(isMax(cells[i]) && r_type(cells[i]) == (numVt - 1))
            continue;
        if(cells[i].isDontTouch)
            continue;
        // if(cells[i].depth > THRESHOLD)
        //    continue;

        entry tmpEntry;
        tmpEntry.id = i;
        if(NO_TOPO) {
            tmpEntry.tie_break = i;
        }
        else
            tmpEntry.tie_break = map2topoidx[i];

        vector< double > delta_impact_size, delta_impact_type;
        vector< int > size_step, type_step;

        for(int k = 1; k <= MULTI_STEP; ++k) {
            // upsizing
            if(isSizable(cells[i], k)) {
                double sf = 0.0;
                if(!mmmcOn) {
                    sf = CalSens(cells[i], k, 0, 5, 1.0, alpha);
                }  // SF 6 for TNS, originally 5
                else {
                    sf = CalSensMMMC(cells[i], k, 0, 5, 1.0, alpha);
                }  // SF 6 for TNS, originally 5
                delta_impact_size.push_back(1.0 / sf);
                size_step.push_back(k);
            }

            // upgrading
            if(isSwappable(cells[i], k)) {
                double sf = 0.0;
                if(!mmmcOn) {
                    sf = CalSens(cells[i], 0, k, 5, 1.0, alpha);
                }
                else {
                    sf = CalSensMMMC(cells[i], 0, k, 5, 1.0, alpha);
                }
                delta_impact_type.push_back(1.0 / sf);
                type_step.push_back(k);
            }
        }

        double max_delta_impact = 0.0;
        int step = 0;
        bool isUpSize = true;

        for(unsigned k = 0; k < delta_impact_size.size(); ++k) {
            if(max_delta_impact > delta_impact_size[k]) {
                step = size_step[k];
                max_delta_impact = delta_impact_size[k];
            }
        }

        for(unsigned k = 0; k < delta_impact_type.size(); ++k) {
            if(max_delta_impact > delta_impact_type[k]) {
                isUpSize = false;
                step = type_step[k];
                max_delta_impact = delta_impact_type[k];
            }
        }

        if(isUpSize) {
            tmpEntry.change = UPSIZE;
        }
        else {
            tmpEntry.change = UPTYPE;
        }

        tmpEntry.delta_impact = max_delta_impact;
        tmpEntry.step = step;

        if(tmpEntry.delta_impact < -0.0001)
            targets.insert(tmpEntry);
    }

    ecc::logInfo() << "# priority targets  : " << targets.size() << endl;
    unsigned count = 0;

    for(set< entry >::iterator it = targets.begin(); it != targets.end();
        it++) {
        unsigned cur = it->id;
        prv_slk = GetCellSlack(cells[cur]);

        double delta_delay = 0.0;
        bool change = false;

        if(it->change == UPSIZE) {
            // ecc::logInfo() << "KICK TARGET = UPSIZE (" << it->step << "): " <<
            // cells[cur].name << " " << cells[cur].type;

            if(HOLD_CHECK) {
                delta_delay = EstDeltaDelay(cells[cur], it->step, 0);
            }

            change = cell_resize(cells[cur], it->step);
        }
        else {
            // ecc::logInfo() << "KICK TARGET = UPTYPE (" << it->step << "): " <<
            // cells[cur].name << " " << cells[cur].type;
            if(HOLD_CHECK) {
                delta_delay = EstDeltaDelay(cells[cur], 0, it->step);
            }
            change = cell_retype(cells[cur], it->step);
        }

        OneTimer(cells[cur], STA_MARGIN);
        cur_slk = GetCellSlack(cells[cur]);

        bool restore = false;

        double fi_nslack = FiNSlackSUM(cells[cur]);

        if(cur_slk - prv_slk < KICK_SLACK || fi_nslack < 0.0) {
            restore = true;
        }

        if(!restore && HOLD_CHECK) {
            if(EstHoldVio(cells[cur], delta_delay)) {
                restore = true;
            }
        }

        if(restore && change) {  // restore
            if(it->change == UPSIZE) {
                cell_resize(cells[cur], -it->step);
            }
            else {
                cell_retype(cells[cur], -it->step);
            }
            OneTimer(cells[cur], STA_MARGIN);
            if(VERBOSE >= 1)
                ecc::logInfo() << "-> " << cells[cur].type << " -- Restored" << endl;
        }
        else {  // accept
            if(VERBOSE >= 1)
                ecc::logInfo() << "-> " << cells[cur].type << " -- Accepted" << endl;
            count++;
        }
        if(count > (double)targets.size() * kick_ratio)
            break;
    }
    ecc::logInfo() << "#Swaps to increase slack: " << count << endl;
    targets.clear();
    return count;
}

unsigned Sizer::IncrSlackRandom(double kick_ratio, double kick_slack) {
    ecc::logInfo() << endl << "Increase slack random .." << endl;

    int cnt = 0;

    unsigned i = 0;

    for(unsigned j = 0; j < topolist.size(); j++) {
        i = topolist[j];
        cells[i].touched = false;
        if(cells[i].isClockCell)
            continue;
        if(isMax(cells[i]) && r_type(cells[i]) == (numVt - 1))
            continue;
        if(cells[i].isDontTouch)
            continue;

        double prv_slk = min(GetCellSlack(cells[i]), GetFICellSlack(cells[i]));
        bool change = false;
        if(prv_slk < kick_slack) {
            double delta_delay = 0.0;

            // upgrading
            if(isSwappable(cells[i], KICK_STEP)) {
                ecc::logInfo() << "KICK TARGET = UPTYPE (" << KICK_STEP
                     << "): " << cells[i].name << " " << cells[i].type;

                bool attack = true;

                if(HOLD_CHECK) {
                    for(unsigned view = 0; view < numViews; ++view) {
                        delta_delay =
                            EstDeltaDelay(cells[i], 0, KICK_STEP, view);
                        if(EstHoldVio(cells[i], delta_delay)) {
                            attack = false;
                        }
                    }
                }

                if(attack) {
                    change = cell_retype(cells[i], KICK_STEP);
                    ecc::logInfo() << "-> " << cells[i].type << " -- Accepted" << endl;

                    cells[i].touched = true;
                    if(change) {
                        for(unsigned view = 0; view < numViews; ++view) {
                            OneTimer(cells[i], STA_MARGIN, view);
                        }
                        cnt++;
                    }
                }
                else {
                    ecc::logInfo() << "-> " << cells[i].type << " -- Rejected" << endl;
                }
            }
            else if(isSizable(cells[i], KICK_STEP)) {
                // upsizing

                bool attack = true;
                for(unsigned view = 0; view < numViews; ++view) {
                    unsigned corner = 0;  // mmmcViewList[view].corner;
                    double max_slew = 0.0;
                    for(unsigned k = 0; k < cells[i].inpins.size(); ++k) {
                        if(IsTranVio(pins[view][cells[i].inpins[k]])) {
                            attack = false;
                            break;
                        }
                    }
                }

                if(attack) {
                    ecc::logInfo() << "KICK TARGET = UPSIZE (" << KICK_STEP
                         << "): " << cells[i].name << " " << cells[i].type;

                    change = cell_resize(cells[i], KICK_STEP);

                    if(change) {
                        for(unsigned view = 0; view < numViews; ++view) {
                            OneTimer(cells[i], STA_MARGIN, view);
                        }
                    }

                    double cur_slk =
                        min(GetCellSlack(cells[i]), GetFICellSlack(cells[i]));

                    if(cur_slk < prv_slk) {
                        attack = false;
                    }

                    if(HOLD_CHECK) {
                        for(unsigned view = 0; view < numViews; ++view) {
                            delta_delay =
                                EstDeltaDelay(cells[i], 0, KICK_STEP, view);
                            if(EstHoldVio(cells[i], delta_delay)) {
                                attack = false;
                            }
                        }
                    }

                    if(attack) {
                        ecc::logInfo() << "-> " << cells[i].type << " -- Accepted"
                             << endl;
                        cnt++;
                    }
                    else {
                        ecc::logInfo() << "-> " << cells[i].type << " -- Rejected"
                             << endl;
                        if(change) {
                            cell_resize(cells[i], -KICK_STEP);
                            for(unsigned view = 0; view < numViews; ++view) {
                                OneTimer(cells[i], STA_MARGIN, view);
                            }
                        }
                    }
                }
            }
        }
        if(cnt >= numcells * kick_ratio) {
            break;
        }
    }

    ecc::logInfo() << "KICK OPT # SWAP = " << cnt << endl;
    return cnt;
}

// FIXME:
unsigned Sizer::ReducePowerLegal(int thread_id, int option, int iter,
                                 double alpha, double toler, bool isPeephole,
                                 bool &updated_local,
                                 vector< CELL > &best_cells_local) {
    unsigned view = 0;
    if(alpha == -1) {
        alpha = ALPHA;
    }

    double init_time = cpuTime();

    ecc::logInfo() << "(" << thread_id
         << ") Start Power Reduction .. with SF = " << option
         << " alpha = " << alpha;

    if(isPeephole) {
        ecc::logInfo() << " PEEPHOLE" << endl;
    }
    else {
        ecc::logInfo() << " NO PEEPHOLE" << endl;
    }

    set< entry > targets;

    for(unsigned i = 0; i < numcells; i++) {
        if(VERBOSE >= 100) {
            ecc::logInfo() << "START CAL SENS : " << option << " " << alpha << " "
                 << cells[i].name << " " << cells[i].type << " "
                 << r_type(cells[i]) << " " << default_vtype << endl;
        }
        if(cells[i].isClockCell)
            continue;
        if(cells[i].isDontTouch)
            continue;
        if(r_type(cells[i]) == 0 && isMin(cells[i]))
            continue;

        if(r_type(cells[i]) == default_vtype)
            continue;

        bool skip_type = false;
        bool skip_size = false;

        if(tabuNum > 0) {
            unsigned pre_sol = cells[i].c_size * 10 + cells[i].c_vtype;

            unsigned cur_sol1 = pre_sol - 1;
            unsigned cur_sol2 = pre_sol - 10;

            list< unsigned >::iterator iter;
            for(iter = cells[i].tabu.begin(); iter != cells[i].tabu.end();
                iter++) {
                if(*iter == cur_sol1) {
                    skip_type = true;
                }
                if(*iter == cur_sol2) {
                    skip_size = true;
                }
            }
        }

        if(!isPeephole) {
            entry tmpEntry;
            tmpEntry.id = i;
            if(NO_TOPO) {
                tmpEntry.tie_break = i;
            }
            else {
                tmpEntry.tie_break = map2topoidx[i];
            }

            double delta_impact_type = 0.0;
            double delta_impact_size = 0.0;

            // upgrading
            if(r_type(cells[i]) != 0 && !skip_type) {
                double sf = 0.0;
                if(!mmmcOn) {
                    sf = CalSens(cells[i], 0, -1, option, 1.0, alpha);
                }
                else {
                    sf = CalSensMMMC(cells[i], 0, -1, option, 1.0, alpha);
                }
                delta_impact_type = sf;
            }

            // downsizing
            if(!isMin(cells[i]) && !skip_size) {
                double sf = 0.0;
                if(!mmmcOn) {
                    sf = CalSens(cells[i], -1, 0, option, 1.0, alpha);
                }
                else {
                    sf = CalSensMMMC(cells[i], -1, 0, option, 1.0, alpha);
                }
                delta_impact_size = sf;
            }

            if(VERBOSE >= 1) {
                ecc::logInfo() << "CAL SENS : " << option << " " << alpha << " "
                     << cells[i].name << " " << cells[i].type << " "
                     << delta_impact_type << "/" << delta_impact_size << endl;
            }

            if(!ALL_MOVE) {
                if(delta_impact_size > delta_impact_type) {
                    tmpEntry.change = DNTYPE;
                    tmpEntry.delta_impact = delta_impact_type;
                    if(tmpEntry.delta_impact != 0.0)
                        targets.insert(tmpEntry);
                }
                else {
                    tmpEntry.change = DNSIZE;
                    tmpEntry.delta_impact = delta_impact_size;
                    if(tmpEntry.delta_impact != 0.0)
                        targets.insert(tmpEntry);
                }
            }
            else {
                tmpEntry.change = DNTYPE;
                tmpEntry.delta_impact = delta_impact_type;
                if(tmpEntry.delta_impact != 0.0)
                    targets.insert(tmpEntry);

                tmpEntry.change = DNSIZE;
                tmpEntry.delta_impact = delta_impact_size;
                if(tmpEntry.delta_impact != 0.0)
                    targets.insert(tmpEntry);
            }
        }
        else {
            int num_cell = PEEPHOLE_NUMCELL;

            vector< unsigned > cell_list = GetCellList(i, num_cell);
            entry tmpEntrySet;

            int seq_num = (int)pow(2.0, (double)num_cell);

            vector< unsigned > change_list;

            for(unsigned k = 0; k < num_cell; k++) {
                change_list.push_back(0);
            }

            for(unsigned m = 0; m < seq_num; m++) {
                for(unsigned k = 0; k < num_cell; k++) {
                    change_list[k] = 0;
                }

                int seq = m;
                unsigned l = 0;
                while(seq != 0) {
                    change_list[l] = seq % 2;
                    seq /= 2;
                    l++;
                }

                // DNSIZE = 2; DNTYPE = 3;
                for(unsigned k = 0; k < change_list.size(); k++) {
                    change_list[k] += 2;
                }

                if(VERBOSE >= 300) {
                    for(unsigned k = 0; k < change_list.size(); k++) {
                        ecc::logInfo() << change_list[k];
                    }
                    ecc::logInfo() << endl;
                }

                double score =
                    CalSensSet(cell_list, change_list, option, 1.0, alpha);
                if(tmpEntrySet.delta_impact > score) {
                    tmpEntrySet.delta_impact = score;
                    tmpEntrySet.ids = cell_list;
                    tmpEntrySet.changes = change_list;
                }
            }

            if(tmpEntrySet.delta_impact != 0.0)
                targets.insert(tmpEntrySet);
        }
    }
    ecc::logInfo() << "# priority targets  : " << targets.size() << endl;
    unsigned target_size = targets.size();
    unsigned accept = 0;
    unsigned update_cnt = 0;
    unsigned accum_update_cnt = 0;
    unsigned restore = 0;
    unsigned cnt = 0;

    vector< bool > changed;
    changed.resize(numcells);
    for(unsigned i = 0; i < numcells; i++)
        changed[i] = false;

    // double top_impact=targets.begin()->delta_impact;
    // double total_impact=0.;

    if(!isPeephole) {
        while(!targets.empty()) {
            set< entry >::iterator it = targets.begin();
            unsigned cur = it->id;

            if(changed[cur]) {
                targets.erase(it);
                continue;
            }

            LibCellInfo *lib_cell_info = getLibCellInfo(cells[cur]);

            if(lib_cell_info == NULL) {
                targets.erase(it);
                continue;
            }

            bool flag_find_vt = false;

            for(unsigned i = 0; i < MAX_TARGETS; ++i) {
                if(it->change == DNTYPE) {
                    flag_find_vt = true;
                    break;
                }
                ++it;
            }

            if(!flag_find_vt) {
                it = targets.begin();
            }

            cur = it->id;

            cnt++;
            unsigned pre_sol = cells[cur].c_size * 10 + cells[cur].c_vtype;

            if(VERBOSE >= 1) {
                if(it->change == DNSIZE) {
                    ecc::logInfo() << "(" << targets.size() << ") REDUCE = DNSIZE : ";
                }
                else {
                    ecc::logInfo() << "(" << targets.size() << ") REDUCE = DNTYPE : ";
                }

                ecc::logInfo() << cells[cur].name << " " << cells[cur].type << " "
                     << it->delta_impact;

                for(unsigned view1 = 0; view1 < numViews; ++view1) {
                    ecc::logInfo() << " (" << view1 << "/"
                         << GetCellSlack(cells[cur], view1) << "/"
                         << GetCellDelay(cells[cur], view1) << "/"
                         << GetCellTranSlack(cells[cur], view1) << "/"
                         << GetCellLeak(cells[cur], view1) << ")";
                }
                ecc::logInfo() << " --->";
            }

            bool change = false;

            if(it->change == DNSIZE)
                change = cell_resize(cells[cur], -1);
            else
                change = cell_retype(cells[cur], -1);

            if(USE_FULL_STA) {
                for(unsigned view1 = 0; view1 < numViews; ++view1) {
                    CallTimer(view1);
                }
            }
            else {
                for(unsigned view1 = 0; view1 < numViews; ++view1) {
                    OneTimer(cells[cur], STA_MARGIN, false, view1);
                }
            }

            if(VERBOSE > 0 || VERBOSE >= 5)
                ecc::logInfo() << cells[cur].type << " ";

            // check slack, max cap constraints
            bool restore_flag = false;

            if(VERBOSE > 0 || VERBOSE >= 5) {
                for(unsigned view1 = 0; view1 < numViews; ++view1) {
                    ecc::logInfo() << " (" << view1 << "/"
                         << GetCellSlack(cells[cur], view1) << "/"
                         << GetCellDelay(cells[cur], view1) << "/"
                         << GetCellTranSlack(cells[cur], view1) << "/"
                         << GetCellLeak(cells[cur], view1) << ")";
                }
            }

            for(unsigned view1 = 0; view1 < numViews; ++view1) {
                unsigned corner1 = mmmcViewList[view1].corner;
                double maxCap, curCap;
                // CalcStats((unsigned)thread_id, true, "CORR_PWR_OPT", view1,
                // false);

                if(GetCellSlack(cells[cur], view1) < toler) {
                    restore_flag = true;
                    if(VERBOSE >= 1)
                        ecc::logInfo() << "RESTORED DUE TO SLACK " << view << " "
                             << " " << GetCellSlack(cells[cur], view1) << " "
                             << viewWNS[view1] << " " << toler << " ";
                    break;
                }
                for(unsigned k = 0; k < cells[cur].outpins.size(); ++k) {
                    // double slack =
                    // min(pins[view1][cells[cur].outpins[k]].rslk,
                    // pins[view1][cells[cur].outpins[k]].fslk);
                    maxCap =
                        lib_cell_info
                            ->pins[pins[view1][cells[cur].outpins[k]].lib_pin]
                            .maxCapacitance;
                    if(use_margin) {
                        maxCap *= cap_margin;
                    }
                    curCap = pins[view1][cells[cur].outpins[k]].totcap;
                    if(curCap > maxCap) {
                        if(VERBOSE >= 1)
                            ecc::logInfo() << "RESTORED DUE TO CAP " << curCap << " "
                                 << maxCap << " ";
                        restore_flag = true;
                        break;
                    }
                    if(IsTranVio(pins[view1][cells[cur].outpins[k]])) {
                        if(VERBOSE >= 1)
                            ecc::logInfo() << "RESTORED DUE TO TRAN "
                                 << pins[view1][cells[cur].outpins[k]].rtran
                                 << "/"
                                 << pins[view1][cells[cur].outpins[k]].ftran
                                 << " "
                                 << pins[view1][cells[cur].outpins[k]].max_tran
                                 << " ";
                        restore_flag = true;
                        break;
                    }
                }

                if(isff(cells[cur])) {
                    for(unsigned k = 0; k < cells[cur].inpins.size(); ++k) {
                        if(libs[0]
                               .find(cells[cur].type)
                               ->second
                               .pins[pins[view1][cells[cur].inpins[k]].lib_pin]
                               .isClock)
                            continue;

                        double slack =
                            min(pins[view1][cells[cur].inpins[k]].rslk,
                                pins[view1][cells[cur].inpins[k]].rslk);

                        if(slack < 0 || slack < toler) {
                            restore_flag = true;
                            if(VERBOSE >= 1)
                                ecc::logInfo() << "RESTORED DUE TO SLACK " << slack << " "
                                     << toler << " ";
                            break;
                        }
                    }
                }
            }

            if(restore_flag && change) {  // restore
                if(VERBOSE >= 5) {
                    if(it->change == DNSIZE)
                        cell_resize(cells[cur], 1, true);
                    else
                        cell_retype(cells[cur], 1, true);
                }
                else {
                    if(it->change == DNSIZE)
                        cell_resize(cells[cur], 1);
                    else
                        cell_retype(cells[cur], 1);
                }
                if(USE_FULL_STA) {
                    for(unsigned view1 = 0; view1 < numViews; ++view1) {
                        CallTimer(view1);
                    }
                }
                else {
                    for(unsigned view1 = 0; view1 < numViews; ++view1) {
                        OneTimer(cells[cur], STA_MARGIN, false, view1);
                    }
                }
                restore++;
                if(VERBOSE > 0 || VERBOSE >= 5)
                    ecc::logInfo() << " Restore" << endl;
                entry tmpEntry(*it);
                targets.erase(it);
                // ecc::logInfo() << "x";
            }
            else {
                if(VERBOSE > 0 || VERBOSE >= 5)
                    ecc::logInfo() << " Accept" << endl;
                changed[cur] = true;
                cells[cur].downsized = true;
                if(tabuNum > 0) {
                    if(cells[cur].tabu.size() < tabuNum) {
                        cells[cur].tabu.push_back(pre_sol);
                    }
                    else {
                        cells[cur].tabu.pop_front();
                        cells[cur].tabu.push_back(pre_sol);
                    }

                    ecc::logInfo() << cells[cur].name << "'s tabu list is";
                    list< unsigned >::iterator iter;
                    for(iter = cells[cur].tabu.begin();
                        iter != cells[cur].tabu.end(); iter++) {
                        ecc::logInfo() << " " << *iter;
                    }
                    ecc::logInfo() << endl;
                }

                if(VERBOSE > 0 || VERBOSE >= 5)
                    ecc::logInfo() << " Accept"
                         << " " << cells[cur].type << endl;
                accept++;
                update_cnt++;
                accum_update_cnt++;
                entry tmpEntry(*it);
                targets.erase(it);
                // ecc::logInfo() << "o";
                if(UPDATE_LIST) {
                    // UPDATE Sensitivity
                    //
                    //
                    double delta_impact_type = 0.0;
                    double delta_impact_size = 0.0;

                    // upgrading
                    if(r_type(cells[cur]) != 0) {
                        double sf = 0.0;
                        if(!mmmcOn) {
                            sf = CalSens(cells[cur], 0, -1, option, 1.0, alpha);
                        }
                        else {
                            sf = CalSensMMMC(cells[cur], 0, -1, option, 1.0,
                                             alpha);
                        }
                        delta_impact_type = sf;
                    }

                    // downsizing
                    if(!isMin(cells[cur])) {
                        double sf = 0.0;
                        if(!mmmcOn) {
                            sf = CalSens(cells[cur], -1, 0, option, 1.0, alpha);
                        }
                        else {
                            sf = CalSensMMMC(cells[cur], -1, 0, option, 1.0,
                                             alpha);
                        }
                        delta_impact_size = sf;
                    }

                    if(delta_impact_size > delta_impact_type) {
                        tmpEntry.change = DNTYPE;
                        tmpEntry.delta_impact = delta_impact_type;
                    }
                    else {
                        tmpEntry.change = DNSIZE;
                        tmpEntry.delta_impact = delta_impact_size;
                    }

                    if(tmpEntry.delta_impact == 0.0) {
                        // ecc::logInfo() << endl;
                    }
                    else {
                        if(tmpEntry.change == DNTYPE) {
                            ecc::logInfo() << "-- Update Sensitivity DNTYPE -- "
                                 << tmpEntry.delta_impact << endl;
                        }
                        else {
                            ecc::logInfo() << "-- Update Sensitivity DNSIZE -- "
                                 << tmpEntry.delta_impact << endl;
                        }
                        targets.insert(tmpEntry);
                    }
                }
                else {
                    if(VERBOSE > 0 || VERBOSE >= 5)
                        ;
                    //    ecc::logInfo() << endl;
                }
            }

            if(cnt > numcells * PWR_CORR_RATIO && CORR_PT) {
                double worst_slew_vio = 0.0;
                for(unsigned view1 = 0; view1 < numViews; ++view1) {
                    CorrelatePT((unsigned)thread_id, view1);
                    CalcStats((unsigned)thread_id, true, "CORR_PWR_OPT", view1);
                    cnt = 0;
                    if(worst_slew_vio < slew_violation) {
                        worst_slew_vio = slew_violation;
                    }
                }

                double wns = worst_slack_worst;
                if(VERBOSE >= 1) {
                    ecc::logInfo() << "Current WNS : " << wns << endl;
                }

                if(SAFE_LEAK_OPT) {
                    if(wns < SAFE_LEAK_OPT_GB) {
                        ecc::logInfo() << "BREAK DUE TO WNS " << wns << " "
                             << skew_violation << endl;
                        break;
                    }
                    if(skew_violation != 0.0) {
                        ecc::logInfo() << "BREAK DUE TO TNS " << wns << " "
                             << skew_violation << endl;
                        break;
                    }
                    if(worst_slew_vio > SLEW_GB) {
                        ecc::logInfo() << "BREAK DUE TO SLEW " << worst_slew_vio << endl;
                        break;
                    }
                }

                if(wns > 0) {
                    if(power < best_score_local) {
                        ecc::logInfo() << "(" << thread_id
                             << ") Local best power is updated (in reduce "
                                "power function) "
                             << power << "/" << best_score_local << endl;
                        best_score_local = power;
                        best_alpha_local = local_alpha;
                        char opt_str[250];
                        sprintf(opt_str, "%d", thread_id);
                        string temp = (string)opt_str + "_feasible";
                        SizeOut(outputDir);
                        for(unsigned view = 0; view < numViews; ++view) {
                            // if(useOpenSTA) {
                            //     string find_timing =
                            //         T[view]->doOneCmd("find_timing");
                            // }
                            double wns =
                                T[view]->getWorstSlack(clk_name[worst_corner]);
                            double tns = T[view]->getTNS();

                            double leak = 0.0;
                            double tot = 0.0;

                            leak = T[view]->getLeakPower();
                            tot = T[view]->getTotPower();

                            double tran_tot, tran_max;
                            tran_tot = tran_max = 0.0;
                            int tran_num = 0;
                            T[view]->getTranVio(tran_tot, tran_max, tran_num);

                            ecc::logInfo() << "[view " << view
                                 << "] Local best power  WNS from Timer    : "
                                 << wns << " ps (init: " << init_wns[view]
                                 << ")" << endl;
                            ecc::logInfo() << "[view " << view
                                 << "] Local best power  TNS            : "
                                 << tns << " ps (init: " << init_tns[view]
                                 << ")" << endl;
                            ecc::logInfo() << "[view " << view
                                 << "] Local best power Leakage Power    : "
                                 << leak << endl;
                            ecc::logInfo()
                                << "[view " << view
                                << "] Local best power Total Power    : " << tot
                                << endl;

                            ecc::logInfo() << "[view " << view
                                 << "] Local best power Tran           : "
                                 << tran_tot << " ps " << tran_num << " "
                                 << tran_max << " ps" << endl;

                            unsigned swap_count = 0;
                            unsigned uptype_swap_count = 0;
                            unsigned downtype_swap_count = 0;
                            unsigned upsize_swap_count = 0;
                            unsigned downsize_swap_count = 0;
                            unsigned hvt_init_count = 0;
                            unsigned hvt_count = 0;
                            for(unsigned i = 0; i < numcells; ++i) {
                                if(g_cells[i].c_vtype == s) {
                                    ++hvt_init_count;
                                }
                            }

                            for(unsigned i = 0; i < numcells; ++i) {
                                if(g_cells[i].type != cells[i].type) {
                                    ++swap_count;
                                    if(cells[i].c_vtype > g_cells[i].c_vtype) {
                                        uptype_swap_count++;
                                    }
                                    else if(cells[i].c_vtype <
                                            g_cells[i].c_vtype) {
                                        downtype_swap_count++;
                                    }

                                    if(cells[i].c_size > g_cells[i].c_size) {
                                        upsize_swap_count++;
                                    }
                                    else if(cells[i].c_size <
                                            g_cells[i].c_size) {
                                        downsize_swap_count++;
                                    }
                                }
                                if(cells[i].c_vtype == s) {
                                    ++hvt_count;
                                }
                            }

                            double hvt_ratio =
                                (double)hvt_count / (double)numcells;
                            double hvt_init_ratio =
                                (double)hvt_init_count / (double)numcells;

                            ecc::logInfo() << "[view " << view
                                 << "] # Swap             : " << swap_count
                                 << "(" << uptype_swap_count << "/"
                                 << downtype_swap_count << "/"
                                 << upsize_swap_count << "/"
                                 << downsize_swap_count << ")" << endl;
                            ecc::logInfo() << "[view " << view
                                 << "] % HVT cells        : "
                                 << hvt_ratio * 100.0 << endl;
                            ecc::logInfo() << "[view " << view
                                 << "] % HVT cells (init) : "
                                 << hvt_init_ratio * 100.0 << endl;
                        }
                        Profile();
                        for(unsigned j = 0; j < numcells; ++j) {
                            best_cells_local[j] = cells[j];
                        }
                        updated_local = true;
                    }
                }
            }

            if(TEST_MODE == "SLK_CORR_TEST" && (accum_update_cnt) % 50 == 0 &&
               accum_update_cnt != 0) {
                ecc::logInfo() << "# Cell Swap : " << accum_update_cnt << endl;
                ReportSlackErr();
            }
            if((double)accept / (double)(target_size) >= TRIAL_RATE) {
                break;
            }

            if(restore + accept > MAX_TRIALS) {
                break;
            }

            if(TRIAL_MOVE && accept > TRIAL_MOVE_NUM) {
                break;
            }
        }
    }
    else {
        while(!targets.empty()) {
            set< entry >::iterator it = targets.begin();
            bool change = false;

            // ecc::logInfo() << "TRY --- " ;
            for(unsigned m = 0; m < it->ids.size(); ++m) {
                unsigned cur = it->ids[m];
                // ecc::logInfo() << cells[cur].name << "/" << it->changes[m] << " ";

                if(changed[cur])
                    continue;

                LibCellInfo *lib_cell_info = getLibCellInfo(cells[cur]);

                if(lib_cell_info == NULL) {
                    continue;
                }

                cnt++;

                if(it->changes[m] == DNSIZE)
                    change = cell_resize(cells[cur], -1);
                else
                    change = cell_retype(cells[cur], -1);

                for(unsigned view1 = 0; view1 < numViews; ++view1) {
                    OneTimer(cells[cur], STA_MARGIN, false, view1);
                }
            }
            // ecc::logInfo() << endl;

            bool restore_flag = false;
            for(unsigned m = 0; m < it->ids.size(); ++m) {
                unsigned cur = it->ids[m];

                // check slack, max cap constraints

                LibCellInfo *lib_cell_info = getLibCellInfo(cells[cur]);

                for(unsigned view1 = 0; view1 < numViews; ++view1) {
                    unsigned corner1 = mmmcViewList[view1].corner;

                    double maxCap, curCap;
                    // CalcStats((unsigned)thread_id, true, "CORR_PWR_OPT",
                    // view1, false);

                    if(GetCellSlack(cells[cur], view1) < toler) {
                        restore_flag = true;
                        if(VERBOSE >= 1)
                            ecc::logInfo() << "RESTORED DUE TO SLACK " << view << " "
                                 << " " << GetCellSlack(cells[cur], view1)
                                 << " " << viewWNS[view1] << " " << toler
                                 << " ";
                        break;
                    }

                    for(unsigned k = 0; k < cells[cur].outpins.size(); ++k) {
                        // double slack =
                        // min(pins[view1][cells[cur].outpins[k]].rslk,
                        // pins[view1][cells[cur].outpins[k]].fslk);
                        // if ( slack < toler ) {
                        //    if ( VERBOSE >= 1 )
                        //    ecc::logInfo() << "RESTORED DUE TO SLACK " << slack << " "
                        //    << toler << " ";
                        //    restore_flag = true;
                        //    break;
                        //}

                        maxCap = lib_cell_info
                                     ->pins[pins[view1][cells[cur].outpins[k]]
                                                .lib_pin]
                                     .maxCapacitance;
                        if(use_margin) {
                            maxCap *= cap_margin;
                        }
                        curCap = pins[view1][cells[cur].outpins[k]].totcap;
                        if(curCap > maxCap) {
                            if(VERBOSE >= 1)
                                ecc::logInfo() << "RESTORED DUE TO CAP " << curCap << " "
                                     << maxCap << " ";
                            restore_flag = true;
                            break;
                        }
                        if(IsTranVio(pins[view1][cells[cur].outpins[k]])) {
                            if(VERBOSE >= 1)
                                ecc::logInfo() << "RESTORED DUE TO TRAN "
                                     << pins[view1][cells[cur].outpins[k]].rtran
                                     << "/"
                                     << pins[view1][cells[cur].outpins[k]].ftran
                                     << " "
                                     << pins[view1][cells[cur].outpins[k]]
                                            .max_tran
                                     << " ";
                            restore_flag = true;
                            break;
                        }
                    }

                    if(isff(cells[cur])) {
                        for(unsigned k = 0; k < cells[cur].inpins.size(); ++k) {
                            if(libs[0]
                                   .find(cells[cur].type)
                                   ->second
                                   .pins[pins[view1][cells[cur].inpins[k]]
                                             .lib_pin]
                                   .isClock)
                                continue;

                            double slack =
                                min(pins[view1][cells[cur].inpins[k]].rslk,
                                    pins[view1][cells[cur].inpins[k]].rslk);

                            if(slack < 0 || slack < toler) {
                                if(VERBOSE >= 1)
                                    ecc::logInfo() << "RESTORED DUE TO SLACK " << slack
                                         << " " << toler << " ";
                                restore_flag = true;
                                break;
                            }
                        }
                    }
                }
            }

            if(restore_flag && change) {  // restore
                for(unsigned m = 0; m < it->ids.size(); ++m) {
                    unsigned cur = it->ids[m];

                    if(it->changes[m] == DNSIZE)
                        cell_resize(cells[cur], 1);
                    else
                        cell_retype(cells[cur], 1);
                    OneTimer(cells[cur], STA_MARGIN);
                    restore++;
                }
                if(VERBOSE > 0 || VERBOSE >= 5)
                    ecc::logInfo() << " Restore" << endl;
                entry tmpEntry(*it);
                targets.erase(it);
            }
            else {
                if(VERBOSE > 0 || VERBOSE >= 5)
                    ecc::logInfo() << " Accept" << endl;

                for(unsigned m = 0; m < it->ids.size(); ++m) {
                    unsigned cur = it->ids[m];
                    changed[cur] = true;
                    if(tabuNum > 0) {
                        unsigned pre_sol =
                            cells[cur].c_size * 10 + cells[cur].c_vtype;
                        if(cells[cur].tabu.size() < tabuNum) {
                            cells[cur].tabu.push_back(pre_sol);
                        }
                        else {
                            cells[cur].tabu.pop_front();
                            cells[cur].tabu.push_back(pre_sol);
                        }

                        // ecc::logInfo() << cells[cur].name << "'s tabu list is";
                        list< unsigned >::iterator iter;
                        for(iter = cells[cur].tabu.begin();
                            iter != cells[cur].tabu.end(); iter++) {
                            //    ecc::logInfo() << " " << *iter ;
                        }
                        ecc::logInfo() << endl;
                    }
                }
                accept += PEEPHOLE_NUMCELL;
                update_cnt += PEEPHOLE_NUMCELL;
                accum_update_cnt += PEEPHOLE_NUMCELL;
                entry tmpEntry(*it);
                targets.erase(it);
            }

            if(cnt > numcells * PWR_CORR_RATIO && CORR_PT) {
                for(unsigned view1 = 0; view1 < numViews; ++view1) {
                    CorrelatePT((unsigned)thread_id, view1);
                    CalcStats((unsigned)thread_id, true, "CORR_PWR_OPT", view1);
                    cnt = 0;
                }
                double wns = worst_slack_worst;
                if(VERBOSE >= 1) {
                    ecc::logInfo() << "Current WNS : " << wns << endl;
                }

                if(SAFE_LEAK_OPT) {
                    if(wns < SAFE_LEAK_OPT_GB) {
                        break;
                    }
                }
            }

            //            for ( unsigned view1 = 0; view1 < numViews; ++view1 )
            //            {
            //                CalcStats((unsigned)thread_id, false,
            //                "PWR_OPT_INT", view1);
            //            }

            if(TEST_MODE == "SLK_CORR_TEST" && (accum_update_cnt) % 50 == 0 &&
               accum_update_cnt != 0) {
                ecc::logInfo() << "# Cell Swap : " << accum_update_cnt << endl;
                ReportSlackErr();
            }
            if((double)accept / (double)(target_size) >= TRIAL_RATE) {
                break;
            }
            if(restore + accept > MAX_TRIALS) {
                break;
            }

            if(TRIAL_MOVE && accept > TRIAL_MOVE_NUM) {
                break;
            }
        }
    }

    // ecc::logInfo() << endl;
    ecc::logInfo() << "#Swaps for power reduction: " << accept << endl;
    ecc::logInfo() << "#Trials for power reduction: " << restore + accept << endl;
    ecc::logInfo() << "Accept rate: " << (double)accept / (double)(target_size) << endl;
    targets.clear();
    ecc::logInfo() << "Reduce power time : " << cpuTime() - init_time << endl;
    return accept;
}

bool Sizer::CheckMaxCap(CELL &cell) {
    LibCellInfo *lib_cell_info = getLibCellInfo(cell);

    if(lib_cell_info == NULL) {
        return true;
    }

    double maxCap, curCap;
    for(unsigned k = 0; k < cell.outpins.size(); ++k) {
        for(unsigned view = 0; view < numViews; ++view) {
            maxCap = lib_cell_info->pins[pins[view][cell.outpins[k]].lib_pin]
                         .maxCapacitance;
            if(use_margin) {
                maxCap *= cap_margin;
            }
            curCap = pins[view][cell.outpins[k]].totcap;
            if(curCap > maxCap) {
                return false;
            }
        }
    }
    return true;
}
void Sizer::initSingleMode() {
    numCorners = 1;
    numModes = 1;
    numViews = 1;

    view _view;
    mmmcViewList.push_back(_view);
    viewTNS.push_back(0.0);
    viewRuntime.push_back(0.0);
    viewWNS.push_back(0.0);
    viewWeight.push_back(1.0);
    viewWeightTime.push_back(1.0);

    viewVioCnt.push_back(0);
    viewWorstSlack.push_back(0.0);
    viewTNSHold.push_back(0.0);
    viewWNSHold.push_back(0.0);
    viewRuntime.push_back(0.0);
    viewPower.push_back(0.0);
    init_wns.push_back(0.0);
    init_wns_worst = DBL_MAX;
    init_tns.push_back(0.0);
    init_leak.push_back(0.0);
    init_score.push_back(DBL_MAX);
    init_tot.push_back(0.0);
    viewSlackMargin.push_back(slack_margin);
    viewTNSMargin.push_back(0.0);
}

void Sizer::main(unsigned thread_id, bool postGTR) {
    string size_str;

    int step = 0;
    while(1) {
        step++;
        cells = new CELL[numcells];
        pins = new PIN *[numViews];

        for(unsigned i = 0; i < numViews; ++i) {
            pins[i] = new PIN[numpins];
            for(unsigned j = 0; j < numpins; j++)
                pins[i][j] = g_pins[i][j];
        }

        nets = new NET *[numCorners];

        for(unsigned i = 0; i < numCorners; ++i) {
            nets[i] = new NET[numnets];
            for(unsigned j = 0; j < numnets; j++)
                nets[i][j] = g_nets[i][j];
        }

        // pthread_mutex_lock(&mutex1);
        if(search_queue.empty()) {
            // pthread_mutex_unlock(&mutex1);
            break;
        }
        double ratio = (search_queue.front()).first;
        double exponent = (search_queue.front()).second;
        search_queue.pop();

        char filename[100];
        if(!postGTR) {
            sprintf(filename, "%s%s%s_%d", directory.c_str(), benchname.c_str(),
                    "_gtr_no_pt", thread_id);
            ofstream out_file(filename, ios::app);
            out_file << "RATIO : " << ratio << "\tEXP : " << exponent << "\n";
            out_file.close();
        }
        else {
            sprintf(filename, "%s%s%s_%d", directory.c_str(), benchname.c_str(),
                    "gtr_pt", thread_id);
            ofstream out_file(filename, ios::app);
            out_file << "RATIO : " << ratio << "\tEXP : " << exponent << "\n";
            out_file.close();
        }

        if(CORR_PT)
            T = PTimer[thread_id];

        // bool worse_than_second_best=false;

        // make a thread-local copy of netlist
        ecc::logInfo() << "Generating thread local instance of circuit .. thread "
             << thread_id << " ratio : " << ratio
             << " leakage exponent : " << exponent
             << " guardband: " << guardband[worst_corner] << endl;

        // if (postGTR) SetGB(guardband[worst_corner]);

        if(postGTR && TOPX == ratio && TOPEXP == exponent && gtr1_feasible) {
            ecc::logInfo() << "thread" << thread_id << " post gtr - best solution"
                 << endl;
            for(unsigned i = 0; i < numcells; i++)
                cells[i] = best_cells[i];
        }
        else if(postGTR && !GTRWPT_ONLY) {
            if(gtr1_feasible) {
                ecc::logInfo() << "thread" << thread_id << " post gtr - feasible" << endl;
                for(unsigned i = 0; i < numcells; i++)
                    cells[i] = best_cells[i];
            }
            else {
                ecc::logInfo() << "thread" << thread_id << " post gtr - infeasible"
                     << endl;
                // for(unsigned i = 0; i < numcells; i++)
                // cells[i] = best_failed_cells[i]; //FIXME:
            }
        }
        else {
            for(unsigned i = 0; i < numcells; i++)
                cells[i] = g_cells[i];

            if(GTRWPT_ONLY) {
                SizeIn(outputDir);
                ecc::logInfo() << "2nd gtr only -- size in" << endl;
            }
            else {
                ecc::logInfo() << "thread" << thread_id << " other" << endl;
            }
        }

        // pthread_mutex_unlock(&mutex1);

        if(FIX_CAP_REPAIR) {
            FwdFixCapViolation();
            BwdFixCapViolation();
        }

        double oneTMR = cpuTime();
        CallTimer();
        oneTMR = (cpuTime() - oneTMR);
        CorrelatePT(thread_id);
        // pthread_mutex_lock(&mutex1);
        CalcStats(thread_id);
        // pthread_mutex_unlock(&mutex1);

        ecc::logInfo() << "Single timing simulation runtime = " << oneTMR << endl;
        ecc::logInfo() << "Violations with minimum sizes = " << tot_violations << endl;
        oneTMR *= 1.1;  // can vary up to 10%

        double prev_ss = slew_violation + skew_violation;
        double prev_wns = min(max_neg_rslk, max_neg_fslk);

        double curr_wns = prev_wns;
        double curr_ss = prev_ss;
        double pprev_ss = prev_ss;

        vector< CELL > int_best_cells;
        int_best_cells.resize(numcells);
        for(unsigned i = 0; i < numcells; i++)
            int_best_cells[i] = cells[i];

        double cur_best_tns = skew_violation;

        if(prev_ss != 0.0) {
            unsigned maxSimulN = min(
                (int)floor((0.9 * RuntimeLimit - (cpuTime() - global_begin)) /
                           (oneTMR * 3)),
                100);
            unsigned same_ss_count = 0;
            unsigned swap_cnt;
            unsigned i = 0;
            int release_cnt = 0;
            int round = 0;
            int worse_ss_count = 0;
            while(i < maxSimulN) {
                ecc::logInfo() << "gtr(i/same/worse/release/wns/tns) -- " << i << " "
                     << same_ss_count << " " << worse_ss_count << " "
                     << release_cnt << " " << min(max_neg_rslk, max_neg_fslk)
                     << " " << skew_violation << " " << tot_violations << endl;

                if(skew_violation == 0.0)
                    break;
                if(round > maxSimulN * 5)
                    break;

                // hyein: to apply guardband
                if(postGTR && GetGB() != 0)
                    CalcSlack();

                if(GTR_METRIC1 == SF1)
                    CountNPaths();

                double wns;
                wns = min(max_neg_rslk, max_neg_fslk);

                if(cur_best_tns > skew_violation) {
                    cur_best_tns = skew_violation;
                    for(unsigned j = 0; j < numcells; j++)
                        int_best_cells[j] = cells[j];
                }

                if(!postGTR)  // this should be a multiple of numcells
                {
                    swap_cnt = Attack(i + 1, GLOBAL, ratio, exponent);
                    if(swap_cnt == -1) {
                        swap_cnt = 0;
                    }
                }
                else {
                    swap_cnt = Attack(i + 1, GLOBAL, ratio, exponent);
                    if(swap_cnt == -1) {
                        swap_cnt = 0;
                    }
                }
                if(FIX_CAP_REPAIR) {
                    FwdFixCapViolation();
                    BwdFixCapViolation();
                }
                if(FIX_SLEW_REPAIR) {
                    FwdFixSlewViolation(1.0);
                }
                CallTimer();
                ecc::logInfo() << "from our timer ";
                CalcStats(i, thread_id);
                pprev_ss = prev_ss;
                prev_ss = curr_ss;
                prev_wns = curr_wns;

                if(postGTR) {
                    CorrelatePT(thread_id);
                    ecc::logInfo() << "after correlation ";
                    CalcStats(thread_id);
                }

                wns = min(max_neg_rslk, max_neg_fslk);
                if(postGTR)
                    ecc::logInfo() << "2ndOPT" << thread_id
                         << " itr/wns/TNS/PWR/swap/GB : " << i << " " << wns
                         << " "
                         << " " << skew_violation << " " << power << " "
                         << swap_cnt << " " << GetGB() << endl;
                else
                    ecc::logInfo() << "OPT" << thread_id
                         << " itr/wns/TNS/PWR/swap/GB : " << i << " " << wns
                         << " "
                         << " " << skew_violation << " " << power << " "
                         << swap_cnt << " " << GetGB() << endl;
                curr_ss = slew_violation + skew_violation;
                curr_wns = wns;

                if(curr_ss == 0) {
                    ecc::logInfo() << endl
                         << "A feasible solution has been found. Global OPT is "
                            "done. "
                         << endl;
                    break;
                }

                if(prev_ss < curr_ss) {
                    swap_cnt += OptWNSPath(DNSIZE);
                    ecc::logInfo() << "OPT" << thread_id << " OptWNSPath" << endl;
                }
                same_ss_count = (prev_ss > curr_ss) && swap_cnt > 0
                                    ? 0
                                    : same_ss_count + 1;  // slack degradation
                if(pprev_ss == curr_ss)
                    same_ss_count++;

                if(cur_best_tns * 3.0 < skew_violation) {
                    worse_ss_count++;
                }

                ecc::logInfo() << "Delta TNS = " << curr_ss - prev_ss;
                ecc::logInfo() << " Delta WNS = " << -(curr_wns - prev_wns) << endl;

                if(postGTR && same_ss_count > SAME_SS_LIMIT && curr_ss != 0 &&
                   release_cnt < 5) {
                    break;

                    ecc::logInfo() << "Stuck -- Releasing cells. Downsizing every cell "
                            "on critical paths by one step..."
                         << endl;
                    Release(false, CRIT_LEGALIZE);
                    release_cnt++;
                    same_ss_count = 0;
                    continue;
                }

                if(postGTR && worse_ss_count > 10 && release_cnt < 5) {
                    break;

                    // revert the solution
                    for(unsigned j = 0; j < numcells; j++)
                        cells[j] = int_best_cells[j];
                    ecc::logInfo() << "Stuck -- going back to the best and downsizing "
                            "every cell by one step..."
                         << endl;
                    Release(false, LEGALIZE);
                    release_cnt++;
                    same_ss_count = 0;
                    worse_ss_count = 0;
                    continue;
                }

                if(postGTR && skew_violation_cnt < 10) {
                    break;
                }

                if(same_ss_count > SAME_SS_LIMIT + 2) {
                    break;
                }

                i++;
                round++;
            }
        }

        // if the final solution is worse than the local optimal
        if(skew_violation > 0) {
            if(cur_best_tns < skew_violation) {
                ecc::logInfo() << "Revert to the best solution.. " << endl;
                for(unsigned i = 0; i < numcells; i++)
                    cells[i] = int_best_cells[i];
            }

            CallTimer();
            if(postGTR) {
                CorrelatePT(thread_id);
            }
            CalcStats(thread_id);

            string size_str = "2ndgtr";
            SizeOut(outputDir);
        }

        if(postGTR) {
            // mutex lock is applied. Update the best soluation if necessary
            // pthread_mutex_lock(&mutex1);
            if(skew_violation == 0.) {
                gtr2_feasible = true;
            }
            // pthread_mutex_unlock(&mutex1);
        }

        // mutex lock is applied. Update the best soluation if necessary
        // pthread_mutex_lock(&mutex1);
        if(skew_violation == 0.) {
            feasible = true;
            if(score < best_score) {
                if(best_cells.size() != 0) {
                    second_best_power = best_score;
                    SEC_TOPX = TOPX;
                    SEC_TOPEXP = TOPEXP;
                }

                best_score = score;

                if(!postGTR)
                    size_str = "1stgtr";
                else
                    size_str = "2ndgtr";

                SizeOut(outputDir);

                TOPX = ratio;
                TOPEXP = exponent;

                best_cells.resize(numcells);
                best_thread = thread_id;
                for(unsigned i = 0; i < numcells; i++)
                    best_cells[i] = cells[i];
            }
        }
        else {
            if(tot_violations < best_failed) {
                best_failed = tot_violations;

                if(!postGTR)
                    size_str = "1stgtr";
                else
                    size_str = "2ndgtr";
                SizeOut(outputDir);

                // best_failed_cells.resize(numcells); //FIXME:
                best_failed_thread = thread_id;
                for(unsigned i = 0; i < numcells; i++)
                    // best_failed_cells[i] = cells[i];  //FIXME:

                    if(!postGTR) {
                        FTOPX = ratio;
                        FTOPEXP = exponent;
                    }
            }
        }
        // pthread_mutex_unlock(&mutex1);

        if(skew_violation == 0.)
            ecc::logInfo() << "Feasible power = " << power << " uW ( @ X = " << ratio
                 << " EXP = " << exponent << ")" << endl;
        else
            ecc::logInfo() << "Infeasible violations = " << tot_violations
                 << " ( @ X = " << ratio << " EXP = " << exponent << ")"
                 << endl;

        for(unsigned i = 0; i < numViews; ++i) {
            delete[] pins[i];
        }
        for(unsigned i = 0; i < numCorners; ++i) {
            delete[] nets[i];
        }
        delete[] pins;
        delete[] cells;
        delete[] nets;
    }
    // pthread_mutex_lock(&mutex1);
    // ecc::logInfo() << "\nthread " << thread_id << " leaving main with PT " << postGTR
    // << "\n";
    // pthread_mutex_unlock(&mutex1);
    return;
}

void Sizer::Profile() {
    ecc::logInfo() << "Profiling ----" << endl;
    double tot_time = cpuTime() - global_begin;

    double timer_time = 0;
    for(unsigned i = 0; i < MAX_THREAD; i++) {
        for(int view = 0; view < numViews; view++) {
            timer_time += PTimer[i][view]->pt_time;
        }
    }

    ecc::logInfo() << "Total runtime      : " << tot_time << " sec. (" << tot_time / 60
         << " min.)" << endl;
    ecc::logInfo() << "Timer runtime      : " << timer_time << " sec. ("
         << timer_time / 60 << " min.)" << endl;
}

void ecc::runCellSizing(ord::Design& design, const RepairConfig& config,
                        const std::function<void()>& refresh,
                        bool extracted_parasitics) {
    // The original sizing search retains its analysis arrays; the only mutable
    // physical design and all input parsing belong to the OpenROAD session.
    Sizer sizer(0, nullptr);
    sizer.exeOp = 1;
    sizer.swapOp = 1;
    sizer.stopCond = 0;
    sizer.maxTrCheck = false;
    GUARD_BAND = 0.0;
    sizer.swapstep = 10000;
    sizer.sdcFile = "";
    sizer.timerSdcFile = "";
    sizer.defFile = "";
    sizer.spefFile = "";
    sizer.verilogFile = "";
    sizer.clockName = "clk";
    sizer.reportFile = "result.rpt";
    sizer.verilogOutFile = "";
    sizer.defOutFile = "";
    sizer.spefOutFile = "";
    sizer.ptCmd = "";
    sizer.ptOption = "";
    sizer.ptLogSave = true;
    sizer.exePNRFlag = false;
    sizer.chkWNSFlag = false;
    sizer.totPWRFlag = false;
    sizer.mmmcOn = false;
    sizer.useETS = false;
    sizer.useTempus = false;
    sizer.useOpenSTA = true;
    sizer.noDEF = false;
    sizer.noSPEF = false;
    sizer.saifFile = "";
    sizer.swapFile = "";
    sizer.optEffort = 1;
    sizer.soceScriptFile = "";
    sizer.etsScriptFile = "";
    sizer.ptScriptFile = "";
    sizer.ptLaunchScriptFile = "";
    sizer.tcfFile = "";
    sizer.soceBin = "";
    sizer.sensFunc = 1;  // SF 6 is TNS, originally 0
    sizer.sensFunc2 = 0;
    sizer.sensFuncT = 5;  // SF 6 is TNS only, originally 5
    sizer.falsePathFile = "";
    sizer.timerTestCnt = 0;
    sizer.timerTestCell = 0;
    sizer.timerTestMove = 0;
    sizer.use_gr_rc = design.getResizer()->getEstimateParasitics()->getParasiticsSrc()
                      == est::ParasiticsSrc::kGlobalRouting;

    sizer.benchname = design.getBlock()->getName();
    sizer.numVt = 1;
    sizer.numCorners = sizer.numModes = sizer.numViews = 1;
    sizer.RuntimeLimit = 3600;
    sizer.global_begin = cpuTime();
    sizer.time_IO = sizer.time_SizeOut = sizer.time_WNSOpt = 0;
    sizer.time_Coarse = sizer.time_Zoomin = sizer.time_KickOpt = sizer.time_LeakOpt = 0;
    sizer.time_OneTimer = sizer.time_CallTimer = sizer.time_Fineswap = 0;
    sizer.count_OneTimer = sizer.count_CallTimer = 0;
    sizer.nativeRepairConfig = config;
    sizer.refresh_design = refresh;
    sizer.noSPEF = false;  // Both estimated and extracted RC come from STA.
    sizer.spefFile = extracted_parasitics ? "loaded" : "";
    sizer.initSingleMode();
    NO_FOOTPRINT = true;
    PRESERVE_PLACEMENT = extracted_parasitics;
    sizer.Parser(design);
    sizer.Parallel_Sizer_Launcher();
    sizer.FinalReport();
}
