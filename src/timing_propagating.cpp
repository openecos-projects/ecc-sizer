#include <algorithm>
#include <cassert>
#include <climits>
#include <cstdio>
#include <iostream>
#include <map>
#include <sstream>
#include "sizing_model.h"
#include "ord/Timing.h"
#include "cell_sizing.h"
#include <limits>
#include <unordered_map>
#include <vector>
#include "float.h"
#include "utils.h"
#include <thread>
#include <mutex>
#include <deque>
#include <unordered_set>

// Forward timing propagation for one net arc.
void Sizer::net_arc_forward_timing(int net_inpin, int corner, float margin) {
    unsigned curnet = g_pins[corner][net_inpin].net;
    unsigned inpin = g_nets[corner][curnet].inpin;
    auto& pin = g_pins[corner][net_inpin];
    if(inpin == UINT_MAX) {
        return;
    }
    if(pin.owner == UINT_MAX) {
        return;
    }
    if(pin.isPI) {
        return;
    }
    if(pin.isPO) {
        return;
    }
    for(unsigned j = 0; j < nets[corner][curnet].outpins.size(); j++) {
        unsigned fopin = nets[corner][curnet].outpins[j];
        timing_lookup wire_delay = get_wire_delay(curnet, fopin, corner);
        timing_lookup wire_tran =
            get_wire_tran(curnet, fopin, pin.rtran, pin.ftran, corner);

        double prv_rtran = pins[corner][fopin].rtran;
        double prv_ftran = pins[corner][fopin].ftran;
        pins[corner][fopin].rtran =
            wire_tran.rise + pins[corner][fopin].rtran_ofs;
        pins[corner][fopin].ftran =
            wire_tran.fall + pins[corner][fopin].ftran_ofs;
        double prv_rAAT = pins[corner][fopin].rAAT;
        double prv_fAAT = pins[corner][fopin].fAAT;
        if(CORR_AAT) {
            pins[corner][fopin].rAAT =
                pin.rAAT + wire_delay.rise + pins[corner][fopin].rAAT_ofs;
            pins[corner][fopin].fAAT =
                pin.fAAT + wire_delay.fall + pins[corner][fopin].fAAT_ofs;
        }
        else {
            pins[corner][fopin].rAAT = pin.rAAT + wire_delay.rise;
            pins[corner][fopin].fAAT = pin.fAAT + wire_delay.fall;
        }

        pin.rslk = pin.rRAT - pin.rAAT + pin.rslk_ofs + pin.slk_gb;
        pin.fslk = pin.fRAT - pin.fAAT + pin.fslk_ofs + pin.slk_gb;
        double diff_tran = computeMaxDiff(prv_ftran, pins[corner][fopin].rtran,
                                          prv_ftran, pins[corner][fopin].ftran);
        double diff_AAT = computeMaxDiff(prv_rAAT, pins[corner][fopin].rAAT,
                                         prv_fAAT, pins[corner][fopin].fAAT);

        logPinState(pins[corner][fopin], corner, "UPDATE PIN AAT - NEW ", 2);
    }
}
