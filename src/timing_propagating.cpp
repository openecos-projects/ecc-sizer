#include <algorithm>
#include <cassert>
#include <climits>
#include <cstdio>
#include <iostream>
#include <map>
#include <sstream>
#include "ckt.h"
#include "ord/Timing.h"
#include "sizer.h"
#include <limits>
#include <unordered_map>
#include <vector>
#include "float.h"
#include "utils.h"
#include <vector>
#include <thread>
#include <mutex>
#include <deque>
#include <unordered_set>
#include <unordered_map>
#include <iostream>

#if 0
void process_pin_range(vector< unsigned >& fwpins, int start, int end,
                       vector< unsigned >& bwpins,
                       unordered_set< unsigned >& bwpins_set,
                       vector< unsigned >& endpins,
                       unordered_map< unsigned, int >& visited, int view,
                       int corner, float margin, mutex& bwpins_mutex,
                       mutex& endpins_mutex, mutex& visited_mutex,
                       deque< unsigned >& local_fwpins,
                       unordered_set< unsigned >& local_fwpins_set);

// Main entry point
void Sizer::parallel_bfs(vector< unsigned >& fwpins, vector< unsigned >& bwpins,
                         unordered_set< unsigned >& fwpins_set,
                         unordered_set< unsigned >& bwpins_set,
                         vector< unsigned >& endpins,
                         unordered_map< unsigned, int >& visited, int view,
                         int corner, float margin) {
    int num_threads = thread::hardware_concurrency();  // Number of hardware-supported threads
    vector< thread > threads;
    mutex bwpins_mutex, endpins_mutex, visited_mutex;

    // Per-thread local fwpins queues
    vector< deque< unsigned > > local_fwpins(num_threads);
    vector< unordered_set< unsigned > > local_fwpins_set(num_threads);

    // Split work across threads
    int chunk_size = (fwpins.size() + num_threads - 1) / num_threads;
    for(int t = 0; t < num_threads; ++t) {
        int start = t * chunk_size;
        int end = min(start + chunk_size, (int)fwpins.size());
        if(start < end) {
            threads.emplace_back(process_pin_range, ref(fwpins), start, end,
                                 ref(bwpins), ref(bwpins_set), ref(endpins),
                                 ref(visited), view, corner, margin,
                                 ref(bwpins_mutex), ref(endpins_mutex),
                                 ref(visited_mutex), ref(local_fwpins[t]),
                                 ref(local_fwpins_set[t]));
        }
    }

    // Wait for all threads to finish
    for(auto& t : threads)
        t.join();

    // Merge local fwpins into the global queue if further processing is needed.
    for(int t = 0; t < num_threads; ++t) {
        while(!local_fwpins[t].empty()) {
            unsigned fopin = local_fwpins[t].front();
            local_fwpins[t].pop_front();
            if(fwpins_set.count(fopin) == 0) {
                fwpins.push_back(fopin);
                fwpins_set.insert(fopin);
            }
        }
    }
}

// Parallel worker function
void process_pin_range(PIN** pins, NET** nets, vector< unsigned >& fwpins,
                       int start, int end, vector< unsigned >& bwpins,
                       unordered_set< unsigned >& bwpins_set,
                       vector< unsigned >& endpins,
                       unordered_map< unsigned, int >& visited, int view,
                       int corner, float margin, mutex& bwpins_mutex,
                       mutex& endpins_mutex, mutex& visited_mutex,
                       deque< unsigned >& local_fwpins,
                       unordered_set< unsigned >& local_fwpins_set) {
    for(int iter = start; iter < end; ++iter) {
        unsigned fipin = fwpins[iter];
        if(VERBOSE >= 2)
            cout << "--- UPDATE PIN TIMING START "
                 << getFullPinName(pins[view][fipin]) << endl;

        bool change = updatePinTiming(pins[view][fipin], margin, view);

        if(VERBOSE >= 2)
            cout << "--- UPDATE PIN TIMING END "
                 << getFullPinName(pins[view][fipin]) << endl;

        unsigned curnet = pins[view][fipin].net;
        if(VERBOSE >= 2)
            cout << "----- NET " << nets[corner][curnet].name << " "
                 << nets[corner][curnet].outpins.size() << endl;

        // Synchronize visit counts
        {
            lock_guard< mutex > lock(visited_mutex);
            if(!visited.count(curnet))
                visited[curnet] = 0;
            int t_visit = ++visited[curnet];
            if(t_visit > MAX_VISIT)
                change = false;
        }

        if(change) {
            for(unsigned j = 0; j < nets[corner][curnet].outpins.size(); ++j) {
                unsigned curpin = nets[corner][curnet].outpins[j];
                unsigned curfo = pins[view][curpin].owner;
                if(VERBOSE >= 2)
                    cout << "------- CUR PIN "
                         << getFullPinName(pins[view][curpin]) << endl;

                if(curfo == UINT_MAX) {  // PO
                    if(VERBOSE >= 2)
                        cout << "REACH PO -- ADD BW PIN "
                             << getFullPinName(pins[view][curpin]) << endl;
                    lock_guard< mutex > lock(bwpins_mutex);
                    if(bwpins_set.count(curpin) == 0) {
                        bwpins.push_back(curpin);
                        bwpins_set.insert(curpin);
                    }
                    continue;
                }

                if(getLibCellInfo(cells[curfo], corner) != nullptr) {
                    if(isff(cells[curfo])) {
                        if(!libs[corner]
                                .find(cells[curfo].type)
                                ->second.pins[pins[view][curpin].lib_pin]
                                .isClock) {
                            if(VERBOSE >= 2)
                                cout << "REACH FF -- ADD BW PIN "
                                     << getFullPinName(pins[view][curpin])
                                     << endl;
                            lock_guard< mutex > lock(bwpins_mutex);
                            if(bwpins_set.count(curpin) == 0) {
                                bwpins.push_back(curpin);
                                bwpins_set.insert(curpin);
                            }
                        }
                        lock_guard< mutex > lock(endpins_mutex);
                        for(unsigned k = 0; k < cells[curfo].outpins.size();
                            ++k) {
                            endpins.push_back(cells[curfo].outpins[k]);
                        }
                        continue;
                    }
                }

                for(unsigned k = 0; k < cells[curfo].outpins.size(); ++k) {
                    unsigned fopin = cells[curfo].outpins[k];
                    if(VERBOSE >= 2)
                        cout << "ADD FW PIN FO "
                             << getFullPinName(pins[view][fopin]) << endl;
                    if(local_fwpins_set.count(fopin) == 0) {
                        local_fwpins.push_back(fopin);
                        local_fwpins_set.insert(fopin);
                    }
                }
            }
        }
        else {
            for(unsigned j = 0; j < nets[corner][curnet].outpins.size(); ++j) {
                unsigned curpin = nets[corner][curnet].outpins[j];
                unsigned curfo = pins[view][curpin].owner;
                if(VERBOSE >= 2)
                    cout << "ADD BW PIN NO CHANGE "
                         << getFullPinName(pins[view][curpin]) << endl;

                lock_guard< mutex > lock(bwpins_mutex);
                if(bwpins_set.count(curpin) == 0) {
                    bwpins.push_back(curpin);
                    bwpins_set.insert(curpin);
                }
                lock_guard< mutex > lock2(endpins_mutex);
                if(curfo != UINT_MAX) {
                    for(unsigned k = 0; k < cells[curfo].outpins.size(); ++k) {
                        endpins.push_back(cells[curfo].outpins[k]);
                    }
                }
                else {
                    endpins.push_back(curpin);
                }
            }
        }
    }
}
#endif

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
