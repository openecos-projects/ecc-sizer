// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair_design_ord.h"

#include <chrono>
#include <unordered_map>

#include "db_sta/dbNetwork.hh"
#include "repair_acceptance.h"
#include "repair_backend.h"
#include "sta/Graph.hh"

namespace ecc {

OrdDesignRepairResult OpenDbOrdDesignRepairer::repair(
    odb::dbBlock* block, sta::dbSta* sta,
    const std::function< void() >& refresh,
    const DesignRepairOptions& options) {
    OrdDesignRepairResult result;
    try {
        options.validate();
    }
    catch(const std::exception& error) {
        result.message = error.what();
        return result;
    }
    if(!block || !sta || !refresh || sta->scenes().size() != 1 ||
       sta->getDbNetwork()->block() != block) {
        result.message = "invalid OpenROAD-style repair backend";
        return result;
    }

    detail::Repair backend(block, sta, options);
    if(!backend.hasCells()) {
        result.message = "no usable Liberty cell models";
        return result;
    }

    const auto analyze = [&](DesignRepairMetrics& metrics) {
        return backend.analyze(metrics, true);
    };
    auto states = analyze(result.before);
    result.after = result.before;
    const auto* network = sta->getDbNetwork();

    for(unsigned pass = 0;
        pass < options.max_passes && result.after.severity > 0; ++pass) {
        bool pass_progress = false;
        unsigned batch_limit = options.ord_driver_batch_size;
        std::unordered_set< odb::dbNet* > processed;
        while(true) {
            // RepairDesign walks levelized driver vertices from sinks back to
            // sources. Rebuild the map after each accepted ECO because buffer
            // insertion changes the OpenSTA graph and net ownership.
            std::unordered_map< odb::dbNet*, detail::NetState* > by_net;
            by_net.reserve(states.size());
            for(auto& state : states)
                by_net.emplace(state.net, &state);

            std::vector< odb::dbNet* > selected_nets;
            const auto& drivers = sta->levelizedDrvrVertices();
            for(int index = int(drivers.size()) - 1;
                index >= 0 && selected_nets.size() < batch_limit; --index) {
                auto* vertex = drivers[index];
                if(!vertex || !vertex->pin())
                    continue;
                auto* object = network->staToDb(vertex->pin());
                if(!object || object->getObjectType() != odb::dbITermObj)
                    continue;
                auto* driver = static_cast< odb::dbITerm* >(object);
                auto* net = driver->getNet();
                const auto found = by_net.find(net);
                if(found == by_net.end() || !found->second->violating() ||
                   processed.count(net))
                    continue;
                processed.insert(net);
                selected_nets.push_back(net);
            }
            if(selected_nets.empty())
                break;

            odb::dbDatabase::beginEco(block);
            bool active = true;
            unsigned inserted = 0, resized_buffers = 0, resized_gates = 0,
                     vt_swaps = 0, repaired = 0;
            try {
                for(auto* net : selected_nets) {
                    const auto found = by_net.find(net);
                    if(found == by_net.end())
                        continue;
                    if(backend.edit(*found->second, inserted, resized_buffers,
                                    resized_gates,
                                    options.max_inserted_buffers -
                                        result.inserted_buffers,
                                    true, vt_swaps))
                        ++repaired;
                }
                if(repaired == 0) {
                    odb::dbDatabase::undoEco(block);
                    active = false;
                    for(auto* net : selected_nets)
                        processed.erase(net);
                    break;
                }
                if(options.prepare)
                    options.prepare();
                refresh();
                DesignRepairMetrics after;
                auto next_states = analyze(after);
                // OpenROAD repair_design commits a DRV-improving driver edit;
                // setup/hold protection belongs to repair_timing.  Keeping
                // this acceptance rule separate is essential for a faithful
                // A/B reference and is the main difference from Native's
                // timing-preserving transaction policy.
                if(after.severity + options.tolerances.drv_severity <
                   result.after.severity) {
                    odb::dbDatabase::commitEco(block);
                    active = false;
                    result.after = after;
                    states = std::move(next_states);
                    result.repaired_nets += repaired;
                    result.inserted_buffers += inserted;
                    result.resized_buffers += resized_buffers;
                    result.resized_gates += resized_gates;
                    result.vt_swaps += vt_swaps;
                    pass_progress = true;
                    continue;
                }
                odb::dbDatabase::undoEco(block);
                active = false;
                refresh();
                states = analyze(result.after);
                for(auto* net : selected_nets)
                    processed.erase(net);
                ++result.rejected_passes;
                if(batch_limit > 1) {
                    batch_limit = std::max(1u, batch_limit / 2);
                    continue;
                }
                result.message =
                    "OpenROAD-style batch rejected: DRV or timing regressed";
                break;
            }
            catch(const std::exception& error) {
                result.message = error.what();
                if(active) {
                    odb::dbDatabase::undoEco(block);
                    for(auto* net : selected_nets)
                        processed.erase(net);
                    try {
                        refresh();
                        states = analyze(result.after);
                    }
                    catch(const std::exception& restore_error) {
                        result.message +=
                            std::string("; refresh after rollback failed: ") +
                            restore_error.what();
                    }
                }
                return result;
            }
            break;
        }
        if(pass_progress)
            ++result.passes;
        else if(result.message.empty())
            result.message = "no feasible OpenROAD-style edit";
        if(!pass_progress)
            break;
    }

    result.ok = true;
    result.converged = result.after.severity <= options.tolerances.drv_severity;
    if(result.message.empty())
        result.message = result.converged
                             ? "OpenROAD-style DRV repair converged"
                             : "OpenROAD-style pass limit reached; violations remain";
    return result;
}

}  // namespace ecc
