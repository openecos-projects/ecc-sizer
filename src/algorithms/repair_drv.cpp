// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "algorithms/repair_drv.h"
#include "design_analysis.h"
#include "algorithms/cell_candidates.h"
#include "algorithms/repair_setup.h"
#include "sta/Sdc.hh"
#include "sta/Mode.hh"
#include "sta/Liberty.hh"
#include "db_sta/dbSta.hh"
#include <chrono>
#include <unordered_map>
#include "db_sta/dbNetwork.hh"
#include "sta/Graph.hh"
#include <algorithm>
#include <deque>
#include <unordered_set>
#include <utility>
#include <vector>
#include "odb/db.h"
#include "odb/dbTypes.h"
#include "sta/PowerClass.hh"
#include "power/Power.hh"
#include <set>
#include "spdlog/spdlog.h"

namespace ecc {
using detail::CellModel;
using detail::DesignAnalysis;
using detail::Load;
using detail::NetState;

static const CellModel *selectDrvResize(DesignAnalysis &analysis,
                                        detail::CellCandidates &cells,
                                        const NetState &state) {
    // Keep electrically feasible drive-strength repair first. A same-size
    // VT change is a fallback for DRV; setup compares its delay directly,
    // and power recovery can reclaim drive/VT after timing is established.
    const auto sized = cells.resize(state);
    if (!sized.empty())
        return sized.front();
    const auto vt = cells.resize(state, false, true);
    if (vt.empty())
        return nullptr;
    return *std::min_element(vt.begin(), vt.end(), [&](const auto *a, const auto *b) {
        const float sa =
            analysis.predictedSlew(*a, state.capacitance, state.input_slew);
        const float sb =
            analysis.predictedSlew(*b, state.capacitance, state.input_slew);
        return sa != sb ? sa < sb : a->master->getName() < b->master->getName();
    });
}

DrvNetPlan planDrvNetRepair(detail::DesignAnalysis &analysis,
                            detail::CellCandidates &cells, sta::dbSta *sta,
                            const detail::NetState &input, unsigned available,
                            bool allow_resize, const RepairCommonOptions &options) {
    DrvNetPlan plan;
    if (input.loads.empty())
        return plan;
    if (!options.actions.buffer)
        available = 0;
    auto state = input;
    // Buffer trees use recovered input slew for sizing. Existing gate
    // replacements use the measured input slew on every incoming arc.
    float in_slew = std::isfinite(state.slew_limit)
                        ? state.slew_limit * options.recovered_slew_fraction
                        : state.slew;
    const CellModel *replacement = allow_resize && options.actions.resize
                                       ? selectDrvResize(analysis, cells, state)
                                       : nullptr;
    // Spatial ordering groups nearby loads; estimated wire/external-port
    // capacitance is apportioned conservatively from the current net total.
    std::stable_sort(state.loads.begin(), state.loads.end(),
                     [](const Load &a, const Load &b) {
                         return a.x != b.x ? a.x < b.x : a.y < b.y;
                     });
    float pin_sum = 0;
    for (const auto &load : state.loads)
        pin_sum += load.capacitance;
    float extra = std::max(0.0f, state.capacitance - pin_sum) / state.loads.size();
    size_t max_group = options.max_fanout;
    if (state.fanout_ratio > 1) {
        max_group = std::min(
            max_group,
            std::max(size_t(1), size_t(state.loads.size() / state.fanout_ratio)));
    }
    // Limiting an overloaded driver's group size provides relief even if a
    // large buffer could carry the entire net. Subsequent passes fix the
    // root.
    if (state.cap_ratio > 1 || state.slew_ratio > 1)
        max_group = std::min(
            max_group,
            std::max(size_t(1), size_t(std::ceil(state.loads.size() *
                                                 options.overload_group_fraction))));
    struct Group {
        size_t begin, end;
        const CellModel *buffer;
    };
    std::vector<Group> groups;
    double buffer_area = 0;
    for (size_t begin = 0; begin < state.loads.size();) {
        size_t end = begin;
        float group_cap = 0;
        const CellModel *selected = nullptr;
        while (end < state.loads.size() && end - begin < max_group) {
            float next_cap = group_cap + state.loads[end].capacitance + extra;
            const CellModel *next =
                analysis.select(next_cap, state.slew_limit, in_slew);
            if (!next)
                break;
            group_cap = next_cap;
            selected = next;
            ++end;
        }
        if (end == begin) {
            ++begin;
            continue;
        }

        // A singleton that is already a low-capacitance buffer input cannot
        // benefit from repeatedly adding another identical buffer.
        if (end - begin == 1 &&
            state.loads[begin].term->getObjectType() == odb::dbITermObj) {
            auto *load_pin = static_cast<odb::dbITerm *>(state.loads[begin].term);
            if (sta->getDbNetwork()->libertyCell(load_pin->getInst()) &&
                sta->getDbNetwork()->libertyCell(load_pin->getInst())->isBuffer() &&
                selected->input->capacitance() >= state.loads[begin].capacitance) {
                begin = end;
                continue;
            }
        }
        groups.push_back({begin, end, selected});
        buffer_area += selected->area;
        begin = end;
    }
    // Compare feasible edit area before the shared STA acceptance step.
    // Count budgeting never forbids a gate replacement that adds no cell.
    if (replacement &&
        (groups.empty() || groups.size() > available ||
         replacement->area - detail::area(state.driver->getInst()->getMaster()) <=
             buffer_area)) {
        plan.replacement =
            CellReplacement{state.driver->getInst(), replacement->master};
    }
    for (const auto &group : groups) {
        if (plan.buffers.size() >= available)
            break;
        const auto begin = group.begin, end = group.end;
        const auto *selected = group.buffer;
        std::vector<odb::dbObject *> terms;
        long long x = 0, y = 0;
        for (size_t i = begin; i < end; ++i) {
            terms.push_back(state.loads[i].term);
            x += state.loads[i].x;
            y += state.loads[i].y;
        }
        BufferInsertionOptions options;
        options.location = BufferLocation{int(x / terms.size()), int(y / terms.size())};
        options.buffer_name_prefix = "sizer_drv_buf";
        options.net_name_prefix = "sizer_drv_net";
        plan.buffers.push_back(
            {state.net, std::move(terms), selected->master, std::move(options)});
    }
    return plan;
}

static bool applyNetRepair(DesignAnalysis &analysis, detail::CellCandidates &cells,
                           sta::dbSta *sta, NetState &state, unsigned &inserted,
                           unsigned &resized_buffers, unsigned &resized_gates,
                           unsigned remaining, bool allow_resize, unsigned &vt_swaps,
                           const RepairCommonOptions &options) {
    const auto plan = planDrvNetRepair(analysis, cells, sta, state,
                                       remaining > inserted ? remaining - inserted : 0,
                                       allow_resize, options);
    const bool buffer =
        sta->getDbNetwork()->libertyCell(state.driver->getInst())->isBuffer();
    const bool vt =
        plan.replacement && analysis.isVtSwap(state.driver->getInst()->getMaster(),
                                              plan.replacement->master);
    // DRV prefers its selected replacement and falls back to the buffer plan.
    // Actions perform only the explicit mutation selected here.
    const bool replaced = plan.replacement && replaceCell(*plan.replacement);
    const unsigned buffered = replaced ? 0 : insertBuffers(plan.buffers);
    inserted += buffered;
    if (replaced) {
        vt_swaps += vt;
        if (buffer)
            ++resized_buffers;
        else
            ++resized_gates;
    }
    return replaced || buffered != 0;
}

DrvRepairResult DrvRepairer::repair(odb::dbBlock *block, sta::dbSta *sta,
                                    const std::function<void()> &refresh,
                                    const DrvRepairOptions &options) {
    DrvRepairResult result;
    try {
        options.validate();
    } catch (const std::exception &error) {
        result.message = error.what();
        return result;
    }
    if (!block || !sta || !refresh || sta->scenes().size() != 1 ||
        sta->getDbNetwork()->block() != block) {
        result.message = "invalid repair analysis (one STA scene required)";
        return result;
    }
    DesignAnalysis repair(block, sta, options);
    detail::CellCandidates cells(repair, sta, options);
    if (!repair.hasCells()) {
        result.message = "no usable Liberty cell models";
        return result;
    }
    const bool collect_all =
        options.actions.load_downsize && options.max_load_downsizes_per_pass > 0;
    const auto analyze = [&](DesignMetrics &metrics) {
        return repair.analyze(metrics, collect_all);
    };
    const auto violating = [](const DesignMetrics &metrics) {
        return metrics.fanout_nets || metrics.capacitance_nets || metrics.slew_nets;
    };
    auto states = analyze(result.before);
    result.after = result.before;
    bool allow_resize = true;
    bool allow_load_downsize = collect_all;
    for (unsigned pass = 0; pass < options.max_passes && violating(result.after);
         ++pass) {
        const bool overloaded =
            std::any_of(states.begin(), states.end(), [](const NetState &state) {
                return state.fanout_ratio > 1 || state.cap_ratio > 1;
            });
        std::vector<detail::LoadDownsizeCandidate> load_plans;
        if (allow_load_downsize) {
            detail::LoadDownsizePlanner planner(repair, sta, states, options);
            std::unordered_set<odb::dbInst *> selected;
            for (const auto &state : states) {
                if (state.fanout_ratio > 1 ||
                    (overloaded ? state.cap_ratio <= 1
                                : state.slew_ratio <= 1 || state.propagated_slew))
                    continue;
                for (const auto &candidate : planner.candidates(state)) {
                    if (load_plans.size() >= options.max_load_downsizes_per_pass)
                        break;
                    if (selected.insert(candidate.inst).second)
                        load_plans.push_back(candidate);
                }
                if (load_plans.size() >= options.max_load_downsizes_per_pass)
                    break;
            }
        }
        EcoTransaction transaction(block);
        unsigned inserted = 0, resized = 0, resized_gates = 0, downsized = 0,
                 vt_swaps = 0;
        try {
            // Try relief at slack-rich loads before adding drive strength or
            // buffers. All candidates are measured before any ECO mutation.
            if (!load_plans.empty()) {
                for (const auto &plan : load_plans)
                    if (replaceCell({plan.inst, plan.master}))
                        ++downsized;
            } else {
                for (auto &state : states) {
                    if (!state.violating())
                        continue;
                    const bool load_violation =
                        state.fanout_ratio > 1 || state.cap_ratio > 1;
                    if (overloaded ? !load_violation : state.propagated_slew)
                        continue;
                    applyNetRepair(repair, cells, sta, state, inserted, resized,
                                   resized_gates,
                                   remainingBufferBudget(options.max_inserted_buffers,
                                                         result.inserted_buffers),
                                   allow_resize, vt_swaps, options);
                }
            }
            if (inserted == 0 && resized == 0 && resized_gates == 0 && downsized == 0) {
                transaction.rollback();
                result.message = "no feasible edit for remaining violations";
                break;
            }
            if (options.prepare)
                options.prepare();
            refresh();
            DesignMetrics after;
            auto next_states = analyze(after);
            // Accept only measured DRV improvement without setup/hold WNS
            // regression beyond configured tolerance. Never claim convergence
            // just because a batch or insertion budget has been exhausted.
            if (after.severity + options.tolerances.drv_severity <
                    result.after.severity &&
                detail::timingNonRegressing(result.after, after, options.tolerances) &&
                (!(downsized || vt_swaps) || detail::noNewDrv(result.after, after))) {
                transaction.commit();
                result.inserted_buffers += inserted;
                result.resized_buffers += resized;
                result.resized_gates += resized_gates;
                result.vt_swaps += vt_swaps;
                result.downsized_loads += downsized;
                allow_load_downsize = collect_all;
                ++result.accepted_passes;
                result.after = after;
                states = std::move(next_states);
            } else {
                transaction.rollback();
                refresh();
                states = analyze(result.after);
                ++result.rejected_passes;
                if (downsized) {
                    ++result.rejected_load_batches;
                    allow_load_downsize = false;
                    continue; // Preserve existing source resize/buffer
                              // fallback.
                }
                if (allow_resize && (resized || resized_gates)) {
                    allow_resize = false;
                    continue; // Retry with buffering if upsizing hurt upstream
                              // timing.
                }
                result.message = "batch rejected: DRV severity or WNS did not improve";
                break;
            }
        } catch (const std::exception &error) {
            result.message = error.what();
            if (transaction.active()) {
                transaction.rollback();
                try {
                    refresh();
                    analyze(result.after);
                } catch (const std::exception &restore_error) {
                    result.message +=
                        std::string("; timing refresh after rollback failed: ") +
                        restore_error.what();
                }
            }
            return result;
        }
    }
    result.ok = true;
    result.converged = !violating(result.after);
    if (result.message.empty())
        result.message = result.converged ? "eligible nets repaired"
                                          : "pass limit reached; violations remain";
    return result;
}

OrdDrvRepairResult OrdDrvRepairer::repair(odb::dbBlock *block, sta::dbSta *sta,
                                          const std::function<void()> &refresh,
                                          const DrvRepairOptions &options) {
    OrdDrvRepairResult result;
    try {
        options.validate();
    } catch (const std::exception &error) {
        result.message = error.what();
        return result;
    }
    if (!block || !sta || !refresh || sta->scenes().size() != 1 ||
        sta->getDbNetwork()->block() != block) {
        result.message = "invalid OpenROAD-style repair analysis";
        return result;
    }

    detail::DesignAnalysis analysis(block, sta, options);
    detail::CellCandidates cells(analysis, sta, options);
    if (!analysis.hasCells()) {
        result.message = "no usable Liberty cell models";
        return result;
    }

    const auto analyze = [&](DesignMetrics &metrics) {
        return analysis.analyze(metrics, true);
    };
    auto states = analyze(result.before);
    result.after = result.before;
    const auto *network = sta->getDbNetwork();

    for (unsigned pass = 0; pass < options.max_passes && result.after.severity > 0;
         ++pass) {
        bool pass_progress = false;
        unsigned batch_limit = options.ord_driver_batch_size;
        std::unordered_set<odb::dbNet *> processed;
        while (true) {
            // RepairDesign walks levelized driver vertices from sinks back to
            // sources. Rebuild the map after each accepted ECO because buffer
            // insertion changes the OpenSTA graph and net ownership.
            std::unordered_map<odb::dbNet *, detail::NetState *> by_net;
            by_net.reserve(states.size());
            for (auto &state : states)
                by_net.emplace(state.net, &state);

            std::vector<odb::dbNet *> selected_nets;
            const auto &drivers = sta->levelizedDrvrVertices();
            for (int index = int(drivers.size()) - 1;
                 index >= 0 && selected_nets.size() < batch_limit; --index) {
                auto *vertex = drivers[index];
                if (!vertex || !vertex->pin())
                    continue;
                auto *object = network->staToDb(vertex->pin());
                if (!object || object->getObjectType() != odb::dbITermObj)
                    continue;
                auto *driver = static_cast<odb::dbITerm *>(object);
                auto *net = driver->getNet();
                const auto found = by_net.find(net);
                if (found == by_net.end() || !found->second->violating() ||
                    processed.count(net))
                    continue;
                processed.insert(net);
                selected_nets.push_back(net);
            }
            if (selected_nets.empty())
                break;

            EcoTransaction transaction(block);
            unsigned inserted = 0, resized_buffers = 0, resized_gates = 0, vt_swaps = 0,
                     repaired = 0;
            try {
                for (auto *net : selected_nets) {
                    const auto found = by_net.find(net);
                    if (found == by_net.end())
                        continue;
                    if (applyNetRepair(
                            analysis, cells, sta, *found->second, inserted,
                            resized_buffers, resized_gates,
                            remainingBufferBudget(options.max_inserted_buffers,
                                                  result.inserted_buffers),
                            true, vt_swaps, options))
                        ++repaired;
                }
                if (repaired == 0) {
                    transaction.rollback();
                    for (auto *net : selected_nets)
                        processed.erase(net);
                    break;
                }
                if (options.prepare)
                    options.prepare();
                refresh();
                DesignMetrics after;
                auto next_states = analyze(after);
                // OpenROAD repair_design commits a DRV-improving driver edit;
                // setup/hold protection belongs to repair_setup.  Keeping
                // this acceptance rule separate is essential for a faithful
                // A/B reference and is the main difference from Native's
                // timing-preserving transaction policy.
                if (after.severity + options.tolerances.drv_severity <
                    result.after.severity) {
                    transaction.commit();
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
                transaction.rollback();
                refresh();
                states = analyze(result.after);
                for (auto *net : selected_nets)
                    processed.erase(net);
                ++result.rejected_passes;
                if (batch_limit > 1) {
                    batch_limit = std::max(1u, batch_limit / 2);
                    continue;
                }
                result.message =
                    "OpenROAD-style batch rejected: DRV or timing regressed";
                break;
            } catch (const std::exception &error) {
                result.message = error.what();
                if (transaction.active()) {
                    transaction.rollback();
                    for (auto *net : selected_nets)
                        processed.erase(net);
                    try {
                        refresh();
                        states = analyze(result.after);
                    } catch (const std::exception &restore_error) {
                        result.message +=
                            std::string("; refresh after rollback failed: ") +
                            restore_error.what();
                    }
                }
                return result;
            }
            break;
        }
        if (pass_progress)
            ++result.passes;
        else if (result.message.empty())
            result.message = "no feasible OpenROAD-style edit";
        if (!pass_progress)
            break;
    }

    result.ok = true;
    result.converged = result.after.severity <= options.tolerances.drv_severity;
    if (result.message.empty())
        result.message = result.converged
                             ? "OpenROAD-style DRV repair converged"
                             : "OpenROAD-style pass limit reached; violations remain";
    return result;
}

static FanoutRepairResult failure(FanoutRepairStatus status, std::string message) {
    FanoutRepairResult result;
    result.status = status;
    result.message = std::move(message);
    return result;
}

static bool isLoad(odb::dbObject *term) {
    if (term == nullptr) {
        return false;
    }
    if (term->getObjectType() == odb::dbITermObj) {
        const odb::dbIoType io_type = static_cast<odb::dbITerm *>(term)->getIoType();
        return io_type == odb::dbIoType::INPUT || io_type == odb::dbIoType::INOUT;
    }
    if (term->getObjectType() == odb::dbBTermObj) {
        const odb::dbIoType io_type = static_cast<odb::dbBTerm *>(term)->getIoType();
        return io_type == odb::dbIoType::OUTPUT || io_type == odb::dbIoType::INOUT;
    }
    return false;
}

static std::vector<odb::dbObject *> loads(odb::dbNet *net) {
    std::vector<odb::dbObject *> result;
    if (net == nullptr) {
        return result;
    }
    result.reserve(net->getITerms().size() + net->getBTerms().size());
    for (odb::dbITerm *term : net->getITerms()) {
        if (isLoad(term)) {
            result.push_back(term);
        }
    }
    for (odb::dbBTerm *term : net->getBTerms()) {
        if (isLoad(term)) {
            result.push_back(term);
        }
    }
    return result;
}

static bool
shouldRepair(odb::dbNet *net, bool repair_clock_nets,
             const std::unordered_set<odb::dbNet *> *excluded_nets = nullptr) {
    if (net == nullptr || net->isDoNotTouch() || net->isSpecial() ||
        (excluded_nets != nullptr && excluded_nets->count(net) > 0)) {
        return false;
    }
    const odb::dbSigType signal_type = net->getSigType();
    if (signal_type == odb::dbSigType::POWER || signal_type == odb::dbSigType::GROUND) {
        return false;
    }
    return repair_clock_nets || signal_type != odb::dbSigType::CLOCK;
}

static unsigned
maxFanoutWithPolicy(odb::dbBlock *block, bool include_clock_nets,
                    const std::unordered_set<odb::dbNet *> *excluded_nets) {
    unsigned max_fanout = 0;
    if (block == nullptr) {
        return max_fanout;
    }
    for (odb::dbNet *net : block->getNets()) {
        if (shouldRepair(net, include_clock_nets, excluded_nets)) {
            max_fanout = std::max(max_fanout, OpenDbFanoutRepairer::loadCount(net));
        }
    }
    return max_fanout;
}

static bool isSimpleBuffer(odb::dbMaster *master) {
    if (master == nullptr || !master->isFrozen()) {
        return false;
    }
    unsigned input_count = 0;
    unsigned output_count = 0;
    for (odb::dbMTerm *term : master->getMTerms()) {
        if (term->getIoType() == odb::dbIoType::INPUT) {
            ++input_count;
        } else if (term->getIoType() == odb::dbIoType::OUTPUT) {
            ++output_count;
        }
    }
    return input_count == 1 && output_count == 1;
}

static void addAffectedNet(FanoutRepairResult &result,
                           std::unordered_set<odb::dbNet *> &seen, odb::dbNet *net) {
    if (net != nullptr && seen.insert(net).second) {
        result.affected_nets.push_back(net);
    }
}

unsigned OpenDbFanoutRepairer::loadCount(odb::dbNet *net) { return loads(net).size(); }

unsigned OpenDbFanoutRepairer::maxFanout(odb::dbBlock *block, bool include_clock_nets) {
    return maxFanoutWithPolicy(block, include_clock_nets, nullptr);
}

FanoutRepairResult OpenDbFanoutRepairer::repair(odb::dbBlock *block,
                                                odb::dbMaster *buffer_master,
                                                const FanoutRepairOptions &options) {
    if (block == nullptr) {
        return failure(FanoutRepairStatus::kInvalidBlock,
                       "fanout repair requires a block");
    }
    if (!isSimpleBuffer(buffer_master)) {
        return failure(FanoutRepairStatus::kInvalidBufferMaster,
                       "fanout repair requires a frozen buffer master with "
                       "one input and one output");
    }
    if (options.max_fanout < 2) {
        return failure(FanoutRepairStatus::kInvalidMaxFanout,
                       "max fanout must be at least two");
    }

    FanoutRepairResult result;
    result.status = FanoutRepairStatus::kSuccess;
    const std::unordered_set<odb::dbNet *> excluded_nets(options.excluded_nets.begin(),
                                                         options.excluded_nets.end());
    result.before_max_fanout =
        maxFanoutWithPolicy(block, options.repair_clock_nets, &excluded_nets);
    std::unordered_set<odb::dbNet *> seen_affected_nets;

    // Keep the tree construction atomic. OpenDB supports nested ECO journals,
    // so this also composes with a caller-owned transaction.
    EcoTransaction transaction(block);
    const auto rollback = [&](FanoutRepairStatus status, const std::string &message) {
        transaction.rollback();
        result.status = status;
        result.message = message;
        result.inserted_buffers.clear();
        result.affected_nets.clear();
        result.after_max_fanout =
            maxFanoutWithPolicy(block, options.repair_clock_nets, &excluded_nets);
        return result;
    };

    std::vector<odb::dbNet *> candidate_nets;
    candidate_nets.reserve(block->getNets().size());
    for (odb::dbNet *net : block->getNets()) {
        if (shouldRepair(net, options.repair_clock_nets, &excluded_nets)) {
            candidate_nets.push_back(net);
        }
    }

    for (odb::dbNet *net : candidate_nets) {
        ++result.nets_examined;
        std::vector<odb::dbObject *> net_loads = loads(net);
        if (net_loads.size() <= options.max_fanout) {
            continue;
        }
        ++result.violating_nets;

        struct TreeNode {
            odb::dbObject *term;
            unsigned depth;
        };
        std::deque<TreeNode> root_loads;
        for (odb::dbObject *load : net_loads) {
            root_loads.push_back({load, 0});
        }

        // Each insertion reduces the root fanout by at most K - 1. Always
        // take that full reduction except for the last insertion, which
        // reaches K exactly. Taking the shallowest nodes first also keeps the
        // resulting minimum-size tree balanced.
        while (root_loads.size() > options.max_fanout) {
            const size_t excess = root_loads.size() - options.max_fanout;
            const size_t group_size =
                std::min(static_cast<size_t>(options.max_fanout), excess + 1);
            std::vector<odb::dbObject *> group;
            group.reserve(group_size);
            unsigned group_depth = 0;
            for (size_t index = 0; index < group_size; ++index) {
                group.push_back(root_loads.front().term);
                group_depth = std::max(group_depth, root_loads.front().depth);
                root_loads.pop_front();
            }
            const unsigned buffer_depth = group_depth + 1;
            if (buffer_depth > options.max_levels) {
                return rollback(FanoutRepairStatus::kMaxLevelsExceeded,
                                "fanout repair exceeded the maximum tree depth");
            }

            BufferInsertionResult insertion =
                OpenDbBufferInserter::insertBufferBeforeLoads(net, group, buffer_master,
                                                              options.insertion);
            if (!insertion) {
                return rollback(FanoutRepairStatus::kInsertionFailed,
                                insertion.message);
            }
            result.inserted_buffers.push_back(insertion.buffer);
            for (odb::dbNet *affected : insertion.affected_nets) {
                addAffectedNet(result, seen_affected_nets, affected);
            }
            root_loads.push_back({insertion.input_pin, buffer_depth});
        }
    }

    result.after_max_fanout =
        maxFanoutWithPolicy(block, options.repair_clock_nets, &excluded_nets);
    if (result.after_max_fanout > options.max_fanout) {
        return rollback(FanoutRepairStatus::kInsertionFailed,
                        "fanout repair finished with an unresolved violation");
    }
    transaction.commit();
    return result;
}

struct PowerCandidate {
    odb::dbInst *inst;
    odb::dbMaster *master;
    double saving;
    float slack;
    bool vt;
    bool needs_recovery;
};
// Characterized leakage is only a ranking estimate. Acceptance uses
// whole-design OpenSTA power including conditional leakage/activity effects.
static float rankLeakage(sta::LibertyCell *cell) {
    // Conditional tables (e.g. ASAP7) often have no cell_leakage_power.
    // Use the sum of per-supply maxima as a cheap, activity-independent rank.
    // This is not reported as measured leakage or used for acceptance.
    if (!cell->leakagePowers().empty()) {
        std::unordered_map<const sta::LibertyPort *, float> peaks;
        for (const auto &entry : cell->leakagePowers()) {
            if (!std::isfinite(entry.power()) || entry.power() < 0)
                return -1;
            auto &peak = peaks[entry.relatedPgPort()];
            peak = std::max(peak, entry.power());
        }
        float sum = 0;
        for (const auto &entry : peaks)
            sum += entry.second;
        return sum;
    }
    float value = 0;
    bool exists = false;
    cell->leakagePower(value, exists);
    return exists && std::isfinite(value) && value >= 0 ? value : -1;
}
static std::vector<PowerCandidate>
powerCandidates(detail::DesignAnalysis &analysis, sta::dbSta *sta,
                const std::vector<detail::NetState> &states,
                const PowerRepairOptions &options) {
    std::vector<PowerCandidate> plans;
    if (!options.common.actions.resize)
        return plans;
    detail::CellCandidates cells(analysis, sta, options.common);
    for (const auto &state : states) {
        auto *inst = state.driver->getInst();
        if (analysis.protectedInputs(inst))
            continue;
        const auto original = analysis.driverModel(state);
        if (!original.cell)
            continue;
        const float old_leakage = rankLeakage(original.cell);
        if (old_leakage <= 0)
            continue;
        const float slack = float(sta->slack(sta->getDbNetwork()->dbToSta(state.driver),
                                             sta::RiseFallBoth::riseFall(),
                                             sta->scenes(), sta::MinMax::max()));
        // Unconstrained outputs have no verified timing budget.
        if (!std::isfinite(slack) || slack >= sta::INF || slack < options.slack_margin)
            continue;
        const float old_delay =
            analysis.predictedDelay(original, state.capacitance, state.input_slew);
        auto replacements = cells.resize(state, true);
        const auto vt = cells.resize(state, false, true);
        replacements.insert(replacements.end(), vt.begin(), vt.end());
        for (const auto *replacement : replacements) {
            const float new_leakage = rankLeakage(replacement->cell);
            if (new_leakage < 0 || new_leakage >= old_leakage)
                continue;
            const float delay = analysis.predictedDelay(*replacement, state.capacitance,
                                                        state.input_slew);
            if (!std::isfinite(old_delay) || !std::isfinite(delay))
                continue;
            plans.push_back(
                {inst, replacement->master, old_leakage - new_leakage, slack,
                 analysis.isVtSwap(inst->getMaster(), replacement->master),
                 slack < options.slack_margin + std::max(0.0f, delay - old_delay)});
        }
    }
    std::sort(plans.begin(), plans.end(), [](const auto &a, const auto &b) {
        // Try slack-budgeted moves first, then bounded recovery trials.
        if (a.needs_recovery != b.needs_recovery)
            return !a.needs_recovery;
        if (a.saving != b.saving)
            return a.saving > b.saving;
        if (a.slack != b.slack)
            return a.slack > b.slack;
        if (a.inst->getName() != b.inst->getName())
            return a.inst->getName() < b.inst->getName();
        return a.master->getName() < b.master->getName();
    });
    if (plans.size() > options.max_candidates)
        plans.resize(options.max_candidates);
    return plans;
}
static std::vector<detail::NetState> measurePower(detail::DesignAnalysis &analysis,
                                                  sta::dbSta *sta,
                                                  PowerRepairMetrics &metrics) {
    auto states = analysis.analyze(metrics.design, true);
    sta::PowerResult total, seq, comb, clock, macro, pad;
    // This vendored STA's powerInvalid() clears the instance map without
    // invalidating its validity flag. activitiesInvalid() invalidates both
    // activity and instance power caches and preserves user annotations.
    sta->power()->activitiesInvalid();
    sta->power(*sta->scenes().begin(), total, seq, comb, clock, macro, pad);
    metrics.leakage = total.leakage();
    metrics.internal = total.internal();
    metrics.switching = total.switching();
    metrics.total = total.total();
    for (const double value :
         {metrics.leakage, metrics.internal, metrics.switching, metrics.total})
        if (!std::isfinite(value) || value < 0)
            throw std::runtime_error("invalid STA power result");
    return states;
}
static bool powerImproves(const PowerRepairMetrics &before,
                          const PowerRepairMetrics &after,
                          const PowerRepairOptions &options) {
    const double old_power =
        options.objective == ecc::PowerObjective::Total ? before.total : before.leakage;
    const double new_power =
        options.objective == ecc::PowerObjective::Total ? after.total : after.leakage;
    const double minimum = std::max(options.min_absolute_improvement,
                                    old_power * options.min_relative_improvement);
    return new_power < old_power - minimum;
}
static bool acceptable(const PowerRepairMetrics &before,
                       const PowerRepairMetrics &after,
                       const PowerRepairOptions &options) {
    return powerImproves(before, after, options) &&
           after.design.area <= before.design.area &&
           detail::timingNonRegressing(before.design, after.design,
                                       options.common.tolerances) &&
           detail::noNewDrv(before.design, after.design) &&
           after.design.severity <=
               before.design.severity + options.common.tolerances.drv_severity;
}
PowerRepairResult OpenDbPowerRepairer::repair(odb::dbBlock *block, sta::dbSta *sta,
                                              const std::function<void()> &refresh,
                                              const PowerRepairOptions &options) {
    PowerRepairResult result;
    const auto start = std::chrono::steady_clock::now();
    std::optional<EcoTransaction> transaction;
    try {
        options.validate();
        if (!block || !sta || !refresh || sta->getDbNetwork()->block() != block ||
            sta->scenes().size() != 1)
            throw std::invalid_argument(
                "invalid power analysis (one STA scene required)");
        detail::DesignAnalysis analysis(block, sta, options.common);
        auto states = measurePower(analysis, sta, result.before);
        result.after = result.before;
        const auto update = [&]() {
            ++result.refreshes;
            refresh();
        };
        for (unsigned pass = 0;
             pass < options.max_passes && result.attempted_trials < options.max_trials;
             ++pass) {
            const auto plans = powerCandidates(analysis, sta, states, options);
            size_t cursor = 0,
                   width = std::min(size_t(options.batch_size), plans.size());
            std::set<std::vector<size_t>> rejected;
            bool accepted = false;
            const auto backoff = [&]() {
                if (width > 1)
                    width = std::min(
                        width - 1,
                        std::max(size_t(1), size_t(width * options.batch_backoff)));
                else {
                    ++cursor;
                    width = std::min(size_t(options.batch_size), plans.size() - cursor);
                }
            };
            while (cursor < plans.size() &&
                   result.attempted_trials < options.max_trials) {
                std::vector<size_t> selected;
                std::unordered_set<odb::dbInst *> touched;
                for (size_t i = cursor; i < std::min(cursor + width, plans.size()); ++i)
                    if (touched.insert(plans[i].inst).second)
                        selected.push_back(i);
                if (rejected.count(selected)) {
                    backoff();
                    continue;
                }
                transaction.emplace(block);
                unsigned vt_swaps = 0, downsizes = 0;
                for (const auto i : selected) {
                    const auto &plan = plans[i];
                    if (!replaceCell({plan.inst, plan.master}))
                        throw std::runtime_error("power cell swap failed");
                    if (plan.vt)
                        ++vt_swaps;
                    else
                        ++downsizes;
                }
                ++result.attempted_trials;
                if (options.common.prepare)
                    options.common.prepare();
                update();
                PowerRepairMetrics after;
                auto next = measurePower(analysis, sta, after);
                const auto localSlackOk = [&]() {
                    bool local_slack_ok = true;
                    for (const auto i : selected) {
                        for (auto *pin : plans[i].inst->getITerms()) {
                            if (pin->getSigType() != odb::dbSigType::SIGNAL ||
                                pin->getIoType() != odb::dbIoType::OUTPUT)
                                continue;
                            const float slack =
                                float(sta->slack(sta->getDbNetwork()->dbToSta(pin),
                                                 sta::RiseFallBoth::riseFall(),
                                                 sta->scenes(), sta::MinMax::max()));
                            local_slack_ok &= std::isfinite(slack) &&
                                              slack < sta::INF &&
                                              slack >= options.slack_margin;
                        }
                    }
                    return local_slack_ok;
                };
                // Migrate the old power -> timing recovery strategy onto the
                // same OpenDB journal and STA models as native repair. A child
                // commit remains provisional until whole-design acceptance.
                if (powerImproves(result.after, after, options) &&
                    (!localSlackOk() || !acceptable(result.after, after, options)) &&
                    result.attempted_trials < options.max_trials) {
                    transaction->suspend();
                    ++result.recovery_attempts;
                    auto recovery_common = options.common;
                    // Retain the proposed saving during recovery. Otherwise
                    // setup can simply undo it and waste the recovery budget.
                    for (const auto i : selected)
                        recovery_common.excluded_resize_instances.push_back(
                            plans[i].inst);
                    if (!detail::noNewDrv(result.after.design, after.design) &&
                        result.attempted_trials < options.max_trials) {
                        DrvRepairOptions drv;
                        static_cast<RepairCommonOptions &>(drv) = recovery_common;
                        drv.max_passes = options.max_trials - result.attempted_trials;
                        drv.max_inserted_buffers =
                            static_cast<unsigned>(selected.size());
                        const auto repaired =
                            DrvRepairer::repair(block, sta, update, drv);
                        result.attempted_trials +=
                            repaired.accepted_passes + repaired.rejected_passes;
                        if (!repaired.ok)
                            throw std::runtime_error(repaired.message);
                        next = measurePower(analysis, sta, after);
                    }
                    if (result.attempted_trials < options.max_trials) {
                        SetupRepairOptions setup;
                        setup.common = recovery_common;
                        setup.max_trials = options.max_trials - result.attempted_trials;
                        setup.max_passes = setup.max_trials;
                        setup.max_candidates = options.max_candidates;
                        setup.batch_size = options.batch_size;
                        setup.batch_backoff = options.batch_backoff;
                        // Recovery can spend area released by the power move.
                        // Final acceptance still forbids net area growth.
                        if (after.design.area > 0)
                            setup.max_area_growth = std::max(
                                0.0,
                                result.after.design.area / after.design.area - 1.0);
                        const auto repaired =
                            SetupRepairer::repair(block, sta, update, setup);
                        spdlog::debug(
                            "Power setup recovery: {} ({} trials, {} accepted)",
                            repaired.message, repaired.attempted_batches,
                            repaired.accepted_batches);
                        result.attempted_trials += repaired.attempted_batches;
                        if (!repaired.ok)
                            throw std::runtime_error(repaired.message);
                    }
                    next = measurePower(analysis, sta, after);
                }
                spdlog::debug(
                    "Power trial: local_slack={} leakage {} -> {}, setup {} -> "
                    "{}, hold {} -> {}, DRV {} -> {}",
                    localSlackOk(), result.after.leakage, after.leakage,
                    result.after.design.setup_wns, after.design.setup_wns,
                    result.after.design.hold_wns, after.design.hold_wns,
                    result.after.design.severity, after.design.severity);
                if (localSlackOk() && acceptable(result.after, after, options)) {
                    transaction->commit();
                    result.after = after;
                    states = std::move(next);
                    ++result.accepted_batches;
                    result.vt_swaps += vt_swaps;
                    result.downsized_cells += downsizes;
                    accepted = true;
                    break;
                }
                transaction->rollback();
                update(); // Always restore current timing/power before
                          // returning or replanning.
                states = measurePower(analysis, sta, result.after);
                ++result.rejected_batches;
                rejected.insert(selected);
                backoff();
            }
            if (!accepted)
                break;
        }
        result.ok = true;
        result.message = result.attempted_trials >= options.max_trials
                             ? "power trial budget exhausted"
                         : result.accepted_batches >= options.max_passes
                             ? "power pass budget exhausted"
                             : "no further acceptable power candidate";
    } catch (const std::exception &error) {
        result.message = error.what();
        if (transaction && transaction->active()) {
            transaction->rollback();
            try {
                ++result.refreshes;
                refresh();
                sta->power()->activitiesInvalid();
            } catch (const std::exception &restore) {
                result.message +=
                    std::string("; rollback refresh failed: ") + restore.what();
            }
        }
    }
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - start)
                            .count();
    return result;
}
} // namespace ecc
