// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <memory>

namespace ecc::path_detail {

// Never accept using only the incremental precheck. Also revisit electrical
// rejections: small STA roundoff near a DRV boundary can change their verdict.
inline bool needsFullVerification(const std::vector<std::string>& reasons) {
    return std::all_of(reasons.begin(), reasons.end(), [](const auto& reason) {
        return reason == "slew" || reason == "cap" || reason == "fanout" ||
               reason == "drv_severity";
    });
}

// The full serialization is the key. unordered_map may hash it, but equality
// always checks the complete action rather than trusting a digest.
struct ActionNode {
    std::string master;
    int x = 0, y = 0;
    std::vector<std::string> terminals;
    std::vector<size_t> children;
};
inline std::string actionFingerprint(const std::string& net,
                                     const std::vector<ActionNode>& nodes,
                                     bool physical = true) {
    std::vector<std::string> encoded;
    for (const auto& node : nodes) {
        auto terminals = node.terminals;
        std::sort(terminals.begin(), terminals.end());
        std::vector<std::string> children;
        children.reserve(node.children.size());
        for (auto child : node.children) {
            children.push_back(encoded.at(child));
        }
        std::sort(children.begin(), children.end());
        std::ostringstream out;
        out << '(';
        if (physical) {
            out << node.master.size() << ':' << node.master << '@' << node.x << ','
                << node.y;
        }
        out << '[';
        for (const auto& term : terminals) {
            out << term.size() << ':' << term;
        }
        out << ']';
        for (const auto& child : children) {
            out << child.size() << ':' << child;
        }
        out << ')';
        encoded.push_back(out.str());
    }
    return std::to_string(net.size()) + ':' + net +
           (encoded.empty() ? "()" : encoded.back());
}

// Prefix sums make a full endpoint forecast logarithmic per load/variant.
// Include nonviolating endpoints too: a negative gain can make them violate.
// This models the current worst path, not future competing paths; rank only.
class EndpointSlackProfile {
  public:
    void add(double slack) {
        if (std::isfinite(slack)) {
            slacks_.push_back(slack);
        }
    }
    void finish() {
        std::sort(slacks_.begin(), slacks_.end());
        prefix_.assign(slacks_.size() + 1, 0);
        for (size_t i = 0; i < slacks_.size(); ++i) {
            prefix_[i + 1] = prefix_[i] + slacks_[i];
        }
        before_ = negativeSum(0);
    }
    double gain(double delay_gain) const { return negativeSum(delay_gain) - before_; }
    size_t size() const { return slacks_.size(); }

  private:
    double negativeSum(double gain) const {
        const auto count =
            std::lower_bound(slacks_.begin(), slacks_.end(), -gain) - slacks_.begin();
        return prefix_.at(count) + count * gain;
    }
    std::vector<double> slacks_, prefix_{0};
    double before_ = 0;
};

// Soft scheduling memory, not a certificate that global timing is unchanged.
// Unrelated commits do not erase exploration of the same load group. Changes
// to its structural context reopen it. No candidate is pruned using this count.
class ContextCoverage {
  public:
    unsigned attempts(const std::string& family, const std::string& context) const {
        const auto it = counts_.find(family);
        return it != counts_.end() && it->second.context == context
                   ? it->second.attempts
                   : 0;
    }
    void observe(const std::string& family, const std::string& context) {
        auto& count = counts_[family];
        if (count.context != context) {
            count = {context, 0};
        }
        ++count.attempts;
    }

  private:
    struct Count {
        std::string context;
        unsigned attempts = 0;
    };
    std::unordered_map<std::string, Count> counts_;
};

struct PathForecast {
    size_t endpoint = 0;
    double before = 0, after = 0;
    bool affected = false;
};
struct ObjectiveForecast {
    double wns_gain = 0, tns_gain = 0;
    unsigned affected_endpoints = 0;
    // Unclamped gain of the worst affected sampled path. This distinguishes
    // strengths after an unaffected path caps the global objective. Ranking
    // only: it must never replace the global acceptance/pruning objective.
    double path_gain = 0;
};

struct CandidateQuality {
    double electrical_excess = 0;
    double path_gain = 0;
};

inline bool betterElectricalAlternative(const CandidateQuality& a,
                                        const CandidateQuality& b) {
    return a.electrical_excess < b.electrical_excess ||
           (a.electrical_excess == b.electrical_excess && a.path_gain > b.path_gain);
}

// These are estimates for scheduling, not upper bounds. In particular, a
// sampled endpoint list cannot prove that no other endpoint has a TNS gain.
inline ObjectiveForecast
forecastObjective(const std::vector<PathForecast>& paths,
                  const std::map<size_t, double>& endpoint_slacks, double global_wns,
                  bool local = false) {
    struct Endpoint {
        double before = 0, after = 0;
        bool initialized = false, affected = false;
    };
    std::map<size_t, Endpoint> endpoints;
    for (const auto& path : paths) {
        auto& end = endpoints[path.endpoint];
        if (!end.initialized) {
            end.before = path.before;
            end.after = path.after;
            end.initialized = true;
        } else {
            end.before = std::min(end.before, path.before);
            end.after = std::min(end.after, path.after);
        }
        end.affected |= path.affected;
    }
    ObjectiveForecast result;
    double affected_before = INFINITY, affected_after = INFINITY;
    for (const auto& path : paths) {
        if (path.affected) {
            affected_before = std::min(affected_before, path.before);
            affected_after = std::min(affected_after, path.after);
        }
    }
    if (std::isfinite(affected_before) && std::isfinite(affected_after)) {
        result.path_gain = affected_after - affected_before;
    }
    double before_wns = 0, after_wns = 0, sampled_wns = 0;
    for (auto& [id, end] : endpoints) {
        const auto found = endpoint_slacks.find(id);
        if (found != endpoint_slacks.end() && found->second < end.before) {
            // Another check/path at this endpoint is already more critical
            // than our samples: preserve that competing slack.
            end.before = found->second;
            end.after = std::min(end.after, found->second);
        }
        sampled_wns = std::min(sampled_wns, end.before);
        if (local && !end.affected) {
            continue;
        }
        before_wns = std::min(before_wns, end.before);
        after_wns = std::min(after_wns, end.after);
        if (end.affected) {
            ++result.affected_endpoints;
            result.tns_gain += std::min(0.0, end.after) - std::min(0.0, end.before);
        }
    }
    if (!local) {
        before_wns = std::min(0.0, global_wns);
        if (global_wns < sampled_wns) {
            after_wns = std::min(after_wns, global_wns);
        }
    }
    result.wns_gain = after_wns - before_wns;
    return result;
}

// A data-derived fit, without a fixed penalty, retry count or time budget.
// Use it only to rank siblings; it is not a proof for pruning them.
struct GainCalibration {
    double xx = 0, xy = 0, measured_sum = 0;
    unsigned samples = 0;
    void observe(double predicted, double measured) {
        if (!std::isfinite(predicted) || !std::isfinite(measured)) {
            return;
        }
        xx += predicted * predicted;
        xy += predicted * measured;
        measured_sum += measured;
        ++samples;
    }
    double predict(double raw) const {
        return xx > 0 ? raw * xy / xx : samples ? measured_sum / samples : raw;
    }
};

// Empirical frontier for one load/topology family on one committed design.
// A forecast is NOT a mathematical upper bound. Suspend only points dominated
// by a measured rejection, with both gains below acceptance even after adding
// the largest observed underprediction. An independent dominated probe must
// confirm that envelope before it can suspend siblings. New state, a better
// forecast, or an underestimated probe reopens exploration. No retry/time cap.
class ExplorationFrontier {
  public:
    struct Gains {
        double wns = 0, tns = 0;
    };
    enum class Decision { Explore, Audit, Suspend };
    Decision decide(Gains predicted, Gains floor) const {
        if (!finite(predicted) || !finite(floor)) {
            return Decision::Explore;
        }
        const bool dominated =
            std::any_of(rejected_.begin(), rejected_.end(), [&](Gains p) {
                return p.wns >= predicted.wns && p.tns >= predicted.tns;
            });
        if (!dominated || predicted.wns + error_.wns > floor.wns ||
            predicted.tns + error_.tns > floor.tns) {
            return Decision::Explore;
        }
        return audited_ ? Decision::Suspend : Decision::Audit;
    }
    void observe(Gains predicted, Gains actual, bool rejected, bool audit) {
        if (!finite(predicted) || !finite(actual)) {
            audited_ = false;
            return;
        }
        const Gains next{std::max(error_.wns, actual.wns - predicted.wns),
                         std::max(error_.tns, actual.tns - predicted.tns)};
        const bool expanded = next.wns > error_.wns || next.tns > error_.tns;
        error_ = next;
        if (expanded || !rejected) {
            audited_ = false;
        } else if (audit) {
            audited_ = true;
        }
        if (rejected) {
            // Keep the Pareto frontier rather than every near-identical probe.
            if (std::any_of(rejected_.begin(), rejected_.end(), [&](Gains p) {
                    return p.wns >= predicted.wns && p.tns >= predicted.tns;
                })) {
                return;
            }
            rejected_.erase(std::remove_if(rejected_.begin(), rejected_.end(),
                                           [&](Gains p) {
                                               return predicted.wns >= p.wns &&
                                                      predicted.tns >= p.tns;
                                           }),
                            rejected_.end());
            rejected_.push_back(predicted);
        }
    }

  private:
    static bool finite(Gains p) { return std::isfinite(p.wns) && std::isfinite(p.tns); }
    Gains error_;
    std::vector<Gains> rejected_;
    bool audited_ = false;
};

struct FailureEvidence {
    std::string context;
    std::vector<std::string> reasons;
    unsigned generation = 0;
    double wns_gain = 0, tns_gain = 0;
    double predicted_wns_gain = 0, predicted_tns_gain = 0;
};

// Coverage is a scheduling preference, never a cap on attempts. A new
// committed design starts a fresh wave; rejections advance only the tried
// family, so its near-identical variants cannot occupy the entire queue.
class FamilyCoverage {
  public:
    unsigned attempts(const std::string& family, unsigned generation) const {
        const auto it = counts_.find(family);
        return it != counts_.end() && it->second.generation == generation
                   ? it->second.attempts
                   : 0;
    }
    void observe(const std::string& family, unsigned generation) {
        auto& count = counts_[family];
        if (count.generation != generation) {
            count = {generation, 0};
        }
        ++count.attempts;
    }

  private:
    struct Count {
        unsigned generation = 0, attempts = 0;
    };
    std::unordered_map<std::string, Count> counts_;
};

enum class Retry { Fresh, Invalidated, Deferred, Duplicate };
class FailureHistory {
  public:
    Retry status(const std::string& action, const std::string& context,
                 unsigned generation) const {
        const auto found = records_.find(action);
        if (found == records_.end()) {
            return Retry::Fresh;
        }
        if (found->second.context != context) {
            return Retry::Invalidated;
        }
        if (found->second.generation == generation) {
            return Retry::Duplicate;
        }
        // Matching sampled boundaries do not prove that global STA or an
        // arbitrary placement callback is unchanged. Defer across commits,
        // but require a fresh STA trial before declaring the search exhausted.
        return Retry::Deferred;
    }
    void record(std::string action, FailureEvidence evidence) {
        records_[std::move(action)] = std::move(evidence);
    }
    const FailureEvidence* find(const std::string& action) const {
        const auto found = records_.find(action);
        return found == records_.end() ? nullptr : &found->second;
    }
    bool opportunityChanged(const std::string& action, double wns, double tns,
                            double wns_floor, double tns_floor) const {
        const auto* evidence = find(action);
        return evidence && (wns > evidence->predicted_wns_gain + wns_floor ||
                            tns > evidence->predicted_tns_gain + tns_floor);
    }

  private:
    std::unordered_map<std::string, FailureEvidence> records_;
};

// A rejected TNS delta can be reused only while its entire delay cone is
// untouched. WNS/no-gain failures are deliberately excluded: an unrelated
// commit can expose a different global bottleneck and make them worthwhile.
class DependencyFailures {
  public:
    using Cone = std::unordered_set<unsigned>;
    void record(const std::string& action, Cone cone, float before, float after) {
        record(action, std::make_shared<const Cone>(std::move(cone)), before, after);
    }
    void record(const std::string& action, std::shared_ptr<const Cone> cone,
                float before, float after) {
        if (std::isfinite(before) && std::isfinite(after) && after < before) {
            failures_[action] = {std::move(cone), before, after};
        }
    }
    bool rejects(const std::string& action, float current, double tolerance) const {
        const auto it = failures_.find(action);
        if (it == failures_.end() || !std::isfinite(current)) {
            return false;
        }
        const auto& f = it->second;
        const double loss = double(f.before) - f.after;
        // Whole-design TNS is stored as a float. Allow outward rounding of
        // both recorded totals and both replayed totals, not a tuned margin.
        const auto ulp = [](float value) {
            return double(std::nextafter(std::abs(value),
                                         std::numeric_limits<float>::infinity())) -
                   std::abs(value);
        };
        const double uncertainty =
            ulp(f.before) + ulp(f.after) + ulp(current) + ulp(float(current - loss));
        return loss > tolerance + uncertainty;
    }
    void committed(const Cone& changed) {
        std::unordered_map<const Cone*, bool> invalidated;
        for (auto it = failures_.begin(); it != failures_.end();) {
            const auto& cone = *it->second.cone;
            auto [memo, fresh] = invalidated.emplace(&cone, false);
            if (fresh) {
                const auto& smaller = cone.size() < changed.size() ? cone : changed;
                const auto& larger = cone.size() < changed.size() ? changed : cone;
                memo->second =
                    std::any_of(smaller.begin(), smaller.end(),
                                [&](unsigned id) { return larger.contains(id); });
            }
            if (memo->second) {
                it = failures_.erase(it);
            } else {
                ++it;
            }
        }
    }

  private:
    struct Evidence {
        std::shared_ptr<const Cone> cone;
        float before, after;
    };
    std::unordered_map<std::string, Evidence> failures_;
};

// A single unchanged endpoint with slack below the current global WNS proves
// rejection, even if unrelated endpoints or global TNS changed. Dependencies
// include its complete fanin, capture-clock/check arcs and launch clocks.
class EndpointFailures {
  public:
    using Cone = DependencyFailures::Cone;
    void record(const std::string& action, std::shared_ptr<const Cone> fanin,
                float slack) {
        if (std::isfinite(slack)) {
            failures_[action] = {std::move(fanin), slack};
        }
    }
    bool rejects(const std::string& action, float current_wns, double tolerance) const {
        const auto it = failures_.find(action);
        if (it == failures_.end() || !std::isfinite(current_wns)) {
            return false;
        }
        const auto slack = it->second.slack;
        const auto ulp = [](float v) {
            return double(std::nextafter(std::abs(v),
                                         std::numeric_limits<float>::infinity())) -
                   std::abs(v);
        };
        return double(current_wns) - slack > tolerance + ulp(current_wns) + ulp(slack);
    }
    void committed(unsigned edited_net) {
        for (auto it = failures_.begin(); it != failures_.end();) {
            if (it->second.fanin->contains(edited_net)) {
                it = failures_.erase(it);
            } else {
                ++it;
            }
        }
    }

  private:
    struct Evidence {
        std::shared_ptr<const Cone> fanin;
        float slack;
    };
    std::unordered_map<std::string, Evidence> failures_;
};

} // namespace ecc::path_detail
