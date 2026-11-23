// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "actions/netlist.h"
#include "design_analysis.h"
#include "repair/config.h"
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <map>
#include <unordered_map>
#include <unordered_set>

namespace sta {
class Pin;
class dbSta;
class dbNetwork;
} // namespace sta
namespace ecc {
// Mutable progress shared by the candidate policy and the repair runner.
// Repair owns the committed metrics; scheduling only updates its counters.
struct PathSearchState {
    DesignMetrics before, after;
    unsigned inserted_buffers = 0;
    unsigned candidate_count = 0;
    unsigned duplicate_candidates = 0;
    unsigned screened_candidates = 0;
    unsigned failure_cache_hits = 0;
    unsigned reused_rejections = 0;
    unsigned deferred_candidates = 0;
    unsigned invalidated_failures = 0;
    unsigned deferred_trials = 0;
    unsigned repeated_trials = 0;
    unsigned calibrated_candidates = 0;
    unsigned feedback_updates = 0;
    unsigned frontier_suspended = 0;
    unsigned frontier_audits = 0;
    unsigned frontier_reopened = 0;
    unsigned fanout_precheck_rejections = 0;
};
// Initial candidate frontier for the current STA snapshot. Forecasts rank
// candidates only; callers still own global timing/DRV acceptance after editing.
struct PathCandidate {
    BufferTreePlan plan;
    float slack = 0;
    float predicted_wns_gain = 0;
    float predicted_tns_gain = 0;
    bool recovery = false;
};

class PathSearch {
  public:
    // Read-only generation on one committed STA snapshot. Feedback begins empty;
    // plans expire after changes to topology, placement or the STA scene.
    static std::vector<PathCandidate>
    generateCandidates(odb::dbBlock* block, sta::dbSta* sta,
                       const PathSearchOptions& options = {},
                       const DesignConstraints& constraints = {});
};
} // namespace ecc

namespace ecc::path_detail {
struct PathBufferCandidate;
// Measured changes for one trial. Rejection evidence is borrowed only during
// observe/recoverElectrical; policies copy any history they retain.
struct PathObservation {
    double wns_gain = 0, tns_gain = 0;
    double objective_wns_gain = 0, objective_tns_gain = 0;
    const std::vector<std::string>& rejection_reasons;
    bool metrics_complete = true, frontier_audit = false;
};

// Never accept using only the incremental precheck. Also revisit electrical
// rejections: small STA roundoff near a DRV boundary can change their verdict.
bool needsFullVerification(const std::vector<std::string>& reasons);

// The full serialization is the key. unordered_map may hash it, but equality
// always checks the complete action rather than trusting a digest.
struct ActionNode {
    std::string master;
    int x = 0, y = 0;
    std::vector<std::string> terminals;
    std::vector<size_t> children;
};
std::string actionFingerprint(const std::string& net,
                              const std::vector<ActionNode>& nodes,
                              bool physical = true);

// Prefix sums make a full endpoint forecast logarithmic per load/variant.
// Include nonviolating endpoints too: a negative gain can make them violate.
// This models the current worst path, not future competing paths; rank only.
class EndpointSlackProfile {
  public:
    void add(double slack);
    void finish();
    double gain(double delay_gain) const;
    size_t size() const;

  private:
    double negativeSum(double gain) const;
    std::vector<double> slacks_, prefix_{0};
    double before_ = 0;
};

// Soft scheduling memory, not a certificate that global timing is unchanged.
// Unrelated commits do not erase exploration of the same load group. Changes
// to its structural context reopen it. No candidate is pruned using this count.
class ContextCoverage {
  public:
    unsigned attempts(const std::string& family, const std::string& context) const;
    void observe(const std::string& family, const std::string& context);

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

bool betterElectricalAlternative(const CandidateQuality& a, const CandidateQuality& b);

// These are estimates for scheduling, not upper bounds. In particular, a
// sampled endpoint list cannot prove that no other endpoint has a TNS gain.
ObjectiveForecast forecastObjective(const std::vector<PathForecast>& paths,
                                    const std::map<size_t, double>& endpoint_slacks,
                                    double global_wns, bool local = false);

// A data-derived fit, without a fixed penalty, retry count or time budget.
// Use it only to rank siblings; it is not a proof for pruning them.
struct GainCalibration {
    double xx = 0, xy = 0, measured_sum = 0;
    unsigned samples = 0;
    void observe(double predicted, double measured);
    double predict(double raw) const;
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
    Decision decide(Gains predicted, Gains floor) const;
    void observe(Gains predicted, Gains actual, bool rejected, bool audit);

  private:
    static bool finite(Gains p);
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
    unsigned attempts(const std::string& family, unsigned generation) const;
    void observe(const std::string& family, unsigned generation);

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
                 unsigned generation) const;
    void record(std::string action, FailureEvidence evidence);
    const FailureEvidence* find(const std::string& action) const;
    bool opportunityChanged(const std::string& action, double wns, double tns,
                            double wns_floor, double tns_floor) const;

  private:
    std::unordered_map<std::string, FailureEvidence> records_;
};

// A rejected TNS delta can be reused only while its entire delay cone is
// untouched. WNS/no-gain failures are deliberately excluded: an unrelated
// commit can expose a different global bottleneck and make them worthwhile.
class DependencyFailures {
  public:
    using Cone = std::unordered_set<unsigned>;
    void record(const std::string& action, Cone cone, float before, float after);
    void record(const std::string& action, std::shared_ptr<const Cone> cone,
                float before, float after);
    bool rejects(const std::string& action, float current, double tolerance) const;
    void committed(const Cone& changed);

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
                float slack);
    bool rejects(const std::string& action, float current_wns, double tolerance) const;
    void committed(unsigned edited_net);

  private:
    struct Evidence {
        std::shared_ptr<const Cone> fanin;
        float slack;
    };
    std::unordered_map<std::string, Evidence> failures_;
};

// Per-run learning and retry policy. It never edits the design or accepts an ECO;
// observations come from the search runner after actual STA measurement.
class PathFeedback {
  public:
    PathFeedback(const PathSearchOptions& options, PathSearchState& result);
    ~PathFeedback();
    PathFeedback(const PathFeedback&) = delete;
    PathFeedback& operator=(const PathFeedback&) = delete;
    void beginCollection();
    bool hasFailure(const PathBufferCandidate& candidate) const;
    unsigned generationIndex() const;
    double tnsEstimate(const PathBufferCandidate& candidate) const;
    ExplorationFrontier::Decision explore(const PathBufferCandidate& candidate);
    void committed();
    void rank(PathBufferCandidate& candidate, bool count = false);
    static bool order(const PathBufferCandidate& a, const PathBufferCandidate& b);
    void select(std::vector<PathBufferCandidate>& variants);
    void screen(std::vector<PathBufferCandidate>& candidates);
    void recoverElectrical(const PathBufferCandidate& failed,
                           const PathObservation& trial,
                           std::vector<PathBufferCandidate>& pending, size_t next);
    void covered(const PathBufferCandidate& candidate);
    void observe(const PathBufferCandidate& candidate, const PathObservation& trial);

  private:
    struct Model {
        std::string context;
        path_detail::GainCalibration wns, tns;
    };
    path_detail::FailureHistory failures;
    std::unordered_map<std::string, Model> models;
    path_detail::FamilyCoverage coverage;
    unsigned generation = 0;
    const PathSearchOptions& options;
    PathSearchState& result;
    std::unordered_map<std::string, path_detail::ExplorationFrontier> frontiers;
    path_detail::ContextCoverage group_coverage;
    // One unselected representative per load set/master, not every position
    // and topology. These are activated only by a measured electrical failure
    // with sufficient actual timing gain, on this committed STA snapshot.
    std::vector<PathBufferCandidate> electrical_reserve;
};

const DesignMetrics& globalGainReference(const PathSearchOptions& options,
                                         const DesignMetrics& initial,
                                         const DesignMetrics& current);

// Internal candidate snapshot shared by generation and scheduling. Its DB/STA
// objects belong to the current committed design; rebuild after a commit.
// A topology plan is immutable until the entire ECO is accepted. Nodes are
// bottom-up: every child index precedes its parent, and the root is last.
struct PathTreeNode {
    BufferLocation location;
    std::vector<size_t> loads, children;
    float cap = 0;
};
struct PathTree {
    std::vector<PathTreeNode> nodes;
    std::vector<odb::dbObject*> terminals;
};

struct PathBufferCandidate {
    odb::dbNet* net = nullptr;
    std::vector<odb::dbObject*> loads;
    odb::dbMaster* master = nullptr;
    BufferLocation location;
    float slack = 0;
    float predicted_gain = 0;
    size_t path_index = 0;
    float predicted_wns_gain = 0;
    float predicted_tns_gain = 0;
    path_detail::CandidateQuality quality;
    bool electrical_retry = false;
    unsigned affected_endpoints = 0;
    unsigned variant_rank = 0;
    std::string feedback_key;
    std::string group_key;
    std::string load_group_key;
    std::shared_ptr<const std::string> feedback_context;
    double ranked_wns_gain = 0, ranked_tns_gain = 0, rank_score = 0;
    bool deferred_retry = false;
    unsigned feedback_priority = 0;
    unsigned family_attempts = 0;
    double covered_tns_gain = 0;
    unsigned covered_endpoints = 0;
    std::vector<odb::dbInst*> cone;
    bool recovery = false;
    std::shared_ptr<const PathTree> tree;
    // Saved on the committed netlist, never read from deferred rollback STA.
    std::vector<std::pair<const sta::Pin*, float>> endpoints;
    double objective_wns = 0, objective_tns = 0;
};

struct CandidateCollection {
    std::vector<PathBufferCandidate> candidates;
    unsigned duplicate_candidates = 0;
};

BufferTreePlan bufferPlan(const PathBufferCandidate& candidate);
float fanoutWeight(sta::dbNetwork* network, odb::dbObject* term);
CandidateCollection collectCandidates(detail::DesignAnalysis& analysis, sta::dbSta* sta,
                                      const std::vector<detail::NetState>& states,
                                      const PathSearchOptions& options,
                                      const DesignMetrics& metrics,
                                      PathFeedback* feedback);

std::string terminalKey(odb::dbObject* term);
std::string actionKey(const PathBufferCandidate& candidate, bool physical = true);
bool gainOrder(const PathBufferCandidate& a, const PathBufferCandidate& b);

struct PathBatch {
    std::vector<size_t> indices;
    double area = 0;
    unsigned buffers = 0;
};

// One candidate wave on one committed design. Queue operations never inspect
// live STA or edit the netlist, including while an ECO's refresh is deferred.
class PathSchedule {
  public:
    PathSchedule(std::vector<PathBufferCandidate> candidates,
                 const PathSearchOptions& options, PathSearchState& result,
                 PathFeedback& feedback);
    const std::vector<PathBufferCandidate>& candidates() const { return plans; }
    bool hasNext() const { return cursor < plans.size(); }
    bool prepareNext();
    bool frontierAudit() const { return frontier_audit; }
    PathBatch selectBatch(double area_limit);
    void skipBatch();
    void backoff();
    void rerank();
    void rejected();
    void fanoutRejected(const PathBufferCandidate& candidate);
    void recoverElectrical(size_t failed, const PathObservation& trial);

  private:
    static std::string topologyKey(const PathBufferCandidate& candidate);
    void reopen();
    std::vector<PathBufferCandidate> plans, suspended;
    const PathSearchOptions& options;
    PathSearchState& result;
    PathFeedback& feedback;
    std::unordered_set<std::string> infeasible_topologies;
    size_t cursor = 0, width = 0;
    bool single_candidate_mode = false, frontier_audit = false;
};

} // namespace ecc::path_detail
