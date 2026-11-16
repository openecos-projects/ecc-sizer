// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "repair_pbbi_solver.h"
#include <map>
#include <string>
#include <unordered_map>
namespace odb {
class dbBlock;
class dbInst;
class dbMaster;
class dbNet;
class dbObject;
}  // namespace odb
namespace sta {
class dbSta;
class Vertex;
}
namespace ecc {
// Endpoint-only setup slack for the command scene, indexed by rise/fall.
// Internal endpoints must exclude required times propagated from downstream.
std::array<double, 2> pbbiEndpointSlacks(sta::dbSta* sta, sta::Vertex* vertex);

// Actual candidate net response, calibrated against the committed tree.
// It contains only this ECO's buffers, not speculative edits on other nets.
struct PbbiNetResponse {
    double cap = 0;
    std::function< pbbi::ArcResponse(odb::dbObject*, double, unsigned) > delta;
};
struct PbbiElectricalRisk {
    unsigned slew = 0, cap = 0, unsupported = 0;
    double severity = 0, wns_delta = 0, tns_delta = 0;
    // Actual STA endpoint slack at the committed snapshot. Graph-only values
    // are diagnostic: a graph cut is not necessarily an STA endpoint.
    double baseline_wns = 0, baseline_tns = 0;
    double graph_wns = 0, graph_tns = 0;
    unsigned endpoints = 0, graph_endpoints = 0;
    std::unordered_map< std::string, double > slew_by_pin;
    std::unordered_map< std::string, double > endpoint_slack_by_pin;
    std::vector< std::string > affected_endpoints;
};
// Immutable STA/DB snapshot; prediction does not query mutable STA or modify
// the DB, so it is safe between deferred ECO rollbacks. Liberty is read-only.
class PbbiElectricalModel {
   public:
    PbbiElectricalModel(odb::dbBlock* block, sta::dbSta* sta);
    ~PbbiElectricalModel();
    PbbiElectricalRisk predict(
        const std::vector< std::pair< odb::dbInst*, odb::dbMaster* > >& sizes,
        const std::unordered_map< odb::dbNet*, PbbiNetResponse >& nets,
        bool capture_pins = false);

   private:
    struct Impl;
    std::unique_ptr< Impl > impl_;
};
}  // namespace ecc
