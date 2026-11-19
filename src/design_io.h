// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

namespace ord { class Tech; class Design; }
namespace odb { class dbBlock; }
namespace sta { class dbSta; }

namespace ecc {

struct DesignInputs {
    std::string script;
    std::string odb;
    std::string def;
    std::vector<std::string> lef;
    std::vector<std::string> liberty;
    std::string sdc;
    std::string rc;
};

// Owns the OpenROAD session. OpenDB is the only mutable design model;
// solvers may keep short-lived analysis/candidate data derived from it.
class DesignSession {
public:
    explicit DesignSession(const DesignInputs& inputs);
    ~DesignSession();
    DesignSession(const DesignSession&) = delete;
    DesignSession& operator=(const DesignSession&) = delete;

    ord::Design& design() const;
    odb::dbBlock* block() const;
    sta::dbSta* timing() const;

    // Arguments go through Tcl objects, so filenames and names are never
    // interpolated as Tcl code. Every Tcl failure becomes an exception.
    std::string command(std::initializer_list<std::string> words) const;
    void initializePlacementRC();
    void refresh();
    void reconnectSupplies();
    void write(const std::string& prefix) const;

private:
    std::unique_ptr<ord::Tech> tech_;
    std::unique_ptr<ord::Design> design_;
    unsigned baseline_missing_supplies_ = 0;
};

}  // namespace ecc
