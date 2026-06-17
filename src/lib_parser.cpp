#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "FuncExpr.hh"
#include "LeakagePower.hh"
#include "Liberty.hh"
#include "LibertyClass.hh"
#include "MinMax.hh"
#include "NetworkClass.hh"
#include "Sequential.hh"
#include "Transition.hh"
#include "sizer.h"
#include "ckt.h"

#include "sta/InternalPower.hh"
#include "sta/Sta.hh"

void skip(istream& is) {
    bool finishedReading = false;

    int check = 1;

    std::vector< string > tokens;
    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);

        if(check == 0) {
            finishedReading = true;
        }
    }
}

int check_brac(char c) {
    if(c == '{') {
        return 1;
    }
    if(c == '}') {
        return -1;
    }
    return 0;
}

int read_line_as_tokens_chk(istream& is, std::vector< string >& tokens) {
    tokens.clear();
    string line;
    std::getline(is, line);

    int chkBrac = 0;

    string token = "";

    for(unsigned i = 0; i < line.size(); ++i) {
        char currChar = line[i];
        bool isSpecialChar = is_special_char(currChar);
        chkBrac += check_brac(currChar);

        if(std::isspace(currChar) || isSpecialChar) {
            if(!token.empty()) {
                // Add the current token to the list of tokens
                tokens.push_back(token);
                token.clear();
            }
        }
        else {
            // Add the char to the current token
            token.push_back(currChar);
        }
    }

    if(!token.empty())
        tokens.push_back(token);

    return chkBrac;
}

bool read_line_as_tokens(istream& is, std::vector< string >& tokens,
                         bool includeSpecialChars) {
    tokens.clear();

    string line;
    std::getline(is, line);

    while(is && tokens.empty()) {
        string token = "";

        for(unsigned i = 0; i < line.size(); ++i) {
            char currChar = line[i];
            bool isSpecialChar = is_special_char(currChar);

            if(std::isspace(currChar) || isSpecialChar) {
                if(!token.empty()) {
                    // Add the current token to the list of tokens
                    tokens.push_back(token);
                    token.clear();
                }

                if(includeSpecialChars && isSpecialChar) {
                    tokens.push_back(string(1, currChar));
                }
            }
            else {
                // Add the char to the current token
                token.push_back(currChar);
            }
        }

        if(!token.empty())
            tokens.push_back(token);

        if(tokens.empty()) {
            // Previous line read was empty. Read the next one.
            std::getline(is, line);
        }
    }

    return !tokens.empty();
}

// Unified unit conversion helper.
struct UnitScaler {
    double time;  // 1e-9 (ns)
    double cap;   // 1e-15 (fF)
    double volt;

    UnitScaler(const sta::Units* u) {
        time = u->timeUnit()->scale() / 1e-9;
        cap = u->capacitanceUnit()->scale() / 1e-15;
        volt = u->voltageUnit()->scale();
    }

    double to_ns(double val) const {
        return val / 1e-9;
    }
    double to_fF(double val) const {
        return val / 1e-15;
    }
};

// Get or insert a pin ID.
static unsigned get_pin_id(LibCellInfo& cell, const std::string& name) {
    auto& pin_map = cell.lib_pin2id_map;
    if(pin_map.find(name) == pin_map.end()) {
        unsigned id = pin_map.size();
        pin_map.emplace(name, id);
        return id;
    }
    return pin_map.at(name);
}

// Extract axis values.
static void extract_axis(const sta::TableAxis* axis, const sta::Units* units,
                         std::vector< double >& out_vec) {
    if(!axis) {
        return;
    }

    float scale = 1.0;
    if(axis->variable() != sta::TableAxisVariable::unknown) {
        scale = tableVariableUnit(axis->variable(), units)->scale();
    }

    const auto& vals = axis->values();
    out_vec.reserve(vals.size());
    for(double v : vals) {
        out_vec.push_back(v / scale);
    }
}

static void extract_lut(sta::LibertyLibrary* sta_lib,
                        const sta::TableModel* sta_model, LibLUT& out_lut) {
    out_lut.templ = sta_model->tblTemplate()->name();

    const auto* units = sta_lib->units();
    UnitScaler scale(units);
    int order = sta_model->order();    // 1 or 2
    assert(order == 1 || order == 2);  // Only 1D/2D tables are supported.

    const auto* ax1 = sta_model->axis1();
    bool ax1_is_trans =
        (std::string(ax1->variableString()).find("transition") !=
         std::string::npos);

    auto& load_vec = out_lut.loadIndices;
    auto& trans_vec = out_lut.transitionIndices;

    auto& ax1_dest = ax1_is_trans ? trans_vec : load_vec;
    extract_axis(ax1, units, ax1_dest);

    if(const auto* ax2 = sta_model->axis2(); ax2) {
        auto& ax2_dest = ax1_is_trans ? load_vec : trans_vec;
        extract_axis(ax2, units, ax2_dest);
    }

    auto normalize_axis = [&](std::vector< double >& vec, double scale_factor) {
        if(vec.empty()) {  // Missing dimension in a 1D table; synthesize axis [0, 100(scaled)].
            vec = {0.0, 100.0 * scale_factor};
            return;
        }
        for(double& v : vec) {
            v *= scale_factor;
        }
    };

    normalize_axis(load_vec, scale.cap);
    normalize_axis(trans_vec, scale.time);

    size_t rows = load_vec.size();
    size_t cols = trans_vec.size();

    out_lut.tableVals.assign(rows, std::vector< double >(cols));
    // work for now
    for(size_t i = 0; i < rows; ++i) {  // Always: index1 = load; index2 = tran
        for(size_t j = 0; j < cols; ++j) {
            int idx1 = ax1_is_trans ? j : i;
            int idx2 = ax1_is_trans ? i : j;
            idx2 = order == 1 ? 0 : idx2;
            out_lut.tableVals[i][j] =
                scale.to_ns(sta_model->value(idx1, idx2, 0));
        }
    }
}

// TODO: OpenSTA Power
void Circuit::parse_power(sta::LibertyLibrary* sta_lib,
                          sta::InternalPower* sta_pwr, LibPowerInfo& out_pwr) {
    out_pwr.toPin = sta_pwr->port() ? sta_pwr->port()->name() : "";
    out_pwr.relatedPin =
        sta_pwr->relatedPort() ? sta_pwr->relatedPort()->name() : "";
}

void Circuit::parse_timing(sta::LibertyLibrary* sta_lib,
                           sta::TimingArcSet* sta_arcset,
                           LibTimingInfo& out_timing) {
    out_timing.isFunc = (sta_arcset->cond() != nullptr);
    out_timing.cnt0 = out_timing.cnt1 = out_timing.cnt2 = out_timing.cnt3 = 1;

    out_timing.toPin = sta_arcset->to()->name();
    out_timing.fromPin = sta_arcset->from()->name();

    // Set timing sense.
    auto s = sta_arcset->sense();
    out_timing.timingSense =
        (s == sta::TimingSense::negative_unate) ? 'n' : 'p';

    if(sta_arcset->role() == sta::TimingRole::setup() ||
       sta_arcset->role() == sta::TimingRole::hold()) {
        out_timing.timingSense = 'c';
    }

    // Extract rise/fall delay and slew.
    for(const auto* rf : sta::RiseFall::range()) {
        auto* arc = sta_arcset->arcTo(rf);
        if(!arc) {
            continue;
        }

        bool is_fall = (rf == sta::RiseFall::fall());
        auto& d_lut = is_fall ? out_timing.fallDelay : out_timing.riseDelay;
        auto& s_lut =
            is_fall ? out_timing.fallTransition : out_timing.riseTransition;

        if(auto* gate = dynamic_cast< sta::GateTableModel* >(arc->model())) {
            extract_lut(sta_lib, gate->delayModel(), d_lut);
            extract_lut(sta_lib, gate->slewModel(), s_lut);
        }
        else if(auto* check =
                    dynamic_cast< sta::CheckTableModel* >(arc->model())) {
            // NOTE: setup is stored in delay; hold is stored in transition.
            if(sta_arcset->role() == sta::TimingRole::setup()) {
                extract_lut(sta_lib, check->checkModel(), d_lut);
            }
            else if(sta_arcset->role() == sta::TimingRole::hold()) {
                extract_lut(sta_lib, check->checkModel(), s_lut);
            }
        }
    }
}

void Circuit::parse_pin(sta::LibertyLibrary* sta_lib,
                        sta::LibertyPort* sta_port, LibCellInfo& out_cell,
                        LibPinInfo& out_pin) {
    UnitScaler scale(sta_lib->units());

    out_pin.name = sta_port->name();
    out_pin.capacitance = scale.to_fF(sta_port->capacitance());

    // Limits
    bool exists;
    float max_cap, max_tran;
    sta_port->capacitanceLimit(sta::MinMax::max(), max_cap, exists);
    out_pin.maxCapacitance =
        exists ? scale.to_fF(max_cap) : std::numeric_limits< double >::max();

    sta_port->slewLimit(sta::MinMax::max(), max_tran, exists);
    out_pin.maxTran = exists ? scale.to_ns(max_tran) : 0;

    // Direction
    auto dir = sta_port->direction();
    out_pin.isInput = dir->isInput() || dir->isBidirect();
    out_pin.isOutput = dir->isOutput() || dir->isBidirect();
    if(!out_pin.isInput && !out_pin.isOutput && !dir->isInternal()) {
        assert(false && "Unknown pin direction!");
    }

    // Clock & Sequential
    if(sta_port->isClock() || sta_port->isClockGateClock()) {
        out_pin.isClock = true;
        out_cell.isSequential = true;
    }
    out_pin.isData = sta_port->isLatchData() || sta_port->isClockGateEnable();

    // IQN Function Check
    if(auto* func = sta_port->function()) {
        if(func->to_string().find("IQN") != string::npos) {
            out_cell.hasQN = true;
            out_pin.IQN = true;
        }
    }
}

// Extract footprint according to technology-node naming rules.
static std::string determine_footprint(Sizer* sizer,
                                       const std::string& cell_name) {
    if(ASAP7) {
        return std::to_string(sizer->cellName2EquaivaID.at(cell_name));
    }

    // Extract the substring between two underscores.
    auto extract_mid = [](std::string& s) {
        size_t p1 = s.find('_');
        if(p1 == string::npos) {
            return;
        }
        size_t p2 = s.find('_', p1 + 1);
        if(p2 != string::npos) {
            s = s.substr(p1 + 1, p2 - p1 - 1);
        }
    };

    std::string fp = cell_name;

    if(STM28) {
        // Extract STM28-style suffix: _X...
        size_t x_pos = fp.find_last_of('X');
        if(x_pos != string::npos) {
            fp.erase(x_pos);
        }
        extract_mid(fp);
        return fp;
    }

    if(C40) {
        extract_mid(fp);
        return fp;
    }

    size_t u_pos = fp.find_first_of('_');
    return (u_pos != string::npos) ? fp.substr(0, u_pos) : "NA";
}

// Determine Vt type.
static cell_vtypes determine_vt_type(const std::string& name, Sizer* sizer) {
    if(sizer->numVt == 3) {
        if(name.find(sizer->suffixLVT) != std::string::npos) {
            return f;
        }
        if(name.find(sizer->suffixHVT) != std::string::npos) {
            return s;
        }
        return m;
    }
    if(sizer->numVt == 2) {
        if(name.find(sizer->suffixHVT) != std::string::npos) {
            return s;
        }
        return m;
    }
    return s;
}

void Circuit::parse_cell(sta::LibertyLibrary* sta_lib,
                         sta::LibertyCell* sta_cell, LibCellInfo& out_cell) {
    out_cell.name = sta_cell->name();
    out_cell.libname = sta_lib->name();

    // Footprint
    out_cell.footprint = sta_cell->footprint();
    if(out_cell.footprint.empty() || NO_FOOTPRINT) {
        out_cell.footprint = determine_footprint(_sizer, out_cell.name);
    }

    // Vt Type
    out_cell.c_vtype = determine_vt_type(out_cell.name, _sizer);

    // Leakage Power
    bool exists;
    float leak;
    sta_cell->leakagePower(leak, exists);
    if(exists) {
        out_cell.leakagePower = leak / _sizer->sw_adj;
    }
    else if(const auto& pwrs = sta_cell->leakagePowers(); !pwrs.empty()) {
        float sum = 0.0;
        for(const auto& p : pwrs) {
            sum += p.power();
        }
        out_cell.leakagePower = sum / _sizer->sw_adj / pwrs.size();
    }
    else {
        out_cell.leakagePower = 0.0;
    }

    out_cell.area = sta_cell->area();
    out_cell.width = sta_cell->area();
    out_cell.isSequential = sta_cell->hasSequentials();
    out_cell.dontTouch = sta_cell->dontUse();
    out_cell.dontUse = isDontUse(out_cell.name);

    float def_slew;
    sta_lib->defaultMaxSlew(def_slew, exists);
    out_cell.max_tran = exists ? def_slew / 1e-9 : 0.0;

    sta::LibertyCellPortIterator port_iter(sta_cell);
    while(port_iter.hasNext()) {
        auto* sta_port = port_iter.next();
        LibPinInfo pin;
        parse_pin(sta_lib, sta_port, out_cell, pin);

        // Update isData from sequential definitions
        for(const auto& seq : sta_cell->sequentials()) {
            if(seq.data()->hasPort(sta_port)) {
                pin.isData = true;
            }
        }

        // Timing Arcs
        auto arcs = sta_cell->timingArcSets(nullptr, sta_port);
        // Handle buses: addr[0] -> addr.
        if(arcs.empty() && sta_port->isBus()) {
            if(auto* mems = sta_port->memberPorts()) {
                arcs = sta_cell->timingArcSets(
                    nullptr, dynamic_cast< sta::LibertyPort* >(mems->at(0)));
            }
        }

        for(auto* sta_arc : arcs) {
            if(!sta_arc->from() || !sta_arc->to()) {
                continue;
            }

            LibTimingInfo ti;
            parse_timing(sta_lib, sta_arc, ti);
            ti.toPin = sta_port->name();

            // Logic Fixes
            if(ti.timingSense == '-') {
                ti.timingSense = pin.IQN ? 'n' : 'p';
            }
            if(ti.timingSense == 'c') {
                pin.isData = true;
            }

            unsigned from_id = get_pin_id(out_cell, ti.fromPin);
            unsigned to_id = get_pin_id(out_cell, ti.toPin);

            auto insert = [&](unsigned idx) {
                if(out_cell.timingArcs.count(idx)) {
                    merge_timingArcs(out_cell.timingArcs[idx], ti);
                }
                else if(!ti.riseDelay.tableVals.empty() ||
                        !ti.fallDelay.tableVals.empty() ||
                        !ti.fallTransition.tableVals.empty() ||
                        !ti.riseTransition.tableVals.empty()) {
                    out_cell.timingArcs.emplace(idx, ti);
                }
            };

            if(from_id != to_id) {
                insert(from_id + (to_id * 100));
                // PB4W Special
                if(pin.isOutput && pin.isInput && out_cell.name == "PB4W") {
                    std::swap(ti.fromPin, ti.toPin);
                    insert((from_id * 100) + to_id);
                }
            }
        }

        // TODO: Power Table

        if(pin.isInput || pin.isOutput) {
            out_cell.pins.emplace(get_pin_id(out_cell, pin.name), pin);
        }
    }

    for(auto& [name, ti] : out_cell.timingArcs) {
        if(ti.cnt0 + ti.cnt1 + ti.cnt2 + ti.cnt3 > 4) {
            average_lut(ti.fallDelay, ti.cnt0);
            average_lut(ti.riseDelay, ti.cnt1);
            average_lut(ti.fallTransition, ti.cnt2);
            average_lut(ti.riseTransition, ti.cnt3);
            ti.cnt0 = ti.cnt1 = ti.cnt2 = ti.cnt3 = 1;
        }
    }
}

void Circuit::lib_parser(string filename, unsigned corner) {
    cout << "Reading .lib file " << filename << " for corner " << corner
         << endl;

    sta::LibertyLibrary* sta_lib = _sta->readLiberty(
        filename.c_str(), _sta->cmdScene(), sta::MinMaxAll::all(), true);

    LibInfo lib;
    lib.name = sta_lib->name();

    const auto* units = sta_lib->units();
    lib.time_unit = units->timeUnit()->scale();
    lib.voltage_unit = units->voltageUnit()->scale();
    lib.current_unit = units->currentUnit()->scale();
    lib.cap_unit = units->capacitanceUnit()->scale();
    lib.leak_power_unit = units->powerUnit()->scale();

    // Default Max Slew
    bool exists;
    float slew;
    sta_lib->defaultMaxSlew(slew, exists);
    lib.max_transition = exists ? (slew / lib.time_unit) : 0.0;

    lib.volt = sta_lib->nominalVoltage();
    // Derived units: IntPower = V^2 * C / T, SwPower = V^2 * C
    double v_sq = std::pow(lib.voltage_unit, 2);
    lib.int_power_unit = v_sq * lib.cap_unit / lib.time_unit;
    lib.sw_power_unit = v_sq * lib.cap_unit;

    // Parse templates.
    for(const auto* sta_tmpl : sta_lib->tableTemplates()) {
        LibTableTempl tmpl;
        tmpl.name = sta_tmpl->name();
        tmpl.loadFirst = tmpl.tranFirst = false;

        auto parse_ax = [&](const sta::TableAxis* ax, bool is_ax1) {
            if(!ax) {
                return;
            }
            string var(ax->variableString());

            if(is_ax1) {
                tmpl.tranFirst = (var.find("transition") != string::npos);
                tmpl.loadFirst = (var.find("capacitance") != string::npos);
            }

            auto& target = (is_ax1 == tmpl.tranFirst) ? tmpl.transitionIndices
                                                      : tmpl.loadIndices;
            extract_axis(ax, units, target);
        };

        parse_ax(sta_tmpl->axis1(), true);
        parse_ax(sta_tmpl->axis2(), false);

        if(sta_tmpl->axis1()) {
            lib.templs.emplace(tmpl.name, tmpl);
        }
    }

    // Parse cells.
    std::vector< LibCellInfo > cells;
    sta::LibertyCellIterator it(sta_lib);
    while(it.hasNext()) {
        LibCellInfo c;
        parse_cell(sta_lib, it.next(), c);
        cells.push_back(std::move(c));
    }

    _sizer->LIBs.insert(pair< string, LibInfo >(lib.name, lib));

    // JLPWR
    _sizer->sw_adj = 1e-6;
    _sizer->res_unit = 1e6;
    _sizer->cap_unit = 1e-15;
    _sizer->time_unit = 1e-9;

    LibCellInfo cur_cell;

    for(auto& cell : cells) {
        _sizer->libs[corner].emplace(cell.name, cell);
    }

    for(unsigned i = 0; i < cells.size(); i++) {
        cur_cell = cells[i];
        map< string, unsigned >::iterator temp_iter;
        // if(cur_cell.footprint == "152"){
        //     cout << "152" << endl;
        // }
        if((temp_iter = _sizer->func2id.find(cur_cell.footprint)) !=
           _sizer->func2id.end()) {
            if(!cur_cell.dontUse) {
                _sizer->func_lib_cell_list[corner][cur_cell.footprint]
                    .push_back(
                        &_sizer->libs[corner].find(cur_cell.name)->second);
            }
        }
        else {
            _sizer->func2id.insert(pair< string, unsigned >(
                cur_cell.footprint, _sizer->func2id.size()));
            if(!cur_cell.dontUse) {
                list< LibCellInfo* > cell_list;
                cell_list.push_back(
                    &_sizer->libs[corner].find(cur_cell.name)->second);
                _sizer->func_lib_cell_list[corner].insert(
                    std::pair< string, list< LibCellInfo* > >(
                        cur_cell.footprint, cell_list));
            }
        }
    }
#if 1
    int t_corner = 0;
    for(auto it = _sizer->func_lib_cell_list[t_corner].begin();
        it != _sizer->func_lib_cell_list[t_corner].end(); ++it) {
        for(LibCellInfo* lib_cell_info : it->second) {
            double partial_order = 0;
            int partial_count = 0;
            for(auto [id, pin] : lib_cell_info->pins) {
                if(pin.isInput) {
                    partial_order += pin.capacitance;
                    partial_count++;
                }
            }
            lib_cell_info->partial_order = partial_order / partial_count;
        }
        (it->second).sort([&](LibCellInfo* c1, LibCellInfo* c2) {
            return c1->leakagePower < c2->leakagePower;
        });
    }
#endif
}

string Circuit::read_lib_name(istream& is) {
    bool finishedReading = false;

    std::vector< string > tokens;

    while(!finishedReading) {
        read_line_as_tokens_chk(is, tokens);

        if(tokens.size() == 2 && tokens[0] == "library") {
            return tokens[1];
        }
    }
    return "";
}

void Circuit::average_lut(LibLUT& lut, int num) {
    if(num <= 0) {
        return;
    }
    for(unsigned i = 0; i < lut.loadIndices.size(); ++i) {
        for(unsigned j = 0; j < lut.transitionIndices.size(); ++j) {
            lut.tableVals[i][j] /= double(num);
        }
    }
}

void Circuit::add_pg_pin(LibPowerInfo& powerTables, string pg_pin) {
    for(unsigned i = 0; i < powerTables.pgPins.size(); ++i) {
        if(powerTables.pgPins[i] == pg_pin) {
            return;
        }
    }
    powerTables.pgPins.push_back(pg_pin);
}

void Circuit::update_lut(LibLUT& toLut, LibLUT fromLut) {
    if(toLut.tableVals.empty()) {
        toLut.templ = fromLut.templ;
        toLut.loadIndices = fromLut.loadIndices;
        toLut.transitionIndices = fromLut.transitionIndices;
        toLut.tableVals = fromLut.tableVals;
    }
    else {
        for(unsigned i = 0; i < toLut.loadIndices.size(); ++i) {
            for(unsigned j = 0; j < toLut.transitionIndices.size(); ++j) {
                toLut.tableVals[i][j] += fromLut.tableVals[i][j];
            }
        }
    }
}

void Circuit::merge_timingArcs(LibTimingInfo& timing1, LibTimingInfo& timing2) {
    if(timing1.isFunc && !timing2.isFunc) {
        if(timing2.fallDelay.templ == "") {
            timing1.fallDelay.tableVals.clear();
        }
        else {
            timing1.fallDelay = timing2.fallDelay;
        }
        if(timing2.riseDelay.templ == "") {
            timing1.riseDelay.tableVals.clear();
        }
        else {
            timing1.riseDelay = timing2.riseDelay;
        }
        if(timing2.fallTransition.templ == "") {
            timing1.fallTransition.tableVals.clear();
        }
        else {
            timing1.fallTransition = timing2.fallTransition;
        }
        if(timing2.riseTransition.templ == "") {
            timing1.riseTransition.tableVals.clear();
        }
        else {
            timing1.riseTransition = timing2.riseTransition;
        }
        timing1.isFunc = false;
        timing1.cnt0 = 1;
        timing1.cnt1 = 1;
        timing1.cnt2 = 1;
        timing1.cnt3 = 1;
    }
    else if(timing1.isFunc == timing2.isFunc) {
        if(timing2.fallDelay.templ != "") {
            if(timing1.fallDelay.templ != "") {
                ++timing1.cnt0;
            }
            update_lut(timing1.fallDelay, timing2.fallDelay);
        }
        if(timing2.riseDelay.templ != "") {
            if(timing1.riseDelay.templ != "") {
                ++timing1.cnt1;
            }
            update_lut(timing1.riseDelay, timing2.riseDelay);
        }
        if(timing2.fallTransition.templ != "") {
            if(timing1.fallTransition.templ != "") {
                ++timing1.cnt2;
            }
            update_lut(timing1.fallTransition, timing2.fallTransition);
        }
        if(timing2.riseTransition.templ != "") {
            if(timing1.riseTransition.templ != "") {
                ++timing1.cnt3;
            }
            update_lut(timing1.riseTransition, timing2.riseTransition);
        }
    }
}

void Circuit::merge_powerTables(LibPowerInfo& power1, LibPowerInfo& power2) {
    if(power1.isFunc && !power2.isFunc) {
        if(power2.fallPower.templ != "") {
            power1.fallPower = power2.fallPower;
        }
        if(power2.risePower.templ != "") {
            power1.risePower = power2.risePower;
        }
        power1.isFunc = false;
        power1.cnt = 1;
    }
    else if(!power1.isFunc && !power2.isFunc) {
        bool flag = false;
        for(unsigned i = 0; i < power1.pgPins.size(); ++i) {
            if(power1.pgPins[i] == power2.pgPins[0]) {
                flag = true;
                break;
            }
        }
        if(!flag) {
            if(power2.fallPower.templ != "") {
                update_lut(power1.fallPower, power2.fallPower);
            }
            if(power2.risePower.templ != "") {
                update_lut(power1.risePower, power2.risePower);
            }
            add_pg_pin(power1, power2.pgPins[0]);
        }
    }
    else if(power1.isFunc && power2.isFunc) {
        if(power2.fallPower.templ != "") {
            update_lut(power1.fallPower, power2.fallPower);
        }
        if(power2.risePower.templ != "") {
            update_lut(power1.risePower, power2.risePower);
        }
        add_pg_pin(power1, power2.pgPins[0]);
        ++power1.cnt;
    }
}
