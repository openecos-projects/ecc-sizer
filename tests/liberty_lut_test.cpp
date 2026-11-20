// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <iostream>
#include <memory>

#include "liberty_lut.h"
#include "sta/Liberty.hh"
#include "sta/TableModel.hh"
#include "sta/Units.hh"

namespace {
bool check(const LibLUT& lut, const std::vector<std::vector<double>>& values) {
    if(lut.tableVals.size() != values.size()) return false;
    for(size_t i = 0; i < values.size(); ++i) {
        if(lut.tableVals[i].size() != values[i].size()) return false;
        for(size_t j = 0; j < values[i].size(); ++j) {
            if(std::abs(lut.tableVals[i][j] - values[i][j]) > 1e-5) return false;
        }
    }
    return true;
}
}

int main() {
    sta::Units units;
    units.timeUnit()->setScale(1e-9);
    units.capacitanceUnit()->setScale(1e-12);
    sta::TableTemplate templ("test");
    auto convert = [&](sta::TablePtr table) {
        sta::TableModel model(table, &templ, sta::ScaleFactorType::cell,
                              sta::RiseFall::rise());
        LibLUT lut;
        extract_lut(&units, &model, lut);
        return lut;
    };
    bool ok = check(convert(std::make_shared<sta::Table>(2.5e-9F)),
                    {{2.5, 2.5}, {2.5, 2.5}});
    auto trans = std::make_shared<sta::TableAxis>(
        sta::TableAxisVariable::input_net_transition,
        sta::FloatSeq{0.1e-9F, 0.2e-9F});
    auto load = std::make_shared<sta::TableAxis>(
        sta::TableAxisVariable::total_output_net_capacitance,
        sta::FloatSeq{10e-15F, 20e-15F});
    ok &= check(convert(std::make_shared<sta::Table>(
        sta::FloatSeq{1e-9F, 2e-9F}, trans)), {{1, 2}, {1, 2}});
    ok &= check(convert(std::make_shared<sta::Table>(
        sta::FloatSeq{1e-9F, 2e-9F}, load)), {{1, 1}, {2, 2}});
    auto singleton = std::make_shared<sta::TableAxis>(
        sta::TableAxisVariable::input_net_transition, sta::FloatSeq{0.1e-9F});
    ok &= check(convert(std::make_shared<sta::Table>(
        sta::FloatSeq{3e-9F}, singleton)), {{3, 3}, {3, 3}});
    auto lut = convert(std::make_shared<sta::Table>(
        sta::FloatTable{{1e-9F, 2e-9F}, {3e-9F, 4e-9F}}, trans, load));
    ok &= check(lut, {{1, 3}, {2, 4}});
    ok &= std::abs(lut.loadIndices[0] - 10) < 1e-5;
    ok &= std::abs(lut.transitionIndices[1] - 0.2) < 1e-5;
    ok &= check(convert(std::make_shared<sta::Table>(
        sta::FloatTable{{1e-9F, 3e-9F}, {2e-9F, 4e-9F}}, load, trans)),
        {{1, 3}, {2, 4}});
    if(!ok) std::cerr << "Liberty LUT conversion failed\n";
    return ok ? 0 : 1;
}
