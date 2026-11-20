// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "liberty_lut.h"

#include <algorithm>
#include <stdexcept>

#include "sta/Liberty.hh"
#include "sta/TableModel.hh"
#include "sta/Units.hh"

void extract_lut(const sta::Units* units, const sta::TableModel* model,
                 LibLUT& lut) {
    const int order = model->order();
    if(order < 0 || order > 2) {
        throw std::runtime_error("Sizer requires scalar, 1D or 2D timing tables");
    }
    lut = {};
    lut.templ = model->tblTemplate()->name();
    const auto* ax1 = model->axis1();
    const auto* ax2 = model->axis2();
    const bool trans_first = ax1 &&
        ax1->variableString().find("transition") != std::string::npos;

    auto copy_axis = [&](const sta::TableAxis* axis,
                         std::vector<double>& dest, double output_scale) {
        if(axis) {
            const auto* unit = sta::tableVariableUnit(axis->variable(), units);
            const double input_scale = unit ? unit->scale() : 1.0;
            for(double value : axis->values()) {
                dest.push_back(value / input_scale * output_scale);
            }
        }
        // Sizer's bilinear interpolation requires two points per axis.
        // A missing or singleton axis is constant: extend its coordinates,
        // and repeat the corresponding value below rather than indexing past it.
        if(dest.empty()) {
            dest = {0.0, 100.0 * output_scale};
        }
        else if(dest.size() == 1) {
            dest.push_back(dest.front() + 100.0 * output_scale);
        }
    };
    copy_axis(trans_first ? ax2 : ax1, lut.loadIndices,
              units->capacitanceUnit()->scale() / 1e-15);
    copy_axis(trans_first ? ax1 : ax2, lut.transitionIndices,
              units->timeUnit()->scale() / 1e-9);

    auto source_index = [](const sta::TableAxis* axis, size_t index) {
        return axis && axis->size() ? std::min(index, axis->size() - 1) : 0;
    };
    lut.tableVals.assign(lut.loadIndices.size(),
                         std::vector<double>(lut.transitionIndices.size()));
    for(size_t load = 0; load < lut.loadIndices.size(); ++load) {
        for(size_t trans = 0; trans < lut.transitionIndices.size(); ++trans) {
            lut.tableVals[load][trans] = model->value(
                source_index(ax1, trans_first ? trans : load),
                source_index(ax2, trans_first ? load : trans), 0) / 1e-9;
        }
    }
}
