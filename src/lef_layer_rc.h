#pragma once

#include <optional>

namespace ecc {

struct LefLayerRC {
    double resistance_per_meter;
    double capacitance_per_meter;
};

std::optional<LefLayerRC> lefLayerRC(double width_microns,
                                     double resistance_per_square,
                                     double area_cap_pf_per_square_micron,
                                     double edge_cap_pf_per_micron);
LefLayerRC fillMissingLayerRC(LefLayerRC configured, LefLayerRC lef);

}  // namespace ecc
