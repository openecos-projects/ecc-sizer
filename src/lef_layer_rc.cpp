#include "lef_layer_rc.h"

#include <cmath>

namespace ecc {

std::optional<LefLayerRC> lefLayerRC(double width_microns,
                                     double resistance_per_square,
                                     double area_cap_pf_per_square_micron,
                                     double edge_cap_pf_per_micron)
{
    if(!std::isfinite(width_microns) || width_microns <= 0.0 ||
       !std::isfinite(resistance_per_square) || resistance_per_square < 0.0 ||
       !std::isfinite(area_cap_pf_per_square_micron) ||
       area_cap_pf_per_square_micron < 0.0 ||
       !std::isfinite(edge_cap_pf_per_micron) || edge_cap_pf_per_micron < 0.0) {
        return std::nullopt;
    }

    // Match OpenROAD's global-route LEF fallback at the layer's default width.
    const double resistance = resistance_per_square / width_microns * 1e6;
    const double capacitance =
        (width_microns * area_cap_pf_per_square_micron +
         2.0 * edge_cap_pf_per_micron) * 1e-6;
    if(!std::isfinite(resistance) || !std::isfinite(capacitance)) {
        return std::nullopt;
    }
    return LefLayerRC{resistance, capacitance};
}

LefLayerRC fillMissingLayerRC(LefLayerRC configured, LefLayerRC lef)
{
    if(configured.resistance_per_meter == 0.0) {
        configured.resistance_per_meter = lef.resistance_per_meter;
    }
    if(configured.capacitance_per_meter == 0.0) {
        configured.capacitance_per_meter = lef.capacitance_per_meter;
    }
    return configured;
}

}  // namespace ecc
