#include <cmath>
#include <iostream>
#include <limits>

#include "lef_layer_rc.h"

namespace {

bool expect(bool condition, const char* message)
{
    if(!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

}  // namespace

int main()
{
    bool passed = true;
    const auto rc = ecc::lefLayerRC(0.2, 0.04, 0.08, 0.03);
    passed &= expect(rc.has_value(), "valid LEF geometry produces layer RC");
    if(rc) {
        passed &= expect(std::abs(rc->resistance_per_meter - 200000.0) < 1e-9,
                         "sheet resistance is divided by default width");
        passed &= expect(std::abs(rc->capacitance_per_meter - 7.6e-8) < 1e-18,
                         "area and both edge capacitances are included");
    }

    const auto edge_only = ecc::lefLayerRC(0.2, 0.04, 0.0, 0.03);
    passed &= expect(edge_only && edge_only->capacitance_per_meter > 0.0,
                     "edge-only LEF capacitance is usable");
    passed &= expect(!ecc::lefLayerRC(0.0, 0.04, 0.08, 0.03),
                     "zero-width layer cannot be converted");
    passed &= expect(!ecc::lefLayerRC(0.2, -0.04, 0.08, 0.03),
                     "negative sheet resistance is rejected");
    passed &= expect(!ecc::lefLayerRC(
                         0.2, 0.04, std::numeric_limits<double>::infinity(), 0.03),
                     "non-finite LEF values are rejected");

    if(rc) {
        const ecc::LefLayerRC configured{123.0, 0.0};
        const auto merged = ecc::fillMissingLayerRC(configured, *rc);
        passed &= expect(merged.resistance_per_meter == 123.0 &&
                             merged.capacitance_per_meter == rc->capacitance_per_meter,
                         "configured R is retained while missing C is filled");
        const ecc::LefLayerRC complete{123.0, 456.0};
        const auto unchanged = ecc::fillMissingLayerRC(complete, *rc);
        passed &= expect(unchanged.resistance_per_meter == 123.0 &&
                             unchanged.capacitance_per_meter == 456.0,
                         "fully configured layer RC is not overridden");
    }
    return passed ? 0 : 1;
}
