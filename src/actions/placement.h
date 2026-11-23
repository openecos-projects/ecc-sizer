// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <vector>
namespace odb {
class dbBlock;
class dbInst;
} // namespace odb
namespace dpl {
class Opendp;
}
namespace ecc {
enum class PlacementLegalizer { OpenDP, Abacus };

// Legalize and validate; protect DNT/explicit exclusions without changing
// their persistent placement status. Throws on failure. May run inside an ECO.
void legalizeRepairPlacement(odb::dbBlock* block, dpl::Opendp* placer,
                             const std::vector<odb::dbInst*>& protected_instances = {},
                             bool incremental = false,
                             PlacementLegalizer legalizer = PlacementLegalizer::OpenDP);
} // namespace ecc
