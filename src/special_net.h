// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace odb {
class dbBlock;
}

namespace ecc {

int connectMissingWildcardSpecialNetPins(odb::dbBlock* block);

}  // namespace ecc
