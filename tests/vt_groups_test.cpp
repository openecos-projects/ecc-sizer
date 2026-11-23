// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vt_groups.h"

namespace {
int failures = 0;

void require(bool condition, const std::string& what)
{
  if (!condition) {
    std::cerr << "FAIL: " << what << std::endl;
    ++failures;
  }
}

template <typename Fn>
bool throws(Fn fn)
{
  try {
    fn();
  } catch (const std::runtime_error&) {
    return true;
  }
  return false;
}

// Ascending leakage is slowest-first, matching cell_vtypes {s, m, f}.
void check_vt_axis_follows_leakage()
{
  const std::vector<std::pair<std::string, double>> cells{
      {"lib_lvt", 4.6},
      {"lib_hvt", 1.6},
      {"lib_svt", 2.8},
      {"lib_lvt", 4.7},
      {"lib_hvt", 1.7},
      {"lib_svt", 2.9},
  };
  const VtGroups groups = derive_vt_groups(cells, 3);
  require(groups.count == 3, "three libraries become three groups");
  require(groups.vt_of("lib_hvt") == 0, "least leaky library is the slowest");
  require(groups.vt_of("lib_svt") == 1, "middle leakage is the middle group");
  require(groups.vt_of("lib_lvt") == 2, "leakiest library is the fastest");
  require(groups.vt_of("not_a_library") == 0,
          "an unknown library falls back to the slowest slot");
}

void check_single_library_is_single_vt()
{
  const VtGroups groups = derive_vt_groups({{"only", 1.0}, {"only", 2.0}}, 3);
  require(groups.count == 1, "one library is one group");
  require(groups.vt_of("only") == 0, "its cells take the only slot");
}

// The sizing tables cannot represent more groups than cell_vtypes has slots,
// so an over-wide library set must fail instead of being truncated.
void check_too_many_libraries_is_an_error()
{
  require(throws([] {
            derive_vt_groups({{"a", 1.0}, {"b", 2.0}, {"c", 3.0}, {"d", 4.0}},
                             3);
          }),
          "four libraries exceed a three-slot axis");
}

// Guessing an order would mis-rank every retype decision, so refuse instead.
void check_unorderable_libraries_are_an_error()
{
  require(throws([] { derive_vt_groups({{"a", 2.0}, {"b", 2.0}}, 3); }),
          "equal mean leakage cannot order two libraries");
  require(throws([] { derive_vt_groups({{"a", 1.0}, {"b", 0.0}}, 3); }),
          "a library without leakage data cannot be ordered");
}

// Area names a drive, except where a PDK gives two drives the same area; then
// the leakage order among equal-area cells separates them, identically in
// every threshold library.
void check_drive_keys_separate_equal_area_cells()
{
  const std::vector<FamilyCell> family{
      {"hvt", 10.08, 1.0},  // ADDFCINX0P5 equivalent
      {"hvt", 10.08, 2.0},  // ADDFCINX1: same area, leakier
      {"lvt", 10.08, 5.0},
      {"lvt", 10.08, 9.0},
      {"hvt", 20.00, 3.0},  // a genuinely larger drive
  };
  const std::vector<DriveKey> keys = derive_drive_keys(family);

  require(!(keys[0] == keys[1]), "equal-area drives get distinct keys");
  require(keys[0] == keys[2], "the weaker drive pairs across thresholds");
  require(keys[1] == keys[3], "the stronger drive pairs across thresholds");
  require(!(keys[0] == keys[4]), "a different area is a different drive");
  require(keys[0] < keys[1], "weaker drives sort first");
  require(keys[1] < keys[4], "smaller areas sort before larger ones");
}

void check_drive_keys_are_stable_for_equal_leakage()
{
  const std::vector<FamilyCell> family{
      {"hvt", 5.0, 1.0},
      {"hvt", 5.0, 1.0},
  };
  const std::vector<DriveKey> keys = derive_drive_keys(family);
  require(keys[0].rank == 0 && keys[1].rank == 1,
          "indistinguishable cells keep their input order");
}
}  // namespace

int main()
{
  check_vt_axis_follows_leakage();
  check_single_library_is_single_vt();
  check_too_many_libraries_is_an_error();
  check_unorderable_libraries_are_an_error();
  check_drive_keys_separate_equal_area_cells();
  check_drive_keys_are_stable_for_equal_leakage();
  if (failures != 0) {
    std::cerr << failures << " check(s) failed" << std::endl;
    return 1;
  }
  std::cout << "vt_groups: all checks passed" << std::endl;
  return 0;
}
