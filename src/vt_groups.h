// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

// Threshold-voltage grouping derived from the loaded Liberty.
//
// A PDK ships one Liberty per threshold flavour, so a cell's source library
// identifies its VT without reading the cell's name. A higher threshold leaks
// less and switches slower, so ordering those libraries by ascending leakage
// orders them slowest-first -- the order cell_vtypes {s, m, f} already uses.
struct VtGroups {
    // Source library name -> VT index in [0, count).
    std::map< std::string, int > rank;
    int count = 1;

    // VT index of a cell from *libname*; 0 when the library is unknown, which
    // is the single-VT answer and the enum's slowest slot.
    int vt_of(const std::string& libname) const;
};

// A cell's drive identity inside one equivalence family. Cells that are the
// same drive strength at different thresholds share it, so it indexes the
// size rows of a VT/size table.
//
// Area alone is not enough: a PDK may give two drives of one function the
// same footprint and area (ICS55 v2 does so for 111 of its families), and
// only their timing separates them. Within a single library those cells are
// ordered by leakage, which rises with transistor width, and that order is
// identical in every threshold library of the set -- so area plus the
// position among equal-area cells names a drive without reading any name.
struct DriveKey {
    double area = 0.0;
    int rank = 0;

    bool operator<(const DriveKey& other) const {
        return area != other.area ? area < other.area : rank < other.rank;
    }
    bool operator==(const DriveKey& other) const {
        return area == other.area && rank == other.rank;
    }
};

// One cell of a family, as the drive derivation sees it.
struct FamilyCell {
    std::string libname;
    double area = 0.0;
    double leakage = 0.0;
};

// DriveKey per entry of *family*, positionally.
std::vector< DriveKey > derive_drive_keys(
    const std::vector< FamilyCell >& family);

// Build the grouping from one (library, leakage) observation per cell.
//
// *max_groups* caps the axis because cell_vtypes cannot represent more; a
// Liberty set with more threshold libraries than that is an error rather than
// a silent truncation. Multi-library sets whose leakage cannot order them are
// also an error: guessing the order would mis-rank every retype decision.
VtGroups derive_vt_groups(
    const std::vector< std::pair< std::string, double > >& cell_library_leakage,
    int max_groups);
