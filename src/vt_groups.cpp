// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "vt_groups.h"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <tuple>

int VtGroups::vt_of(const std::string& libname) const {
    const auto it = rank.find(libname);
    return it == rank.end() ? 0 : it->second;
}

std::vector< DriveKey > derive_drive_keys(
    const std::vector< FamilyCell >& family) {
    // Group by (library, area): the cells that area alone cannot separate.
    std::map< std::pair< std::string, double >, std::vector< std::size_t > >
        buckets;
    for(std::size_t index = 0; index < family.size(); ++index) {
        buckets[{family[index].libname, family[index].area}].push_back(index);
    }

    std::vector< DriveKey > keys(family.size());
    for(auto& [footprint, members] : buckets) {
        // Ascending leakage is ascending transistor width, hence ascending
        // drive. Equal leakage keeps the input order so the result is stable.
        std::stable_sort(members.begin(), members.end(),
                         [&](std::size_t lhs, std::size_t rhs) {
                             return family[lhs].leakage < family[rhs].leakage;
                         });
        for(std::size_t position = 0; position < members.size(); ++position) {
            keys[members[position]].area = footprint.second;
            keys[members[position]].rank = static_cast< int >(position);
        }
    }
    return keys;
}

VtGroups derive_vt_groups(
    const std::vector< std::pair< std::string, double > >& cell_library_leakage,
    int max_groups) {
    // Mean over the cells that report leakage. Libraries hold the same cell
    // set across thresholds, so their means are directly comparable; cells
    // without a leakage figure are skipped rather than counted as zero, which
    // would drag a library's mean toward another's.
    std::map< std::string, std::pair< double, long > > totals;
    for(const auto& [libname, leakage] : cell_library_leakage) {
        if(libname.empty()) {
            continue;
        }
        auto& entry = totals[libname];
        if(leakage > 0.0) {
            entry.first += leakage;
            entry.second += 1;
        }
    }

    VtGroups groups;
    if(totals.empty()) {
        return groups;
    }
    if(totals.size() == 1) {
        groups.rank.emplace(totals.begin()->first, 0);
        groups.count = 1;
        return groups;
    }
    if(static_cast< int >(totals.size()) > max_groups) {
        throw std::runtime_error(
            "Liberty declares more threshold-voltage libraries than the sizer "
            "can represent; reduce the library set to at most " +
            std::to_string(max_groups));
    }

    std::vector< std::tuple< double, std::string > > ordered;
    for(const auto& [libname, sum_count] : totals) {
        if(sum_count.second == 0) {
            throw std::runtime_error(
                "Liberty library '" + libname +
                "' reports no cell leakage, so its threshold voltage cannot be "
                "ordered against the other libraries");
        }
        ordered.emplace_back(sum_count.first / sum_count.second, libname);
    }
    // Ascending leakage is slowest-first. Ties break on the library name so a
    // given Liberty set always produces the same axis.
    std::sort(ordered.begin(), ordered.end());
    for(std::size_t index = 0; index + 1 < ordered.size(); ++index) {
        if(std::get< 0 >(ordered[index]) == std::get< 0 >(ordered[index + 1])) {
            throw std::runtime_error(
                "Liberty libraries '" + std::get< 1 >(ordered[index]) +
                "' and '" + std::get< 1 >(ordered[index + 1]) +
                "' report equal mean leakage, so their threshold voltages "
                "cannot be ordered");
        }
    }
    for(std::size_t index = 0; index < ordered.size(); ++index) {
        groups.rank.emplace(std::get< 1 >(ordered[index]),
                            static_cast< int >(index));
    }
    groups.count = static_cast< int >(ordered.size());
    return groups;
}
