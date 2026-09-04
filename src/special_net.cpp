// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include "special_net.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "odb/db.h"

namespace ecc {

int connectMissingWildcardSpecialNetPins(odb::dbBlock* block)
{
    // DEF wildcard connections cover only the instances present while reading.
    // Extend them to later insertions without overriding an explicit connection.
    std::unordered_map<std::string, std::vector<odb::dbITerm*>>
        iterms_by_mterm_name;
    for(odb::dbInst* inst : block->getInsts()) {
        for(odb::dbITerm* iterm : inst->getITerms()) {
            iterms_by_mterm_name[iterm->getMTerm()->getName()].push_back(iterm);
        }
    }

    int connected = 0;
    for(odb::dbNet* net : block->getNets()) {
        if(!net->isSpecial() || !net->isWildConnected() ||
           net->isDoNotTouch()) {
            continue;
        }

        std::unordered_set<std::string> wildcard_mterm_names;
        for(odb::dbITerm* iterm : net->getITerms()) {
            if(iterm->isSpecial()) {
                wildcard_mterm_names.insert(iterm->getMTerm()->getName());
            }
        }

        for(const std::string& mterm_name : wildcard_mterm_names) {
            const auto found = iterms_by_mterm_name.find(mterm_name);
            if(found == iterms_by_mterm_name.end()) {
                continue;
            }

            bool has_conflict = false;
            for(odb::dbITerm* iterm : found->second) {
                odb::dbNet* current_net = iterm->getNet();
                if((current_net != nullptr && current_net != net) ||
                   (current_net == nullptr && iterm->getInst()->isDoNotTouch())) {
                    has_conflict = true;
                    break;
                }
            }
            if(has_conflict) {
                continue;
            }

            for(odb::dbITerm* iterm : found->second) {
                if(iterm->getNet() == nullptr) {
                    iterm->connect(net);
                    iterm->setSpecial();
                    connected++;
                }
            }
        }
    }

    return connected;
}

}  // namespace ecc
