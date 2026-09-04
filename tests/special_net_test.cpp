// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

// Regression for preserving wildcard special nets after buffer insertion.
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "odb/db.h"
#include "odb/defout.h"
#include "special_net.h"
#include "utl/Logger.h"

namespace {

bool expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

odb::dbMaster* makeBufferMaster(odb::dbLib* lib)
{
    odb::dbMaster* master = odb::dbMaster::create(lib, "BUF");
    master->setWidth(100);
    master->setHeight(100);
    master->setType(odb::dbMasterType::CORE);
    odb::dbMTerm::create(
        master, "A", odb::dbIoType::INPUT, odb::dbSigType::SIGNAL);
    odb::dbMTerm::create(
        master, "Y", odb::dbIoType::OUTPUT, odb::dbSigType::SIGNAL);
    odb::dbMTerm::create(
        master, "VDD", odb::dbIoType::INOUT, odb::dbSigType::POWER);
    odb::dbMTerm::create(
        master, "VSS", odb::dbIoType::INOUT, odb::dbSigType::GROUND);
    master->setFrozen();
    return master;
}

odb::dbNet* makeWildcardSpecialNet(odb::dbBlock* block,
                                   odb::dbInst* existing_inst,
                                   const char* net_name,
                                   odb::dbSigType sig_type)
{
    odb::dbNet* net = odb::dbNet::create(block, net_name);
    net->setSpecial();
    net->setSigType(sig_type);
    net->setWildConnected();
    odb::dbITerm* iterm = existing_inst->findITerm(net_name);
    iterm->connect(net);
    iterm->setSpecial();
    return net;
}

}  // namespace

int main()
{
    utl::Logger* logger = utl::Logger::defaultLogger();
    odb::dbDatabase* db = odb::dbDatabase::create();
    db->setLogger(logger);

    odb::dbTech* tech = odb::dbTech::create(db, "tech");
    odb::dbLib* lib = odb::dbLib::create(db, "lib", tech, '/');
    odb::dbMaster* master = makeBufferMaster(lib);
    odb::dbChip* chip = odb::dbChip::create(db, tech);
    odb::dbBlock* block = odb::dbBlock::create(chip, "top");

    odb::dbInst* existing_inst
        = odb::dbInst::create(block, master, "existing_buf");
    odb::dbNet* vdd = makeWildcardSpecialNet(
        block, existing_inst, "VDD", odb::dbSigType::POWER);
    odb::dbNet* vss = makeWildcardSpecialNet(
        block, existing_inst, "VSS", odb::dbSigType::GROUND);

    odb::dbInst* inserted_inst
        = odb::dbInst::create(block, master, "load_slew1");

    bool passed = true;
    passed &= expect(inserted_inst->findITerm("VDD")->getNet() == nullptr,
                     "test precondition: inserted VDD starts unconnected");
    passed &= expect(inserted_inst->findITerm("VSS")->getNet() == nullptr,
                     "test precondition: inserted VSS starts unconnected");

    const int connected = ecc::connectMissingWildcardSpecialNetPins(block);
    passed &= expect(connected == 2, "exactly two missing PG pins are connected");
    passed &= expect(inserted_inst->findITerm("VDD")->getNet() == vdd,
                     "inserted VDD is connected to the wildcard VDD net");
    passed &= expect(inserted_inst->findITerm("VSS")->getNet() == vss,
                     "inserted VSS is connected to the wildcard VSS net");
    passed &= expect(inserted_inst->findITerm("VDD")->isSpecial(),
                     "inserted VDD is marked special");
    passed &= expect(inserted_inst->findITerm("VSS")->isSpecial(),
                     "inserted VSS is marked special");

    const std::filesystem::path output_path
        = std::filesystem::temp_directory_path()
          / "ecc_sizer_special_net_test.def";
    odb::DefOut writer(logger);
    passed &= expect(writer.writeBlock(block, output_path.c_str()),
                     "DEF writer succeeds");

    std::ifstream output_file(output_path);
    std::ostringstream output;
    output << output_file.rdbuf();
    const std::string def = output.str();
    passed &= expect(def.find("( * VDD )") != std::string::npos,
                     "VDD remains a wildcard SPECIALNET connection");
    passed &= expect(def.find("( * VSS )") != std::string::npos,
                     "VSS remains a wildcard SPECIALNET connection");
    passed &= expect(def.find("( existing_buf VDD )") == std::string::npos,
                     "VDD is not expanded into explicit instances");
    passed &= expect(def.find("( existing_buf VSS )") == std::string::npos,
                     "VSS is not expanded into explicit instances");

    passed &= expect(ecc::connectMissingWildcardSpecialNetPins(block) == 0,
                     "repeating the repair is idempotent");

    odb::dbNet* switched_vdd = odb::dbNet::create(block, "VDD_SWITCHED");
    odb::dbInst* switched_domain_inst
        = odb::dbInst::create(block, master, "switched_domain_buf");
    switched_domain_inst->findITerm("VDD")->connect(switched_vdd);
    odb::dbInst* later_inst
        = odb::dbInst::create(block, master, "load_slew2");

    passed &= expect(ecc::connectMissingWildcardSpecialNetPins(block) == 2,
                     "only the two non-conflicting VSS pins are connected");
    passed &= expect(switched_domain_inst->findITerm("VDD")->getNet()
                         == switched_vdd,
                     "an explicit VDD connection is not overridden");
    passed &= expect(later_inst->findITerm("VDD")->getNet() == nullptr,
                     "an ambiguous unconnected VDD pin is left unchanged");
    passed &= expect(later_inst->findITerm("VSS")->getNet() == vss,
                     "an independent non-conflicting VSS pin is connected");

    std::filesystem::remove(output_path);
    odb::dbDatabase::destroy(db);
    return passed ? 0 : 1;
}
