// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include <iostream>
#include <string>
#include <vector>

#include "design_edit.h"
#include "odb/db.h"
#include "utl/Logger.h"

namespace {

bool expect(bool condition, const std::string& message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
  }
  return condition;
}

odb::dbMaster* makeMaster(odb::dbLib* lib, const char* name)
{
  odb::dbMaster* master = odb::dbMaster::create(lib, name);
  master->setWidth(100);
  master->setHeight(100);
  master->setType(odb::dbMasterType::CORE);
  odb::dbMTerm::create(
      master, "A", odb::dbIoType::INPUT, odb::dbSigType::SIGNAL);
  odb::dbMTerm::create(
      master, "Y", odb::dbIoType::OUTPUT, odb::dbSigType::SIGNAL);
  master->setFrozen();
  return master;
}

struct Fixture
{
  utl::Logger* logger = utl::Logger::defaultLogger();
  odb::dbDatabase* db = odb::dbDatabase::create();
  odb::dbTech* tech = nullptr;
  odb::dbLib* lib = nullptr;
  odb::dbMaster* master = nullptr;
  odb::dbChip* chip = nullptr;
  odb::dbBlock* block = nullptr;

  Fixture()
  {
    db->setLogger(logger);
    tech = odb::dbTech::create(db, "tech");
    lib = odb::dbLib::create(db, "lib", tech, '/');
    master = makeMaster(lib, "BUF");
    chip = odb::dbChip::create(db, tech);
    block = odb::dbBlock::create(chip, "top");
    block->setCoreArea(odb::Rect(0, 0, 1000, 1000));
  }

  ~Fixture() { odb::dbDatabase::destroy(db); }

  odb::dbInst* makeInst(const char* name, int x, int y)
  {
    odb::dbInst* inst = odb::dbInst::create(block, master, name);
    inst->setLocation(x, y);
    inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    return inst;
  }

  odb::dbNet* makeDrivenNet(const char* name,
                            odb::dbInst* driver,
                            const std::vector<odb::dbInst*>& loads)
  {
    odb::dbNet* net = odb::dbNet::create(block, name);
    driver->findITerm("Y")->connect(net);
    for (odb::dbInst* load : loads) {
      load->findITerm("A")->connect(net);
    }
    return net;
  }
};

bool testAfterDriver()
{
  Fixture fixture;
  odb::dbInst* driver = fixture.makeInst("driver", 100, 100);
  odb::dbInst* load = fixture.makeInst("load", 700, 700);
  odb::dbNet* net = fixture.makeDrivenNet("n1", driver, {load});

  ecc::BufferInsertionOptions options;
  options.location = ecc::BufferLocation{980, 980};
  options.buffer_name_prefix = "sizer_buf";
  options.net_name_prefix = "sizer_net";
  const ecc::BufferInsertionResult result
      = ecc::OpenDbBufferInserter::insertBufferAfterDriver(
          net, fixture.master, options);

  bool passed = true;
  passed &= expect(result.ok(), "after-driver insertion succeeds");
  passed &= expect(result.buffer != nullptr, "buffer instance is returned");
  passed &= expect(result.buffer->getSourceType() == odb::dbSourceType::TIMING,
                   "inserted buffer is marked as a timing source");
  passed &= expect(result.input_net != net,
                   "driver side receives a new input net");
  passed &= expect(result.output_net == net,
                   "load side preserves the original net");
  passed &= expect(driver->findITerm("Y")->getNet() == result.input_net,
                   "driver is rewired to the new input net");
  passed &= expect(load->findITerm("A")->getNet() == net,
                   "load remains on the original net");
  passed &= expect(result.affected_nets.size() == 2,
                   "both changed nets are reported");
  int x = 0;
  int y = 0;
  result.buffer->getLocation(x, y);
  passed &= expect(x == 900 && y == 900,
                   "explicit location is clamped inside the core");
  return passed;
}

bool testLargeFanout()
{
  Fixture fixture;
  odb::dbInst* driver = fixture.makeInst("driver", 100, 100);
  std::vector<odb::dbInst*> load_instances;
  std::vector<odb::dbObject*> load_pins;
  constexpr int kLoadCount = 256;
  load_instances.reserve(kLoadCount);
  load_pins.reserve(kLoadCount);
  for (int index = 0; index < kLoadCount; ++index) {
    const std::string name = "load_" + std::to_string(index);
    odb::dbInst* load = fixture.makeInst(name.c_str(), 200 + index, 300);
    load_instances.push_back(load);
    load_pins.push_back(load->findITerm("A"));
  }
  odb::dbNet* net
      = fixture.makeDrivenNet("large_fanout", driver, load_instances);
  ecc::BufferInsertionOptions options;
  options.location = ecc::BufferLocation{500, 300};

  const ecc::BufferInsertionResult result
      = ecc::OpenDbBufferInserter::insertBufferBeforeLoads(
          net, load_pins, fixture.master, options);

  bool passed = expect(result.ok(), "large-fanout insertion succeeds");
  for (odb::dbInst* load : load_instances) {
    passed &= expect(load->findITerm("A")->getNet() == result.output_net,
                     "large-fanout load is rewired");
  }
  passed &= expect(result.affected_nets.size() == 2,
                   "large-fanout insertion reports two affected nets");
  return passed;
}

bool testBeforeLoad()
{
  Fixture fixture;
  odb::dbInst* driver = fixture.makeInst("driver", 100, 100);
  odb::dbInst* load1 = fixture.makeInst("load1", 600, 500);
  odb::dbInst* load2 = fixture.makeInst("load2", 700, 500);
  odb::dbNet* net = fixture.makeDrivenNet("n1", driver, {load1, load2});

  ecc::BufferInsertionOptions options;
  options.location = ecc::BufferLocation{600, 500};
  const ecc::BufferInsertionResult result
      = ecc::OpenDbBufferInserter::insertBufferBeforeLoad(
          load1->findITerm("A"), fixture.master, options);

  bool passed = true;
  passed &= expect(result.ok(), "before-load insertion succeeds");
  passed &= expect(result.input_net == net,
                   "buffer input stays on the original net");
  passed
      &= expect(result.output_net != net, "buffer output receives a new net");
  passed &= expect(load1->findITerm("A")->getNet() == result.output_net,
                   "selected load moves to the buffer output net");
  passed &= expect(load2->findITerm("A")->getNet() == net,
                   "unselected load stays on the original net");
  return passed;
}

bool testBeforeLoads()
{
  Fixture fixture;
  odb::dbInst* driver = fixture.makeInst("driver", 100, 100);
  odb::dbInst* load1 = fixture.makeInst("load1", 500, 400);
  odb::dbInst* load2 = fixture.makeInst("load2", 600, 400);
  odb::dbInst* load3 = fixture.makeInst("load3", 700, 400);
  odb::dbNet* net = fixture.makeDrivenNet("n1", driver, {load1, load2, load3});
  std::vector<odb::dbObject*> loads{load1->findITerm("A"),
                                    load2->findITerm("A")};

  ecc::BufferInsertionOptions options;
  options.location = ecc::BufferLocation{550, 400};
  const ecc::BufferInsertionResult result
      = ecc::OpenDbBufferInserter::insertBufferBeforeLoads(
          net, loads, fixture.master, options);

  bool passed = true;
  passed &= expect(result.ok(), "before-loads insertion succeeds");
  passed &= expect(load1->findITerm("A")->getNet() == result.output_net,
                   "first selected load moves to the output net");
  passed &= expect(load2->findITerm("A")->getNet() == result.output_net,
                   "second selected load moves to the output net");
  passed &= expect(load3->findITerm("A")->getNet() == net,
                   "unselected load remains on the original net");
  return passed;
}

bool testTopLevelPorts()
{
  Fixture fixture;
  odb::dbNet* net = odb::dbNet::create(fixture.block, "io_net");
  odb::dbBTerm* input = odb::dbBTerm::create(net, "input");
  input->setIoType(odb::dbIoType::INPUT);
  odb::dbBTerm* output = odb::dbBTerm::create(net, "output");
  output->setIoType(odb::dbIoType::OUTPUT);

  const ecc::BufferInsertionResult after_driver
      = ecc::OpenDbBufferInserter::insertBufferAfterDriver(net, fixture.master);
  bool passed = true;
  passed &= expect(after_driver.ok(),
                   "top-level input can drive an inserted buffer");
  passed &= expect(input->getNet() == after_driver.input_net,
                   "top-level input moves to the buffer input net");
  passed &= expect(after_driver.output_net == net,
                   "top-level output stays on the original net");

  const ecc::BufferInsertionResult before_load
      = ecc::OpenDbBufferInserter::insertBufferBeforeLoad(output,
                                                          fixture.master);
  passed &= expect(before_load.ok(),
                   "buffer can be inserted before a top-level output");
  passed &= expect(before_load.input_net == net,
                   "top-level output buffer is driven by the original net");
  passed &= expect(output->getNet() == before_load.output_net,
                   "top-level output moves to the buffer output net");
  return passed;
}

bool testValidationDoesNotMutate()
{
  Fixture fixture;
  odb::dbInst* load = fixture.makeInst("load", 500, 500);
  odb::dbNet* undriven = odb::dbNet::create(fixture.block, "undriven");
  load->findITerm("A")->connect(undriven);
  odb::dbInst* other_load = fixture.makeInst("other_load", 600, 500);
  odb::dbNet* other_net = odb::dbNet::create(fixture.block, "other");
  other_load->findITerm("A")->connect(other_net);
  const size_t instance_count = fixture.block->getInsts().size();

  const ecc::BufferInsertionResult missing_driver
      = ecc::OpenDbBufferInserter::insertBufferAfterDriver(undriven,
                                                           fixture.master);
  const ecc::BufferInsertionResult invalid_load
      = ecc::OpenDbBufferInserter::insertBufferBeforeLoad(load->findITerm("Y"),
                                                          fixture.master);
  const std::vector<odb::dbObject*> different_net_loads{
      load->findITerm("A"), other_load->findITerm("A")};
  const ecc::BufferInsertionResult different_nets
      = ecc::OpenDbBufferInserter::insertBufferBeforeLoads(
          undriven, different_net_loads, fixture.master);

  bool passed = true;
  passed &= expect(
      missing_driver.status == ecc::BufferInsertionStatus::kMissingDriver,
      "missing driver is reported");
  passed
      &= expect(invalid_load.status == ecc::BufferInsertionStatus::kInvalidLoad,
                "an unconnected output is rejected");
  passed &= expect(different_nets.status
                       == ecc::BufferInsertionStatus::kLoadsOnDifferentNets,
                   "loads on different nets are rejected by default");
  passed &= expect(fixture.block->getInsts().size() == instance_count,
                   "validation failures do not create instances");
  return passed;
}

}  // namespace

int main()
{
  const bool passed = testAfterDriver() && testBeforeLoad() && testBeforeLoads()
                      && testLargeFanout() && testTopLevelPorts()
                      && testValidationDoesNotMutate();
  return passed ? 0 : 1;
}
