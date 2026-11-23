// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "algorithms/repair_drv.h"
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

struct Fixture
{
  utl::Logger* logger = utl::Logger::defaultLogger();
  odb::dbDatabase* db = odb::dbDatabase::create();
  odb::dbTech* tech = nullptr;
  odb::dbLib* lib = nullptr;
  odb::dbMaster* master = nullptr;
  odb::dbChip* chip = nullptr;
  odb::dbBlock* block = nullptr;
  odb::dbNet* net = nullptr;
  std::vector<odb::dbInst*> loads;

  explicit Fixture(unsigned load_count)
  {
    db->setLogger(logger);
    tech = odb::dbTech::create(db, "tech");
    lib = odb::dbLib::create(db, "lib", tech, '/');
    master = odb::dbMaster::create(lib, "BUF");
    master->setWidth(100);
    master->setHeight(100);
    master->setType(odb::dbMasterType::CORE);
    odb::dbMTerm::create(
        master, "A", odb::dbIoType::INPUT, odb::dbSigType::SIGNAL);
    odb::dbMTerm::create(
        master, "Y", odb::dbIoType::OUTPUT, odb::dbSigType::SIGNAL);
    master->setFrozen();

    chip = odb::dbChip::create(db, tech);
    block = odb::dbBlock::create(chip, "top");
    block->setCoreArea(odb::Rect(0, 0, 10000, 10000));
    odb::dbInst* driver = makeInst("driver", 100, 100);
    net = odb::dbNet::create(block, "high_fanout");
    driver->findITerm("Y")->connect(net);
    for (unsigned index = 0; index < load_count; ++index) {
      const std::string name = "load_" + std::to_string(index);
      odb::dbInst* load = makeInst(name.c_str(), 1000 + 200 * index, 1000);
      load->findITerm("A")->connect(net);
      loads.push_back(load);
    }
  }

  ~Fixture() { odb::dbDatabase::destroy(db); }

  odb::dbInst* makeInst(const char* name, int x, int y)
  {
    odb::dbInst* inst = odb::dbInst::create(block, master, name);
    inst->setLocation(x, y);
    inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    return inst;
  }
};

bool testBuildsBoundedTree()
{
  Fixture fixture(10);
  ecc::FanoutRepairOptions options;
  options.max_fanout = 3;
  options.insertion.location = ecc::BufferLocation{5000, 5000};
  options.insertion.buffer_name_prefix = "sizer_fanout_buf";
  options.insertion.net_name_prefix = "sizer_fanout_net";

  const ecc::FanoutRepairResult result = ecc::OpenDbFanoutRepairer::repair(
      fixture.block, fixture.master, options);

  bool passed = true;
  passed &= expect(result.ok(), "fanout repair succeeds");
  passed &= expect(result.before_max_fanout == 10,
                   "original maximum fanout is reported");
  passed &= expect(result.after_max_fanout == 3,
                   "repaired maximum fanout is bounded");
  passed &= expect(result.violating_nets == 1,
                   "one original violating net is reported");
  passed &= expect(result.inserted_buffers.size() == 4,
                   "a minimum-size two-level buffer tree is inserted");
  passed &= expect(ecc::OpenDbFanoutRepairer::loadCount(fixture.net) == 3,
                   "the root net obeys the requested fanout");
  for (odb::dbNet* net : fixture.block->getNets()) {
    passed &= expect(ecc::OpenDbFanoutRepairer::loadCount(net) <= 3,
                     "every resulting net obeys the fanout bound");
  }
  for (odb::dbInst* load : fixture.loads) {
    passed &= expect(load->findITerm("A")->getNet() != nullptr,
                     "each original load remains connected");
  }
  return passed;
}

bool testFailureRollsBack()
{
  Fixture fixture(10);
  const size_t original_instances = fixture.block->getInsts().size();
  const size_t original_nets = fixture.block->getNets().size();
  ecc::FanoutRepairOptions options;
  options.max_fanout = 3;
  options.max_levels = 1;
  options.insertion.location = ecc::BufferLocation{5000, 5000};

  const ecc::FanoutRepairResult result = ecc::OpenDbFanoutRepairer::repair(
      fixture.block, fixture.master, options);

  bool passed = true;
  passed &= expect(result.status == ecc::FanoutRepairStatus::kMaxLevelsExceeded,
                   "tree depth failure is reported");
  passed &= expect(result.inserted_buffers.empty(),
                   "rolled-back result has no dangling instance pointers");
  passed &= expect(fixture.block->getInsts().size() == original_instances,
                   "rollback restores the instance count");
  passed &= expect(fixture.block->getNets().size() == original_nets,
                   "rollback restores the net count");
  passed &= expect(ecc::OpenDbFanoutRepairer::loadCount(fixture.net) == 10,
                   "rollback restores the original connectivity");
  return passed;
}

bool testLargeFanoutUsesMinimumBuffers()
{
  // This exceeds K^2 so a level-by-level tree builder would over-insert.
  constexpr unsigned kLoadCount = 5000;
  constexpr unsigned kMaxFanout = 64;
  Fixture fixture(kLoadCount);
  ecc::FanoutRepairOptions options;
  options.max_fanout = kMaxFanout;
  options.insertion.location = ecc::BufferLocation{5000, 5000};

  const auto begin = std::chrono::steady_clock::now();
  const ecc::FanoutRepairResult result = ecc::OpenDbFanoutRepairer::repair(
      fixture.block, fixture.master, options);
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - begin);

  const unsigned minimum_buffers
      = (kLoadCount - kMaxFanout + (kMaxFanout - 2)) / (kMaxFanout - 1);
  bool passed = true;
  passed &= expect(result.ok(), "large fanout repair succeeds");
  passed &= expect(result.inserted_buffers.size() == minimum_buffers,
                   "large fanout repair uses the minimum buffer count");
  passed &= expect(result.after_max_fanout == kMaxFanout,
                   "large fanout repair reaches the requested bound");
  std::cout << "large fanout repair: " << kLoadCount << " loads, "
            << result.inserted_buffers.size() << " buffers, " << elapsed.count()
            << " ms\n";
  return passed;
}

bool testClockPolicy()
{
  Fixture fixture(5);
  fixture.net->setSigType(odb::dbSigType::CLOCK);
  ecc::FanoutRepairOptions options;
  options.max_fanout = 2;
  options.insertion.location = ecc::BufferLocation{5000, 5000};

  const ecc::FanoutRepairResult skipped = ecc::OpenDbFanoutRepairer::repair(
      fixture.block, fixture.master, options);
  bool passed = expect(skipped.ok() && skipped.inserted_buffers.empty(),
                       "clock nets are skipped by default");

  fixture.net->setSigType(odb::dbSigType::SIGNAL);
  options.excluded_nets.push_back(fixture.net);
  const ecc::FanoutRepairResult explicitly_excluded
      = ecc::OpenDbFanoutRepairer::repair(
          fixture.block, fixture.master, options);
  passed &= expect(
      explicitly_excluded.ok() && explicitly_excluded.inserted_buffers.empty(),
      "caller-identified clock nets can be excluded");

  options.excluded_nets.clear();
  options.repair_clock_nets = true;
  const ecc::FanoutRepairResult repaired = ecc::OpenDbFanoutRepairer::repair(
      fixture.block, fixture.master, options);
  passed &= expect(repaired.ok() && repaired.after_max_fanout <= 2,
                   "clock nets can be repaired explicitly");
  return passed;
}

}  // namespace

int main()
{
  const bool passed = testBuildsBoundedTree() && testFailureRollsBack()
                      && testLargeFanoutUsesMinimumBuffers()
                      && testClockPolicy();
  return passed ? 0 : 1;
}
