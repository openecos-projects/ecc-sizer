// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <tuple>

#include "buffer_inserter.h"
#include "repair_fanout.h"
#include "repair_design.h"
#include "repair_gbi.h"
#include "repair_timing.h"
#include "repair_load.h"
#include "repair_power.h"
#include "sta/Sdc.hh"
#include "sta/Mode.hh"
#include "repair_placement.h"
#include "sta/EquivCells.hh"
#include <stdexcept>
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbReadVerilog.hh"
#include "est/EstimateParasitics.h"
#include "gtest/gtest.h"
#include "odb/db.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/Search.hh"
#include "sta/VerilogReader.hh"
#include "sta/VerilogWriter.hh"
#include "tst/IntegratedFixture.h"

namespace {

class BufferInserterStaTest : public tst::IntegratedFixture
{
 protected:
  BufferInserterStaTest()
      : tst::IntegratedFixture(tst::IntegratedFixture::Technology::kNangate45,
                               "_main/src/est/test/")
  {
  }

  void readTestDesign(const char* file_name = "buffer_inserter_sta.v")
  {
    ord::dbVerilogNetwork verilog_network(sta_.get());
    sta::VerilogReader verilog_reader(&verilog_network);
    const std::string path
        = std::string(ECC_SIZER_TEST_DATA_DIR) + "/" + file_name;
    verilog_reader.read(path.c_str());

    ord::dbLinkDesign("top", &verilog_network, db_.get(), &logger_, true);
    sta_->postReadDb(db_.get());

    block_ = db_->getChip()->getBlock();
    ASSERT_NE(block_, nullptr);
    block_->setDefUnits(lib_->getTech()->getLefUnits());
    block_->setDieArea(odb::Rect(0, 0, 10000, 10000));
    block_->setCoreArea(odb::Rect(0, 0, 10000, 10000));

    int x = 1000;
    for (odb::dbInst* inst : block_->getInsts()) {
      inst->setLocation(x, 1000);
      inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
      x += 2000;
    }
    sta_->postReadDef(block_);
    for (sta::Scene* scene : sta_->scenes()) {
      ep_.setHWireSignalRC(scene, 1.0e6, 1.0e-10);
      ep_.setVWireSignalRC(scene, 1.0e6, 1.0e-10);
    }
    initStaDefaultSdc();
    sta_->updateTiming(true);
    ASSERT_TRUE(sta_->search()->arrivalsValid());
  }

  void readVtDesign()
  {
    for (const auto* suffix : {"H", "L", "B", "X"})
      readLiberty(std::string(ECC_SIZER_TEST_DATA_DIR) + "/repair_vt_" + suffix + ".lib");
    ASSERT_TRUE(updateLib(lib_, std::string(ECC_SIZER_TEST_DATA_DIR) + "/repair_vt.lef"));
    readTestDesign("repair_vt.v");
    addLegalRows();
    int y = 1000;
    for (auto* term : block_->getBTerms()) {
      auto* pin = odb::dbBPin::create(term);
      odb::dbBox::create(pin, block_->getTech()->findLayer("metal2"), 20000, y, 20100, y + 100);
      pin->setPlacementStatus(odb::dbPlacementStatus::PLACED);
      y += 1000;
    }
    refreshRepairTiming();
  }

  void addLegalRows()
  {
    auto* site = lib_->findMaster("BUF_X1")->getSite();
    ASSERT_NE(site, nullptr);
    const int w = site->getWidth(), h = site->getHeight();
    block_->setDieArea(odb::Rect(0, 0, 200 * w, 20 * h));
    block_->setCoreArea(odb::Rect(0, 0, 200 * w, 20 * h));
    for (int i = 0; i < 20; ++i) {
      const auto name = "repair_row_" + std::to_string(i);
      odb::dbRow::create(block_,
                         name.c_str(),
                         site,
                         0,
                         i * h,
                         i % 2 ? odb::dbOrientType::MX : odb::dbOrientType::R0,
                         odb::dbRowDir::HORIZONTAL,
                         200,
                         w);
    }
    int x = 0;
    for (auto* inst : block_->getInsts()) {
      inst->setOrient(odb::dbOrientType::R0);
      inst->setLocation(x, 0);
      x += 6 * w;
    }
    ecc::legalizeRepairPlacement(block_, &dp_);
  }

  void constrainGateCap()
  {
    auto* gate = db_network_->libertyCell(block_->findInst("driver"));
    auto* load = db_network_->libertyCell(block_->findInst("load0"));
    gate->findLibertyPort("ZN")->setCapacitanceLimit(
        3 * load->findLibertyPort("A")->capacitance(), sta::MinMax::max());
  }

  void constrainSetup(bool only_out0 = false)
  {
    auto* sdc = sta_->cmdMode()->sdc();
    for (auto* port : block_->getBTerms()) {
      if (port->getIoType() != odb::dbIoType::OUTPUT
          || (only_out0 && port->getName() != "out0"))
        continue;
      sta_->setOutputDelay(db_network_->dbToSta(port),
                           sta::RiseFallBoth::riseFall(),
                           sdc->findClock("clk"),
                           sta::RiseFall::rise(),
                           nullptr,
                           false,
                           false,
                           sta::MinMaxAll::max(),
                           false,
                           0.49e-9f,
                           sdc);
    }
  }

  void refreshRepairTiming()
  {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  }

  void oversizeLoads(bool keep_critical_small = false)
  {
    for (unsigned i = keep_critical_small ? 1 : 0; i < 10; ++i) {
      auto* load = block_->findInst(("load" + std::to_string(i)).c_str());
      ASSERT_TRUE(load->swapMaster(lib_->findMaster("BUF_X8")));
    }
  }

  void verifyStaState(const ecc::BufferInsertionResult& result)
  {
    ASSERT_TRUE(result.ok()) << result.message;
    ASSERT_NE(result.buffer, nullptr);
    ASSERT_NE(result.input_pin, nullptr);
    ASSERT_NE(result.output_pin, nullptr);
    ASSERT_NE(result.input_net, nullptr);
    ASSERT_NE(result.output_net, nullptr);

    sta::Instance* sta_buffer = db_network_->dbToSta(result.buffer);
    sta::Pin* sta_input = db_network_->dbToSta(result.input_pin);
    sta::Pin* sta_output = db_network_->dbToSta(result.output_pin);
    sta::Net* sta_input_net = db_network_->dbToSta(result.input_net);
    sta::Net* sta_output_net = db_network_->dbToSta(result.output_net);
    ASSERT_NE(sta_buffer, nullptr);
    ASSERT_NE(sta_input, nullptr);
    ASSERT_NE(sta_output, nullptr);
    ASSERT_NE(sta_input_net, nullptr);
    ASSERT_NE(sta_output_net, nullptr);

    EXPECT_EQ(db_network_->net(sta_input), sta_input_net);
    EXPECT_EQ(db_network_->net(sta_output), sta_output_net);
    sta::Graph* graph = sta_->ensureGraph();
    EXPECT_NE(graph->pinLoadVertex(sta_input), nullptr);
    EXPECT_NE(graph->pinDrvrVertex(sta_output), nullptr);

    EXPECT_TRUE(ep_.hasParasiticsInvalid());
    resizer_.updateParasiticsAndTiming();
    EXPECT_FALSE(ep_.hasParasiticsInvalid());
    sta_->updateTiming(true);
    EXPECT_TRUE(sta_->search()->arrivalsValid());
    db_network_->checkAxioms();
  }
};

TEST_F(BufferInserterStaTest, InsertAfterDriverUpdatesStaAndParasitics)
{
  readTestDesign();
  odb::dbNet* net = block_->findNet("n1");
  odb::dbMaster* buffer_master = lib_->findMaster("BUF_X1");
  ASSERT_NE(net, nullptr);
  ASSERT_NE(buffer_master, nullptr);

  est::IncrementalParasiticsGuard parasitics_guard(&ep_);
  const ecc::BufferInsertionResult result
      = ecc::OpenDbBufferInserter::insertBufferAfterDriver(net, buffer_master);

  verifyStaState(result);
}

TEST_F(BufferInserterStaTest, InsertBeforeLoadsUpdatesStaAndParasitics)
{
  readTestDesign();
  odb::dbNet* net = block_->findNet("n1");
  odb::dbInst* load1 = block_->findInst("load1");
  odb::dbInst* load2 = block_->findInst("load2");
  odb::dbMaster* buffer_master = lib_->findMaster("BUF_X1");
  ASSERT_NE(net, nullptr);
  ASSERT_NE(load1, nullptr);
  ASSERT_NE(load2, nullptr);
  ASSERT_NE(buffer_master, nullptr);
  std::vector<odb::dbObject*> loads{load1->findITerm("A"),
                                    load2->findITerm("A")};

  est::IncrementalParasiticsGuard parasitics_guard(&ep_);
  const ecc::BufferInsertionResult result
      = ecc::OpenDbBufferInserter::insertBufferBeforeLoads(
          net, loads, buffer_master);

  verifyStaState(result);
  EXPECT_EQ(load1->findITerm("A")->getNet(), result.output_net);
  EXPECT_EQ(load2->findITerm("A")->getNet(), result.output_net);
}

TEST_F(BufferInserterStaTest, GbiOneShotLeavesCleanDesignUnchanged)
{
  readTestDesign();
  const size_t instances_before = block_->getInsts().size();
  const size_t nets_before = block_->getNets().size();
  ecc::GbiRepairOptions options;
  options.common.max_fanout = 64;
  options.max_buffers = 4;
  const auto result = ecc::OpenDbGbiRepairer::repair(
      block_, sta_.get(), [this]() { refreshRepairTiming(); }, &resizer_, options);
  EXPECT_TRUE(result.ok) << result.message;
  EXPECT_FALSE(result.committed);
  EXPECT_EQ(result.plans_collected, 0u);
  EXPECT_EQ(block_->getInsts().size(), instances_before);
  EXPECT_EQ(block_->getNets().size(), nets_before);
}

TEST_F(BufferInserterStaTest, GbiOneShotAppliesOneAtomicPlan)
{
  readTestDesign("fanout_repair_sta.v");
  // The CLI has not necessarily called a Resizer command before GBI. Verify
  // that GBI initializes DBU conversion even with a fresh Resizer instance.
  rsz::Resizer cold_resizer(&logger_, db_.get(), sta_.get(), &stt_, &grt_,
                            &dp_, &ep_);
  // Give the timing-driven VG solver a wire-dominated net, where buffering
  // improves delay as well as fanout (short wires may correctly be rejected).
  block_->setDieArea(odb::Rect(0, 0, 2000000, 2000000));
  block_->setCoreArea(odb::Rect(0, 0, 2000000, 2000000));
  for(auto* inst : block_->getInsts()) {
    int x, y;
    inst->getLocation(x, y);
    inst->setLocation(x * 100, y);
  }
  refreshRepairTiming();
  const size_t instances_before = block_->getInsts().size();
  ecc::GbiRepairOptions options;
  // This integration target deliberately links the CPU stub: requesting
  // CUDA must fall back before any ECO, then apply the same atomic plan.
  options.backend = "cuda";
  options.common.max_fanout = 3;
  options.max_buffers = 64;
  options.max_group_loads = 3;
  options.require_drv_reduction = true;
  const auto result = ecc::OpenDbGbiRepairer::repair(
      block_, sta_.get(), [this]() { refreshRepairTiming(); }, &cold_resizer, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.backend_used, "cpu");
  EXPECT_FALSE(result.cuda_fallback.empty());
  EXPECT_TRUE(result.committed);
  EXPECT_GT(result.plans_collected, 0u);
  EXPECT_GT(result.inserted_buffers, 0u);
  EXPECT_GT(block_->getInsts().size(), instances_before);
  EXPECT_EQ(block_->getInsts().size() - instances_before, result.inserted_buffers);
  EXPECT_LT(result.after.severity, result.before.severity);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();

  // The next repairs must analyze the inserted topology, not a pre-GBI
  // snapshot. Check the stage boundary metrics and instance accounting.
  ecc::DesignRepairOptions design_options;
  design_options.max_fanout = options.common.max_fanout;
  const auto refresh = [this]() { refreshRepairTiming(); };
  const auto design = ecc::OpenDbDesignRepairer::repair(
      block_, sta_.get(), refresh, design_options);
  ASSERT_TRUE(design.ok) << design.message;
  EXPECT_NEAR(design.before.severity, result.after.severity, 1e-6);
  EXPECT_NEAR(design.before.setup_wns, result.after.setup_wns, 1e-15);
  const size_t after_design = block_->getInsts().size();
  EXPECT_EQ(after_design, instances_before + result.inserted_buffers +
                             design.inserted_buffers);
  ecc::SetupRepairOptions setup_options;
  setup_options.common = design_options;
  const auto setup = ecc::OpenDbSetupRepairer::repair(
      block_, sta_.get(), refresh, setup_options);
  ASSERT_TRUE(setup.ok) << setup.message;
  EXPECT_NEAR(setup.before.design.setup_wns, design.after.setup_wns, 1e-15);
  EXPECT_GE(setup.after.design.setup_wns + 1e-15, setup.before.design.setup_wns);
  EXPECT_EQ(block_->getInsts().size(), after_design + setup.inserted_buffers);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_TRUE(sta_->search()->arrivalsValid());
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, GbiOneShotRollsBackWhenTimingGetsWorse)
{
  readTestDesign("fanout_repair_sta.v");
  const size_t instances_before = block_->getInsts().size();
  ecc::GbiRepairOptions options;
  options.common.max_fanout = 3;
  options.max_buffers = 3;
  options.max_group_loads = 3;
  const auto refresh = [&]() {
    const bool edited = block_->getInsts().size() > instances_before;
    for(auto* scene : sta_->scenes()) {
      ep_.setHWireSignalRC(scene, edited ? 1e12 : 1e6,
                            edited ? 1e-4 : 1e-10);
      ep_.setVWireSignalRC(scene, edited ? 1e12 : 1e6,
                            edited ? 1e-4 : 1e-10);
    }
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  const auto result =
      ecc::OpenDbGbiRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_FALSE(result.committed);
  EXPECT_EQ(block_->getInsts().size(), instances_before);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, RepairFanoutBuildsTreeAndKeepsStaValid)
{
  readTestDesign("fanout_repair_sta.v");
  odb::dbNet* net = block_->findNet("high_fanout");
  odb::dbMaster* buffer_master = lib_->findMaster("BUF_X1");
  ASSERT_NE(net, nullptr);
  ASSERT_NE(buffer_master, nullptr);
  ASSERT_EQ(ecc::OpenDbFanoutRepairer::loadCount(net), 10);

  ecc::FanoutRepairOptions options;
  options.max_fanout = 3;
  options.insertion.buffer_name_prefix = "sizer_fanout_buf";
  options.insertion.net_name_prefix = "sizer_fanout_net";

  est::IncrementalParasiticsGuard parasitics_guard(&ep_);
  const ecc::FanoutRepairResult result
      = ecc::OpenDbFanoutRepairer::repair(block_, buffer_master, options);

  ASSERT_TRUE(result.ok()) << result.message;
  EXPECT_EQ(result.before_max_fanout, 10);
  EXPECT_EQ(result.after_max_fanout, 3);
  EXPECT_EQ(result.inserted_buffers.size(), 4);
  for (odb::dbNet* result_net : block_->getNets()) {
    EXPECT_LE(ecc::OpenDbFanoutRepairer::loadCount(result_net), 3)
        << result_net->getName();
  }
  sta::Graph* graph = sta_->ensureGraph();
  for (odb::dbInst* buffer : result.inserted_buffers) {
    ASSERT_NE(db_network_->dbToSta(buffer), nullptr);
    odb::dbITerm* input = nullptr;
    odb::dbITerm* output = nullptr;
    for (odb::dbITerm* term : buffer->getITerms()) {
      if (term->getIoType() == odb::dbIoType::INPUT) {
        input = term;
      } else if (term->getIoType() == odb::dbIoType::OUTPUT) {
        output = term;
      }
    }
    ASSERT_NE(input, nullptr);
    ASSERT_NE(output, nullptr);
    EXPECT_NE(graph->pinLoadVertex(db_network_->dbToSta(input)), nullptr);
    EXPECT_NE(graph->pinDrvrVertex(db_network_->dbToSta(output)), nullptr);
  }

  EXPECT_TRUE(ep_.hasParasiticsInvalid());
  resizer_.updateParasiticsAndTiming();
  EXPECT_FALSE(ep_.hasParasiticsInvalid());
  sta_->updateTiming(true);
  EXPECT_TRUE(sta_->search()->arrivalsValid());
  db_network_->checkAxioms();

  const std::string output_path
      = std::string(ECC_SIZER_TEST_OUTPUT_DIR) + "/fanout_repair_demo.v";
  sta::writeVerilog(output_path.c_str(), false, {}, sta_->network());
  std::ifstream output(output_path);
  const std::string verilog((std::istreambuf_iterator<char>(output)),
                            std::istreambuf_iterator<char>());
  EXPECT_NE(verilog.find("sizer_fanout_buf"), std::string::npos);
  std::cout << "fanout demo: max 10 -> 3, inserted 4 buffers, output "
            << output_path << '\n';
}

TEST_F(BufferInserterStaTest, NativeDesignRepairUsesStaAndHonorsBudget)
{
  readTestDesign("fanout_repair_sta.v");
  ecc::DesignRepairOptions options;
  options.max_fanout = 3;
  options.max_inserted_buffers = 20;
  const auto refresh = [&]() {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  refresh();
  const auto result
      = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.accepted_passes, 0);
  EXPECT_LT(result.after.severity, result.before.severity);
  EXPECT_LE(result.inserted_buffers, options.max_inserted_buffers);
  EXPECT_GE(result.after.setup_wns + 1e-15f, result.before.setup_wns);
  db_network_->checkAxioms();
  EXPECT_TRUE(sta_->search()->arrivalsValid());
}

TEST_F(BufferInserterStaTest,
       NativeDesignRepairRestoresTopologyOnRefreshFailure)
{
  readTestDesign("fanout_repair_sta.v");
  auto* net = block_->findNet("high_fanout");
  const auto insts = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  ecc::DesignRepairOptions options;
  options.max_fanout = 3;
  unsigned updates = 0;
  const auto refresh = [&]() {
    if (++updates == 1)
      throw std::runtime_error("injected timing update failure");
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  const auto result
      = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(block_->getInsts().size(), insts);
  EXPECT_EQ(block_->getNets().size(), nets);
  EXPECT_EQ(ecc::OpenDbFanoutRepairer::loadCount(net), 10);
  EXPECT_EQ(updates, 2);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest,
       NativeDesignRepairSkipsProtectedLoadAndExcludedCells)
{
  readTestDesign("fanout_repair_sta.v");
  block_->findInst("load0")->setDoNotTouch(true);
  ecc::DesignRepairOptions options;
  options.max_fanout = 3;
  const auto refresh = [&]() { sta_->updateTiming(true); };
  const auto result
      = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(
      ecc::OpenDbFanoutRepairer::loadCount(block_->findNet("high_fanout")), 10);
  for (auto* master : lib_->getMasters())
    options.excluded_masters.push_back(master);
  const auto excluded
      = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  EXPECT_FALSE(excluded.ok);
}

TEST_F(BufferInserterStaTest,
       NativeDesignRepairHandlesCapWithoutFanoutViolation)
{
  readTestDesign("fanout_repair_sta.v");
  auto* cell = db_network_->libertyCell(block_->findInst("driver"));
  sta::LibertyPort *input, *output;
  cell->bufferPorts(input, output);
  output->setCapacitanceLimit(input->capacitance() * 3, sta::MinMax::max());
  ecc::DesignRepairOptions options;
  options.max_fanout = 64;
  const auto refresh = [&]() {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  refresh();
  const auto result
      = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.before.fanout_nets, 0);
  EXPECT_GT(result.before.capacitance_nets, 0);
  EXPECT_LT(result.after.capacitance_nets, result.before.capacitance_nets);
  EXPECT_GT(result.resized_buffers + result.inserted_buffers, 0);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, NativeDesignRepairReportsUnresolvedAtZeroBudget)
{
  readTestDesign("fanout_repair_sta.v");
  ecc::DesignRepairOptions options;
  options.max_fanout = 3;
  options.max_inserted_buffers = 0;
  const auto count = block_->getInsts().size();
  const auto result = ecc::OpenDbDesignRepairer::repair(
      block_, sta_.get(), [&]() { sta_->updateTiming(true); }, options);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.converged);
  EXPECT_GT(result.after.fanout_nets, 0);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(block_->getInsts().size(), count);
}

TEST_F(BufferInserterStaTest,
       NativeDesignRepairHandlesSlewWithoutFanoutViolation)
{
  readTestDesign("fanout_repair_sta.v");
  const auto refresh = [&]() {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  refresh();
  sta_->checkSlewsPreamble();
  auto* driver = block_->findInst("driver");
  sta::Slew slew;
  float limit, slack;
  const sta::RiseFall* rf = nullptr;
  const sta::Scene* scene = nullptr;
  sta_->checkSlew(db_network_->dbToSta(driver->findITerm("Z")),
                  sta_->scenes(),
                  sta::MinMax::max(),
                  false,
                  slew,
                  limit,
                  slack,
                  rf,
                  scene);
  ASSERT_GT(float(slew), 0);
  sta::LibertyPort *input, *output;
  db_network_->libertyCell(driver)->bufferPorts(input, output);
  output->setSlewLimit(float(slew) * 0.75f, sta::MinMax::max());
  const auto result
      = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.before.fanout_nets, 0);
  EXPECT_EQ(result.before.capacitance_nets, 0);
  EXPECT_GT(result.before.slew_nets, 0);
  EXPECT_LT(result.after.slew_nets, result.before.slew_nets);
  EXPECT_GT(result.resized_buffers + result.inserted_buffers, 0);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, NativeDesignRepairRejectsTimingRegression)
{
  readTestDesign("fanout_repair_sta.v");
  ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
  sta_->updateTiming(true);
  const auto insts = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  unsigned updates = 0;
  const auto refresh = [&]() {
    const bool candidate = ++updates == 1;
    // Make the candidate's parasitics deliberately poor, then restore the
    // original RC on rollback. Exercise the measured rejection path.
    for (auto* scene : sta_->scenes()) {
      ep_.setHWireSignalRC(
          scene, candidate ? 1.0e12 : 1.0e6, candidate ? 1.0e-4 : 1.0e-10);
      ep_.setVWireSignalRC(
          scene, candidate ? 1.0e12 : 1.0e6, candidate ? 1.0e-4 : 1.0e-10);
    }
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  ecc::DesignRepairOptions options;
  options.max_fanout = 3;
  const auto result
      = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.rejected_passes, 1);
  EXPECT_EQ(result.accepted_passes, 0);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(block_->getInsts().size(), insts);
  EXPECT_EQ(block_->getNets().size(), nets);
  EXPECT_EQ(
      ecc::OpenDbFanoutRepairer::loadCount(block_->findNet("high_fanout")), 10);
  EXPECT_NEAR(result.after.severity, result.before.severity, 1e-6);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, NativeDesignRepairRecognizesSdcClockNets)
{
  readTestDesign("fanout_repair_sta.v");
  sta::PinSet pins(db_network_);
  pins.insert(db_network_->dbToSta(block_->findInst("driver")->findITerm("Z")));
  sta_->makeClock("internal_clock",
                  pins,
                  false,
                  1e-9,
                  sta::FloatSeq{0, 0.5e-9},
                  "",
                  sta_->cmdMode());
  auto* net = block_->findNet("high_fanout");
  ASSERT_EQ(net->getSigType(), odb::dbSigType::SIGNAL);
  ecc::DesignRepairOptions options;
  options.max_fanout = 3;
  const auto result = ecc::OpenDbDesignRepairer::repair(
      block_, sta_.get(), [&]() { sta_->updateTiming(true); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_TRUE(sta_->isClock(db_network_->dbToSta(net), sta_->cmdMode()));
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(ecc::OpenDbFanoutRepairer::loadCount(net), 10);
}

TEST_F(BufferInserterStaTest, NativeDesignRepairReportsFailedRollbackRefresh)
{
  readTestDesign("fanout_repair_sta.v");
  ecc::DesignRepairOptions options;
  options.max_fanout = 3;
  const auto insts = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  const auto result = ecc::OpenDbDesignRepairer::repair(
      block_,
      sta_.get(),
      []() { throw std::runtime_error("timing unavailable"); },
      options);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("timing refresh after rollback failed"),
            std::string::npos);
  EXPECT_EQ(block_->getInsts().size(), insts);
  EXPECT_EQ(block_->getNets().size(), nets);
  EXPECT_EQ(
      ecc::OpenDbFanoutRepairer::loadCount(block_->findNet("high_fanout")), 10);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, NativeGateResizeUsesEquivalentFunctionAndKeepsNets)
{
  readTestDesign("gate_repair_sta.v");
  constrainGateCap();
  auto* driver = block_->findInst("driver");
  auto* original = db_network_->libertyCell(driver);
  auto* in1 = driver->findITerm("A1")->getNet();
  auto* in2 = driver->findITerm("A2")->getNet();
  auto* out = driver->findITerm("ZN")->getNet();
  ecc::DesignRepairOptions options;
  options.max_inserted_buffers = 0;
  const auto refresh = [&]() {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  refresh();
  const auto result
      = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.resized_gates, 1);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(result.after.capacitance_nets, 0);
  EXPECT_TRUE(sta::equivCellsArcs(original, db_network_->libertyCell(driver)));
  EXPECT_EQ(driver->findITerm("A1")->getNet(), in1);
  EXPECT_EQ(driver->findITerm("A2")->getNet(), in2);
  EXPECT_EQ(driver->findITerm("ZN")->getNet(), out);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest,
       NativeGateResizeRejectsWrongFunctionAndExcludedMasters)
{
  readTestDesign("gate_repair_sta.v");
  constrainGateCap();
  auto* driver = block_->findInst("driver");
  auto* original = driver->getMaster();
  ecc::DesignRepairOptions options;
  options.max_inserted_buffers = 0;
  // Other gates such as NOR2 have the same physical pin names but a different
  // Boolean function. They must not be considered when NAND upsizes are banned.
  options.excluded_masters
      = {lib_->findMaster("NAND2_X2"), lib_->findMaster("NAND2_X4")};
  const auto result = ecc::OpenDbDesignRepairer::repair(
      block_, sta_.get(), [&]() { sta_->updateTiming(true); }, options);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.resized_gates, 0);
  EXPECT_EQ(driver->getMaster(), original);
  EXPECT_GT(result.after.capacitance_nets, 0);
}

TEST_F(BufferInserterStaTest, NativeRepairLegalizesEachAcceptedBatch)
{
  readTestDesign("fanout_repair_sta.v");
  addLegalRows();
  ecc::DesignRepairOptions options;
  options.max_fanout = 3;
  unsigned prepared = 0;
  options.prepare = [&]() {
    ecc::legalizeRepairPlacement(block_, &dp_);
    ++prepared;
  };
  const auto refresh = [&]() {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  refresh();
  const auto result
      = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.accepted_passes, 0);
  EXPECT_EQ(prepared, result.accepted_passes + result.rejected_passes);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest,
       NativeRepairRestoresResizeAndPlacementOnPrepareFailure)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainGateCap();
  struct Snapshot
  {
    odb::dbInst* inst;
    odb::dbMaster* master;
    odb::Point location;
    odb::dbOrientType orient;
    odb::dbPlacementStatus status;
  };
  std::vector<Snapshot> snapshots;
  for (auto* inst : block_->getInsts())
    snapshots.push_back({inst,
                         inst->getMaster(),
                         inst->getLocation(),
                         inst->getOrient(),
                         inst->getPlacementStatus()});
  ecc::DesignRepairOptions options;
  options.max_inserted_buffers = 0;
  options.prepare = [&]() {
    ecc::legalizeRepairPlacement(block_, &dp_);
    block_->findInst("load9")->setLocation(1333, 2777);
    throw std::runtime_error("injected legalization failure");
  };
  const auto refresh = [&]() {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  refresh();
  const auto result
      = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.resized_gates, 0);
  for (const auto& snap : snapshots) {
    EXPECT_EQ(snap.inst->getMaster(), snap.master);
    EXPECT_EQ(snap.inst->getLocation(), snap.location);
    EXPECT_EQ(snap.inst->getOrient(), snap.orient);
    EXPECT_EQ(snap.inst->getPlacementStatus(), snap.status);
  }
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, RepairPlacementPreservesProtectedInstances)
{
  readTestDesign("fanout_repair_sta.v");
  addLegalRows();
  auto* protected_inst = block_->findInst("driver");
  const auto location = protected_inst->getLocation();
  const auto status = protected_inst->getPlacementStatus();
  auto* load = block_->findInst("load0");
  load->setLocation(location.x(), location.y());
  ASSERT_NO_THROW(ecc::legalizeRepairPlacement(block_, &dp_, {protected_inst}));
  EXPECT_EQ(protected_inst->getLocation(), location);
  EXPECT_EQ(protected_inst->getPlacementStatus(), status);
  EXPECT_NE(load->getLocation(), location);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
}

TEST_F(BufferInserterStaTest, NativeSetupRepairImprovesTimingWithoutNewDrv)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.max_area_growth = 0.5;
  options.common.prepare
      = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::OpenDbSetupRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_LT(result.before.design.setup_wns, 0);
  EXPECT_GT(result.accepted_batches, 0);
  EXPECT_GT(result.after.design.setup_wns, result.before.design.setup_wns);
  EXPECT_GT(result.after.setup_tns, result.before.setup_tns);
  EXPECT_GE(result.after.design.hold_wns, result.before.design.hold_wns);
  EXPECT_LE(result.after.cap_violations, result.before.cap_violations);
  EXPECT_LE(result.after.slew_violations, result.before.slew_violations);
  EXPECT_LE(result.after.fanout_violations, result.before.fanout_violations);
  EXPECT_LE(result.attempted_batches, options.max_trials);
  EXPECT_LE(result.after.design.area, result.before.design.area * 1.5);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, NativeSetupRepairBuffersNoncriticalBranches)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  auto* critical_net = block_->findInst("load0")->findITerm("A")->getNet();
  ecc::SetupRepairOptions options;
  options.common.allow_gate_resize = false;
  options.max_path_ends = 1;
  options.max_passes = 1;
  options.batch_size = 1;
  options.max_area_growth = 0.5;
  options.common.prepare
      = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::OpenDbSetupRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.inserted_buffers, 0);
  EXPECT_GT(result.after.design.setup_wns, result.before.design.setup_wns);
  EXPECT_EQ(block_->findInst("load0")->findITerm("A")->getNet(), critical_net);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest,
       NativeSetupRepairHonorsZeroBudgetAndProtectedDriver)
{
  readTestDesign("gate_repair_sta.v");
  constrainSetup(true);
  refreshRepairTiming();
  const auto count = block_->getInsts().size();
  const auto master = block_->findInst("driver")->getMaster();
  ecc::SetupRepairOptions options;
  options.max_trials = 0;
  auto result = ecc::OpenDbSetupRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.attempted_batches, 0);
  EXPECT_EQ(block_->getInsts().size(), count);
  EXPECT_EQ(block_->findInst("driver")->getMaster(), master);
  options.max_trials = 8;
  for (auto* inst : block_->getInsts())
    options.common.excluded_instances.push_back(inst);
  result = ecc::OpenDbSetupRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.attempted_batches, 0);
  EXPECT_EQ(block_->getInsts().size(), count);
}

TEST_F(BufferInserterStaTest,
       NativeSetupRepairRejectsBadParasiticsAndRestoresPlacement)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  std::vector<
      std::tuple<odb::dbInst*, odb::dbMaster*, odb::Point, odb::dbOrientType>>
      saved;
  for (auto* inst : block_->getInsts())
    saved.emplace_back(
        inst, inst->getMaster(), inst->getLocation(), inst->getOrient());
  const auto net_count = block_->getNets().size();
  ecc::SetupRepairOptions options;
  options.max_area_growth = 0.5;
  options.max_trials = 1;
  options.common.prepare
      = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned updates = 0;
  const auto refresh = [&]() {
    const bool candidate = ++updates == 1;
    for (auto* scene : sta_->scenes()) {
      ep_.setHWireSignalRC(
          scene, candidate ? 1e12 : 1e6, candidate ? 1e-4 : 1e-10);
      ep_.setVWireSignalRC(
          scene, candidate ? 1e12 : 1e6, candidate ? 1e-4 : 1e-10);
    }
    refreshRepairTiming();
  };
  const auto result
      = ecc::OpenDbSetupRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.accepted_batches, 0);
  EXPECT_EQ(result.rejected_batches, 1);
  EXPECT_EQ(block_->getInsts().size(), saved.size());
  EXPECT_EQ(block_->getNets().size(), net_count);
  for (const auto& [inst, master, point, orient] : saved) {
    EXPECT_EQ(inst->getMaster(), master);
    EXPECT_EQ(inst->getLocation(), point);
    EXPECT_EQ(inst->getOrient(), orient);
  }
  EXPECT_NEAR(
      result.after.design.setup_wns, result.before.design.setup_wns, 1e-15);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
}

TEST_F(BufferInserterStaTest, NativeSetupRepairRestoresOnPreparationFailure)
{
  readTestDesign("gate_repair_sta.v");
  constrainSetup();
  refreshRepairTiming();
  const auto count = block_->getInsts().size();
  auto* driver = block_->findInst("driver");
  auto* master = driver->getMaster();
  ecc::SetupRepairOptions options;
  options.max_area_growth = 0.5;
  options.common.prepare = []() {
    throw std::runtime_error("injected setup preparation failure");
  };
  const auto result = ecc::OpenDbSetupRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.accepted_batches, 0);
  EXPECT_EQ(block_->getInsts().size(), count);
  EXPECT_EQ(driver->getMaster(), master);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, NativeSetupRepairRejectsHoldLossDespiteSetupGain)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.max_area_growth = 0.5;
  options.batch_size = 1;
  options.max_trials = 1;
  options.common.prepare
      = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned updates = 0;
  float candidate_setup = 0, candidate_hold = 0;
  const auto refresh = [&]() {
    const bool candidate = ++updates == 1;
    auto* sdc = sta_->cmdMode()->sdc();
    for (auto* pin : block_->getBTerms()) {
      if (pin->getIoType() != odb::dbIoType::OUTPUT)
        continue;
      sta_->setOutputDelay(db_network_->dbToSta(pin),
                           sta::RiseFallBoth::riseFall(),
                           sdc->findClock("clk"),
                           sta::RiseFall::rise(),
                           nullptr,
                           false,
                           false,
                           sta::MinMaxAll::min(),
                           false,
                           candidate ? -1e-9f : 0.0f,
                           sdc);
    }
    refreshRepairTiming();
    if (candidate) {
      candidate_setup = float(sta_->worstSlack(sta::MinMax::max()));
      candidate_hold = float(sta_->worstSlack(sta::MinMax::min()));
    }
  };
  const auto result
      = ecc::OpenDbSetupRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(candidate_setup, result.before.design.setup_wns);
  EXPECT_LT(candidate_hold, result.before.design.hold_wns);
  EXPECT_EQ(result.accepted_batches, 0);
  EXPECT_EQ(result.rejected_batches, 1);
  EXPECT_NEAR(
      result.after.design.setup_wns, result.before.design.setup_wns, 1e-15);
  EXPECT_EQ(result.after.design.hold_wns, result.before.design.hold_wns);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
}

TEST_F(BufferInserterStaTest, NativeSetupRepairHonorsZeroAreaGrowth)
{
  readTestDesign("gate_repair_sta.v");
  constrainSetup();
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.max_area_growth = 0;
  const auto result = ecc::OpenDbSetupRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.attempted_batches, 0);
  EXPECT_EQ(result.after.design.area, result.before.design.area);
  EXPECT_EQ(result.after.design.setup_wns, result.before.design.setup_wns);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(result.resized_cells, 0);
}

TEST_F(BufferInserterStaTest, NativeRepairConfigZeroPassAndInvalidPolicyDoNotEdit)
{
  readTestDesign("gate_repair_sta.v");
  constrainGateCap();
  refreshRepairTiming();
  const auto count = block_->getInsts().size();
  const auto master = block_->findInst("driver")->getMaster();
  ecc::RepairConfig config;
  config.set("design.max_passes", "0");
  unsigned refreshes = 0;
  auto refresh = [&]() { ++refreshes; refreshRepairTiming(); };
  auto result = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, config.design);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.accepted_passes, 0);
  EXPECT_EQ(refreshes, 0);
  EXPECT_EQ(block_->getInsts().size(), count);
  EXPECT_EQ(block_->findInst("driver")->getMaster(), master);
  config.design.overload_group_fraction = 1;
  result = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, config.design);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(refreshes, 0);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
}

TEST_F(BufferInserterStaTest, NativeSetupConfigControlsGainAndTrialBudgets)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  ecc::RepairConfig config;
  config.design.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  config.set("setup.min_improvement_ns", "1000000");
  config.set("setup.max_area_growth", "0.5");
  auto refresh = [&]() { refreshRepairTiming(); };
  auto result = ecc::OpenDbSetupRepairer::repair(block_, sta_.get(), refresh, config.setupOptions());
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.attempted_batches, 0);
  EXPECT_EQ(result.after.design.setup_wns, result.before.design.setup_wns);
  config.set("setup.min_improvement_ns", "0.0001");
  config.set("setup.max_trials", "1");
  config.set("setup.batch_size", "1");
  result = ecc::OpenDbSetupRepairer::repair(block_, sta_.get(), refresh, config.setupOptions());
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.attempted_batches, 1);
  EXPECT_EQ(result.accepted_batches, 1);
  EXPECT_GT(result.after.design.setup_wns, result.before.design.setup_wns);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, NativeSetupSkipsDuplicateEditsAndDefersRollbackRefresh)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  auto* driver = block_->findInst("driver");
  auto* original = driver->getMaster();
  const auto inst_count = block_->getInsts().size();
  const auto net_count = block_->getNets().size();
  unsigned refresh_count = 0;
  const auto refresh = [&]() {
    ++refresh_count;
    const bool edited = driver->getMaster() != original || block_->getInsts().size() != inst_count;
    for (auto* scene : sta_->scenes()) {
      ep_.setHWireSignalRC(scene, edited ? 1e12 : 1e6, edited ? 1e-4 : 1e-10);
      ep_.setVWireSignalRC(scene, edited ? 1e12 : 1e6, edited ? 1e-4 : 1e-10);
    }
    refreshRepairTiming();
  };
  ecc::SetupRepairOptions options;
  options.max_passes = 1;
  options.max_trials = 64;
  options.max_area_growth = 0.5;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  options.skip_duplicate_trials = false;
  options.defer_rollback_refresh = false;
  const auto eager = ecc::OpenDbSetupRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(eager.ok) << eager.message;
  ASSERT_EQ(eager.accepted_batches, 0);
  EXPECT_EQ(eager.refreshes, 2 * eager.rejected_batches);
  EXPECT_EQ(eager.refreshes, refresh_count);
  options.skip_duplicate_trials = true;
  options.defer_rollback_refresh = true;
  refresh_count = 0;
  const auto optimized = ecc::OpenDbSetupRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(optimized.ok) << optimized.message;
  EXPECT_EQ(optimized.accepted_batches, 0);
  EXPECT_GT(optimized.skipped_duplicate_batches, 0);
  EXPECT_LT(optimized.attempted_batches, eager.attempted_batches);
  EXPECT_LT(optimized.refreshes, eager.refreshes);
  EXPECT_EQ(optimized.rollback_refreshes, 1);
  EXPECT_EQ(optimized.refreshes, refresh_count);
  EXPECT_EQ(driver->getMaster(), original);
  EXPECT_EQ(block_->getInsts().size(), inst_count);
  EXPECT_EQ(block_->getNets().size(), net_count);
  EXPECT_NEAR(optimized.after.design.setup_wns, eager.after.design.setup_wns, 1e-15);
  EXPECT_NEAR(optimized.after.setup_tns, eager.after.setup_tns, 1e-15);
  EXPECT_NEAR(float(sta_->worstSlack(sta::MinMax::max())), optimized.after.design.setup_wns, 1e-15);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, NativeSetupDeferredRollbackSurvivesLaterPrepareFailure)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  auto* driver = block_->findInst("driver");
  auto* original = driver->getMaster();
  const auto count = block_->getInsts().size();
  const float initial_wns = float(sta_->worstSlack(sta::MinMax::max()));
  const auto refresh = [&]() {
    const bool edited = driver->getMaster() != original || block_->getInsts().size() != count;
    for (auto* scene : sta_->scenes()) {
      ep_.setHWireSignalRC(scene, edited ? 1e12 : 1e6, edited ? 1e-4 : 1e-10);
      ep_.setVWireSignalRC(scene, edited ? 1e12 : 1e6, edited ? 1e-4 : 1e-10);
    }
    refreshRepairTiming();
  };
  ecc::SetupRepairOptions options;
  options.max_area_growth = 0.5;
  options.skip_duplicate_trials = false;
  unsigned prepares = 0;
  options.common.prepare = [&]() {
    if (++prepares == 2) throw std::runtime_error("later prepare failure");
    ecc::legalizeRepairPlacement(block_, &dp_);
  };
  const auto result = ecc::OpenDbSetupRepairer::repair(block_, sta_.get(), refresh, options);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(prepares, 2);
  EXPECT_EQ(result.rejected_batches, 1);
  EXPECT_EQ(result.rollback_refreshes, 1);
  EXPECT_EQ(driver->getMaster(), original);
  EXPECT_EQ(block_->getInsts().size(), count);
  EXPECT_NEAR(float(sta_->worstSlack(sta::MinMax::max())), initial_wns, 1e-15);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, LoadDownsizeRepairsCapWithoutGrowingDriverOrAddingBuffers)
{
  readTestDesign("gate_repair_sta.v");
  oversizeLoads();
  addLegalRows();
  constrainGateCap();
  refreshRepairTiming();
  const auto count = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  auto* driver = block_->findInst("driver");
  auto* original = driver->getMaster();
  ecc::DesignRepairOptions options;
  options.allow_gate_resize = false;
  options.max_inserted_buffers = 0;
  options.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.before.capacitance_nets, 0);
  EXPECT_EQ(result.after.capacitance_nets, 0);
  EXPECT_GT(result.downsized_loads, 0);
  EXPECT_LT(result.after.area, result.before.area);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(driver->getMaster(), original);
  EXPECT_EQ(block_->getInsts().size(), count);
  EXPECT_EQ(block_->getNets().size(), nets);
  EXPECT_GE(result.after.setup_wns, result.before.setup_wns);
  EXPECT_GE(result.after.hold_wns, result.before.hold_wns);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, LoadDownsizeIsConfigurableAndHonorsBatchBudget)
{
  readTestDesign("gate_repair_sta.v");
  oversizeLoads();
  addLegalRows();
  constrainGateCap();
  refreshRepairTiming();
  ecc::DesignRepairOptions options;
  options.allow_load_downsize = false;
  options.allow_gate_resize = false;
  options.max_inserted_buffers = 0;
  auto refresh = [&]() { refreshRepairTiming(); };
  auto result = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.downsized_loads, 0);
  options.allow_load_downsize = true;
  options.max_load_downsizes_per_pass = 1;
  options.max_passes = 1;
  result = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.downsized_loads, 1);
  EXPECT_LT(result.after.severity, result.before.severity);
}

TEST_F(BufferInserterStaTest, LoadDownsizePlannerHonorsProtectionSlackAndEquivalentCells)
{
  readTestDesign("gate_repair_sta.v");
  oversizeLoads();
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  auto* fixed = block_->findInst("load1");
  fixed->setPlacementStatus(odb::dbPlacementStatus::FIRM);
  block_->findInst("load2")->setDoNotTouch(true);
  block_->findInst("load4")->findITerm("Z")->getNet()->setDoNotTouch(true);
  ecc::RepairCommonOptions options;
  options.excluded_instances.push_back(block_->findInst("load3"));
  options.excluded_masters.push_back(lib_->findMaster("BUF_X1"));
  ecc::detail::Repair backend(block_, sta_.get(), options);
  ecc::DesignRepairMetrics metrics;
  const auto states = backend.analyze(metrics, true);
  // Source net is conservatively skipped when it includes a protected load.
  // Query its state from an unprotected snapshot solely to test planner guards.
  block_->findInst("load2")->setDoNotTouch(false);
  ecc::RepairCommonOptions source_options;
  ecc::detail::Repair source_backend(block_, sta_.get(), source_options);
  const auto source_states = source_backend.analyze(metrics, true);
  const auto it = std::find_if(source_states.begin(), source_states.end(), [&](const auto& state) {
    return state.driver->getInst() == block_->findInst("driver");
  });
  ASSERT_NE(it, source_states.end());
  block_->findInst("load2")->setDoNotTouch(true);
  ecc::detail::LoadDownsizePlanner planner(backend, sta_.get(), states, options);
  const auto candidates = planner.candidates(*it);
  ASSERT_FALSE(candidates.empty());
  for (const auto& candidate : candidates) {
    EXPECT_NE(candidate.inst, block_->findInst("load0")); // Negative setup slack.
    EXPECT_NE(candidate.inst, fixed);
    EXPECT_NE(candidate.inst, block_->findInst("load2"));
    EXPECT_NE(candidate.inst, block_->findInst("load3"));
    EXPECT_NE(candidate.inst, block_->findInst("load4"));
    EXPECT_NE(candidate.master, lib_->findMaster("BUF_X1"));
    EXPECT_TRUE(sta::equivCellsArcs(db_network_->libertyCell(candidate.inst),
      db_network_->libertyCell(db_network_->dbToSta(candidate.master))));
    EXPECT_LT(ecc::detail::area(candidate.master), ecc::detail::area(candidate.inst->getMaster()));
  }
  options.load_downsize_slack_margin = 1e-6f;
  EXPECT_TRUE(planner.candidates(*it).empty());
}

TEST_F(BufferInserterStaTest, LoadDownsizeImprovesSetupAtZeroAreaGrowth)
{
  readTestDesign("gate_repair_sta.v");
  oversizeLoads(true);
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  auto* driver = block_->findInst("driver");
  auto* master = driver->getMaster();
  auto* critical = block_->findInst("load0")->getMaster();
  ecc::SetupRepairOptions options;
  options.common.allow_gate_resize = false;
  options.max_inserted_buffers = 0;
  options.max_area_growth = 0;
  options.batch_size = 1;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::OpenDbSetupRepairer::repair(block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.downsized_loads, 0);
  EXPECT_GT(result.after.design.setup_wns, result.before.design.setup_wns);
  EXPECT_GT(result.after.setup_tns, result.before.setup_tns);
  EXPECT_LT(result.after.design.area, result.before.design.area);
  EXPECT_EQ(driver->getMaster(), master);
  EXPECT_EQ(block_->findInst("load0")->getMaster(), critical);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, LoadDownsizeRejectionRestoresCellsAndFallsBackToDriverResize)
{
  readTestDesign("gate_repair_sta.v");
  oversizeLoads();
  addLegalRows();
  constrainGateCap();
  refreshRepairTiming();
  ecc::DesignRepairOptions options;
  options.max_inserted_buffers = 0;
  options.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  bool saw_downsize = false;
  const auto refresh = [&]() {
    bool downsized = false;
    for (unsigned i = 0; i < 10; ++i)
      downsized |= block_->findInst(("load" + std::to_string(i)).c_str())->getMaster() != lib_->findMaster("BUF_X8");
    saw_downsize |= downsized;
    for (auto* scene : sta_->scenes()) {
      ep_.setHWireSignalRC(scene, downsized ? 1e12 : 1e6, downsized ? 1e-4 : 1e-10);
      ep_.setVWireSignalRC(scene, downsized ? 1e12 : 1e6, downsized ? 1e-4 : 1e-10);
    }
    refreshRepairTiming();
  };
  const auto result = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_TRUE(saw_downsize);
  EXPECT_GT(result.rejected_load_batches, 0);
  EXPECT_EQ(result.downsized_loads, 0);
  EXPECT_GT(result.resized_gates, 0);
  for (unsigned i = 0; i < 10; ++i)
    EXPECT_EQ(block_->findInst(("load" + std::to_string(i)).c_str())->getMaster(), lib_->findMaster("BUF_X8"));
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, LoadDownsizeWorksWithGateOnlyChoicesAndTiedInputs)
{
  readTestDesign("load_repair_sta.v");
  addLegalRows();
  auto* driver = block_->findInst("driver");
  driver->setPlacementStatus(odb::dbPlacementStatus::FIRM);
  auto* small = db_network_->libertyCell(db_network_->dbToSta(lib_->findMaster("NAND2_X1")));
  small->findLibertyPort("ZN")->setCapacitanceLimit(
      25 * small->findLibertyPort("A1")->capacitance(), sta::MinMax::max());
  refreshRepairTiming();
  ecc::DesignRepairOptions options;
  options.max_inserted_buffers = 0;
  for (auto* master : lib_->getMasters()) {
    auto* cell = db_network_->libertyCell(db_network_->dbToSta(master));
    if (cell && cell->isBuffer()) options.excluded_masters.push_back(master);
  }
  const auto result = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.downsized_loads, 10);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(result.resized_gates, 0);
  EXPECT_EQ(driver->getMaster(), lib_->findMaster("NAND2_X1"));
  for (unsigned i = 0; i < 10; ++i) {
    auto* inst = block_->findInst(("load" + std::to_string(i)).c_str());
    EXPECT_EQ(inst->getMaster(), lib_->findMaster("NAND2_X1"));
    EXPECT_EQ(inst->findITerm("A1")->getNet(), inst->findITerm("A2")->getNet());
  }
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, VtSetupUsesExplicitMappingAcrossLibraries)
{
  readVtDesign();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.common.allow_load_downsize = false;
  options.max_area_growth = 0;
  options.max_inserted_buffers = 0;
  auto refresh = [&]() { refreshRepairTiming(); };
  auto disabled = ecc::OpenDbSetupRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(disabled.ok);
  EXPECT_EQ(disabled.vt_swaps, 0);
  options.common.vt_suffixes = {"_H", "_L"};
  const auto result = ecc::OpenDbSetupRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.vt_swaps, 1) << result.message << " attempts=" << result.attempted_batches;
  EXPECT_GT(result.after.design.setup_wns, result.before.design.setup_wns);
  EXPECT_EQ(result.after.design.area, result.before.design.area);
  EXPECT_EQ(block_->findInst("driver")->getMaster()->getName(), "VTBUF_L");
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, VtDesignRepairFixesSlewWithoutAreaGrowth)
{
  readVtDesign();
  auto* driver = block_->findInst("driver");
  auto* cell = db_network_->libertyCell(driver);
  cell->findLibertyPort("Z")->setSlewLimit(0.04e-9f, sta::MinMax::max());
  refreshRepairTiming();
  ecc::DesignRepairOptions options;
  options.vt_suffixes = {"_H", "_L"};
  options.allow_load_downsize = false;
  options.max_inserted_buffers = 0;
  options.max_passes = 1;
  const auto result = ecc::OpenDbDesignRepairer::repair(block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.vt_swaps, 1);
  EXPECT_GT(result.before.slew_nets, 0);
  EXPECT_EQ(result.after.slew_nets, 0);
  EXPECT_EQ(result.after.area, result.before.area);
  EXPECT_EQ(driver->getMaster()->getName(), "VTBUF_L");
}

TEST_F(BufferInserterStaTest, VtCandidatesRejectWrongFunctionCornerAndProtection)
{
  readVtDesign();
  ecc::RepairCommonOptions options;
  options.vt_suffixes = {"_H", "_L", "_B", "_X"};
  auto* driver = block_->findInst("driver");
  const auto inspect = [&](const ecc::RepairCommonOptions& common) {
    ecc::detail::Repair backend(block_, sta_.get(), common);
    ecc::DesignRepairMetrics metrics;
    const auto states = backend.analyze(metrics, true);
    std::vector<const ecc::detail::Buffer*> found;
    for (const auto& state : states)
      if (state.driver->getInst() == driver) {
        found = backend.resizeCandidates(state, false, true);
        EXPECT_LE(found.size(), 1);
        if (!found.empty()) EXPECT_EQ(found.front()->master->getName(), "VTBUF_L");
      }
    return found.size();
  };
  EXPECT_EQ(inspect(options), 1);
  driver->setDoNotTouch(true);
  EXPECT_EQ(inspect(options), 0);
  driver->setDoNotTouch(false);
  driver->setPlacementStatus(odb::dbPlacementStatus::LOCKED);
  EXPECT_EQ(inspect(options), 0);
  driver->setPlacementStatus(odb::dbPlacementStatus::PLACED);
  options.excluded_masters = {lib_->findMaster("VTBUF_L")};
  EXPECT_EQ(inspect(options), 0);
  options.excluded_masters.clear();
  driver->findITerm("A")->getNet()->setDoNotTouch(true);
  EXPECT_EQ(inspect(options), 0);
}

TEST_F(BufferInserterStaTest, PowerRecoveryLowersLeakageWithVtAndPreservesTiming)
{
  readVtDesign();
  constrainSetup(true); // Keep an unrelated negative-slack path unchanged.
  refreshRepairTiming();
  ecc::PowerRepairOptions options;
  options.enabled = true;
  options.common.vt_suffixes = {"_H", "_L"};
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::OpenDbPowerRepairer::repair(block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.vt_swaps, 1);
  EXPECT_EQ(result.downsized_cells, 0);
  EXPECT_GT(result.after.leakage, 0);
  EXPECT_LT(result.after.leakage, result.before.leakage);
  EXPECT_LT(result.after.total, result.before.total);
  EXPECT_EQ(result.after.design.setup_wns, result.before.design.setup_wns);
  EXPECT_EQ(result.after.design.hold_wns, result.before.design.hold_wns);
  EXPECT_EQ(block_->findInst("side")->getMaster()->getName(), "VTBUF_H");
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PowerRecoveryHonorsDisableBudgetSlackAndTotalObjective)
{
  readVtDesign();
  ecc::PowerRepairOptions options;
  options.common.vt_suffixes = {"_H", "_L"};
  const auto run = [&]() { return ecc::OpenDbPowerRepairer::repair(block_, sta_.get(), [&]() { refreshRepairTiming(); }, options); };
  EXPECT_EQ(run().attempted_batches, 0);
  options.enabled = true;
  options.max_trials = 0;
  EXPECT_EQ(run().attempted_batches, 0);
  options.max_trials = 1;
  options.slack_margin = 1;
  EXPECT_EQ(run().attempted_batches, 0);
  options.slack_margin = 0;
  options.objective = "total";
  const auto result = run();
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.attempted_batches, 1);
  EXPECT_EQ(result.vt_swaps, 1);
  EXPECT_LT(result.after.total, result.before.total);
}

TEST_F(BufferInserterStaTest, PowerRecoveryRollsBackBadTimingAndPrepareException)
{
  readVtDesign();
  auto* side = block_->findInst("side");
  auto* original = side->getMaster();
  ecc::PowerRepairOptions options;
  options.enabled = true;
  options.max_trials = 1;
  options.common.vt_suffixes = {"_H", "_L"};
  const auto refresh = [&]() {
    const bool changed = side->getMaster() != original;
    for (auto* scene : sta_->scenes()) {
      ep_.setHWireSignalRC(scene, changed ? 1e12 : 1e6, changed ? 1e-4 : 1e-10);
      ep_.setVWireSignalRC(scene, changed ? 1e12 : 1e6, changed ? 1e-4 : 1e-10);
    }
    refreshRepairTiming();
  };
  auto result = ecc::OpenDbPowerRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.rejected_batches, 1);
  EXPECT_EQ(result.accepted_batches, 0);
  EXPECT_EQ(result.before.leakage, result.after.leakage);
  EXPECT_EQ(side->getMaster(), original);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  options.common.prepare = []() { throw std::runtime_error("power prepare failure"); };
  result = ecc::OpenDbPowerRepairer::repair(block_, sta_.get(), refresh, options);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("power prepare failure"), std::string::npos);
  EXPECT_EQ(side->getMaster(), original);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PowerRecoveryDownsizesSlackRichCells)
{
  readTestDesign("gate_repair_sta.v");
  oversizeLoads();
  addLegalRows();
  refreshRepairTiming();
  ecc::PowerRepairOptions options;
  options.enabled = true;
  options.max_trials = 2;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::OpenDbPowerRepairer::repair(block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.downsized_cells, 0);
  EXPECT_EQ(result.vt_swaps, 0);
  EXPECT_LT(result.after.leakage, result.before.leakage);
  EXPECT_LT(result.after.design.area, result.before.design.area);
}

}  // namespace
