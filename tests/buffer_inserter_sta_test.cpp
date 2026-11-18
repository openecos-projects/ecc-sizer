#include "repair_pbbi_electrical.h"
// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include <fstream>
#include <array>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <tuple>
#include <regex>
#include <set>
#include <sstream>

#include "buffer_inserter.h"
#include "repair_fanout.h"
#include "repair_design.h"
#include "repair_vg.h"
#include "repair_path.h"
#include "repair_backend.h"
#include "repair_pbbi.h"
#include "repair_timing.h"
#include "repair_load.h"
#include "repair_power.h"
#include "sta/Sdc.hh"
#include "sta/Mode.hh"
#include "sta/Parasitics.hh"
#include "sta/Scene.hh"
#include "repair_placement.h"
#include "sta/EquivCells.hh"
#include <stdexcept>
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbReadVerilog.hh"
#include "est/EstimateParasitics.h"
#include "repair_parasitics.h"
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

  void excludeAllBuffers(ecc::DesignRepairOptions& options)
  {
    for (auto* master : lib_->getMasters()) {
      auto* cell = db_network_->libertyCell(db_network_->dbToSta(master));
      if (cell && cell->isBuffer())
        options.excluded_masters.push_back(master);
    }
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

TEST_F(BufferInserterStaTest, IncrementalPlacementMatchesFullRcAfterMoveOrientationAndRollback)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  const auto metrics = [&]() {
    return std::array<double, 4>{sta_->worstSlack(sta::MinMax::max()),
                                sta_->totalNegativeSlack(sta::MinMax::max()),
                                sta_->worstSlack(sta::MinMax::min()),
                                sta_->totalNegativeSlack(sta::MinMax::min())};
  };
  const auto initial = metrics();
  auto* moved = block_->findInst("load9");
  const auto original_orientation = moved->getOrient();
  {
    ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
    const auto check = [&]() {
      incremental.update();
      const auto observed = metrics();
      std::vector<float> endpoints;
      for(auto* port : block_->getBTerms())
        if(port->getIoType() == odb::dbIoType::OUTPUT)
          endpoints.push_back(sta_->slack(db_network_->dbToSta(port),
              sta::RiseFallBoth::riseFall(), sta_->scenes(), sta::MinMax::max()));
      refreshRepairTiming();
      const auto full = metrics();
      for(size_t i = 0; i < full.size(); ++i) EXPECT_DOUBLE_EQ(observed[i], full[i]);
      size_t i = 0;
      for(auto* port : block_->getBTerms())
        if(port->getIoType() == odb::dbIoType::OUTPUT)
          EXPECT_FLOAT_EQ(endpoints[i++], sta_->slack(db_network_->dbToSta(port),
              sta::RiseFallBoth::riseFall(), sta_->scenes(), sta::MinMax::max()));
    };
    odb::dbDatabase::beginEco(block_);
    auto inserted = ecc::OpenDbBufferInserter::insertBufferBeforeLoads(
        block_->findNet("high_fanout"), {block_->findInst("load0")->findITerm("A")},
        lib_->findMaster("BUF_X2"));
    ASSERT_TRUE(inserted.ok()) << inserted.message;
    moved->setLocation(35000, 5000);
    moved->setOrient(odb::dbOrientType::MY);
    check();
    odb::dbDatabase::endEco(block_);
    odb::dbDatabase::undoEco(block_);
    EXPECT_EQ(moved->getOrient(), original_orientation);
    check();
    EXPECT_EQ(metrics(), initial);

    // Cover a deferred rollback refresh followed by another ECO, including
    // orientation rollback, which OpenDB restores without a move callback.
    odb::dbDatabase::beginEco(block_);
    moved->setLocation(38000, 7000);
    check();
    odb::dbDatabase::endEco(block_);
    odb::dbDatabase::undoEco(block_);
    odb::dbDatabase::beginEco(block_);
    moved->setOrient(odb::dbOrientType::MX);
    check();
    odb::dbDatabase::endEco(block_);
    odb::dbDatabase::undoEco(block_);
    check();
    EXPECT_EQ(metrics(), initial);

    // Deleted provisional cells must not remain as dangling watched pointers.
    // OpenDB may reuse the instance slot for this next insertion.
    odb::dbDatabase::beginEco(block_);
    auto replacement = ecc::OpenDbBufferInserter::insertBufferBeforeLoads(
        block_->findNet("high_fanout"), {block_->findInst("load1")->findITerm("A")},
        lib_->findMaster("BUF_X8"));
    ASSERT_TRUE(replacement.ok()) << replacement.message;
    replacement.buffer->setLocation(31000, 3000);
    replacement.buffer->setOrient(odb::dbOrientType::MY);
    check();
    odb::dbDatabase::endEco(block_);
    odb::dbDatabase::undoEco(block_);
    check();
    EXPECT_EQ(metrics(), initial);
  }
  EXPECT_FALSE(ep_.isIncrementalParasiticsEnabled());
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, IncrementalElectricalAnalysisMatchesFullScanAcrossEcoAndUndo)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::RepairCommonOptions options;
  ecc::detail::Repair cached(block_, sta_.get(), options);
  ecc::detail::Repair reference(block_, sta_.get(), options);
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto check = [&]() {
    incremental.update();
    incremental.verifyTiming();
    ecc::DesignRepairMetrics actual, expected;
    auto states = cached.analyzeIncremental(actual, incremental.takeAnalysisChanges());
    auto full = reference.analyze(expected, true);
    ASSERT_GT(full.size(), 0u);
    EXPECT_FLOAT_EQ(actual.setup_wns, expected.setup_wns);
    EXPECT_FLOAT_EQ(actual.setup_tns, expected.setup_tns);
    EXPECT_FLOAT_EQ(actual.hold_wns, expected.hold_wns);
    EXPECT_FLOAT_EQ(actual.hold_tns, expected.hold_tns);
    EXPECT_EQ(actual.total_slew_violations, expected.total_slew_violations);
    EXPECT_EQ(actual.total_cap_violations, expected.total_cap_violations);
    EXPECT_EQ(actual.total_fanout_violations, expected.total_fanout_violations);
    EXPECT_EQ(actual.skipped_nets, expected.skipped_nets);
    EXPECT_EQ(actual.slew_nets, expected.slew_nets);
    EXPECT_EQ(actual.capacitance_nets, expected.capacitance_nets);
    EXPECT_EQ(actual.fanout_nets, expected.fanout_nets);
    EXPECT_DOUBLE_EQ(actual.area, expected.area);
    EXPECT_DOUBLE_EQ(actual.severity, expected.severity);
    ASSERT_EQ(states.size(), full.size());
    for(size_t i = 0; i < states.size(); ++i) {
      const auto& a = states[i]; const auto& b = full[i];
      ASSERT_EQ(a.net, b.net);
      EXPECT_EQ(a.driver, b.driver);
      EXPECT_FLOAT_EQ(a.capacitance, b.capacitance);
      EXPECT_FLOAT_EQ(a.cap_ratio, b.cap_ratio);
      EXPECT_FLOAT_EQ(a.slew_ratio, b.slew_ratio);
      EXPECT_FLOAT_EQ(a.fanout_ratio, b.fanout_ratio);
      EXPECT_FLOAT_EQ(a.slew, b.slew);
      EXPECT_FLOAT_EQ(a.slew_limit, b.slew_limit);
      EXPECT_FLOAT_EQ(a.input_slew, b.input_slew);
      EXPECT_EQ(a.propagated_slew, b.propagated_slew);
      ASSERT_EQ(a.loads.size(), b.loads.size());
      for(size_t j = 0; j < a.loads.size(); ++j) {
        EXPECT_EQ(a.loads[j].term, b.loads[j].term);
        EXPECT_FLOAT_EQ(a.loads[j].capacitance, b.loads[j].capacitance);
        EXPECT_EQ(a.loads[j].x, b.loads[j].x);
        EXPECT_EQ(a.loads[j].y, b.loads[j].y);
      }
    }
  };
  check();
  const auto net_checks = cached.analysisStatistics().net_checks;
  const auto instance_checks = cached.analysisStatistics().instance_checks;
  check(); // Unchanged inspection does no electrical re-evaluation.
  EXPECT_EQ(cached.analysisStatistics().net_checks, net_checks);
  EXPECT_EQ(cached.analysisStatistics().instance_checks, instance_checks);
  for(int pass = 0; pass < 3; ++pass) {
    odb::dbDatabase::beginEco(block_);
    auto edit = ecc::OpenDbBufferInserter::insertBufferBeforeLoads(
        block_->findNet("high_fanout"), {block_->findInst("load0")->findITerm("A")},
        lib_->findMaster(pass == 1 ? "BUF_X8" : "BUF_X2"));
    ASSERT_TRUE(edit.ok()) << edit.message;
    auto* moved = block_->findInst("load9");
    moved->setLocation(35000 + 1000 * pass, 5000);
    moved->setOrient(odb::dbOrientType::MY);
    check();
    odb::dbDatabase::endEco(block_);
    odb::dbDatabase::undoEco(block_);
    if(pass != 1) check(); // Also cover deferred rollback and reused DB slots.
  }
  check();
  EXPECT_EQ(cached.analysisStatistics().full_scans, 1u);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, IncrementalElectricalInspectionSkipsUnchangedDownstreamSlew)
{
  readTestDesign("repair_slew_cache_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  incremental.takeAnalysisChanges(); // Seed exact slews, including all scenes.
  auto* moved = block_->findInst("driver");
  int x, y;
  moved->getLocation(x, y);
  moved->setLocation(x + 1000, y);
  moved->setLocation(x, y); // Geometry is unchanged at the inspection boundary.
  incremental.update();
  incremental.verifyTiming();
  const auto visited = incremental.statistics().analysis_vertices;
  auto changed = incremental.takeAnalysisChanges();
  EXPECT_GT(incremental.statistics().analysis_vertices, visited);
  EXPECT_TRUE(changed.instances.count(moved));
  EXPECT_TRUE(changed.instances.count(block_->findInst("load0")));
  EXPECT_FALSE(changed.instances.count(block_->findInst("tail")));
  EXPECT_FALSE(changed.instances.count(block_->findInst("sink")));
  EXPECT_TRUE(changed.nets.count(block_->findNet("n1")));
  EXPECT_FALSE(changed.nets.count(block_->findNet("n3")));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, IncrementalPathEveryTrialMatchesFullSta)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_strategy = "feedback";
  options.path_buffer_max_trials = 8;
  options.path_buffer_max_area_growth = 0.5;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned verified = 0;
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto refresh = [&]() {
    SCOPED_TRACE(verified);
    incremental.update();
    incremental.verifyTiming();
    const auto wns = sta_->worstSlack(sta::MinMax::max());
    const auto tns = sta_->totalNegativeSlack(sta::MinMax::max());
    const auto wire_caps = [&]() {
      std::map<std::string, float> caps;
      auto* parasitics = sta_->scenes().front()->parasitics(sta::MinMax::max());
      for(auto* net : block_->getNets()) {
        auto* network = parasitics->findParasiticNetwork(db_network_->dbToSta(net));
        caps[net->getName()] = network ? parasitics->capacitance(network) : -1;
      }
      return caps;
    };
    const auto incremental_caps = wire_caps();
    ecc::detail::Repair backend(block_, sta_.get(), options.common);
    ecc::DesignRepairMetrics incremental_metrics, full_metrics;
    const auto incremental_states = backend.analyze(incremental_metrics, true);
    // updateTiming(true) only invalidates arrivals; explicitly invalidate
    // delays/slews to compare against a full delay calculation on fixed RC.
    sta_->delaysInvalid();
    sta_->findRequireds();
    const auto full_sta_wns = sta_->worstSlack(sta::MinMax::max());
    const auto full_states = backend.analyze(full_metrics, true);
    EXPECT_DOUBLE_EQ(incremental_metrics.severity, full_metrics.severity);
    EXPECT_EQ(incremental_metrics.total_slew_violations, full_metrics.total_slew_violations);
    EXPECT_EQ(incremental_metrics.total_cap_violations, full_metrics.total_cap_violations);
    ASSERT_EQ(incremental_states.size(), full_states.size());
    for(size_t i = 0; i < full_states.size(); ++i) {
      EXPECT_EQ(incremental_states[i].net, full_states[i].net);
      EXPECT_EQ(incremental_states[i].slew_ratio, full_states[i].slew_ratio);
      EXPECT_EQ(incremental_states[i].cap_ratio, full_states[i].cap_ratio);
    }
    EXPECT_FLOAT_EQ(wns, full_sta_wns) << "RC unchanged, STA recomputed";
    refreshRepairTiming();
    for(const auto& [name, cap] : wire_caps())
      EXPECT_FLOAT_EQ(incremental_caps.at(name), cap) << name;
    EXPECT_FLOAT_EQ(full_sta_wns, sta_->worstSlack(sta::MinMax::max())) << "RC then recomputed";
    EXPECT_FLOAT_EQ(wns, sta_->worstSlack(sta::MinMax::max()));
    EXPECT_FLOAT_EQ(tns, sta_->totalNegativeSlack(sta::MinMax::max()));
    ++verified;
  };
  const auto result = ecc::OpenDbPathRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.inserted_buffers, 0u);
  EXPECT_GE(verified, result.attempted_trials);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, IncrementalPathSequenceMatchesFinalFullSta)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_strategy = "feedback";
  options.path_buffer_max_trials = 16;
  options.path_buffer_max_area_growth = -1;
  options.defer_rollback_refresh = true;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { incremental.update(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.accepted_trials, 0u);
  EXPECT_GT(result.rejected_trials, 0u);
  refreshRepairTiming();
  EXPECT_FLOAT_EQ(result.after.setup_wns, sta_->worstSlack(sta::MinMax::max()));
  EXPECT_FLOAT_EQ(result.after.setup_tns, sta_->totalNegativeSlack(sta::MinMax::max()));
  EXPECT_FLOAT_EQ(result.after.hold_wns, std::min(0.0f, float(sta_->worstSlack(sta::MinMax::min()))));
  EXPECT_FLOAT_EQ(result.after.hold_tns, sta_->totalNegativeSlack(sta::MinMax::min()));
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, IncrementalPathFullyVerifiesBeforeEveryCommit)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_strategy = "feedback";
  options.path_buffer_max_trials = 16;
  options.path_buffer_max_area_growth = -1;
  options.defer_rollback_refresh = true;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned full_refreshes = 0, verified_trials = 0;
  options.path_verification_refresh = [&]() { ++full_refreshes; refreshRepairTiming(); };
  options.path_trial_observer = [&](unsigned, const ecc::PathBufferTrial& trial) {
    if(trial.full_verified) ++verified_trials;
    if(trial.rejection_reasons.empty()) EXPECT_TRUE(trial.full_verified);
  };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { incremental.update(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.accepted_trials, 0u);
  EXPECT_GT(result.rejected_trials, 0u);
  EXPECT_EQ(full_refreshes, verified_trials + result.rollback_refreshes);
  EXPECT_EQ(result.refreshes, result.attempted_trials + verified_trials + result.rollback_refreshes);
  refreshRepairTiming();
  EXPECT_FLOAT_EQ(result.after.setup_wns, sta_->worstSlack(sta::MinMax::max()));
  EXPECT_FLOAT_EQ(result.after.setup_tns, sta_->totalNegativeSlack(sta::MinMax::max()));
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathFinalPlacementRunsOnceAndReportsLegalFullSta)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_strategy = "feedback";
  options.path_buffer_placement = "final";
  options.path_buffer_max_trials = 16;
  options.path_buffer_max_area_growth = -1;
  unsigned prepared = 0, observed = 0;
  options.common.prepare = [&]() {
    ++prepared;
    EXPECT_GT(observed, 0u);
    ecc::legalizeRepairPlacement(block_, &dp_);
  };
  options.path_trial_observer = [&](unsigned, const ecc::PathBufferTrial& trial) {
    EXPECT_TRUE(trial.placement_provisional);
    EXPECT_EQ(prepared, 0u);
    ++observed;
  };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  options.path_verification_refresh = [&]() { refreshRepairTiming(); };
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { incremental.update(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.final_placement.provisional_buffers, 0u);
  EXPECT_GT(result.attempted_trials, 1u);
  EXPECT_EQ(prepared, 1u);
  EXPECT_TRUE(result.final_placement.attempted);
  EXPECT_TRUE(result.final_placement.accepted) << result.message;
  EXPECT_NE(result.final_placement.accepted, result.final_placement.rolled_back);
  for(const auto& trial : result.trials) EXPECT_TRUE(trial.placement_provisional);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  refreshRepairTiming();
  EXPECT_FLOAT_EQ(result.after.setup_wns, sta_->worstSlack(sta::MinMax::max()));
  EXPECT_FLOAT_EQ(result.after.setup_tns, sta_->totalNegativeSlack(sta::MinMax::max()));
  EXPECT_GE(result.after.setup_wns + options.common.tolerances.wns_absolute, result.before.setup_wns);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathFinalPlacementRestoresAllProvisionalEditsAndMoves)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  std::map<std::string, std::tuple<odb::Point, odb::dbOrientType, odb::dbPlacementStatus>> placements;
  std::map<std::string, std::string> connections;
  for(auto* inst : block_->getInsts()) {
    placements.emplace(inst->getName(), std::make_tuple(inst->getLocation(), inst->getOrient(), inst->getPlacementStatus()));
    for(auto* pin : inst->getITerms())
      connections[std::string(inst->getName()) + "/" + pin->getMTerm()->getName()] = pin->getNet() ? pin->getNet()->getName() : "";
  }
  const auto nets = block_->getNets().size();
  const auto original_wns = sta_->worstSlack(sta::MinMax::max());
  const auto original_tns = sta_->totalNegativeSlack(sta::MinMax::max());
  // Both final STA rejection and an exception after DP must restore the
  // entire parent transaction, including accepted child ECOs and old cells.
  for(const auto* verification : {"trial", "final"})
  for(bool throw_after_move : {false, true}) {
    ecc::SetupRepairOptions options;
    options.path_buffer_strategy = "feedback";
    options.path_buffer_placement = "final";
    options.path_buffer_verification = verification;
    options.path_verification_refresh = [&]() { refreshRepairTiming(); };
    options.path_buffer_max_trials = 16;
    options.path_buffer_max_area_growth = -1;
    options.common.prepare = [&]() {
      ecc::legalizeRepairPlacement(block_, &dp_);
      block_->findInst("load9")->setLocation(100000000, 0);
      if(throw_after_move) throw std::runtime_error("injected final placement failure");
    };
    ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
    const auto result = ecc::OpenDbPathRepairer::repair(
        block_, sta_.get(), [&]() { incremental.update(); }, options);
    EXPECT_EQ(result.ok, !throw_after_move) << result.message;
    ASSERT_TRUE(result.final_placement.attempted);
    EXPECT_GT(result.final_placement.provisional_buffers, 0u);
    EXPECT_TRUE(result.final_placement.rolled_back);
    EXPECT_FALSE(result.final_placement.accepted);
    EXPECT_EQ(result.inserted_buffers, 0u);
    EXPECT_EQ(result.accepted_trials, 0u);
    EXPECT_EQ(block_->getInsts().size(), placements.size());
    EXPECT_EQ(block_->getNets().size(), nets);
    for(auto* inst : block_->getInsts()) {
      EXPECT_EQ(std::make_tuple(inst->getLocation(), inst->getOrient(), inst->getPlacementStatus()), placements.at(inst->getName()));
      for(auto* pin : inst->getITerms())
        EXPECT_EQ(pin->getNet() ? pin->getNet()->getName() : "", connections.at(std::string(inst->getName()) + "/" + pin->getMTerm()->getName()));
    }
    refreshRepairTiming();
    EXPECT_FLOAT_EQ(original_wns, sta_->worstSlack(sta::MinMax::max()));
    EXPECT_FLOAT_EQ(original_tns, sta_->totalNegativeSlack(sta::MinMax::max()));
    EXPECT_FLOAT_EQ(result.after.setup_wns, original_wns);
    EXPECT_FLOAT_EQ(result.after.setup_tns, original_tns);
    EXPECT_NO_THROW(dp_.checkPlacement(false));
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
    db_network_->checkAxioms();
  }
}

TEST_F(BufferInserterStaTest, PathFinalVerificationAmortizesFullRcAndRequiresJournal)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_strategy = "feedback";
  options.path_buffer_verification = "final";
  options.path_buffer_max_trials = 16;
  options.path_buffer_max_area_growth = -1;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto instances = block_->getInsts().size();
  unsigned full_refreshes = 0;
  const auto full = [&]() { ++full_refreshes; refreshRepairTiming(); };
  auto rejected = ecc::OpenDbPathRepairer::repair(block_, sta_.get(), full, options);
  EXPECT_FALSE(rejected.ok);
  options.path_buffer_placement = "final";
  rejected = ecc::OpenDbPathRepairer::repair(block_, sta_.get(), full, options);
  EXPECT_FALSE(rejected.ok); // Missing authoritative callback.
  EXPECT_EQ(full_refreshes, 0u);
  EXPECT_EQ(block_->getInsts().size(), instances);
  options.path_verification_refresh = full;
  options.path_trial_observer = [&](unsigned, const ecc::PathBufferTrial& trial) {
    EXPECT_TRUE(trial.placement_provisional);
    EXPECT_FALSE(trial.full_verified);
    EXPECT_EQ(full_refreshes, 0u);
    if(trial.rejection_reasons.empty()) EXPECT_TRUE(trial.metrics_complete);
  };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  options.path_incremental_verification = [&]() { incremental.verifyTiming(); };
  options.path_buffer_incremental_analysis = true;
  options.path_buffer_reuse_rejections = true;
  options.path_analysis_changes = [&]() { return incremental.takeAnalysisChanges(); };
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { incremental.update(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.accepted_trials, 0u);
  EXPECT_GT(result.rejected_trials, 0u);
  EXPECT_TRUE(result.final_placement.accepted) << result.message;
  EXPECT_EQ(full_refreshes, 1u);
  EXPECT_GT(incremental.statistics().verifications, 0u);
  EXPECT_LT(incremental.statistics().verifications, result.attempted_trials);
  ecc::detail::Repair backend(block_, sta_.get(), options.common);
  ecc::DesignRepairMetrics measured;
  refreshRepairTiming();
  backend.analyze(measured, true);
  EXPECT_FLOAT_EQ(result.after.setup_wns, measured.setup_wns);
  EXPECT_FLOAT_EQ(result.after.setup_tns, measured.setup_tns);
  EXPECT_FLOAT_EQ(result.after.hold_wns, measured.hold_wns);
  EXPECT_FLOAT_EQ(result.after.hold_tns, measured.hold_tns);
  EXPECT_EQ(result.after.total_slew_violations, measured.total_slew_violations);
  EXPECT_EQ(result.after.total_cap_violations, measured.total_cap_violations);
  EXPECT_EQ(result.after.total_fanout_violations, measured.total_fanout_violations);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathRejectionStopStillFinalizesAndKeepsAcceptedEdits)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  const auto instances = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  ecc::SetupRepairOptions options;
  options.path_buffer_strategy = "feedback";
  options.path_buffer_placement = "final";
  options.path_buffer_max_trials = 0;
  options.path_buffer_max_consecutive_rejections = 2;
  options.path_buffer_max_area_growth = -1;
  unsigned prepared = 0;
  options.common.prepare = [&]() {
    ++prepared;
    ecc::legalizeRepairPlacement(block_, &dp_);
  };
  options.path_verification_refresh = [&]() { refreshRepairTiming(); };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { incremental.update(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_TRUE(result.stopped_on_rejections) << result.message;
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.consecutive_rejections, 2u);
  ASSERT_GT(result.inserted_buffers, 0u);
  EXPECT_EQ(prepared, 1u);
  EXPECT_TRUE(result.final_placement.accepted) << result.message;
  EXPECT_EQ(block_->getInsts().size(), instances + result.inserted_buffers);
  EXPECT_EQ(block_->getNets().size(), nets + result.inserted_buffers);
  EXPECT_NE(result.message.find("search not exhausted"), std::string::npos);
  ASSERT_GE(result.trials.size(), 3u);
  EXPECT_FALSE(result.trials.back().rejection_reasons.empty());
  EXPECT_FALSE(result.trials[result.trials.size() - 2].rejection_reasons.empty());
  EXPECT_TRUE(result.trials[result.trials.size() - 3].rejection_reasons.empty());
  refreshRepairTiming();
  EXPECT_FLOAT_EQ(result.after.setup_wns, sta_->worstSlack(sta::MinMax::max()));
  EXPECT_FLOAT_EQ(result.after.setup_tns, sta_->totalNegativeSlack(sta::MinMax::max()));
  EXPECT_GE(result.after.setup_wns + options.common.tolerances.wns_absolute, result.before.setup_wns);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathTimingPrecheckMatchesFullMetricsAndNeverAcceptsAlone)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_strategy = "feedback";
  options.path_buffer_adaptive_search = true;
  options.path_buffer_timing_precheck = true;
  options.path_buffer_max_trials = 32;
  options.path_buffer_max_area_growth = -1;
  options.defer_rollback_refresh = true;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  ecc::DesignRepairMetrics independently_measured;
  const auto refresh = [&]() {
    refreshRepairTiming();
    ecc::detail::Repair backend(block_, sta_.get(), options.common);
    backend.analyze(independently_measured, true);
  };
  unsigned partial = 0;
  options.path_verification_refresh = refresh;
  options.path_trial_observer = [&](unsigned, const ecc::PathBufferTrial& trial) {
    EXPECT_FLOAT_EQ(trial.after.setup_wns, independently_measured.setup_wns);
    EXPECT_FLOAT_EQ(trial.after.setup_tns, independently_measured.setup_tns);
    EXPECT_FLOAT_EQ(trial.after.hold_wns, independently_measured.hold_wns);
    EXPECT_FLOAT_EQ(trial.after.hold_tns, independently_measured.hold_tns);
    if(!trial.metrics_complete) {
      ++partial;
      EXPECT_FALSE(trial.rejection_reasons.empty());
      EXPECT_FALSE(trial.full_verified);
    }
    if(trial.rejection_reasons.empty()) {
      EXPECT_TRUE(trial.metrics_complete);
      EXPECT_TRUE(trial.full_verified);
      EXPECT_EQ(trial.after.total_slew_violations, independently_measured.total_slew_violations);
      EXPECT_DOUBLE_EQ(trial.after.severity, independently_measured.severity);
    }
  };
  const auto result = ecc::OpenDbPathRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.accepted_trials, 0u);
  EXPECT_GT(partial, 0u);
  EXPECT_EQ(partial, result.timing_precheck_rejections);
  refreshRepairTiming();
  EXPECT_FLOAT_EQ(result.after.setup_wns, sta_->worstSlack(sta::MinMax::max()));
  EXPECT_FLOAT_EQ(result.after.setup_tns, sta_->totalNegativeSlack(sta::MinMax::max()));
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, IncrementalPathRollsBackIfFullVerificationFails)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  const auto count = block_->getInsts().size();
  const auto wns = sta_->worstSlack(sta::MinMax::max());
  const auto tns = sta_->totalNegativeSlack(sta::MinMax::max());
  ecc::SetupRepairOptions options;
  options.path_buffer_strategy = "feedback";
  options.path_buffer_max_trials = 16;
  options.path_buffer_max_area_growth = -1;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned full_refreshes = 0;
  options.path_verification_refresh = [&]() {
    if(++full_refreshes == 1) throw std::runtime_error("full verification failed");
    refreshRepairTiming();
  };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { incremental.update(); }, options);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("full verification failed"), std::string::npos);
  EXPECT_EQ(result.inserted_buffers, 0u);
  EXPECT_EQ(full_refreshes, 2u);
  EXPECT_EQ(block_->getInsts().size(), count);
  EXPECT_FLOAT_EQ(sta_->worstSlack(sta::MinMax::max()), wns);
  EXPECT_FLOAT_EQ(sta_->totalNegativeSlack(sta::MinMax::max()), tns);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

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

TEST_F(BufferInserterStaTest, VgOneShotLeavesCleanDesignUnchanged)
{
  readTestDesign();
  const size_t instances_before = block_->getInsts().size();
  const size_t nets_before = block_->getNets().size();
  ecc::VgRepairOptions options;
  options.common.max_fanout = 64;
  options.max_buffers = 4;
  const auto result = ecc::OpenDbVgRepairer::repair(
      block_, sta_.get(), [this]() { refreshRepairTiming(); }, &resizer_, options);
  EXPECT_TRUE(result.ok) << result.message;
  EXPECT_FALSE(result.committed);
  EXPECT_EQ(result.plans_collected, 0u);
  EXPECT_EQ(block_->getInsts().size(), instances_before);
  EXPECT_EQ(block_->getNets().size(), nets_before);
}

TEST_F(BufferInserterStaTest, VgOneShotAppliesOneAtomicPlan)
{
  readTestDesign("fanout_repair_sta.v");
  // The CLI has not necessarily called a Resizer command before VG. Verify
  // that VG initializes DBU conversion even with a fresh Resizer instance.
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
  ecc::VgRepairOptions options;
  // This integration target deliberately links the CPU stub: requesting
  // CUDA must fall back before any ECO, then apply the same atomic plan.
  options.backend = "cuda";
  options.common.max_fanout = 3;
  options.max_buffers = 64;
  options.max_group_loads = 3;
  options.require_drv_reduction = true;
  const auto result = ecc::OpenDbVgRepairer::repair(
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

  // The next repairs must analyze the inserted topology, not a pre-VG
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

TEST_F(BufferInserterStaTest, VgOneShotRollsBackWhenTimingGetsWorse)
{
  readTestDesign("fanout_repair_sta.v");
  const size_t instances_before = block_->getInsts().size();
  ecc::VgRepairOptions options;
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
      ecc::OpenDbVgRepairer::repair(block_, sta_.get(), refresh, options);
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

TEST_F(BufferInserterStaTest, NativeDesignRepairHonorsPositiveBufferBudget)
{
  readTestDesign("fanout_repair_sta.v");
  ecc::DesignRepairOptions options;
  options.max_fanout = 3;
  options.max_inserted_buffers = 1;
  const auto count = block_->getInsts().size();
  const auto result = ecc::OpenDbDesignRepairer::repair(
      block_, sta_.get(), [&]() { sta_->updateTiming(true); }, options);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.converged);
  EXPECT_GT(result.after.fanout_nets, 0);
  EXPECT_EQ(result.inserted_buffers, 1);
  EXPECT_EQ(block_->getInsts().size(), count + 1);
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
  options.max_inserted_buffers = 1;
  excludeAllBuffers(options);
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

TEST_F(BufferInserterStaTest, PathBufferOnlyRepairUsesVerifiedPathEdit)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_enabled = true;
  options.path_buffer_only = true;
  options.path_buffer_max_paths = 1;
  options.path_buffer_max_candidates = 4;
  options.path_buffer_max_trials = 4;
  options.path_buffer_max_area_growth = 0.5;
  options.common.allow_gate_resize = false;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::OpenDbSetupRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.path_buffer_attempted, 0u);
  EXPECT_GT(result.path_buffer_accepted, 0u);
  EXPECT_EQ(result.path_buffer_inserted_buffers, result.path_buffer_accepted);
  EXPECT_EQ(result.path_buffer_trials.size(), result.path_buffer_attempted);
  EXPECT_LE(result.refreshes, result.path_buffer_attempted + 1);
  EXPECT_LE(result.rollback_refreshes, 1u);
  EXPECT_GT(result.after.design.setup_wns, result.before.design.setup_wns);
  EXPECT_GT(result.inserted_buffers, 0u);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, StandalonePathRepairWithoutBudgetsUsesVerifiedPathEdit)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_max_paths = 1;
  options.path_buffer_max_candidates = 4;
  options.path_buffer_max_trials = 0;
  options.path_buffer_max_area_growth = 0;
  options.common.allow_gate_resize = false;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto no_growth = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(no_growth.ok) << no_growth.message;
  EXPECT_EQ(no_growth.attempted_trials, 0u);
  EXPECT_EQ(no_growth.inserted_buffers, 0u);
  options.path_buffer_max_area_growth = -1;
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.accepted_trials, 0u);
  EXPECT_EQ(result.inserted_buffers, result.accepted_trials);
  EXPECT_GT(result.after.setup_wns, result.before.setup_wns);
  EXPECT_NE(result.message, "path buffer trial budget exhausted; violations remain");
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathFeedbackReportsCommittedAndRolledBackTrials)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_strategy = "feedback";
  options.path_buffer_max_trials = 4;
  options.path_buffer_max_area_growth = 0.5;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto initial_instances = block_->getInsts().size();
  unsigned reports = 0;
  unsigned accepted = 0;
  options.path_trial_observer = [&](unsigned index, const ecc::PathBufferTrial& trial) {
    EXPECT_EQ(index, ++reports);
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
    if(trial.rejection_reasons.empty()) accepted += trial.buffers;
    EXPECT_EQ(block_->getInsts().size(), initial_instances + accepted);
  };
  // A 100% initial-slack threshold cannot be exceeded by negative slack
  // clamped at zero. All edits must be restored, despite optimistic forecasts.
  options.path_buffer_min_relative_improvement = 1;
  // Exercise actual STA rejection independently of prediction screening.
  options.path_buffer_strategy = "path_gain";
  const auto rejected = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(rejected.ok) << rejected.message;
  EXPECT_GT(reports, 0u);
  EXPECT_EQ(reports, rejected.attempted_trials);
  EXPECT_EQ(accepted, 0u);
  EXPECT_FLOAT_EQ(rejected.before.setup_wns, rejected.after.setup_wns);
  reports = 0;
  options.path_buffer_strategy = "feedback";
  const auto screened = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(screened.ok) << screened.message;
  EXPECT_GT(screened.screened_candidates, 0u);
  EXPECT_EQ(accepted, 0u);
  EXPECT_EQ(reports, screened.attempted_trials);
  EXPECT_EQ(block_->getInsts().size(), initial_instances);
  reports = 0;
  options.path_buffer_min_relative_improvement = 0;
  const auto repaired = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(repaired.ok) << repaired.message;
  EXPECT_EQ(reports, repaired.attempted_trials);
  EXPECT_GT(accepted, 0u);
  EXPECT_EQ(repaired.feedback_updates, repaired.attempted_trials);
  for(const auto& trial : repaired.trials) {
    for(const auto& c : trial.candidates) {
      EXPECT_FALSE(c.fingerprint.empty());
      EXPECT_TRUE(std::isfinite(c.ranked_wns_gain));
      EXPECT_TRUE(std::isfinite(c.ranked_tns_gain));
    }
  }
  EXPECT_GT(repaired.after.setup_wns, repaired.before.setup_wns);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathCoverageFindsUnsampledEndpointsAndKeepsGlobalStaAcceptance)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_strategy = "feedback";
  options.path_buffer_endpoint_coverage = true;
  options.path_buffer_group_schedule = true;
  options.path_buffer_gain_scope = "global";
  options.path_buffer_max_paths = 1;
  options.path_buffer_max_trials = 8;
  options.path_buffer_max_area_growth = 0.5;
  options.path_buffer_min_relative_improvement = 0;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_GT(result.attempted_trials, 0u);
  bool expanded = false;
  for(const auto& trial : result.trials) {
    for(const auto& c : trial.candidates) {
      expanded |= c.covered_endpoints > c.affected_endpoints;
      EXPECT_TRUE(std::isfinite(c.covered_tns_gain));
      EXPECT_FALSE(c.load_group.empty());
    }
    if(trial.rejection_reasons.empty()) {
      EXPECT_GE(trial.after.setup_wns, trial.before.setup_wns - options.common.tolerances.wns_absolute);
      EXPECT_GE(trial.after.setup_tns, trial.before.setup_tns - 1e-12f);
    }
  }
  EXPECT_TRUE(expanded);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathFanoutTreeUsesWeightedLoadsAndRollsBackWholeEco)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  // A fractional limit deliberately differs from raw sink count. Keep the
  // original driver violating even after buffering, so an extra violating
  // branch cannot hide behind removal of the original violation.
  for(auto* master : lib_->getMasters()) {
    auto* cell = db_network_->libertyCell(db_network_->dbToSta(master));
    if(cell && cell->isBuffer()) {
      sta::LibertyPort *input, *output;
      cell->bufferPorts(input, output);
      input->setFanoutLoad(0.5f);
      output->setFanoutLimit(1.5f, sta::MinMax::max());
    }
  }
  db_network_->libertyCell(block_->findInst("driver"))->findLibertyPort("ZN")
      ->setFanoutLimit(0.25f, sta::MinMax::max());
  refreshRepairTiming();
  const auto initial_instances = block_->getInsts().size();
  const auto initial_nets = block_->getNets().size();
  std::vector<std::pair<odb::dbITerm*, odb::dbNet*>> connections;
  for(auto* inst : block_->getInsts())
    for(auto* pin : inst->getITerms()) connections.emplace_back(pin, pin->getNet());
  ecc::SetupRepairOptions options;
  options.path_buffer_tree_candidates = false; // Isolate post-insertion fanout completion.
  options.path_buffer_max_trials = 4;
  options.path_buffer_max_area_growth = -1;
  options.path_buffer_min_relative_improvement = 1; // Force post-STA rollback.
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned tree_trials = 0, diagnostics = 0;
  options.path_trial_observer = [&](unsigned, const ecc::PathBufferTrial& trial) {
    if(trial.buffers > trial.candidates.size()) {
      ++tree_trials;
      EXPECT_LE(trial.after.total_fanout_violations, trial.before.total_fanout_violations);
    }
    EXPECT_FALSE(trial.rejection_reasons.empty());
    EXPECT_EQ(block_->getInsts().size(), initial_instances);
    EXPECT_EQ(block_->getNets().size(), initial_nets);
    for(const auto& [pin, net] : connections) EXPECT_EQ(pin->getNet(), net);
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  };
  options.path_fanout_observer = [&](const ecc::PathFanoutCheck& check) {
    ++diagnostics;
    EXPECT_LT(check.slack, 0);
    EXPECT_FALSE(check.pin.empty());
  };
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(tree_trials, 0u);
  EXPECT_EQ(tree_trials, result.fanout_tree_trials);
  EXPECT_GT(diagnostics, 0u);
  EXPECT_EQ(result.inserted_buffers, 0u);
  EXPECT_FLOAT_EQ(result.before.setup_wns, result.after.setup_wns);
  EXPECT_EQ(result.before.total_fanout_violations, result.after.total_fanout_violations);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathFanoutTreeHonorsSdcAndCompoundBufferBudget)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  for(auto* master : lib_->getMasters()) {
    auto* cell = db_network_->libertyCell(db_network_->dbToSta(master));
    if(cell && cell->isBuffer()) {
      sta::LibertyPort *input, *output;
      cell->bufferPorts(input, output);
      input->setFanoutLoad(0.5f);
      output->setFanoutLimit(100, sta::MinMax::max());
    }
  }
  sta_->setFanoutLimit(db_network_->cell(db_network_->topInstance()),
      sta::MinMax::max(), 1.5f, sta_->cmdMode()->sdc());
  db_network_->libertyCell(block_->findInst("driver"))->findLibertyPort("ZN")
      ->setFanoutLimit(0.25f, sta::MinMax::max());
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_tree_candidates = false; // Isolate post-insertion fanout completion.
  options.path_buffer_max_trials = 0;
  options.path_buffer_max_area_growth = -1;
  options.path_buffer_max_inserted_buffers = 2;
  options.path_buffer_min_relative_improvement = 1;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned largest_trial = 0;
  options.path_trial_observer = [&](unsigned, const ecc::PathBufferTrial& trial) {
    largest_trial = std::max(largest_trial, trial.buffers);
    EXPECT_LE(trial.after.total_fanout_violations, trial.before.total_fanout_violations);
  };
  bool saw_sdc_limit = false;
  options.path_fanout_observer = [&](const ecc::PathFanoutCheck& check) {
    if(check.phase == "candidate_buffer") {
      saw_sdc_limit = true;
      EXPECT_FLOAT_EQ(check.limit, 1.5f); // SDC tighter than Liberty's 100.
    }
  };
  const auto bounded = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(bounded.ok) << bounded.message;
  EXPECT_TRUE(saw_sdc_limit);
  EXPECT_GT(bounded.fanout_precheck_rejections, 0u);
  EXPECT_LE(largest_trial, 2u);
  EXPECT_EQ(bounded.inserted_buffers, 0u);
  options.path_buffer_max_inserted_buffers = 0;
  const auto unbounded = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(unbounded.ok) << unbounded.message;
  EXPECT_GT(unbounded.fanout_tree_trials, 0u);
  EXPECT_GT(largest_trial, 2u);
  EXPECT_EQ(unbounded.inserted_buffers, 0u);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathFanoutTreeAllowsSingletonGroupsWithHeavyLoads)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  for(auto* master : lib_->getMasters()) {
    auto* cell = db_network_->libertyCell(db_network_->dbToSta(master));
    if(cell && cell->isBuffer()) {
      sta::LibertyPort *input, *output;
      cell->bufferPorts(input, output);
      input->setFanoutLoad(master->getName() == "BUF_X1" ? 2.0f : 0.5f);
      output->setFanoutLimit(3, sta::MinMax::max());
    }
  }
  db_network_->libertyCell(block_->findInst("driver"))->findLibertyPort("ZN")
      ->setFanoutLimit(0.25f, sta::MinMax::max());
  refreshRepairTiming();
  const auto initial_instances = block_->getInsts().size();
  ecc::SetupRepairOptions options;
  options.path_buffer_tree_candidates = false; // Isolate post-insertion fanout completion.
  options.common.excluded_masters.push_back(lib_->findMaster("BUF_X1"));
  options.path_buffer_max_area_growth = -1;
  options.path_buffer_min_relative_improvement = 1;
  options.path_buffer_max_trials = 4;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned singleton_trees = 0;
  options.path_trial_observer = [&](unsigned, const ecc::PathBufferTrial& trial) {
    if(trial.buffers > trial.candidates.front().loads) {
      ++singleton_trees;
      EXPECT_LE(trial.after.total_fanout_violations, trial.before.total_fanout_violations);
    }
  };
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(singleton_trees, 0u);
  EXPECT_GT(result.fanout_tree_trials, 0u);
  // Exercise the same heavy-sink/lighter-buffer boundary in the planner,
  // before ECO fanout completion. Singleton leaf groups must remain feasible.
  singleton_trees = 0;
  options.path_buffer_tree_candidates = true;
  const auto planned = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(planned.ok) << planned.message;
  EXPECT_GT(singleton_trees, 0u);
  EXPECT_EQ(planned.inserted_buffers, 0u);
  EXPECT_EQ(block_->getInsts().size(), initial_instances);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathImpossibleFanoutSkipsPlacementAndStaTrials)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  for(auto* master : lib_->getMasters()) {
    auto* cell = db_network_->libertyCell(db_network_->dbToSta(master));
    if(cell && cell->isBuffer()) {
      sta::LibertyPort *input, *output;
      cell->bufferPorts(input, output);
      input->setFanoutLoad(0.5f);
      output->setFanoutLimit(0.25f, sta::MinMax::max());
    }
  }
  db_network_->libertyCell(block_->findInst("driver"))->findLibertyPort("ZN")
      ->setFanoutLimit(0.25f, sta::MinMax::max());
  refreshRepairTiming();
  const auto initial_instances = block_->getInsts().size();
  ecc::SetupRepairOptions options;
  options.path_buffer_tree_candidates = false; // Isolate post-insertion fanout completion.
  options.path_buffer_max_area_growth = -1;
  options.path_buffer_max_trials = 0;
  unsigned prepares = 0;
  options.common.prepare = [&]() { ++prepares; };
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.fanout_precheck_rejections, 0u);
  EXPECT_EQ(result.attempted_trials, 0u);
  EXPECT_EQ(prepares, 0u);
  EXPECT_EQ(result.refreshes, 1u); // Restore final STA once, not per rejection.
  EXPECT_EQ(result.inserted_buffers, 0u);
  EXPECT_EQ(block_->getInsts().size(), initial_instances);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathPlansCapLimitedTreeAndRollsBackAtomically)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows(); constrainSetup(false);
  // A load cap tighter than a complete net, but feasible for buffered groups.
  // No fanout overload is needed to create this topology.
  for(auto* master : lib_->getMasters()) {
    auto* cell = db_network_->libertyCell(db_network_->dbToSta(master));
    if(cell && cell->isBuffer()) {
      sta::LibertyPort *input, *output;
      cell->bufferPorts(input, output);
      output->setCapacitanceLimit(10e-15f, sta::MinMax::max());
      output->setFanoutLimit(100, sta::MinMax::max());
    }
  }
  refreshRepairTiming();
  const auto instances = block_->getInsts().size(), nets = block_->getNets().size();
  std::vector<std::pair<odb::dbITerm*, odb::dbNet*>> connections;
  for(auto* inst : block_->getInsts())
    for(auto* pin : inst->getITerms()) connections.emplace_back(pin, pin->getNet());
  ecc::SetupRepairOptions options;
  options.path_buffer_tree_candidates = true;
  options.path_buffer_strategy = "path_gain";
  options.path_buffer_max_trials = 0;
  options.path_buffer_max_candidates = 64;
  for(auto* master : lib_->getMasters()) {
    auto* cell = db_network_->libertyCell(db_network_->dbToSta(master));
    if(cell && cell->isBuffer() && master->getName() != "BUF_X1")
      options.common.excluded_masters.push_back(master);
  }
  options.path_buffer_min_relative_improvement = 1;
  options.path_buffer_max_area_growth = -1;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned planned_trees = 0;
  options.path_trial_observer = [&](unsigned, const ecc::PathBufferTrial& t) {
    for(const auto& c : t.candidates) if(c.planned_buffers > 1) {
      ++planned_trees;
      EXPECT_EQ(c.loads, 10u);
      EXPECT_EQ(t.buffers, c.planned_buffers);
      EXPECT_LE(t.after.total_fanout_violations, t.before.total_fanout_violations);
    }
    EXPECT_FALSE(t.rejection_reasons.empty());
    EXPECT_EQ(block_->getInsts().size(), instances);
    EXPECT_EQ(block_->getNets().size(), nets);
    for(const auto& [pin, net] : connections) EXPECT_EQ(pin->getNet(), net);
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  };
  auto run = [&]() { return ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options); };
  const auto result = run();
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(planned_trees, 0u);
  EXPECT_EQ(result.inserted_buffers, 0u);
  // The allowance applies to the entire planned tree, before mutating DB.
  planned_trees = 0;
  options.path_buffer_max_inserted_buffers = 1;
  const auto bounded = run();
  ASSERT_TRUE(bounded.ok) << bounded.message;
  EXPECT_EQ(planned_trees, 0u);
  EXPECT_EQ(block_->getInsts().size(), instances);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathRecoveryCandidatesUseActualEndpointObjective)
{
  readTestDesign("path_recovery_sta.v");
  addLegalRows();
  auto* sdc = sta_->cmdMode()->sdc();
  sta_->setInputDelay(db_network_->dbToSta(block_->findBTerm("in")),
      sta::RiseFallBoth::riseFall(), sdc->findClock("clk"), sta::RiseFall::rise(),
      nullptr, false, false, sta::MinMaxAll::max(), false, 0.9e-9, sdc);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_gain_scope = "endpoints";
  options.path_buffer_strategy = "path_gain";
  options.path_buffer_max_area_growth = -1;
  options.path_buffer_max_trials = 4;
  options.path_buffer_min_relative_improvement = 1; // force rollback
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  auto run = [&]() { return ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options); };
  options.path_buffer_recovery = false;
  auto setup_only = run();
  ASSERT_TRUE(setup_only.ok) << setup_only.message;
  for(const auto& t : setup_only.trials)
    for(const auto& c : t.candidates) EXPECT_FALSE(c.recovery);
  options.path_buffer_recovery = true;
  auto recovery = run();
  ASSERT_TRUE(recovery.ok) << recovery.message;
  bool saw_recovery = false;
  for(const auto& t : recovery.trials) {
    for(const auto& c : t.candidates) saw_recovery |= c.recovery;
    EXPECT_GT(t.objective_endpoints, 0u);
    EXPECT_LT(t.objective_wns_before, 0);
    EXPECT_LE(t.objective_tns_before, t.objective_wns_before);
    EXPECT_FALSE(t.rejection_reasons.empty());
  }
  EXPECT_TRUE(saw_recovery);
  EXPECT_EQ(recovery.inserted_buffers, 0u);
  EXPECT_FLOAT_EQ(recovery.before.setup_wns, recovery.after.setup_wns);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathLocalObjectiveRequiresBothEndpointAndGlobalProgress)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows(); constrainSetup(true); refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_gain_scope = "endpoints";
  options.path_buffer_min_relative_improvement = 0.001;
  options.path_buffer_max_trials = 4;
  options.path_buffer_max_area_growth = -1;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.inserted_buffers, 0u);
  for(const auto& t : result.trials) {
    EXPECT_GT(t.objective_endpoints, 0u);
    if(t.rejection_reasons.empty()) {
      EXPECT_TRUE(t.objective_wns_after > t.objective_wns_before ||
                  t.objective_tns_after > t.objective_tns_before);
      EXPECT_TRUE(t.after.setup_wns > t.before.setup_wns ||
                  t.after.setup_tns > t.before.setup_tns);
      EXPECT_LE(t.after.total_fanout_violations, t.before.total_fanout_violations);
    }
  }
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathEndpointScaleIsNotBlockedByUnrelatedRecoveryTns)
{
  readTestDesign("path_objective_sta.v");
  addLegalRows(); constrainSetup(true);
  auto* sdc = sta_->cmdMode()->sdc();
  sta_->setInputDelay(db_network_->dbToSta(block_->findBTerm("rst_bad")),
      sta::RiseFallBoth::riseFall(), sdc->findClock("clk"), sta::RiseFall::rise(),
      nullptr, false, false, sta::MinMaxAll::max(), false, 5e-9, sdc);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_recovery = false; // Isolate the gain-scope change.
  options.path_buffer_strategy = "path_gain";
  options.path_buffer_min_relative_improvement = 0.1;
  options.path_buffer_max_trials = 4;
  options.path_buffer_max_area_growth = -1;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  auto run = [&]() { return ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options); };
  options.path_buffer_gain_scope = "global";
  const auto global = run();
  ASSERT_TRUE(global.ok) << global.message;
  EXPECT_GT(global.attempted_trials, 0u);
  EXPECT_EQ(global.inserted_buffers, 0u);
  // The independent reset path remains globally critical. Improving sampled
  // setup endpoints cannot be reported as a positive global WNS forecast.
  for(const auto& trial : global.trials)
    for(const auto& c : trial.candidates)
      EXPECT_LE(c.predicted_wns_gain, options.common.tolerances.wns_absolute);
  options.path_buffer_gain_scope = "endpoints";
  const auto local = run();
  ASSERT_TRUE(local.ok) << local.message;
  EXPECT_GT(local.inserted_buffers, 0u);
  EXPECT_GT(local.after.setup_tns, local.before.setup_tns);
  EXPECT_GE(local.after.setup_wns + options.common.tolerances.wns_absolute,
            local.before.setup_wns);
  EXPECT_LE(local.after.total_slew_violations, local.before.total_slew_violations);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathBufferOnlyRepairRollsBackStaRegression)
{
  readTestDesign("gate_repair_sta.v");
  constrainSetup(true);
  refreshRepairTiming();
  const auto instances_before = block_->getInsts().size();
  const auto nets_before = block_->getNets().size();
  const float wns_before = float(sta_->worstSlack(sta::MinMax::max()));
  ecc::SetupRepairOptions options;
  options.path_buffer_only = true;
  options.path_buffer_max_paths = 1;
  options.path_buffer_max_trials = 1;
  options.path_buffer_max_area_growth = 0.5;
  unsigned refreshes = 0;
  const auto refresh = [&]() {
    const bool edited = block_->getInsts().size() > instances_before;
    ++refreshes;
    for(auto* scene : sta_->scenes()) {
      ep_.setHWireSignalRC(scene, edited ? 1e12 : 1e6,
                            edited ? 1e-4 : 1e-10);
      ep_.setVWireSignalRC(scene, edited ? 1e12 : 1e6,
                            edited ? 1e-4 : 1e-10);
    }
    refreshRepairTiming();
  };
  const auto result =
      ecc::OpenDbSetupRepairer::repair(block_, sta_.get(), refresh, options);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.path_buffer_accepted, 0u);
  EXPECT_EQ(result.path_buffer_rejected, 1u);
  ASSERT_EQ(result.path_buffer_trials.size(), 1u);
  EXPECT_FALSE(result.path_buffer_trials.front().rejection_reasons.empty());
  EXPECT_LT(result.path_buffer_trials.front().after.setup_wns, wns_before);
  EXPECT_EQ(result.refreshes, refreshes);
  EXPECT_EQ(result.rollback_refreshes, 1u);
  EXPECT_EQ(block_->getInsts().size(), instances_before);
  EXPECT_EQ(block_->getNets().size(), nets_before);
  EXPECT_NEAR(result.after.design.setup_wns, wns_before, 1e-15);
  EXPECT_GT(refreshes, 0u);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathDeferredRefreshPreservesRejectedDesignAndTrials)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  const auto count = block_->getInsts().size();
  ecc::SetupRepairOptions options;
  options.path_buffer_max_paths = 10;
  options.path_buffer_max_trials = 4;
  options.path_buffer_max_candidates = 8;
  options.path_buffer_max_area_growth = 0.5;
  options.min_improvement = 1.0f; // Force every otherwise valid ECO to roll back.
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  auto run = [&]() { return ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options); };
  options.defer_rollback_refresh = false;
  const auto eager = run();
  ASSERT_TRUE(eager.ok) << eager.message;
  ASSERT_EQ(eager.attempted_trials, 4u);
  options.defer_rollback_refresh = true;
  const auto deferred = run();
  ASSERT_TRUE(deferred.ok) << deferred.message;
  EXPECT_EQ(deferred.attempted_trials, eager.attempted_trials);
  EXPECT_EQ(eager.refreshes, 8u);
  EXPECT_EQ(deferred.refreshes, 5u);
  EXPECT_EQ(deferred.rollback_refreshes, 1u);
  EXPECT_EQ(deferred.after.setup_wns, eager.after.setup_wns);
  EXPECT_EQ(deferred.after.setup_tns, eager.after.setup_tns);
  EXPECT_EQ(block_->getInsts().size(), count);
  ASSERT_EQ(deferred.trials.size(), eager.trials.size());
  bool saw_multiple_endpoints = false;
  for(size_t i = 0; i < eager.trials.size(); ++i) {
    EXPECT_EQ(deferred.trials[i].candidates.front().net,
              eager.trials[i].candidates.front().net);
    EXPECT_EQ(deferred.trials[i].candidates.front().master,
              eager.trials[i].candidates.front().master);
    EXPECT_EQ(deferred.trials[i].after.setup_wns, eager.trials[i].after.setup_wns);
    saw_multiple_endpoints |= deferred.trials[i].candidates.front().affected_endpoints > 1;
  }
  EXPECT_TRUE(saw_multiple_endpoints);
  options.path_buffer_max_refreshes = 3;
  const auto limited = run();
  ASSERT_TRUE(limited.ok) << limited.message;
  EXPECT_EQ(limited.attempted_trials, 2u);
  EXPECT_EQ(limited.refreshes, 3u);
  EXPECT_EQ(limited.after.setup_wns, eager.after.setup_wns);
  options.path_buffer_max_seconds = 1e-12;
  const auto expired = run();
  ASSERT_TRUE(expired.ok);
  EXPECT_EQ(expired.attempted_trials, 0u);
  EXPECT_EQ(expired.refreshes, 0u);
  options.path_buffer_max_seconds = 0;
  options.path_buffer_max_refreshes = 0;
  options.path_buffer_max_trials = 0;
  options.path_buffer_max_consecutive_rejections = 2;
  const auto stopped = run();
  ASSERT_TRUE(stopped.ok) << stopped.message;
  EXPECT_TRUE(stopped.stopped_on_rejections);
  EXPECT_FALSE(stopped.converged);
  EXPECT_EQ(stopped.attempted_trials, 2u);
  EXPECT_EQ(stopped.rollback_refreshes, 1u);
  EXPECT_EQ(stopped.inserted_buffers, 0u);
  EXPECT_EQ(stopped.after.setup_wns, eager.after.setup_wns);
  EXPECT_EQ(stopped.after.setup_tns, eager.after.setup_tns);
  EXPECT_EQ(block_->getInsts().size(), count);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathExceptionAfterDeferredRollbackRestoresSta)
{
  readTestDesign("gate_repair_sta.v");
  constrainSetup(true);
  refreshRepairTiming();
  const auto count = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  const float wns = float(sta_->worstSlack(sta::MinMax::max()));
  ecc::SetupRepairOptions options;
  options.path_buffer_max_trials = 4;
  options.path_buffer_max_area_growth = 0.5;
  options.min_improvement = 1.0f;
  unsigned prepares = 0;
  options.common.prepare = [&]() {
    if(++prepares == 2) throw std::runtime_error("injected prepare failure");
  };
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("injected prepare failure"), std::string::npos);
  EXPECT_EQ(block_->getInsts().size(), count);
  EXPECT_EQ(block_->getNets().size(), nets);
  EXPECT_FLOAT_EQ(float(sta_->worstSlack(sta::MinMax::max())), wns);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathBatchSeparatesReconvergentCones)
{
  readTestDesign("path_reconvergent_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_batch_size = 8;
  options.path_buffer_max_paths = 4;
  options.paths_per_endpoint = 4;
  options.path_buffer_max_trials = 1;
  options.path_buffer_max_area_growth = 1;
  options.min_improvement = 1.0f;
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_EQ(result.trials.size(), 1u);
  EXPECT_GT(result.candidate_count, 1u);
  // All candidate nets converge on the merge/sink cone, so distinct net
  // names must not be mistaken for independent batch members.
  EXPECT_EQ(result.trials.front().candidates.size(), 1u);
  EXPECT_EQ(result.inserted_buffers, 0u);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
}

TEST_F(BufferInserterStaTest, PathBatchHonorsBufferCapAndSharedCone)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.path_buffer_batch_size = 8;
  options.path_buffer_max_inserted_buffers = 1;
  options.path_buffer_max_trials = 4;
  options.path_buffer_max_area_growth = 0.5;
  options.common.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::OpenDbPathRepairer::repair(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_GT(result.attempted_trials, 0u);
  EXPECT_LE(result.inserted_buffers, 1u);
  for(const auto& trial : result.trials) EXPECT_EQ(trial.candidates.size(), 1u);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
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
  options.max_inserted_buffers = 1;
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
  options.max_inserted_buffers = 1;
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



void placePbbiPorts(odb::dbBlock* block) {
  int y=1000;
  for(auto* term:block->getBTerms()) {
    auto* pin=odb::dbBPin::create(term);
    odb::dbBox::create(pin,block->getTech()->findLayer("metal2"),20000,y,20100,y+100);
    pin->setPlacementStatus(odb::dbPlacementStatus::PLACED);y+=1000;
  }
}

TEST_F(BufferInserterStaTest, PbbiGateSizingIsGloballyVerified)
{
  readTestDesign("pbbi_sequential_sta.v");
  addLegalRows(); placePbbiPorts(block_); constrainSetup(true); refreshRepairTiming();
  ecc::PbbiRepairOptions o;
  o.max_paths=2; o.max_trials=4; o.max_area_growth=0.5;
  o.max_frontier_states=64;
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_, &dp_);};
  auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  SCOPED_TRACE(r.message);
  ASSERT_TRUE(r.ok)<<r.message;
  EXPECT_EQ(block_->findInst("sink")->getMaster()->getName(),"DFF_X1");
  EXPECT_EQ(r.skipped_paths,0u);
  EXPECT_GT(r.paths,0u);EXPECT_GT(r.expansions,0u);EXPECT_GT(r.attempted,0u);
  EXPECT_GT(r.timing_nets,0u);EXPECT_GT(r.timing_vertices,0u);EXPECT_GT(r.timing_edges,0u);
  EXPECT_GT(r.response_evaluations,0u);
  EXPECT_GT(r.accepted,0u);
  EXPECT_GT(r.inserted_buffers+r.resized_cells,0u);
  EXPECT_GE(r.after.setup_wns+1e-15,r.before.setup_wns);
  EXPECT_GE(r.after.setup_tns+1e-15,r.before.setup_tns);
  EXPECT_LE(r.after.total_slew_violations,r.before.total_slew_violations);
  EXPECT_NO_THROW(dp_.checkPlacement(false)); db_network_->checkAxioms();
}
TEST_F(BufferInserterStaTest, PbbiNoSizingKeepsGateMastersAndRollbackRestores)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows(); placePbbiPorts(block_); constrainSetup(true); refreshRepairTiming();
  std::map<std::string,odb::dbMaster*> masters;
  std::map<std::string,odb::dbNet*> nets;
  for(auto* inst:block_->getInsts()){
    masters[inst->getName()]=inst->getMaster();
    for(auto* pin:inst->getITerms())nets[inst->getName()+"/"+pin->getMTerm()->getName()]=pin->getNet();
  }
  const auto count=block_->getInsts().size(), net_count=block_->getNets().size();
  const float wns=sta_->worstSlack(sta::MinMax::max());
  ecc::PbbiRepairOptions o;o.gate_sizing=false;o.sink_sizing=false;
  o.max_paths=1;o.max_trials=3;o.max_area_growth=0.5;
  o.min_improvement=1; // force every real plan to roll back
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_, &dp_);};
  auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  SCOPED_TRACE(r.message);
  ASSERT_TRUE(r.ok)<<r.message;EXPECT_GT(r.attempted,0u);EXPECT_EQ(r.accepted,0u);
  EXPECT_EQ(r.resized_cells,0u);EXPECT_EQ(block_->getInsts().size(),count);
  EXPECT_EQ(block_->getNets().size(),net_count);
  for(auto* inst:block_->getInsts()){
    EXPECT_EQ(inst->getMaster(),masters.at(inst->getName()));
    for(auto* pin:inst->getITerms())EXPECT_EQ(pin->getNet(),nets.at(inst->getName()+"/"+pin->getMTerm()->getName()));
  }
  EXPECT_NEAR(sta_->worstSlack(sta::MinMax::max()),wns,1e-15);
  EXPECT_LE(r.refreshes,r.attempted+1);
  db_network_->checkAxioms();
}
TEST_F(BufferInserterStaTest, PbbiExceptionUndoesResizesAndInsertedBuffers)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows(); placePbbiPorts(block_);constrainSetup(true);refreshRepairTiming();
  const auto count=block_->getInsts().size();
  auto* master=block_->findInst("driver")->getMaster();
  const float wns=sta_->worstSlack(sta::MinMax::max());
  ecc::PbbiRepairOptions o;o.max_paths=1;o.max_area_growth=0.5;
  o.common.prepare=[](){throw std::runtime_error("injected PBBI placement failure");};
  auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  EXPECT_FALSE(r.ok);EXPECT_NE(r.message.find("injected"),std::string::npos);
  EXPECT_EQ(block_->getInsts().size(),count);EXPECT_EQ(block_->findInst("driver")->getMaster(),master);
  EXPECT_NEAR(sta_->worstSlack(sta::MinMax::max()),wns,1e-15);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PbbiGateOnlyAndZeroTrialBudget)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows(); placePbbiPorts(block_); constrainSetup(true); refreshRepairTiming();
  ecc::PbbiRepairOptions o;
  o.max_paths=1; o.max_trials=0; o.max_area_growth=0.5;
  for(auto* master:lib_->getMasters()) {
    auto* cell=db_network_->libertyCell(db_network_->dbToSta(master));
    if(cell && cell->isBuffer())o.common.excluded_masters.push_back(master);
  }
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  auto run=[&](){return ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);};
  auto zero=run();ASSERT_TRUE(zero.ok)<<zero.message;
  EXPECT_EQ(zero.attempted,0u);EXPECT_EQ(zero.refreshes,0u);
  o.max_trials=4;
  auto r=run();ASSERT_TRUE(r.ok)<<r.message;
  EXPECT_GT(r.resized_cells,0u);EXPECT_EQ(r.inserted_buffers,0u);
  EXPECT_GT(r.after.setup_wns,r.before.setup_wns);
  EXPECT_NO_THROW(dp_.checkPlacement(false));db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PbbiDeferredAndEagerRollbackUseSamePlans)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();placePbbiPorts(block_);constrainSetup();refreshRepairTiming();
  ecc::PbbiRepairOptions o;o.max_paths=2;o.max_trials=4;o.max_area_growth=0.5;
  o.min_improvement=1;o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  auto run=[&](){return ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);};
  o.defer_rollback_refresh=false;const auto eager=run();ASSERT_TRUE(eager.ok)<<eager.message;
  o.defer_rollback_refresh=true;const auto deferred=run();ASSERT_TRUE(deferred.ok)<<deferred.message;
  ASSERT_GT(eager.attempted,1u);ASSERT_EQ(eager.trials.size(),deferred.trials.size());
  std::set<std::string> keys;
  std::set<unsigned> path_ranks;
  for(size_t i=0;i<eager.trials.size();++i){
    path_ranks.insert(deferred.trials[i].path_rank);
    EXPECT_EQ(eager.trials[i].fingerprint,deferred.trials[i].fingerprint);
    EXPECT_TRUE(keys.insert(deferred.trials[i].fingerprint).second);
    EXPECT_NEAR(eager.trials[i].after.setup_wns,deferred.trials[i].after.setup_wns,1e-15);
  }
  EXPECT_GT(path_ranks.size(),1u);
  EXPECT_LT(deferred.refreshes,eager.refreshes);
  EXPECT_EQ(deferred.after.setup_wns,eager.after.setup_wns);
  o.max_refreshes=3;auto capped=run();ASSERT_TRUE(capped.ok)<<capped.message;
  EXPECT_LE(capped.refreshes,3u);EXPECT_LE(capped.attempted,2u);
}
TEST_F(BufferInserterStaTest, PbbiSolvesOnlyPathsWithinTrialBudget)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();placePbbiPorts(block_);constrainSetup();refreshRepairTiming();
  const auto count=block_->getInsts().size();
  const float wns=sta_->worstSlack(sta::MinMax::max());
  ecc::PbbiRepairOptions o;o.max_paths=4;o.max_trials=1;o.max_area_growth=0.5;
  o.min_improvement=1;o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  ASSERT_TRUE(r.ok)<<r.message;EXPECT_GT(r.paths,1u);EXPECT_EQ(r.solved_paths,1u);
  EXPECT_EQ(r.attempted,1u);EXPECT_EQ(r.accepted,0u);
  EXPECT_EQ(block_->getInsts().size(),count);
  EXPECT_NEAR(sta_->worstSlack(sta::MinMax::max()),wns,1e-15);
  db_network_->checkAxioms();
}
TEST_F(BufferInserterStaTest, PbbiForecastRespectsHardBufferBlockage)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();placePbbiPorts(block_);constrainSetup(true);refreshRepairTiming();
  const auto core=block_->getCoreArea();
  auto* blockage=odb::dbBlockage::create(block_,core.xMin(),core.yMin(),core.xMax(),core.yMax());
  const auto count=block_->getInsts().size();
  const float wns=sta_->worstSlack(sta::MinMax::max());
  ecc::PbbiRepairOptions o;o.gate_sizing=false;o.sink_sizing=false;o.max_paths=2;o.max_trials=2;
  auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  ASSERT_TRUE(r.ok)<<r.message;EXPECT_GT(r.timing_nets,0u);EXPECT_EQ(r.timing_buffered_nets,0u);
  EXPECT_EQ(r.attempted,0u);EXPECT_EQ(r.inserted_buffers,0u);EXPECT_EQ(block_->getInsts().size(),count);
  EXPECT_NEAR(sta_->worstSlack(sta::MinMax::max()),wns,1e-15);
  odb::dbBlockage::destroy(blockage);db_network_->checkAxioms();
}
TEST_F(BufferInserterStaTest, PbbiModelsProtectedLogicWithoutEditingIt)
{
  readTestDesign("gate_repair_sta.v");addLegalRows();placePbbiPorts(block_);
  constrainSetup(true);refreshRepairTiming();
  auto* protected_gate=block_->findInst("driver");
  auto* master=protected_gate->getMaster();protected_gate->setDoNotTouch(true);
  ecc::PbbiRepairOptions o;o.max_paths=1;o.max_trials=1;o.gate_sizing=false;o.sink_sizing=false;
  auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  ASSERT_TRUE(r.ok)<<r.message;EXPECT_GT(r.timing_readonly_arcs,0u);EXPECT_EQ(r.timing_fallback_arcs,0u);
  EXPECT_EQ(protected_gate->getMaster(),master);EXPECT_TRUE(protected_gate->isDoNotTouch());
  db_network_->checkAxioms();
}
TEST_F(BufferInserterStaTest, PbbiElectricalSnapshotCoversOtherInputsAndDownstream)
{
  readTestDesign("pbbi_electrical_sta.v");addLegalRows();placePbbiPorts(block_);
  constrainSetup();refreshRepairTiming();
  auto* resized=block_->findInst("resized");auto* original=resized->getMaster();
  auto* larger=lib_->findMaster("NAND2_X4");ASSERT_NE(larger,nullptr);
  auto slew=[&](const char* inst,const char* port) {
    return double(float(sta_->slew(sta_->graph()->pinDrvrVertex(db_network_->dbToSta(block_->findInst(inst)->findITerm(port))),
        sta::RiseFallBoth::riseFall(),sta::SceneSeq{sta_->cmdScene()},sta::MinMax::max())));
  };
  const double side_before=slew("side_driver","Z"),out_before=slew("downstream","Z");
  ecc::PbbiElectricalModel model(block_,sta_.get());
  const auto identity=model.predict({}, {}, true);
  EXPECT_DOUBLE_EQ(identity.wns_delta,0);EXPECT_DOUBLE_EQ(identity.tns_delta,0);
  EXPECT_NEAR(identity.baseline_wns, std::min(0.0f, float(sta_->worstSlack(sta::MinMax::max()))), 1e-15);
  EXPECT_NEAR(identity.baseline_tns, float(sta_->totalNegativeSlack(sta::MinMax::max())), 1e-15);
  EXPECT_GT(identity.endpoints, 0u);
  EXPECT_NEAR(identity.slew_by_pin.at("side_driver/Z"),side_before,1e-18);
  auto prediction=model.predict({{resized,larger}}, {}, true);
  EXPECT_GT(prediction.slew_by_pin.at("side_driver/Z"),side_before);
  EXPECT_NE(prediction.slew_by_pin.at("downstream/Z"),out_before);
  EXPECT_EQ(resized->getMaster(),original);EXPECT_TRUE(sta_->search()->arrivalsValid());
  odb::dbDatabase::beginEco(block_);ASSERT_TRUE(resized->swapMaster(larger));refreshRepairTiming();
  EXPECT_GT(slew("side_driver","Z"),side_before);
  const double out_after=slew("downstream","Z");
  EXPECT_GT((prediction.slew_by_pin.at("downstream/Z")-out_before)*(out_after-out_before),0);
  odb::dbDatabase::undoEco(block_);
  // No RC/STA refresh: the immutable model must be independent of dirty STA.
  const auto reused=model.predict({{resized,larger}}, {}, true);
  EXPECT_EQ(prediction.slew_by_pin,reused.slew_by_pin);
  EXPECT_EQ(resized->getMaster(),original);refreshRepairTiming();db_network_->checkAxioms();
}
TEST_F(BufferInserterStaTest, PbbiElectricalIncludesConstrainedEndpointWithFanout)
{
  readTestDesign("pbbi_electrical_sta.v");addLegalRows();placePbbiPorts(block_);
  constrainSetup();refreshRepairTiming();
  const auto* pin = db_network_->dbToSta(block_->findInst("resized")->findITerm("ZN"));
  auto* pins = new sta::PinSet(db_network_);
  pins->insert(pin);
  auto* sdc = sta_->cmdMode()->sdc();
  auto* to = sta_->makeExceptionTo(pins, nullptr, nullptr,
                                  sta::RiseFallBoth::riseFall(), sta::RiseFallBoth::riseFall(), sdc);
  sta_->makePathDelay(nullptr, nullptr, to, sta::MinMax::max(), false, false,
                      1e-12f, "PBBI internal endpoint regression", sdc);
  refreshRepairTiming();
  auto* vertex = sta_->graph()->pinDrvrVertex(pin);
  ASSERT_TRUE(sta_->search()->isEndpoint(vertex, sta_->cmdMode()));
  ecc::PbbiElectricalModel model(block_, sta_.get());
  const auto identity = model.predict({}, {}, true);
  ASSERT_TRUE(identity.endpoint_slack_by_pin.count("resized/ZN"));
  ASSERT_TRUE(identity.endpoint_slack_by_pin.count("out0"));
  // Vertex slack includes the tighter downstream constraint. It must not be
  // charged again as the internal endpoint's own contribution to TNS.
  EXPECT_GT(identity.endpoint_slack_by_pin.at("resized/ZN"),
            float(sta_->slack(vertex, sta::MinMax::max())) + 1e-15);
  EXPECT_NEAR(identity.baseline_wns, std::min(0.0f, float(sta_->worstSlack(sta::MinMax::max()))), 1e-15);
  EXPECT_NEAR(identity.baseline_tns, float(sta_->totalNegativeSlack(sta::MinMax::max())), 1e-15);
  EXPECT_DOUBLE_EQ(identity.wns_delta, 0);
  EXPECT_DOUBLE_EQ(identity.tns_delta, 0);
}
TEST_F(BufferInserterStaTest, PbbiDiagnosticsRetainRejectedBufferPinsAfterUndo)
{
  readTestDesign("gate_repair_sta.v");addLegalRows();placePbbiPorts(block_);
  constrainSetup(true);refreshRepairTiming();const auto count=block_->getInsts().size();
  ecc::PbbiRepairOptions o;o.max_paths=1;o.max_trials=1;o.max_area_growth=0.5;
  o.gate_sizing=false;o.sink_sizing=false;
  o.common.prepare=[&](){
    ecc::legalizeRepairPlacement(block_,&dp_);
    for(auto* inst:block_->getInsts())if(inst->getName().find("sizer_pbbi_buf")==0) {
      sta::LibertyPort *input=nullptr,*output=nullptr;db_network_->libertyCell(inst)->bufferPorts(input,output);
      output->setSlewLimit(1e-15f,sta::MinMax::max());
    }
  };
  const auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  ASSERT_TRUE(r.ok)<<r.message;ASSERT_EQ(r.trials.size(),1u);EXPECT_EQ(r.rejected,1u);
  bool found=false;
  for(const auto& v:r.trials[0].violations)if(v.relation=="inserted") {
    found=true;EXPECT_FALSE(v.had_before);EXPECT_GT(v.after,v.limit);
    EXPECT_EQ(block_->findInst(v.pin.substr(0,v.pin.rfind('/')).c_str()),nullptr);
  }
  EXPECT_TRUE(found);EXPECT_EQ(block_->getInsts().size(),count);db_network_->checkAxioms();
}
}  // namespace

TEST_F(BufferInserterStaTest, PbbiGlobalPoolRollbackAndRefreshParity)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();placePbbiPorts(block_);constrainSetup();refreshRepairTiming();
  ecc::PbbiRepairOptions o;o.scheduler="global";o.scheduler_window=2;
  o.max_paths=4;o.max_trials=4;o.max_area_growth=0.5;o.min_improvement=1;
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  const auto count=block_->getInsts().size();
  auto run=[&](){return ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);};
  o.defer_rollback_refresh=false;auto eager=run();ASSERT_TRUE(eager.ok)<<eager.message;
  o.defer_rollback_refresh=true;auto deferred=run();ASSERT_TRUE(deferred.ok)<<deferred.message;
  EXPECT_EQ(deferred.accepted,0u);ASSERT_EQ(deferred.attempted,4u);
  EXPECT_EQ(deferred.scheduler_decisions,deferred.attempted);
  EXPECT_GT(deferred.scheduler_peak_candidates,1u);
  EXPECT_GE(deferred.solved_paths,2u);
  EXPECT_EQ(deferred.trials.size(),eager.trials.size());
  for(size_t i=0;i<deferred.trials.size();++i) {
    EXPECT_EQ(deferred.trials[i].fingerprint,eager.trials[i].fingerprint);
    EXPECT_EQ(deferred.trials[i].after.setup_wns,eager.trials[i].after.setup_wns);
    EXPECT_GT(deferred.trials[i].affected_nets,0u);
  }
  EXPECT_LT(deferred.refreshes,eager.refreshes);
  EXPECT_EQ(block_->getInsts().size(),count);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
}
TEST_F(BufferInserterStaTest, PbbiForecastCacheMatchesUncachedAfterCommits)
{
  readTestDesign("pbbi_sequential_sta.v");
  addLegalRows();placePbbiPorts(block_);constrainSetup(true);refreshRepairTiming();
  struct Original {odb::dbInst* inst;odb::dbMaster* master;int x,y;odb::dbOrientType orient;};
  std::vector<Original> original;
  std::unordered_set<odb::dbInst*> instances;
  std::unordered_set<odb::dbNet*> nets;
  std::vector<std::pair<odb::dbITerm*,odb::dbNet*>> connections;
  for(auto* inst:block_->getInsts()) {
    int x,y;inst->getLocation(x,y);
    original.push_back({inst,inst->getMaster(),x,y,inst->getOrient()});instances.insert(inst);
    for(auto* pin:inst->getITerms())connections.emplace_back(pin,pin->getNet());
  }
  for(auto* net:block_->getNets())nets.insert(net);
  ecc::PbbiRepairOptions o;o.max_paths=2;o.max_trials=4;o.max_area_growth=0.5;o.max_frontier_states=64;
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  auto run=[&](){return ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);};
  o.forecast_cache=true;auto cached=run();ASSERT_TRUE(cached.ok)<<cached.message;
  EXPECT_GT(cached.accepted,0u);EXPECT_GT(cached.forecast_cache_hits,0u);
  EXPECT_GT(cached.forecast_cache_invalidations,0u);
  std::vector<odb::dbInst*> added;
  for(auto* inst:block_->getInsts())if(!instances.count(inst))added.push_back(inst);
  for(auto* inst:added)odb::dbInst::destroy(inst);
  for(auto& old:original){ASSERT_TRUE(old.inst->swapMaster(old.master));old.inst->setOrient(old.orient);old.inst->setLocation(old.x,old.y);}
  for(auto [pin,net]:connections){if(net)pin->connect(net);else pin->disconnect();}
  std::vector<odb::dbNet*> added_nets;
  for(auto* net:block_->getNets())if(!nets.count(net))added_nets.push_back(net);
  for(auto* net:added_nets)odb::dbNet::destroy(net);
  refreshRepairTiming();
  o.forecast_cache=false;o.path_cache=false;auto uncached=run();ASSERT_TRUE(uncached.ok)<<uncached.message;
  EXPECT_EQ(cached.after.setup_wns,uncached.after.setup_wns);
  EXPECT_EQ(cached.after.setup_tns,uncached.after.setup_tns);
  ASSERT_EQ(cached.trials.size(),uncached.trials.size());
  // OpenDB's name counter is intentionally monotonic across manual restore.
  // Canonicalize only generated IDs, retaining a bijection across all trials.
  auto keys=[](const auto& result) {
    std::map<std::string,std::string> names;std::vector<std::string> keys;
    const std::regex generated("sizer_pbbi_(buf|net)[0-9]+");
    for(const auto& t:result.trials) {
      std::string key;size_t offset=0;
      for(std::sregex_iterator it(t.fingerprint.begin(),t.fingerprint.end(),generated),end;it!=end;++it) {
        key+=t.fingerprint.substr(offset,it->position()-offset);
        auto [name,inserted]=names.emplace(it->str(),"generated"+std::to_string(names.size()));
        key+=name->second;offset=it->position()+it->length();
      }
      key+=t.fingerprint.substr(offset);keys.push_back(key);
    }
    return keys;
  };
  EXPECT_EQ(keys(cached),keys(uncached));
  for(size_t i=0;i<cached.trials.size();++i) {
    EXPECT_EQ(cached.trials[i].accepted,uncached.trials[i].accepted);
  }
  EXPECT_LT(cached.timing_expansions,uncached.timing_expansions);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PbbiAdaptiveDemandAdmissionAndRollbackParity)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();placePbbiPorts(block_);constrainSetup();refreshRepairTiming();
  ecc::PbbiRepairOptions o;o.scheduler="adaptive";o.scheduler_window=4;
  o.max_paths=4;o.max_trials=4;o.max_area_growth=0.5;o.min_improvement=1;
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  const auto count=block_->getInsts().size(),nets=block_->getNets().size();
  const auto wns=sta_->worstSlack(sta::MinMax::max());
  auto run=[&](){return ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);};
  o.defer_rollback_refresh=false;auto eager=run();ASSERT_TRUE(eager.ok)<<eager.message;
  o.defer_rollback_refresh=true;auto deferred=run();ASSERT_TRUE(deferred.ok)<<deferred.message;
  ASSERT_EQ(deferred.attempted,4u);EXPECT_EQ(deferred.accepted,0u);
  EXPECT_EQ(deferred.trials.front().pool_paths,1u); // No speculative warmup.
  ASSERT_EQ(deferred.trials.size(),eager.trials.size());
  for(size_t i=0;i<deferred.trials.size();++i) {
    EXPECT_EQ(deferred.trials[i].fingerprint,eager.trials[i].fingerprint);
    EXPECT_EQ(deferred.trials[i].after.setup_wns,eager.trials[i].after.setup_wns);
  }
  EXPECT_NE(deferred.trials[0].path_rank,deferred.trials[1].path_rank);
  EXPECT_LT(deferred.refreshes,eager.refreshes);
  EXPECT_EQ(block_->getInsts().size(),count);EXPECT_EQ(block_->getNets().size(),nets);
  EXPECT_NEAR(sta_->worstSlack(sta::MinMax::max()),wns,1e-15);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, VgSetupScopeCollectsCleanNegativeSlackNets)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();placePbbiPorts(block_);constrainSetup(true);refreshRepairTiming();
  ecc::VgRepairOptions o;o.max_buffers=64;o.common.max_fanout=64;
  auto run=[&](){return ecc::OpenDbVgRepairer::repair(block_,sta_.get(),[&](){refreshRepairTiming();},&resizer_,o);};
  auto drv=run();ASSERT_TRUE(drv.ok)<<drv.message;
  ASSERT_LT(drv.before.setup_wns,0);
  ASSERT_EQ(drv.before.severity,0);
  ASSERT_EQ(drv.nets_collected,0u);
  o.scope="setup";
  auto setup=run();ASSERT_TRUE(setup.ok)<<setup.message;
  EXPECT_GT(setup.nets_collected,0u);
  EXPECT_GE(setup.after.setup_wns+o.common.tolerances.wns_absolute,setup.before.setup_wns);
  EXPECT_GE(setup.after.setup_tns+o.common.tolerances.tns_absolute,setup.before.setup_tns);
  EXPECT_EQ(setup.after.total_slew_violations,0u);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
  auto missing=ecc::OpenDbVgRepairer::repair(block_,sta_.get(),[&](){refreshRepairTiming();},o);
  EXPECT_FALSE(missing.ok);
  EXPECT_NE(missing.message.find("Steiner"),std::string::npos);
}

TEST_F(BufferInserterStaTest, PbbiIncrementalPlacementRollbackRestoresGeometry)
{
  readTestDesign("gate_repair_sta.v");addLegalRows();placePbbiPorts(block_);
  constrainSetup(true);refreshRepairTiming();
  struct Original {odb::dbInst* inst;odb::dbMaster* master;int x,y;odb::dbOrientType orient;};
  std::vector<Original> original;
  for(auto* inst:block_->getInsts()) {
    int x,y;inst->getLocation(x,y);
    original.push_back({inst,inst->getMaster(),x,y,inst->getOrient()});
  }
  const auto nets=block_->getNets().size();
  const double wns=sta_->worstSlack(sta::MinMax::max());
  ecc::PbbiRepairOptions o;o.max_paths=2;o.max_trials=3;o.max_area_growth=0.5;
  o.min_improvement=1;o.incremental_placement=true;
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_,{},o.incremental_placement);};
  auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  ASSERT_TRUE(r.ok)<<r.message;EXPECT_GT(r.attempted,0u);EXPECT_EQ(r.accepted,0u);
  EXPECT_EQ(block_->getInsts().size(),original.size());EXPECT_EQ(block_->getNets().size(),nets);
  for(const auto& before:original) {
    int x,y;before.inst->getLocation(x,y);
    EXPECT_EQ(x,before.x);EXPECT_EQ(y,before.y);
    EXPECT_EQ(before.inst->getMaster(),before.master);EXPECT_EQ(before.inst->getOrient(),before.orient);
  }
  EXPECT_NEAR(sta_->worstSlack(sta::MinMax::max()),wns,1e-15);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, IncrementalPlacementRetainsLegalProtectedCells)
{
  readTestDesign("gate_repair_sta.v");addLegalRows();placePbbiPorts(block_);
  auto* fixed=block_->findInst("load0");ASSERT_NE(fixed,nullptr);
  int x,y;fixed->getLocation(x,y);auto status=fixed->getPlacementStatus();
  auto* added=odb::dbInst::create(block_,lib_->findMaster("BUF_X8"),"incremental_test");
  added->setLocation(x,y);added->setPlacementStatus(odb::dbPlacementStatus::PLACED);
  ecc::legalizeRepairPlacement(block_,&dp_,{fixed},true);
  int nx,ny;fixed->getLocation(nx,ny);
  EXPECT_EQ(nx,x);EXPECT_EQ(ny,y);EXPECT_EQ(fixed->getPlacementStatus(),status);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  odb::dbInst::destroy(added);
}

TEST_F(BufferInserterStaTest, PbbiPhysicalRetryIsBudgetedAndRestoresLocks)
{
  readTestDesign("gate_repair_sta.v");
  auto* driver=odb::dbInst::create(block_,lib_->findMaster("BUF_X1"),"feedback_driver");
  auto* load=odb::dbInst::create(block_,lib_->findMaster("BUF_X1"),"feedback_load");
  auto* net=odb::dbNet::create(block_,"feedback_net");
  ASSERT_NE(driver,nullptr);ASSERT_NE(load,nullptr);ASSERT_NE(net,nullptr);
  auto* input=block_->findInst("driver")->findITerm("A1");ASSERT_NE(input,nullptr);
  driver->findITerm("A")->connect(input->getNet());
  driver->findITerm("Z")->connect(net);load->findITerm("A")->connect(net);
  driver->setPlacementStatus(odb::dbPlacementStatus::PLACED);
  load->setPlacementStatus(odb::dbPlacementStatus::PLACED);
  addLegalRows();placePbbiPorts(block_);constrainSetup(true);refreshRepairTiming();
  int x,y;load->getLocation(x,y);const auto status=load->getPlacementStatus();
  const auto instances=block_->getInsts().size();
  ecc::PbbiRepairOptions o;o.max_paths=1;o.max_trials=2;o.max_area_growth=0.5;
  o.placement_feedback=true;o.min_improvement=1;
  o.gate_sizing=false;o.sink_sizing=false;
  unsigned prepares=0;
  bool fail_locked=false;
  o.common.prepare=[&](){
    ++prepares;
    if(fail_locked && load->isFixed())
      throw std::runtime_error("injected protected placement failure");
    ecc::legalizeRepairPlacement(block_,&dp_);
    // Inject off-path wire growth after placement, as a deterministic stand-in
    // for an unrelated cell disturbed by legalization. Retry must protect it.
    if(!load->isFixed())load->setLocation(x+20000000,y);
  };
  auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  ASSERT_TRUE(r.ok)<<r.message;ASSERT_EQ(r.trials.size(),2u);
  EXPECT_EQ(prepares,2u);EXPECT_EQ(r.accepted,0u);
  EXPECT_EQ(r.trials[0].protected_instances,0u);
  EXPECT_GT(r.trials[1].protected_instances,0u);
  EXPECT_EQ(r.trials[0].fingerprint,r.trials[1].fingerprint);
  EXPECT_EQ(load->getPlacementStatus(),status);
  int nx,ny;load->getLocation(nx,ny);EXPECT_EQ(nx,x);EXPECT_EQ(ny,y);
  EXPECT_EQ(block_->getInsts().size(),instances);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
  // Guard-induced placement failure must be rejected without leaking locks.
  fail_locked=true;prepares=0;
  auto failed=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  ASSERT_TRUE(failed.ok)<<failed.message;ASSERT_EQ(failed.trials.size(),2u);
  EXPECT_TRUE(failed.trials.back().placement_failed);
  EXPECT_EQ(failed.trials.back().rejection_reasons,std::vector<std::string>{"placement"});
  EXPECT_EQ(load->getPlacementStatus(),status);load->getLocation(nx,ny);
  EXPECT_EQ(nx,x);EXPECT_EQ(ny,y);EXPECT_EQ(block_->getInsts().size(),instances);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  // A requested retry cannot bypass the trial cap.
  o.max_trials=1;prepares=0;fail_locked=false;
  auto capped=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  ASSERT_TRUE(capped.ok)<<capped.message;EXPECT_EQ(capped.attempted,1u);
  EXPECT_EQ(prepares,1u);EXPECT_EQ(load->getPlacementStatus(),status);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
  // Cross-path schedulers must also retry the measured plan before admission
  // or path cooldown can spend the remaining STA budget on another candidate.
  o.max_trials=2;o.max_paths=2;
  for(const auto* scheduler : {"global", "adaptive"}) {
    SCOPED_TRACE(scheduler);o.scheduler=scheduler;prepares=0;
    auto scheduled=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
    ASSERT_TRUE(scheduled.ok)<<scheduled.message;ASSERT_EQ(scheduled.trials.size(),2u);
    EXPECT_EQ(prepares,2u);EXPECT_GT(scheduled.trials[1].protected_instances,0u);
    EXPECT_EQ(scheduled.trials[0].fingerprint,scheduled.trials[1].fingerprint);
    EXPECT_EQ(load->getPlacementStatus(),status);load->getLocation(nx,ny);
    EXPECT_EQ(nx,x);EXPECT_EQ(ny,y);EXPECT_EQ(block_->getInsts().size(),instances);
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
  }
}

TEST_F(BufferInserterStaTest, PbbiNetScopeDeduplicatesContextsAndRollsBack)
{
  readTestDesign("pbbi_net_scope_sta.v");
  addLegalRows();placePbbiPorts(block_);constrainSetup();refreshRepairTiming();
  struct Original {odb::dbInst* inst;odb::dbMaster* master;int x,y;odb::dbOrientType orient;};
  std::vector<Original> original;
  for(auto* inst:block_->getInsts()) {
    int x,y;inst->getLocation(x,y);
    original.push_back({inst,inst->getMaster(),x,y,inst->getOrient()});
  }
  const auto nets=block_->getNets().size();
  const auto wns=sta_->worstSlack(sta::MinMax::max());
  ecc::PbbiRepairOptions o;o.max_paths=4;o.max_trials=4;
  o.max_frontier_states=64;o.max_area_growth=0.5;o.min_improvement=1;
  o.buffer_aware_timing=false;
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  auto run=[&](){return ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);};
  auto path=run();ASSERT_TRUE(path.ok)<<path.message;
  ASSERT_GT(path.max_problem_nets,1u);EXPECT_EQ(path.accepted,0u);
  o.scope="net";
  auto net=run();ASSERT_TRUE(net.ok)<<net.message;
  EXPECT_GT(net.attempted,0u);EXPECT_EQ(net.accepted,0u);
  EXPECT_EQ(net.max_problem_nets,1u);
  EXPECT_GT(net.duplicate_problems,0u);
  EXPECT_GT(net.collected_problems,path.collected_problems);
  for(const auto& trial:net.trials)EXPECT_EQ(trial.path_nets,1u);
  EXPECT_EQ(block_->getInsts().size(),original.size());EXPECT_EQ(block_->getNets().size(),nets);
  for(const auto& before:original) {
    int x,y;before.inst->getLocation(x,y);
    EXPECT_EQ(x,before.x);EXPECT_EQ(y,before.y);
    EXPECT_EQ(before.inst->getMaster(),before.master);EXPECT_EQ(before.inst->getOrient(),before.orient);
  }
  EXPECT_NEAR(sta_->worstSlack(sta::MinMax::max()),wns,1e-15);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PbbiNetScopeAcceptsStaValidatedSizing)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();placePbbiPorts(block_);constrainSetup(true);refreshRepairTiming();
  ecc::PbbiRepairOptions o;o.scope="net";o.max_paths=2;o.max_trials=8;
  o.max_frontier_states=64;o.max_area_growth=0.5;
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  ASSERT_TRUE(r.ok)<<r.message;EXPECT_GT(r.accepted,0u);EXPECT_GT(r.resized_cells,0u);
  for(const auto& trial:r.trials)EXPECT_EQ(trial.path_nets,1u);
  EXPECT_GE(r.after.setup_wns+1e-15,r.before.setup_wns);
  EXPECT_GE(r.after.setup_tns+1e-15,r.before.setup_tns);
  EXPECT_LE(r.after.total_slew_violations,r.before.total_slew_violations);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PbbiAreaRecoveryMaintainsInputTimingAndRejectsUnsafeEdits)
{
  readTestDesign("gate_repair_sta.v");oversizeLoads(true);
  addLegalRows();placePbbiPorts(block_);constrainSetup(true);refreshRepairTiming();
  ecc::PbbiRepairOptions o;o.scope="net";o.objective="area";
  o.target_wns=-1;o.target_tns=-1; // Clamped to the stronger input timing.
  o.max_paths=2;o.max_trials=4;o.max_area_growth=0.5;o.max_frontier_states=64;
  auto* load=block_->findInst("load0");int x,y;load->getLocation(x,y);
  const auto count=block_->getInsts().size();
  o.common.prepare=[&](){
    ecc::legalizeRepairPlacement(block_,&dp_);
    load->setLocation(x+20000000,y); // Force a real setup/DRV failure.
  };
  auto run=[&](){return ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);};
  auto rejected=run();ASSERT_TRUE(rejected.ok)<<rejected.message;
  EXPECT_GT(rejected.attempted,0u);EXPECT_EQ(rejected.accepted,0u);
  for(const auto& t:rejected.trials) {
    EXPECT_TRUE(t.area_recovery);EXPECT_TRUE(t.target_before);
    EXPECT_LT(t.after.area,t.before.area);
    EXPECT_FALSE(t.target_after);
    EXPECT_NE(std::find(t.rejection_reasons.begin(),t.rejection_reasons.end(),"setup_wns"),t.rejection_reasons.end());
  }
  int nx,ny;load->getLocation(nx,ny);EXPECT_EQ(nx,x);EXPECT_EQ(ny,y);
  EXPECT_EQ(block_->getInsts().size(),count);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  auto accepted=run();ASSERT_TRUE(accepted.ok)<<accepted.message;
  EXPECT_GT(accepted.area_reductions,0u);EXPECT_TRUE(accepted.target_met);
  EXPECT_LT(accepted.after.area,accepted.before.area);
  EXPECT_EQ(accepted.target_wns,accepted.before.setup_wns);
  EXPECT_EQ(accepted.target_tns,accepted.before.setup_tns);
  for(const auto& t:accepted.trials)if(t.accepted) {
    EXPECT_TRUE(t.area_recovery);EXPECT_TRUE(t.target_after);
    EXPECT_LT(t.after.area,t.before.area);
  }
  EXPECT_GE(accepted.after.setup_wns+o.common.tolerances.wns_absolute,accepted.before.setup_wns);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PbbiAreaRecoverySearchesPathsWithPositiveSlack)
{
  readTestDesign("gate_repair_sta.v");oversizeLoads(true);
  addLegalRows();placePbbiPorts(block_);
  auto* sdc=sta_->cmdMode()->sdc();
  sta_->setOutputDelay(db_network_->dbToSta(block_->findBTerm("out0")),
                      sta::RiseFallBoth::riseFall(),sdc->findClock("clk"),
                      sta::RiseFall::rise(),nullptr,false,false,sta::MinMaxAll::max(),
                      false,-10e-9f,sdc);
  refreshRepairTiming();
  ecc::PbbiRepairOptions o;o.scope="net";o.objective="area";
  o.max_paths=2;o.max_trials=4;o.max_frontier_states=64;
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  ASSERT_TRUE(r.ok)<<r.message;EXPECT_GE(r.before.setup_wns,0);
  EXPECT_GT(r.paths,0u);EXPECT_GT(r.area_reductions,0u);
  EXPECT_LT(r.after.area,r.before.area);EXPECT_TRUE(r.target_met);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PbbiAreaTargetRepairsBeforeCostRecovery)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();placePbbiPorts(block_);constrainSetup(true);refreshRepairTiming();
  const double initial=sta_->worstSlack(sta::MinMax::max());ASSERT_LT(initial,0);
  ecc::PbbiRepairOptions o;o.objective="area";o.scope="net";
  o.target_wns=initial*0.75;o.max_paths=2;o.max_trials=8;o.max_frontier_states=64;
  o.max_area_growth=0.5;
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  auto r=ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);
  ASSERT_TRUE(r.ok)<<r.message;ASSERT_FALSE(r.trials.empty());
  EXPECT_FALSE(r.trials.front().target_before);EXPECT_FALSE(r.trials.front().area_recovery);
  EXPECT_GT(r.accepted,0u);EXPECT_TRUE(r.target_met);EXPECT_TRUE(r.converged);
  EXPECT_GE(r.after.setup_wns+o.common.tolerances.wns_absolute,o.target_wns);
  bool reached=false;
  for(const auto& t:r.trials) {
    EXPECT_EQ(t.area_recovery,reached);
    if(t.accepted && t.target_after)reached=true;
    if(t.accepted && t.area_recovery) {
      EXPECT_TRUE(t.target_after);EXPECT_LT(t.after.area,t.before.area);
    }
  }
  EXPECT_TRUE(reached);EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PbbiTimingFailureFeedbackDiversifiesActions) {
  readTestDesign("gate_repair_sta.v");
  addLegalRows();placePbbiPorts(block_);constrainSetup();refreshRepairTiming();
  ecc::PbbiRepairOptions o;o.max_paths=1;o.max_trials=2;
  o.max_frontier_states=128;o.max_area_growth=0.5;o.min_improvement=1;
  o.common.prepare=[&](){ecc::legalizeRepairPlacement(block_,&dp_);};
  const auto instances=block_->getInsts().size(), nets=block_->getNets().size();
  const float wns=sta_->worstSlack(sta::MinMax::max());
  const auto run=[&](){return ecc::OpenDbPbbiRepairer::repair(block_,sta_.get(),&resizer_,[&](){refreshRepairTiming();},o);};
  o.failure_feedback=false;auto plain=run();ASSERT_TRUE(plain.ok)<<plain.message;
  o.failure_feedback=true;auto feedback=run();ASSERT_TRUE(feedback.ok)<<feedback.message;
  ASSERT_EQ(plain.trials.size(),2u);ASSERT_EQ(feedback.trials.size(),2u);
  EXPECT_EQ(plain.accepted,0u);EXPECT_EQ(feedback.accepted,0u);
  EXPECT_EQ(plain.trials[0].fingerprint,feedback.trials[0].fingerprint);
  EXPECT_EQ(plain.trials[0].rejection_reasons,
            (std::vector<std::string>{"no_setup_gain"}));
  const auto actions=[](const std::string& key) {
    std::set<std::string> result;std::istringstream stream(key);std::string action;
    while(std::getline(stream,action,';'))if(!action.empty())result.insert(action);
    return result;
  };
  const auto first=actions(plain.trials[0].fingerprint);
  const auto overlap=[&](const std::string& key) {
    const auto current=actions(key);
    return std::count_if(current.begin(),current.end(),[&](const auto& a){return first.count(a);});
  };
  EXPECT_LT(overlap(feedback.trials[1].fingerprint),overlap(plain.trials[1].fingerprint));
  EXPECT_GT(feedback.feedback_reorders,0u);
  EXPECT_EQ(block_->getInsts().size(),instances);EXPECT_EQ(block_->getNets().size(),nets);
  EXPECT_NEAR(float(sta_->worstSlack(sta::MinMax::max())),wns,1e-15);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
}
