// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include "repair/buffer.h"
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

#include "actions/netlist.h"
#include "actions/placement.h"
#include "repair/drv.h"
#include "algorithms/van_ginneken.h"
#include "algorithms/gobi.h"
#include "design_analysis.h"
#include "algorithms/cell_candidates.h"
#include "repair/setup.h"
#include "sta/Sdc.hh"
#include "sta/Mode.hh"
#include "sta/Parasitics.hh"
#include "sta/Scene.hh"
#include "sta/EquivCells.hh"
#include <stdexcept>
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbReadVerilog.hh"
#include "est/EstimateParasitics.h"
#include "parasitics.h"
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
   ecc::BufferRuntime runtime_;
   BufferInserterStaTest()
       : tst::IntegratedFixture(tst::IntegratedFixture::Technology::kNangate45,
                                "_main/src/est/test/") {}

   void readTestDesign(const char *file_name = "buffer_inserter_sta.v") {
     ord::dbVerilogNetwork verilog_network(sta_.get());
     sta::VerilogReader verilog_reader(&verilog_network);
     const std::string path = std::string(ECC_SIZER_TEST_DATA_DIR) + "/" + file_name;
     verilog_reader.read(path.c_str());

     ord::dbLinkDesign("top", &verilog_network, db_.get(), &logger_, true);
     sta_->postReadDb(db_.get());

     block_ = db_->getChip()->getBlock();
     ASSERT_NE(block_, nullptr);
     block_->setDefUnits(lib_->getTech()->getLefUnits());
     block_->setDieArea(odb::Rect(0, 0, 10000, 10000));
     block_->setCoreArea(odb::Rect(0, 0, 10000, 10000));

     int x = 1000;
     for (odb::dbInst *inst : block_->getInsts()) {
       inst->setLocation(x, 1000);
       inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
       x += 2000;
     }
     sta_->postReadDef(block_);
     for (sta::Scene *scene : sta_->scenes()) {
       ep_.setHWireSignalRC(scene, 1.0e6, 1.0e-10);
       ep_.setVWireSignalRC(scene, 1.0e6, 1.0e-10);
     }
     initStaDefaultSdc();
     sta_->updateTiming(true);
     ASSERT_TRUE(sta_->search()->arrivalsValid());
   }

  void readVtDesign(const char* design = "repair_vt.v")
  {
    for (const auto* suffix : {"H", "L", "B", "X"})
      readLiberty(std::string(ECC_SIZER_TEST_DATA_DIR) + "/repair_vt_" + suffix + ".lib");
    ASSERT_TRUE(updateLib(lib_, std::string(ECC_SIZER_TEST_DATA_DIR) + "/repair_vt.lef"));
    readTestDesign(design);
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

  void makePowerRecoveryDesign()
  {
    readVtDesign("repair_power.v");
    auto* sdc = sta_->cmdMode()->sdc();
    for (const auto* name : {"out1", "out2"})
      sta_->setOutputDelay(db_network_->dbToSta(block_->findBTerm(name)),
                           sta::RiseFallBoth::riseFall(),
                           sdc->findClock("clk"),
                           sta::RiseFall::rise(),
                           nullptr,
                           false,
                           false,
                           sta::MinMaxAll::max(),
                           false,
                           0.38e-9f,
                           sdc);
    ecc::legalizeRepairPlacement(block_, &dp_);
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

  void excludeAllBuffers() {
    for (auto* master : lib_->getMasters()) {
      auto* cell = db_network_->libertyCell(db_network_->dbToSta(master));
      if (cell && cell->isBuffer())
        runtime_.constraints.excluded_masters.push_back(master);
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

TEST_F(BufferInserterStaTest, PreparedDelayMatchesOpenStaInterpolationAndExtrapolation)
{
  readTestDesign("gate_repair_sta.v");
  constrainSetup(true);
  refreshRepairTiming();
  ecc::RepairCommonOptions options;
  ecc::detail::DesignAnalysis backend(block_, sta_.get(), options,
                                      runtime_.constraints);
  ecc::DesignMetrics metrics;
  const auto states = backend.analyze(metrics, true);
  ASSERT_FALSE(states.empty());
  std::vector<ecc::detail::CellModel> models = backend.bufferChoices();
  for(const auto& state : states)
    if(state.driver) models.push_back(backend.driverModel(state));
  ASSERT_FALSE(models.empty());
  // Include table knots, interpolation and far extrapolation. Multi-input
  // gates retain the same worst-arc reduction as the original evaluator.
  for(const auto& model : models)
    for(const float cap : {0.0f, 1e-16f, 1e-15f, 1e-14f, 1e-12f}) {
      const auto prepared = backend.prepareDelayPrediction(model, cap);
      for(const float slew : {0.0f, 1e-12f, 1e-11f, 1e-10f, 1e-8f})
        EXPECT_EQ(prepared(slew), backend.predictedDelay(model, cap, slew))
            << model.cell->name() << " cap=" << cap << " slew=" << slew;
    }
  EXPECT_TRUE(std::isinf(backend.prepareDelayPrediction({}, 0)(0)));
}

TEST_F(BufferInserterStaTest, IncrementalElectricalAnalysisMatchesFullScanAcrossEcoAndUndo)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::RepairCommonOptions options;
  ecc::detail::DesignAnalysis cached(block_, sta_.get(), options, runtime_.constraints);
  ecc::detail::DesignAnalysis reference(block_, sta_.get(), options,
                                        runtime_.constraints);
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto check = [&]() {
    incremental.update();
    incremental.verifyTiming();
    ecc::DesignMetrics actual, expected;
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
  ecc::PathSearchOptions options;
  options.max_trials = 8;
  options.max_area_growth = 0.5;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned verified = 0;
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto refresh = [&]() {
    SCOPED_TRACE(verified);
    incremental.updateVerified();
    const auto traversals = incremental.statistics().verifications;
    incremental.verifyTiming();
    EXPECT_EQ(incremental.statistics().verifications, traversals);
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
    ecc::detail::DesignAnalysis backend(block_, sta_.get(), options.common,
                                        runtime_.constraints);
    ecc::DesignMetrics incremental_metrics, full_metrics;
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
  const auto result =
      ecc::BufferRepairer::run(block_, sta_.get(), refresh, options, runtime_);
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
  ecc::PathSearchOptions options;
  options.max_trials = 16;
  options.max_area_growth = std::nullopt;
  options.defer_rollback_refresh = true;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { incremental.update(); }, options, runtime_);
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
  ecc::PathSearchOptions options;
  options.max_trials = 16;
  options.max_area_growth = std::nullopt;
  options.defer_rollback_refresh = true;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned full_refreshes = 0, verified_trials = 0;
  runtime_.verification_refresh = [&]() {
    ++full_refreshes;
    refreshRepairTiming();
  };
  runtime_.trial_observer = [&](unsigned, const ecc::BufferTrialResult &trial) {
    if(trial.rechecked_full_rc)
      ++verified_trials;
    if(trial.rejection_reasons.empty())
      EXPECT_TRUE(trial.rechecked_full_rc);
  };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { incremental.update(); }, options, runtime_);
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

TEST_F(BufferInserterStaTest, BufferFinalPlacementRunsOnceAndReportsLegalFullSta)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::PathSearchOptions options;
  options.placement = ecc::PathPlacement::Final;
  options.max_trials = 16;
  options.max_area_growth = std::nullopt;
  unsigned prepared = 0, observed = 0;
  runtime_.prepare = [&]() {
    ++prepared;
    EXPECT_GT(observed, 0u);
    ecc::legalizeRepairPlacement(block_, &dp_);
  };
  runtime_.trial_observer = [&](unsigned, const ecc::BufferTrialResult &trial) {
    EXPECT_TRUE(trial.placement_provisional);
    EXPECT_EQ(prepared, 0u);
    ++observed;
  };
  std::vector<ecc::BufferEvent> events;
  runtime_.event_observer = [&](const ecc::BufferEvent& event) {
    events.push_back(event);
    EXPECT_EQ(event.placement_provisional,
              event.kind != ecc::BufferEvent::Kind::FinalPlacementAccepted);
  };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  runtime_.verification_refresh = [&]() { refreshRepairTiming(); };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { incremental.update(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_GE(events.size(), 4u);
  EXPECT_EQ(events.front().kind, ecc::BufferEvent::Kind::SearchStarted);
  EXPECT_EQ(events[events.size() - 3].kind, ecc::BufferEvent::Kind::SearchFinished);
  EXPECT_EQ(events[events.size() - 2].kind, ecc::BufferEvent::Kind::FinalPlacementStarted);
  EXPECT_EQ(events.back().kind, ecc::BufferEvent::Kind::FinalPlacementAccepted);
  EXPECT_EQ(events.back().inserted_buffers, result.inserted_buffers);
  EXPECT_EQ(events.back().setup_wns, result.after.setup_wns);
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

TEST_F(BufferInserterStaTest, BufferFinalPlacementRestoresAllProvisionalEditsAndMoves)
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
  for(const auto verification : {ecc::PathVerification::Trial, ecc::PathVerification::Final})
  for(bool throw_after_move : {false, true}) {
    ecc::PathSearchOptions options;
    options.placement = ecc::PathPlacement::Final;
    options.verification = verification;
    runtime_.verification_refresh = [&]() { refreshRepairTiming(); };
    options.max_trials = 16;
    options.max_area_growth = std::nullopt;
    runtime_.prepare = [&]() {
      ecc::legalizeRepairPlacement(block_, &dp_);
      block_->findInst("load9")->setLocation(100000000, 0);
      if(throw_after_move)
        throw std::runtime_error("injected final placement failure");
    };
    std::vector<ecc::BufferEvent> events;
    runtime_.event_observer = [&](const ecc::BufferEvent& event) {
      events.push_back(event);
    };
    ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
    const auto result = ecc::BufferRepairer::run(
        block_, sta_.get(), [&]() { incremental.update(); }, options, runtime_);
    EXPECT_EQ(result.ok, !throw_after_move) << result.message;
    ASSERT_FALSE(events.empty());
    EXPECT_EQ(events.back().kind, ecc::BufferEvent::Kind::FinalPlacementRolledBack);
    EXPECT_FALSE(events.back().placement_provisional);
    EXPECT_EQ(events.back().inserted_buffers, 0u);
    EXPECT_EQ(events.back().setup_wns, result.after.setup_wns);
    EXPECT_EQ(events.back().setup_tns, result.after.setup_tns);
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
  ecc::PathSearchOptions options;
  options.verification = ecc::PathVerification::Final;
  options.max_trials = 16;
  options.max_area_growth = std::nullopt;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto instances = block_->getInsts().size();
  unsigned full_refreshes = 0;
  const auto full = [&]() { ++full_refreshes; refreshRepairTiming(); };
  auto rejected = ecc::BufferRepairer::run(block_, sta_.get(), full, options, runtime_);
  EXPECT_FALSE(rejected.ok);
  options.placement = ecc::PathPlacement::Final;
  rejected = ecc::BufferRepairer::run(block_, sta_.get(), full, options, runtime_);
  EXPECT_FALSE(rejected.ok); // Missing authoritative callback.
  EXPECT_EQ(full_refreshes, 0u);
  EXPECT_EQ(block_->getInsts().size(), instances);
  runtime_.verification_refresh = full;
  runtime_.trial_observer = [&](unsigned, const ecc::BufferTrialResult &trial) {
    EXPECT_TRUE(trial.placement_provisional);
    EXPECT_FALSE(trial.rechecked_full_rc);
    EXPECT_EQ(full_refreshes, 0u);
    if(trial.rejection_reasons.empty())
      EXPECT_TRUE(trial.metrics_complete);
  };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  runtime_.incremental_verification = [&]() { incremental.verifyTiming(); };
  options.incremental_analysis = true;
  options.reuse_rejections = true;
  runtime_.analysis_changes = [&]() { return incremental.takeAnalysisChanges(); };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { incremental.update(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.accepted_trials, 0u);
  EXPECT_GT(result.rejected_trials, 0u);
  EXPECT_TRUE(result.final_placement.accepted) << result.message;
  EXPECT_EQ(full_refreshes, 1u);
  EXPECT_GT(incremental.statistics().verifications, 0u);
  EXPECT_LT(incremental.statistics().verifications, result.attempted_trials);
  ecc::detail::DesignAnalysis backend(block_, sta_.get(), options.common,
                                      runtime_.constraints);
  ecc::DesignMetrics measured;
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

TEST_F(BufferInserterStaTest, PathPatienceStopStillFinalizesAndKeepsAcceptedEdits)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  const auto instances = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  ecc::PathSearchOptions options;
  options.placement = ecc::PathPlacement::Final;
  options.max_trials = std::nullopt;
  options.search_patience = 2;
  options.max_area_growth = std::nullopt;
  unsigned prepared = 0;
  runtime_.prepare = [&]() {
    ++prepared;
    ecc::legalizeRepairPlacement(block_, &dp_);
  };
  runtime_.verification_refresh = [&]() { refreshRepairTiming(); };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { incremental.update(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_TRUE(result.stopped_on_low_progress) << result.message;
  EXPECT_FALSE(result.converged);
  EXPECT_LT(result.remaining_patience, 1);
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

TEST_F(BufferInserterStaTest, PathPatienceSpentAfterAcceptanceStillFinalizes)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::PathSearchOptions options;
  options.placement = ecc::PathPlacement::Final;
  options.search_patience = 1;
  options.progress_fraction = 1;
  options.max_area_growth = std::nullopt;
  unsigned prepared = 0;
  runtime_.prepare = [&]() {
    ++prepared;
    ecc::legalizeRepairPlacement(block_, &dp_);
  };
  runtime_.verification_refresh = [&]() { refreshRepairTiming(); };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { incremental.update(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.attempted_trials, 1u);
  EXPECT_EQ(result.accepted_trials, 1u);
  EXPECT_TRUE(result.stopped_on_low_progress);
  EXPECT_FALSE(result.converged);
  EXPECT_TRUE(result.final_placement.accepted) << result.message;
  EXPECT_EQ(prepared, 1u);
  EXPECT_GT(result.inserted_buffers, 0u);
  EXPECT_LT(result.after.setup_wns, 0);
  EXPECT_GT(result.after.setup_wns, result.before.setup_wns);
  EXPECT_NE(result.message.find("violations remain"), std::string::npos);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_NO_THROW(dp_.checkPlacement(false));
}

TEST_F(BufferInserterStaTest, PathTimingPrecheckMatchesFullMetricsAndNeverAcceptsAlone)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  ecc::PathSearchOptions options;
  options.adaptive_search = true;
  options.timing_precheck = true;
  options.max_trials = 32;
  options.max_area_growth = std::nullopt;
  options.defer_rollback_refresh = true;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  ecc::DesignMetrics independently_measured;
  const auto refresh = [&]() {
    refreshRepairTiming();
    ecc::detail::DesignAnalysis backend(block_, sta_.get(), options.common,
                                        runtime_.constraints);
    backend.analyze(independently_measured, true);
  };
  unsigned partial = 0;
  runtime_.verification_refresh = refresh;
  runtime_.trial_observer = [&](unsigned, const ecc::BufferTrialResult &trial) {
    EXPECT_FLOAT_EQ(trial.after.setup_wns, independently_measured.setup_wns);
    EXPECT_FLOAT_EQ(trial.after.setup_tns, independently_measured.setup_tns);
    EXPECT_FLOAT_EQ(trial.after.hold_wns, independently_measured.hold_wns);
    EXPECT_FLOAT_EQ(trial.after.hold_tns, independently_measured.hold_tns);
    if(!trial.metrics_complete) {
      ++partial;
      EXPECT_FALSE(trial.rejection_reasons.empty());
      EXPECT_FALSE(trial.rechecked_full_rc);
    }
    if(trial.rejection_reasons.empty()) {
      EXPECT_TRUE(trial.metrics_complete);
      EXPECT_TRUE(trial.rechecked_full_rc);
      EXPECT_EQ(trial.after.total_slew_violations, independently_measured.total_slew_violations);
      EXPECT_DOUBLE_EQ(trial.after.severity, independently_measured.severity);
    }
  };
  const auto result =
      ecc::BufferRepairer::run(block_, sta_.get(), refresh, options, runtime_);
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
  ecc::PathSearchOptions options;
  options.max_trials = 16;
  options.max_area_growth = std::nullopt;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned full_refreshes = 0;
  runtime_.verification_refresh = [&]() {
    if(++full_refreshes == 1)
      throw std::runtime_error("full verification failed");
    refreshRepairTiming();
  };
  ecc::PlacementParasiticUpdater incremental(block_, sta_.get(), &ep_);
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { incremental.update(); }, options, runtime_);
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
  ecc::VgOptions options;
  options.common.max_fanout = 64;
  options.max_buffers = 4;
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [this]() { refreshRepairTiming(); }, &resizer_, options,
      runtime_);
  EXPECT_TRUE(result.ok) << result.message;
  EXPECT_FALSE((result.accepted_trials != 0));
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
  ecc::VgOptions options;
  // This integration target deliberately links the CPU stub: requesting
  // CUDA must fall back before any ECO, then apply the same atomic plan.
  options.backend = ecc::VgBackend::Cuda;
  options.common.max_fanout = 3;
  options.max_buffers = 64;
  options.max_group_loads = 3;
  options.require_drv_non_regression = true;
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [this]() { refreshRepairTiming(); }, &cold_resizer, options,
      runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.backend_used, "cpu");
  EXPECT_FALSE(result.cuda_fallback.empty());
  EXPECT_TRUE((result.accepted_trials != 0));
  EXPECT_GT(result.plans_collected, 0u);
  EXPECT_GT(result.inserted_buffers, 0u);
  EXPECT_GT(block_->getInsts().size(), instances_before);
  EXPECT_EQ(block_->getInsts().size() - instances_before, result.inserted_buffers);
  EXPECT_LT(result.after.severity, result.before.severity);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();

  // The next repairs must analyze the inserted topology, not a pre-VG
  // snapshot. Check the stage boundary metrics and instance accounting.
  ecc::DrvRepairOptions design_options;
  design_options.common.max_fanout = options.common.max_fanout;
  const auto refresh = [this]() { refreshRepairTiming(); };
  const auto design =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, design_options, runtime_);
  ASSERT_TRUE(design.ok) << design.message;
  EXPECT_NEAR(design.before.severity, result.after.severity, 1e-6);
  EXPECT_NEAR(design.before.setup_wns, result.after.setup_wns, 1e-15);
  const size_t after_design = block_->getInsts().size();
  EXPECT_EQ(after_design, instances_before + result.inserted_buffers +
                             design.inserted_buffers);
  ecc::SetupRepairOptions setup_options;
  setup_options.common = design_options.common;
  const auto setup =
      ecc::SetupRepairer::run(block_, sta_.get(), refresh, setup_options, runtime_);
  ASSERT_TRUE(setup.ok) << setup.message;
  EXPECT_NEAR(setup.before.setup_wns, design.after.setup_wns, 1e-15);
  EXPECT_GE(setup.after.setup_wns + 1e-15, setup.before.setup_wns);
  EXPECT_EQ(block_->getInsts().size(), after_design + setup.inserted_buffers);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_TRUE(sta_->search()->arrivalsValid());
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, VgOneShotRollsBackWhenTimingGetsWorse)
{
  readTestDesign("fanout_repair_sta.v");
  const size_t instances_before = block_->getInsts().size();
  ecc::VgOptions options;
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
      ecc::BufferRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_FALSE((result.accepted_trials != 0));
  EXPECT_EQ(block_->getInsts().size(), instances_before);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, BufferNetStrategyRestoresEditsAndPlacementAfterPrepareFailure)
{
  readTestDesign("fanout_repair_sta.v");
  const auto count = block_->getInsts().size();
  auto* inst = *block_->getInsts().begin();
  int x, y;
  inst->getLocation(x, y);
  ecc::VgOptions options;
  options.common.max_fanout = 3;
  options.max_buffers = 3;
  options.max_group_loads = 3;
  unsigned preparations = 0;
  runtime_.prepare = [&]() {
    ++preparations;
    EXPECT_GT(block_->getInsts().size(), count);
    inst->setLocation(x + 100, y + 100);
    throw std::runtime_error("deliberate buffer preparation failure");
  };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [this]() { refreshRepairTiming(); }, options, runtime_);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("deliberate buffer preparation failure"), std::string::npos);
  EXPECT_EQ(preparations, 1u);
  EXPECT_EQ(result.rollback_refreshes, 1u);
  EXPECT_EQ(result.inserted_buffers, 0u);
  EXPECT_EQ(block_->getInsts().size(), count);
  int restored_x, restored_y;
  inst->getLocation(restored_x, restored_y);
  EXPECT_EQ(restored_x, x);
  EXPECT_EQ(restored_y, y);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_TRUE(sta_->search()->arrivalsValid());
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, RepairFanoutBuildsTreeAndKeepsStaValid)
{
  readTestDesign("fanout_repair_sta.v");
  odb::dbNet* net = block_->findNet("high_fanout");
  odb::dbMaster* buffer_master = lib_->findMaster("BUF_X1");
  ASSERT_NE(net, nullptr);
  ASSERT_NE(buffer_master, nullptr);
  ASSERT_EQ(ecc::FanoutRepairer::loadCount(net), 10);

  ecc::FanoutRepairOptions options;
  options.max_fanout = 3;
  options.insertion.buffer_name_prefix = "sizer_fanout_buf";
  options.insertion.net_name_prefix = "sizer_fanout_net";

  est::IncrementalParasiticsGuard parasitics_guard(&ep_);
  const ecc::FanoutRepairResult result
      = ecc::FanoutRepairer::run(block_, buffer_master, options);

  ASSERT_TRUE(result.ok()) << result.message;
  EXPECT_EQ(result.before_max_fanout, 10);
  EXPECT_EQ(result.after_max_fanout, 3);
  EXPECT_EQ(result.inserted_buffers.size(), 4);
  for (odb::dbNet* result_net : block_->getNets()) {
    EXPECT_LE(ecc::FanoutRepairer::loadCount(result_net), 3)
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

TEST_F(BufferInserterStaTest, NetRepairPlanningDoesNotMutateAndActionsCanBeUndone)
{
  readTestDesign("fanout_repair_sta.v");
  ecc::DrvRepairOptions options;
  options.common.max_fanout = 3;
  ecc::detail::DesignAnalysis backend(block_, sta_.get(), options.common,
                                      runtime_.constraints);
  ecc::DesignMetrics metrics;
  const auto states = backend.analyze(metrics);
  const auto found = std::find_if(states.begin(), states.end(), [](const auto& state) {
    return state.net->getName() == "high_fanout";
  });
  ASSERT_NE(found, states.end());
  const auto instances = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  ecc::detail::CellCandidates cells(backend, sta_.get(), options.common);
  const auto plan = ecc::planDrvNetRepair(backend, cells, sta_.get(), *found, 2, false,
                                          options.common);
  ASSERT_FALSE(plan.buffers.empty());
  EXPECT_LE(plan.buffers.size(), 2u);
  EXPECT_FALSE(plan.replacement.has_value());
  EXPECT_EQ(block_->getInsts().size(), instances);
  EXPECT_EQ(block_->getNets().size(), nets);
  EXPECT_EQ(ecc::FanoutRepairer::loadCount(found->net), 10u);
  odb::dbDatabase::beginEco(block_);
  const auto inserted = ecc::insertBuffers(plan.buffers);
  EXPECT_EQ(inserted, plan.buffers.size());
  EXPECT_EQ(block_->getInsts().size(), instances + inserted);
  odb::dbDatabase::endEco(block_);
  odb::dbDatabase::undoEco(block_);
  EXPECT_EQ(block_->getInsts().size(), instances);
  EXPECT_EQ(block_->getNets().size(), nets);
  EXPECT_EQ(ecc::FanoutRepairer::loadCount(found->net), 10u);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, DrvRepairUsesStaAndHonorsBudget)
{
  readTestDesign("fanout_repair_sta.v");
  ecc::DrvRepairOptions options;
  options.common.max_fanout = 3;
  options.max_inserted_buffers = 20;
  const auto refresh = [&]() {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  refresh();
  const auto result =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.accepted_passes, 0);
  EXPECT_LT(result.after.severity, result.before.severity);
  EXPECT_LE(result.inserted_buffers, *options.max_inserted_buffers);
  EXPECT_GE(result.after.setup_wns + 1e-15f, result.before.setup_wns);
  db_network_->checkAxioms();
  EXPECT_TRUE(sta_->search()->arrivalsValid());
}

TEST_F(BufferInserterStaTest,
       DrvRepairRestoresTopologyOnRefreshFailure)
{
  readTestDesign("fanout_repair_sta.v");
  auto* net = block_->findNet("high_fanout");
  const auto insts = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  ecc::DrvRepairOptions options;
  options.common.max_fanout = 3;
  unsigned updates = 0;
  const auto refresh = [&]() {
    if (++updates == 1)
      throw std::runtime_error("injected timing update failure");
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  const auto result =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(block_->getInsts().size(), insts);
  EXPECT_EQ(block_->getNets().size(), nets);
  EXPECT_EQ(ecc::FanoutRepairer::loadCount(net), 10);
  EXPECT_EQ(updates, 2);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest,
       DrvRepairSkipsProtectedLoadAndExcludedCells)
{
  readTestDesign("fanout_repair_sta.v");
  block_->findInst("load0")->setDoNotTouch(true);
  ecc::DrvRepairOptions options;
  options.common.max_fanout = 3;
  const auto refresh = [&]() { sta_->updateTiming(true); };
  const auto result =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(
      ecc::FanoutRepairer::loadCount(block_->findNet("high_fanout")), 10);
  for (auto* master : lib_->getMasters())
    runtime_.constraints.excluded_masters.push_back(master);
  const auto excluded =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  EXPECT_FALSE(excluded.ok);
}

TEST_F(BufferInserterStaTest,
       DrvRepairHandlesCapWithoutFanoutViolation)
{
  readTestDesign("fanout_repair_sta.v");
  auto* cell = db_network_->libertyCell(block_->findInst("driver"));
  sta::LibertyPort *input, *output;
  cell->bufferPorts(input, output);
  output->setCapacitanceLimit(input->capacitance() * 3, sta::MinMax::max());
  ecc::DrvRepairOptions options;
  options.common.max_fanout = 64;
  const auto refresh = [&]() {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  refresh();
  const auto result =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.before.fanout_nets, 0);
  EXPECT_GT(result.before.capacitance_nets, 0);
  EXPECT_LT(result.after.capacitance_nets, result.before.capacitance_nets);
  EXPECT_GT(result.resized_buffers + result.inserted_buffers, 0);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, DrvRepairHonorsPositiveBufferBudget)
{
  readTestDesign("fanout_repair_sta.v");
  ecc::DrvRepairOptions options;
  options.common.max_fanout = 3;
  options.max_inserted_buffers = 1;
  const auto count = block_->getInsts().size();
  const auto result = ecc::DrvRepairer::run(
      block_, sta_.get(), [&]() { sta_->updateTiming(true); }, options, runtime_);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.converged);
  EXPECT_GT(result.after.fanout_nets, 0);
  EXPECT_EQ(result.inserted_buffers, 1);
  EXPECT_EQ(block_->getInsts().size(), count + 1);
}

TEST_F(BufferInserterStaTest,
       DrvRepairHandlesSlewWithoutFanoutViolation)
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
  const auto result =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, {}, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.before.fanout_nets, 0);
  EXPECT_EQ(result.before.capacitance_nets, 0);
  EXPECT_GT(result.before.slew_nets, 0);
  EXPECT_LT(result.after.slew_nets, result.before.slew_nets);
  EXPECT_GT(result.resized_buffers + result.inserted_buffers, 0);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, DrvRepairRejectsTimingRegression)
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
  ecc::DrvRepairOptions options;
  options.common.max_fanout = 3;
  const auto result =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.rejected_passes, 1);
  EXPECT_EQ(result.accepted_passes, 0);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(block_->getInsts().size(), insts);
  EXPECT_EQ(block_->getNets().size(), nets);
  EXPECT_EQ(
      ecc::FanoutRepairer::loadCount(block_->findNet("high_fanout")), 10);
  EXPECT_NEAR(result.after.severity, result.before.severity, 1e-6);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, DrvRepairRecognizesSdcClockNets)
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
  ecc::DrvRepairOptions options;
  options.common.max_fanout = 3;
  const auto result = ecc::DrvRepairer::run(
      block_, sta_.get(), [&]() { sta_->updateTiming(true); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_TRUE(sta_->isClock(db_network_->dbToSta(net), sta_->cmdMode()));
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(ecc::FanoutRepairer::loadCount(net), 10);
}

TEST_F(BufferInserterStaTest, DrvRepairReportsFailedRollbackRefresh)
{
  readTestDesign("fanout_repair_sta.v");
  ecc::DrvRepairOptions options;
  options.common.max_fanout = 3;
  const auto insts = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  const auto result = ecc::DrvRepairer::run(
      block_, sta_.get(), []() { throw std::runtime_error("timing unavailable"); },
      options, runtime_);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("timing refresh after rollback failed"),
            std::string::npos);
  EXPECT_EQ(block_->getInsts().size(), insts);
  EXPECT_EQ(block_->getNets().size(), nets);
  EXPECT_EQ(
      ecc::FanoutRepairer::loadCount(block_->findNet("high_fanout")), 10);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, GateResizeUsesEquivalentFunctionAndKeepsNets)
{
  readTestDesign("gate_repair_sta.v");
  constrainGateCap();
  auto* driver = block_->findInst("driver");
  auto* original = db_network_->libertyCell(driver);
  auto* in1 = driver->findITerm("A1")->getNet();
  auto* in2 = driver->findITerm("A2")->getNet();
  auto* out = driver->findITerm("ZN")->getNet();
  ecc::DrvRepairOptions options;
  options.max_inserted_buffers = 0;
  const auto refresh = [&]() {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  refresh();
  const auto result =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
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
       GateResizeRejectsWrongFunctionAndExcludedMasters)
{
  readTestDesign("gate_repair_sta.v");
  constrainGateCap();
  auto* driver = block_->findInst("driver");
  auto* original = driver->getMaster();
  ecc::DrvRepairOptions options;
  options.max_inserted_buffers = 1;
  excludeAllBuffers();
  // Other gates such as NOR2 have the same physical pin names but a different
  // Boolean function. They must not be considered when NAND upsizes are banned.
  runtime_.constraints.excluded_masters = {lib_->findMaster("NAND2_X2"),
                                           lib_->findMaster("NAND2_X4")};
  const auto result = ecc::DrvRepairer::run(
      block_, sta_.get(), [&]() { sta_->updateTiming(true); }, options, runtime_);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.resized_gates, 0);
  EXPECT_EQ(driver->getMaster(), original);
  EXPECT_GT(result.after.capacitance_nets, 0);
}

TEST_F(BufferInserterStaTest, RepairLegalizesEachAcceptedBatch)
{
  readTestDesign("fanout_repair_sta.v");
  addLegalRows();
  ecc::DrvRepairOptions options;
  options.common.max_fanout = 3;
  unsigned prepared = 0;
  runtime_.prepare = [&]() {
    ecc::legalizeRepairPlacement(block_, &dp_);
    ++prepared;
  };
  const auto refresh = [&]() {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  refresh();
  const auto result =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.accepted_passes, 0);
  EXPECT_EQ(prepared, result.accepted_passes + result.rejected_passes);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest,
       RepairRestoresResizeAndPlacementOnPrepareFailure)
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
  ecc::DrvRepairOptions options;
  options.max_inserted_buffers = 0;
  runtime_.prepare = [&]() {
    ecc::legalizeRepairPlacement(block_, &dp_);
    block_->findInst("load9")->setLocation(1333, 2777);
    throw std::runtime_error("injected legalization failure");
  };
  const auto refresh = [&]() {
    ep_.estimateParasitics(est::ParasiticsSrc::kPlacement);
    sta_->updateTiming(true);
  };
  refresh();
  const auto result =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
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

TEST_F(BufferInserterStaTest, SetupRepairImprovesTimingWithoutNewDrv)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.max_area_growth = 0.5;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::SetupRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_LT(result.before.setup_wns, 0);
  EXPECT_GT(result.accepted_batches, 0);
  EXPECT_GT(result.after.setup_wns, result.before.setup_wns);
  EXPECT_GT(result.after.setup_tns, result.before.setup_tns);
  EXPECT_GE(result.after.hold_wns, result.before.hold_wns);
  EXPECT_LE(result.after.total_cap_violations, result.before.total_cap_violations);
  EXPECT_LE(result.after.total_slew_violations, result.before.total_slew_violations);
  EXPECT_LE(result.after.total_fanout_violations, result.before.total_fanout_violations);
  EXPECT_LE(result.attempted_batches, options.max_trials);
  EXPECT_LE(result.after.area, result.before.area * 1.5);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, SetupRepairBuffersNoncriticalBranches)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  auto* critical_net = block_->findInst("load0")->findITerm("A")->getNet();
  ecc::SetupRepairOptions options;
  options.common.actions.resize = false;
  options.max_path_ends = 1;
  options.max_passes = 2;
  options.max_inserted_buffers = 1;
  options.batch_size = 1;
  options.max_area_growth = 0.5;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::SetupRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.inserted_buffers, 1u);
  EXPECT_GT(result.after.setup_wns, result.before.setup_wns);
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
  ecc::PathSearchOptions options;
  options.max_paths = 1;
  options.max_candidates = 4;
  options.max_trials = 4;
  options.max_area_growth = 0.5;
  options.common.actions.resize = false;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  std::vector<ecc::BufferEvent> events;
  runtime_.event_observer = [&](const ecc::BufferEvent& snapshot) {
    EXPECT_FALSE(snapshot.placement_provisional);
    events.push_back(snapshot);
  };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.front().kind, ecc::BufferEvent::Kind::SearchStarted);
  EXPECT_EQ(events.back().kind, ecc::BufferEvent::Kind::SearchFinished);
  EXPECT_EQ(events.back().attempted_trials, result.attempted_trials);
  EXPECT_EQ(events.back().inserted_buffers, result.inserted_buffers);
  EXPECT_EQ(events.back().setup_wns, result.after.setup_wns);
  EXPECT_EQ(events.back().setup_tns, result.after.setup_tns);
  EXPECT_GT(result.attempted_trials, 0u);
  EXPECT_GT(result.accepted_trials, 0u);
  EXPECT_EQ(result.attempted_trials, result.accepted_trials + result.rejected_trials);
  EXPECT_EQ(result.trials.size(), result.attempted_trials);
  EXPECT_LE(result.refreshes, result.attempted_trials + 1);
  EXPECT_LE(result.rollback_refreshes, 1u);
  EXPECT_GT(result.after.setup_wns, result.before.setup_wns);
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
  ecc::PathSearchOptions options;
  options.max_paths = 1;
  options.max_candidates = 4;
  options.max_trials = std::nullopt;
  options.max_area_growth = 0;
  options.common.actions.resize = false;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto no_growth = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(no_growth.ok) << no_growth.message;
  EXPECT_EQ(no_growth.attempted_trials, 0u);
  EXPECT_EQ(no_growth.inserted_buffers, 0u);
  options.max_area_growth = std::nullopt;
  const auto instances = block_->getInsts().size();
  for (unsigned budget = 0; budget < 3; ++budget) {
    auto zero = options;
    switch (budget) {
    case 0: zero.max_trials = 0; break;
    case 1: zero.max_refreshes = 0; break;
    case 2: zero.search_patience = 0; break;
    }
    const auto stopped = ecc::BufferRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, zero, runtime_);
    ASSERT_TRUE(stopped.ok) << stopped.message;
    EXPECT_EQ(stopped.attempted_trials, 0u);
    EXPECT_EQ(stopped.inserted_buffers, 0u);
    EXPECT_EQ(block_->getInsts().size(), instances);
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  }
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.accepted_trials, 0u);
  EXPECT_EQ(result.attempted_trials, result.accepted_trials + result.rejected_trials);
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
  ecc::PathSearchOptions options;
  options.max_trials = 4;
  options.max_area_growth = 0.5;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto initial_instances = block_->getInsts().size();
  unsigned reports = 0;
  unsigned accepted = 0;
  runtime_.trial_observer = [&](unsigned index, const ecc::BufferTrialResult &trial) {
    EXPECT_EQ(index, ++reports);
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
    if(trial.rejection_reasons.empty())
      accepted += trial.buffers;
    EXPECT_EQ(block_->getInsts().size(), initial_instances + accepted);
  };
  // Exercise actual STA rejection independently of relative-gain screening.
  options.min_relative_improvement = 0;
  options.min_improvement = 1;
  const auto rejected = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(rejected.ok) << rejected.message;
  EXPECT_GT(reports, 0u);
  EXPECT_EQ(reports, rejected.attempted_trials);
  EXPECT_EQ(accepted, 0u);
  EXPECT_FLOAT_EQ(rejected.before.setup_wns, rejected.after.setup_wns);
  reports = 0;
  // A 100% initial-slack threshold cannot be exceeded by negative slack
  // clamped at zero, so this run screens before any edit or STA trial.
  options.min_relative_improvement = 1;
  options.min_improvement = 1e-13f;
  const auto screened = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(screened.ok) << screened.message;
  EXPECT_GT(screened.search.screened_candidates, 0u);
  EXPECT_EQ(accepted, 0u);
  EXPECT_EQ(reports, screened.attempted_trials);
  EXPECT_EQ(block_->getInsts().size(), initial_instances);
  reports = 0;
  options.min_relative_improvement = 0;
  const auto repaired = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(repaired.ok) << repaired.message;
  EXPECT_EQ(reports, repaired.attempted_trials);
  EXPECT_GT(accepted, 0u);
  EXPECT_EQ(repaired.search.feedback_updates, repaired.attempted_trials);
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
  ecc::PathSearchOptions options;
  options.endpoint_coverage = true;
  options.group_schedule = true;
  options.gain_scope = ecc::PathGainScope::Global;
  options.max_paths = 1;
  options.max_trials = 8;
  options.max_area_growth = 0.5;
  options.min_relative_improvement = 0;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
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
  ecc::PathSearchOptions options;
  options.tree_candidates = false; // Isolate post-insertion fanout completion.
  options.max_trials = 4;
  options.max_area_growth = std::nullopt;
  options.min_relative_improvement = 0; // Exercise real STA, without forecast screening.
  options.min_improvement = 1.0f; // Force post-STA rollback.
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned tree_trials = 0, diagnostics = 0;
  runtime_.trial_observer = [&](unsigned, const ecc::BufferTrialResult &trial) {
    if(trial.buffers > trial.candidates.size()) {
      ++tree_trials;
      EXPECT_LE(trial.after.total_fanout_violations, trial.before.total_fanout_violations);
    }
    EXPECT_FALSE(trial.rejection_reasons.empty());
    EXPECT_EQ(block_->getInsts().size(), initial_instances);
    EXPECT_EQ(block_->getNets().size(), initial_nets);
    for(const auto& [pin, net] : connections)
      EXPECT_EQ(pin->getNet(), net);
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  };
  runtime_.fanout_observer = [&](const ecc::BufferFanoutCheck &check) {
    ++diagnostics;
    EXPECT_LT(check.slack, 0);
    EXPECT_FALSE(check.pin.empty());
  };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
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
  ecc::PathSearchOptions options;
  options.tree_candidates = false; // Isolate post-insertion fanout completion.
  options.max_trials = std::nullopt;
  options.max_area_growth = std::nullopt;
  options.max_inserted_buffers = 2;
  options.min_relative_improvement = 0; // Exercise real STA, without forecast screening.
  options.min_improvement = 1.0f; // Force post-STA rollback.
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned largest_trial = 0;
  runtime_.trial_observer = [&](unsigned, const ecc::BufferTrialResult &trial) {
    largest_trial = std::max(largest_trial, trial.buffers);
    EXPECT_LE(trial.after.total_fanout_violations, trial.before.total_fanout_violations);
  };
  bool saw_sdc_limit = false;
  runtime_.fanout_observer = [&](const ecc::BufferFanoutCheck &check) {
    if(check.phase == "candidate_buffer") {
      saw_sdc_limit = true;
      EXPECT_FLOAT_EQ(check.limit, 1.5f); // SDC tighter than Liberty's 100.
    }
  };
  const auto bounded = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(bounded.ok) << bounded.message;
  EXPECT_TRUE(saw_sdc_limit);
  EXPECT_GT(bounded.search.fanout_precheck_rejections, 0u);
  EXPECT_LE(largest_trial, 2u);
  EXPECT_EQ(bounded.inserted_buffers, 0u);
  options.max_inserted_buffers.reset();
  const auto unbounded = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
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
  ecc::PathSearchOptions options;
  options.tree_candidates = false; // Isolate post-insertion fanout completion.
  runtime_.constraints.excluded_masters.push_back(lib_->findMaster("BUF_X1"));
  options.max_area_growth = std::nullopt;
  options.min_relative_improvement = 0; // Exercise real STA, without forecast screening.
  options.min_improvement = 1.0f; // Force post-STA rollback.
  options.max_trials = 4;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned singleton_trees = 0;
  runtime_.trial_observer = [&](unsigned, const ecc::BufferTrialResult &trial) {
    if(trial.buffers > trial.candidates.front().loads) {
      ++singleton_trees;
      EXPECT_LE(trial.after.total_fanout_violations, trial.before.total_fanout_violations);
    }
  };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(singleton_trees, 0u);
  EXPECT_GT(result.fanout_tree_trials, 0u);
  // Exercise the same heavy-sink/lighter-buffer boundary in the planner,
  // before ECO fanout completion. Singleton leaf groups must remain feasible.
  singleton_trees = 0;
  options.tree_candidates = true;
  const auto planned = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
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
  ecc::PathSearchOptions options;
  options.tree_candidates = false; // Isolate post-insertion fanout completion.
  options.max_area_growth = std::nullopt;
  options.max_trials = std::nullopt;
  unsigned prepares = 0;
  runtime_.prepare = [&]() { ++prepares; };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.search.fanout_precheck_rejections, 0u);
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
  ecc::PathSearchOptions options;
  options.tree_candidates = true;
  options.max_trials = std::nullopt;
  options.max_candidates = 64;
  for(auto* master : lib_->getMasters()) {
    auto* cell = db_network_->libertyCell(db_network_->dbToSta(master));
    if(cell && cell->isBuffer() && master->getName() != "BUF_X1")
      runtime_.constraints.excluded_masters.push_back(master);
  }
  options.min_relative_improvement = 0; // Exercise real STA, without forecast screening.
  options.min_improvement = 1.0f; // Force post-STA rollback.
  options.max_area_growth = std::nullopt;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  unsigned planned_trees = 0;
  runtime_.trial_observer = [&](unsigned, const ecc::BufferTrialResult &t) {
    for(const auto& c : t.candidates)
      if (c.planned_buffers > 1) {
        ++planned_trees;
        EXPECT_EQ(c.loads, 10u);
        EXPECT_EQ(t.buffers, c.planned_buffers);
        EXPECT_LE(t.after.total_fanout_violations, t.before.total_fanout_violations);
      }
    EXPECT_FALSE(t.rejection_reasons.empty());
    EXPECT_EQ(block_->getInsts().size(), instances);
    EXPECT_EQ(block_->getNets().size(), nets);
    for(const auto& [pin, net] : connections)
      EXPECT_EQ(pin->getNet(), net);
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  };
  auto run = [&]() {
    return ecc::BufferRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  };
  const auto result = run();
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(planned_trees, 0u);
  EXPECT_EQ(result.inserted_buffers, 0u);
  // The allowance applies to the entire planned tree, before mutating DB.
  planned_trees = 0;
  options.max_inserted_buffers = 1;
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
  ecc::PathSearchOptions options;
  options.gain_scope = ecc::PathGainScope::Endpoints;
  options.max_area_growth = std::nullopt;
  options.max_trials = 4;
  options.min_relative_improvement = 0; // Exercise real STA, without forecast screening.
  options.min_improvement = 1.0f; // Force post-STA rollback.
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  auto run = [&]() {
    return ecc::BufferRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  };
  options.recovery = false;
  auto setup_only = run();
  ASSERT_TRUE(setup_only.ok) << setup_only.message;
  for(const auto& t : setup_only.trials)
    for(const auto& c : t.candidates) EXPECT_FALSE(c.recovery);
  options.recovery = true;
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
  ecc::PathSearchOptions options;
  options.gain_scope = ecc::PathGainScope::Endpoints;
  options.min_relative_improvement = 0.001;
  options.max_trials = 4;
  options.max_area_growth = std::nullopt;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
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

TEST_F(BufferInserterStaTest, PathCurrentGlobalScaleKeepsMeasuredTimingAndElectricalGuards)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows(); constrainSetup(true); refreshRepairTiming();
  ecc::PathSearchOptions options;
  options.gain_scope = ecc::PathGainScope::CurrentGlobal;
  options.min_relative_improvement = 0.01;
  options.max_trials = 8;
  options.max_area_growth = std::nullopt;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.inserted_buffers, 0u);
  for(const auto& trial : result.trials) {
    if(!trial.rejection_reasons.empty()) continue;
    const double wns_floor = std::max(double(options.min_improvement),
        std::abs(double(trial.before.setup_wns)) * options.min_relative_improvement);
    const double tns_floor = std::max({double(options.min_improvement),
        std::abs(double(trial.before.setup_tns)) * options.min_relative_improvement,
        double(ecc::detail::tnsTolerance(trial.before.setup_tns, options.common.tolerances))});
    EXPECT_TRUE(trial.after.setup_wns > trial.before.setup_wns + wns_floor ||
                trial.after.setup_tns > trial.before.setup_tns + tns_floor);
    EXPECT_GE(trial.after.setup_wns + options.common.tolerances.wns_absolute, trial.before.setup_wns);
    EXPECT_GE(trial.after.hold_wns + options.common.tolerances.wns_absolute, trial.before.hold_wns);
    EXPECT_LE(trial.after.total_slew_violations, trial.before.total_slew_violations);
    EXPECT_LE(trial.after.total_cap_violations, trial.before.total_cap_violations);
    EXPECT_LE(trial.after.total_fanout_violations, trial.before.total_fanout_violations);
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
  ecc::PathSearchOptions options;
  options.recovery = false; // Isolate the gain-scope change.
  options.min_relative_improvement = 0.1;
  options.max_trials = 4;
  options.max_area_growth = std::nullopt;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  auto run = [&]() {
    return ecc::BufferRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  };
  options.gain_scope = ecc::PathGainScope::Global;
  const auto global = run();
  ASSERT_TRUE(global.ok) << global.message;
  EXPECT_GT(global.attempted_trials, 0u);
  EXPECT_EQ(global.inserted_buffers, 0u);
  // The independent reset path remains globally critical. Improving sampled
  // setup endpoints cannot be reported as a positive global WNS forecast.
  for(const auto& trial : global.trials)
    for(const auto& c : trial.candidates)
      EXPECT_LE(c.predicted_wns_gain, options.common.tolerances.wns_absolute);
  options.gain_scope = ecc::PathGainScope::Endpoints;
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
  ecc::PathSearchOptions options;
  options.max_paths = 1;
  options.max_trials = 1;
  options.max_area_growth = 0.5;
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
      ecc::BufferRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.accepted_trials, 0u);
  EXPECT_EQ(result.rejected_trials, 1u);
  ASSERT_EQ(result.trials.size(), 1u);
  EXPECT_FALSE(result.trials.front().rejection_reasons.empty());
  EXPECT_LT(result.trials.front().after.setup_wns, wns_before);
  EXPECT_EQ(result.refreshes, refreshes);
  EXPECT_EQ(result.rollback_refreshes, 1u);
  EXPECT_EQ(block_->getInsts().size(), instances_before);
  EXPECT_EQ(block_->getNets().size(), nets_before);
  EXPECT_NEAR(result.after.setup_wns, wns_before, 1e-15);
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
  ecc::PathSearchOptions options;
  options.max_paths = 10;
  options.max_trials = 4;
  options.max_candidates = 8;
  options.max_area_growth = 0.5;
  options.min_relative_improvement = 0;
  options.min_improvement = 1.0f; // Force every otherwise valid ECO to roll back.
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  auto run = [&]() {
    return ecc::BufferRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  };
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
    SCOPED_TRACE(i);
    EXPECT_EQ(deferred.trials[i].candidates.front().net,
              eager.trials[i].candidates.front().net);
    EXPECT_EQ(deferred.trials[i].candidates.front().master,
              eager.trials[i].candidates.front().master);
    EXPECT_EQ(deferred.trials[i].after.setup_wns, eager.trials[i].after.setup_wns);
    EXPECT_EQ(deferred.trials[i].after.setup_tns, eager.trials[i].after.setup_tns);
    EXPECT_EQ(deferred.trials[i].before.setup_tns, eager.trials[i].before.setup_tns);
    EXPECT_EQ(deferred.trials[i].candidates.front().ranked_tns_gain,
              eager.trials[i].candidates.front().ranked_tns_gain);
    saw_multiple_endpoints |= deferred.trials[i].candidates.front().affected_endpoints > 1;
  }
  EXPECT_TRUE(saw_multiple_endpoints);
  options.max_refreshes = 3;
  const auto limited = run();
  ASSERT_TRUE(limited.ok) << limited.message;
  EXPECT_EQ(limited.attempted_trials, 2u);
  EXPECT_EQ(limited.refreshes, 3u);
  EXPECT_EQ(limited.after.setup_wns, eager.after.setup_wns);
  options.max_refreshes = std::nullopt;
  options.max_trials = std::nullopt;
  options.search_patience = 2;
  const auto stopped = run();
  ASSERT_TRUE(stopped.ok) << stopped.message;
  EXPECT_TRUE(stopped.stopped_on_low_progress);
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
  ecc::PathSearchOptions options;
  options.max_trials = 4;
  options.max_area_growth = 0.5;
  options.min_relative_improvement = 0;
  options.min_improvement = 1.0f;
  unsigned prepares = 0;
  runtime_.prepare = [&]() {
    if(++prepares == 2)
      throw std::runtime_error("injected prepare failure");
  };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
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
  ecc::PathSearchOptions options;
  options.batch_size = 8;
  options.max_paths = 4;
  options.paths_per_endpoint = 4;
  options.max_trials = 1;
  options.max_area_growth = 1;
  options.min_relative_improvement = 0;
  options.min_improvement = 1.0f;
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_EQ(result.trials.size(), 1u);
  EXPECT_GT(result.search.candidate_count, 1u);
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
  ecc::PathSearchOptions options;
  options.batch_size = 8;
  options.max_inserted_buffers = 1;
  options.max_trials = 4;
  options.max_area_growth = 0.5;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_GT(result.attempted_trials, 0u);
  EXPECT_EQ(result.inserted_buffers, 1u);
  EXPECT_EQ(result.message, "path buffer insertion budget exhausted; violations remain");
  for(const auto& trial : result.trials) EXPECT_EQ(trial.candidates.size(), 1u);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
}

TEST_F(BufferInserterStaTest,
       SetupRepairHonorsZeroBudgetAndProtectedDriver)
{
  readTestDesign("gate_repair_sta.v");
  constrainSetup(true);
  refreshRepairTiming();
  const auto count = block_->getInsts().size();
  const auto master = block_->findInst("driver")->getMaster();
  ecc::SetupRepairOptions options;
  options.max_trials = 0;
  auto result = ecc::SetupRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.attempted_batches, 0);
  EXPECT_EQ(block_->getInsts().size(), count);
  EXPECT_EQ(block_->findInst("driver")->getMaster(), master);
  options.max_trials = 8;
  for (auto* inst : block_->getInsts())
    runtime_.constraints.excluded_instances.push_back(inst);
  result = ecc::SetupRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.attempted_batches, 0);
  EXPECT_EQ(block_->getInsts().size(), count);
}

TEST_F(BufferInserterStaTest,
       SetupRepairRejectsBadParasiticsAndRestoresPlacement)
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
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
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
  const auto result =
      ecc::SetupRepairer::run(block_, sta_.get(), refresh, options, runtime_);
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
      result.after.setup_wns, result.before.setup_wns, 1e-15);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
}

TEST_F(BufferInserterStaTest, SetupRepairRestoresOnPreparationFailure)
{
  readTestDesign("gate_repair_sta.v");
  constrainSetup();
  refreshRepairTiming();
  const auto count = block_->getInsts().size();
  auto* driver = block_->findInst("driver");
  auto* master = driver->getMaster();
  ecc::SetupRepairOptions options;
  options.max_area_growth = 0.5;
  runtime_.prepare = []() {
    throw std::runtime_error("injected setup preparation failure");
  };
  const auto result = ecc::SetupRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.accepted_batches, 0);
  EXPECT_EQ(block_->getInsts().size(), count);
  EXPECT_EQ(driver->getMaster(), master);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, SetupRepairRejectsHoldLossDespiteSetupGain)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.max_area_growth = 0.5;
  options.batch_size = 1;
  options.max_trials = 1;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
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
  const auto result =
      ecc::SetupRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(candidate_setup, result.before.setup_wns);
  EXPECT_LT(candidate_hold, result.before.hold_wns);
  EXPECT_EQ(result.accepted_batches, 0);
  EXPECT_EQ(result.rejected_batches, 1);
  EXPECT_NEAR(
      result.after.setup_wns, result.before.setup_wns, 1e-15);
  EXPECT_EQ(result.after.hold_wns, result.before.hold_wns);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
}

TEST_F(BufferInserterStaTest, SetupRepairHonorsZeroAreaGrowth)
{
  readTestDesign("gate_repair_sta.v");
  constrainSetup();
  refreshRepairTiming();
  ecc::SetupRepairOptions options;
  options.max_area_growth = 0;
  const auto result = ecc::SetupRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.attempted_batches, 0);
  EXPECT_EQ(result.after.area, result.before.area);
  EXPECT_EQ(result.after.setup_wns, result.before.setup_wns);
  EXPECT_EQ(result.inserted_buffers, 0);
  EXPECT_EQ(result.resized_cells, 0);
}

TEST_F(BufferInserterStaTest, RepairConfigZeroPassAndInvalidPolicyDoNotEdit)
{
  readTestDesign("gate_repair_sta.v");
  constrainGateCap();
  refreshRepairTiming();
  const auto count = block_->getInsts().size();
  const auto master = block_->findInst("driver")->getMaster();
  ecc::RepairConfig config;
  config.set("drv.max_passes", "0");
  unsigned refreshes = 0;
  auto refresh = [&]() { ++refreshes; refreshRepairTiming(); };
  auto result = ecc::DrvRepairer::run(block_, sta_.get(), refresh,
                                         config.drvOptions(), runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.accepted_passes, 0);
  EXPECT_EQ(refreshes, 0);
  EXPECT_EQ(block_->getInsts().size(), count);
  EXPECT_EQ(block_->findInst("driver")->getMaster(), master);
  config.common.overload_group_fraction = 1;
  result = ecc::DrvRepairer::run(block_, sta_.get(), refresh, config.drvOptions(),
                                    runtime_);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(refreshes, 0);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
}

TEST_F(BufferInserterStaTest, SetupConfigControlsGainAndTrialBudgets)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup();
  refreshRepairTiming();
  ecc::RepairConfig config;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  config.set("setup.min_improvement_ns", "1000000");
  config.set("setup.max_area_growth", "0.5");
  auto refresh = [&]() { refreshRepairTiming(); };
  auto result = ecc::SetupRepairer::run(block_, sta_.get(), refresh,
                                           config.setupOptions(), runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.attempted_batches, 0);
  EXPECT_EQ(result.after.setup_wns, result.before.setup_wns);
  config.set("setup.min_improvement_ns", "0.0001");
  config.set("setup.max_trials", "1");
  config.set("setup.batch_size", "1");
  result = ecc::SetupRepairer::run(block_, sta_.get(), refresh,
                                      config.setupOptions(), runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.attempted_batches, 1);
  EXPECT_EQ(result.accepted_batches, 1);
  EXPECT_GT(result.after.setup_wns, result.before.setup_wns);
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, SetupSkipsDuplicateEditsAndDefersRollbackRefresh)
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
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  options.skip_duplicate_trials = false;
  options.defer_rollback_refresh = false;
  const auto eager =
      ecc::SetupRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(eager.ok) << eager.message;
  ASSERT_EQ(eager.accepted_batches, 0);
  EXPECT_EQ(eager.refreshes, 2 * eager.rejected_batches);
  EXPECT_EQ(eager.refreshes, refresh_count);
  options.skip_duplicate_trials = true;
  options.defer_rollback_refresh = true;
  refresh_count = 0;
  const auto optimized =
      ecc::SetupRepairer::run(block_, sta_.get(), refresh, options, runtime_);
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
  EXPECT_NEAR(optimized.after.setup_wns, eager.after.setup_wns, 1e-15);
  EXPECT_NEAR(optimized.after.setup_tns, eager.after.setup_tns, 1e-15);
  EXPECT_NEAR(float(sta_->worstSlack(sta::MinMax::max())), optimized.after.setup_wns, 1e-15);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_NO_THROW(dp_.checkPlacement(false));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, SetupDeferredRollbackSurvivesLaterPrepareFailure)
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
  runtime_.prepare = [&]() {
    if (++prepares == 2)
      throw std::runtime_error("later prepare failure");
    ecc::legalizeRepairPlacement(block_, &dp_);
  };
  const auto result =
      ecc::SetupRepairer::run(block_, sta_.get(), refresh, options, runtime_);
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
  ecc::DrvRepairOptions options;
  options.common.actions.resize = false;
  options.max_inserted_buffers = 1;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::DrvRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
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
  ecc::DrvRepairOptions options;
  options.common.actions.load_downsize = false;
  options.common.actions.resize = false;
  options.max_inserted_buffers = 1;
  auto refresh = [&]() { refreshRepairTiming(); };
  auto result =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.downsized_loads, 0);
  options.common.actions.load_downsize = true;
  options.max_load_downsizes_per_pass = 1;
  options.max_passes = 1;
  result = ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
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
  runtime_.constraints.excluded_instances.push_back(block_->findInst("load3"));
  runtime_.constraints.excluded_masters.push_back(lib_->findMaster("BUF_X1"));
  ecc::detail::DesignAnalysis backend(block_, sta_.get(), options,
                                      runtime_.constraints);
  ecc::DesignMetrics metrics;
  const auto states = backend.analyze(metrics, true);
  // Source net is conservatively skipped when it includes a protected load.
  // Query its state from an unprotected snapshot solely to test planner guards.
  block_->findInst("load2")->setDoNotTouch(false);
  ecc::RepairCommonOptions source_options;
  ecc::detail::DesignAnalysis source_backend(block_, sta_.get(), source_options);
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
  options.common.actions.resize = false;
  options.max_inserted_buffers = 0;
  options.max_area_growth = 0;
  options.batch_size = 1;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::SetupRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.downsized_loads, 0);
  EXPECT_GT(result.after.setup_wns, result.before.setup_wns);
  EXPECT_GT(result.after.setup_tns, result.before.setup_tns);
  EXPECT_LT(result.after.area, result.before.area);
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
  ecc::DrvRepairOptions options;
  options.max_inserted_buffers = 0;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
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
  const auto result =
      ecc::DrvRepairer::run(block_, sta_.get(), refresh, options, runtime_);
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
  ecc::DrvRepairOptions options;
  options.max_inserted_buffers = 0;
  for (auto* master : lib_->getMasters()) {
    auto* cell = db_network_->libertyCell(db_network_->dbToSta(master));
    if (cell && cell->isBuffer())
      runtime_.constraints.excluded_masters.push_back(master);
  }
  const auto result = ecc::DrvRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
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
  options.common.actions.load_downsize = false;
  options.max_area_growth = 0;
  options.max_inserted_buffers = 0;
  auto refresh = [&]() { refreshRepairTiming(); };
  auto disabled =
      ecc::SetupRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(disabled.ok);
  EXPECT_EQ(disabled.vt_swaps, 0);
  options.common.vt_suffixes = {"_H", "_L"};
  const auto result =
      ecc::SetupRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.vt_swaps, 1) << result.message << " attempts=" << result.attempted_batches;
  EXPECT_GT(result.after.setup_wns, result.before.setup_wns);
  EXPECT_EQ(result.after.area, result.before.area);
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
  ecc::DrvRepairOptions options;
  options.common.vt_suffixes = {"_H", "_L"};
  options.common.actions.load_downsize = false;
  options.max_inserted_buffers = 0;
  options.max_passes = 1;
  const auto result = ecc::DrvRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
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
  const auto inspect = [&](const ecc::RepairCommonOptions &common) {
    ecc::detail::DesignAnalysis backend(block_, sta_.get(), common,
                                        runtime_.constraints);
    ecc::DesignMetrics metrics;
    const auto states = backend.analyze(metrics, true);
    ecc::detail::CellCandidates cells(backend, sta_.get(), common);
    std::vector<const ecc::detail::CellModel*> found;
    for (const auto& state : states)
      if (state.driver->getInst() == driver) {
        found = cells.resize(state, false, true);
        EXPECT_LE(found.size(), 1);
        if (!found.empty())
          EXPECT_EQ(found.front()->master->getName(), "VTBUF_L");
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
  runtime_.constraints.excluded_masters = {lib_->findMaster("VTBUF_L")};
  EXPECT_EQ(inspect(options), 0);
  runtime_.constraints.excluded_masters.clear();
  driver->findITerm("A")->getNet()->setDoNotTouch(true);
  EXPECT_EQ(inspect(options), 0);
}

TEST_F(BufferInserterStaTest, PowerRecoveryLowersLeakageWithVtAndPreservesTiming)
{
  readVtDesign();
  constrainSetup(true); // Keep an unrelated negative-slack path unchanged.
  refreshRepairTiming();
  ecc::PowerRepairOptions options;
  options.common.vt_suffixes = {"_H", "_L"};
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::PowerRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
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

TEST_F(BufferInserterStaTest, PowerRecoveryHonorsBudgetSlackAndTotalObjective)
{
  readVtDesign();
  ecc::PowerRepairOptions options;
  options.common.vt_suffixes = {"_H", "_L"};
  const auto run = [&]() {
    return ecc::PowerRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  };
  options.max_trials = 0;
  EXPECT_EQ(run().attempted_trials, 0);
  options.max_trials = 1;
  options.slack_margin = 1;
  EXPECT_EQ(run().attempted_trials, 0);
  options.slack_margin = 0;
  options.objective = ecc::PowerObjective::Total;
  const auto result = run();
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.attempted_trials, 1);
  EXPECT_EQ(result.vt_swaps, 1);
  EXPECT_LT(result.after.total, result.before.total);
}

TEST_F(BufferInserterStaTest, PowerRecoveryRollsBackBadTimingAndPrepareException)
{
  readVtDesign();
  auto* side = block_->findInst("side");
  auto* original = side->getMaster();
  ecc::PowerRepairOptions options;
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
  auto result =
      ecc::PowerRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.rejected_batches, 1);
  EXPECT_EQ(result.accepted_batches, 0);
  EXPECT_EQ(result.before.leakage, result.after.leakage);
  EXPECT_EQ(side->getMaster(), original);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  runtime_.prepare = []() { throw std::runtime_error("power prepare failure"); };
  result =
      ecc::PowerRepairer::run(block_, sta_.get(), refresh, options, runtime_);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("power prepare failure"), std::string::npos);
  EXPECT_EQ(side->getMaster(), original);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

// Two L -> H moves save 18 nW but violate both output constraints. Changing
// their shared driver H -> L repairs both paths for 9 nW: a useful joint move
// which neither the former slack-only power stage nor setup alone can make.
TEST_F(BufferInserterStaTest,
       CompoundPowerRecoveryUsesSetupAndSharedBudget)
{
  makePowerRecoveryDesign();
  ecc::PowerRepairOptions options;
  options.common.vt_suffixes = {"_H", "_L"};
  options.max_passes = 1;
  options.max_trials = 1;
  const auto run = [&]() {
    return ecc::PowerRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  };
  auto result = run();
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.accepted_batches, 0u);
  EXPECT_EQ(result.attempted_trials, 1u);
  EXPECT_EQ(result.recovery_attempts, 0u);
  EXPECT_EQ(block_->findInst("driver")->getMaster()->getName(), "VTBUF_H");
  EXPECT_EQ(block_->findInst("side")->getMaster()->getName(), "VTBUF_L");
  options.max_trials = 8;
  result = run();
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.accepted_batches, 1u);
  EXPECT_EQ(result.attempted_trials, 2u);
  EXPECT_GT(result.recovery_attempts, 0u);
  EXPECT_LE(result.attempted_trials, options.max_trials);
  EXPECT_NEAR(result.before.leakage - result.after.leakage, 9e-9, 1e-12);
  EXPECT_GE(result.after.design.setup_wns, result.before.design.setup_wns);
  EXPECT_GE(result.after.design.hold_wns, result.before.design.hold_wns);
  EXPECT_LE(result.after.design.area, result.before.design.area);
  EXPECT_EQ(block_->findInst("driver")->getMaster()->getName(), "VTBUF_L");
  EXPECT_EQ(block_->findInst("side")->getMaster()->getName(), "VTBUF_H");
  EXPECT_EQ(block_->findInst("side2")->getMaster()->getName(), "VTBUF_H");
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest,
       CompoundPowerRecoveryRollsBackChildCommitsAndErrors)
{
  makePowerRecoveryDesign();
  ecc::PowerRepairOptions options;
  options.common.vt_suffixes = {"_H", "_L"};
  options.max_passes = 1;
  options.max_trials = 8;
  // The initial 18 nW saving passes ranking, but the final 9 nW does not.
  options.min_absolute_improvement = 12e-9;
  const auto run = [&]() {
    return ecc::PowerRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  };
  const auto restored = [&]() {
    EXPECT_EQ(block_->findInst("driver")->getMaster()->getName(), "VTBUF_H");
    EXPECT_EQ(block_->findInst("side")->getMaster()->getName(), "VTBUF_L");
    EXPECT_EQ(block_->findInst("side2")->getMaster()->getName(), "VTBUF_L");
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
    db_network_->checkAxioms();
  };
  auto result = run();
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.recovery_attempts, 0u);
  EXPECT_EQ(result.accepted_batches, 0u);
  EXPECT_EQ(result.before.leakage, result.after.leakage);
  EXPECT_EQ(result.before.design.setup_wns, result.after.design.setup_wns);
  restored();
  options.min_absolute_improvement = 0;
  runtime_.prepare = [&]() {
    if (block_->findInst("driver")->getMaster()->getName() == "VTBUF_L")
      throw std::runtime_error("recovery preparation failed");
  };
  result = run();
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("recovery preparation failed"),
            std::string::npos);
  restored();
  runtime_.prepare = {};
  block_->findInst("driver")->setDoNotTouch(true);
  result = run();
  EXPECT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.accepted_batches, 0u);
  restored();
}

TEST_F(BufferInserterStaTest, PowerRecoveryDownsizesSlackRichCells)
{
  readTestDesign("gate_repair_sta.v");
  oversizeLoads();
  addLegalRows();
  refreshRepairTiming();
  ecc::PowerRepairOptions options;
  options.max_trials = 2;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto result = ecc::PowerRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(result.downsized_cells, 0);
  EXPECT_EQ(result.vt_swaps, 0);
  EXPECT_LT(result.after.leakage, result.before.leakage);
  EXPECT_LT(result.after.design.area, result.before.design.area);
}

}  // namespace

void placeTestPorts(odb::dbBlock* block) {
  int y=1000;
  for(auto* term:block->getBTerms()) {
    auto* pin=odb::dbBPin::create(term);
    odb::dbBox::create(pin,block->getTech()->findLayer("metal2"),20000,y,20100,y+100);
    pin->setPlacementStatus(odb::dbPlacementStatus::PLACED);y+=1000;
  }
}

TEST_F(BufferInserterStaTest, VgSetupScopeCollectsCleanNegativeSlackNets)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();placeTestPorts(block_);constrainSetup(true);refreshRepairTiming();
  ecc::VgOptions o;o.max_buffers=64;o.common.max_fanout=64;
  auto run = [&]() {
    return ecc::BufferRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, &resizer_, o, runtime_);
  };
  auto drv=run();ASSERT_TRUE(drv.ok)<<drv.message;
  ASSERT_LT(drv.before.setup_wns,0);
  ASSERT_EQ(drv.before.severity,0);
  ASSERT_EQ(drv.nets_collected,0u);
  o.scope=ecc::VgScope::Setup;
  auto setup=run();ASSERT_TRUE(setup.ok)<<setup.message;
  EXPECT_GT(setup.nets_collected,0u);
  EXPECT_GE(setup.after.setup_wns+o.common.tolerances.wns_absolute,setup.before.setup_wns);
  EXPECT_GE(setup.after.setup_tns+o.common.tolerances.tns_absolute,setup.before.setup_tns);
  EXPECT_EQ(setup.after.total_slew_violations,0u);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));db_network_->checkAxioms();
  auto missing = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, o, runtime_);
  EXPECT_FALSE(missing.ok);
  EXPECT_NE(missing.message.find("Steiner"),std::string::npos);
}

TEST_F(BufferInserterStaTest, IncrementalPlacementRetainsLegalProtectedCells)
{
  readTestDesign("gate_repair_sta.v");addLegalRows();placeTestPorts(block_);
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

TEST_F(BufferInserterStaTest, CellReplacementChecksProtectionAndCompatibility)
{
  readTestDesign();
  auto* inst = block_->findInst("driver");
  auto* original = inst->getMaster();
  auto* larger = lib_->findMaster("BUF_X4");
  auto* incompatible = lib_->findMaster("AND2_X1");
  ASSERT_TRUE(ecc::compatibleCellMasters(original, larger));
  EXPECT_FALSE(ecc::compatibleCellMasters(original, incompatible));
  EXPECT_FALSE(ecc::replaceCell({inst, incompatible}));
  inst->setDoNotTouch(true);
  EXPECT_FALSE(ecc::replaceCell({inst, larger}));
  EXPECT_TRUE(ecc::replaceCell({inst, original}));
  inst->setDoNotTouch(false);
  inst->setPlacementStatus(odb::dbPlacementStatus::LOCKED);
  EXPECT_FALSE(ecc::replaceCell({inst, larger}));
  inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
  {
    ecc::EcoTransaction transaction(block_);
    EXPECT_TRUE(ecc::replaceCell({inst, larger}));
    EXPECT_EQ(inst->getMaster(), larger);
  }
  EXPECT_EQ(inst->getMaster(), original);
}

TEST_F(BufferInserterStaTest,
       AbacusLegalizationPreservesProtectedCellsAndEcoUndo)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  auto* fixed = block_->findInst("load0");
  auto* moved = block_->findInst("load1");
  fixed->setDoNotTouch(true);
  int fx, fy, mx, my;
  fixed->getLocation(fx, fy);
  moved->getLocation(mx, my);
  const auto status = fixed->getPlacementStatus();
  {
    ecc::EcoTransaction trial(block_);
    moved->setLocation(fx, fy);
    ecc::legalizeRepairPlacement(
        block_, &dp_, {}, false, ecc::PlacementLegalizer::Abacus);
    EXPECT_NO_THROW(dp_.checkPlacement(false));
    int x, y;
    fixed->getLocation(x, y);
    EXPECT_EQ(x, fx);
    EXPECT_EQ(y, fy);
    EXPECT_EQ(fixed->getPlacementStatus(), status);
  }
  int x, y;
  moved->getLocation(x, y);
  EXPECT_EQ(x, mx);
  EXPECT_EQ(y, my);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
}

TEST_F(BufferInserterStaTest, SetupInsertionLimitAndActionSelectionAreIndependent)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  const auto instances = block_->getInsts().size();
  ecc::SetupRepairOptions options;
  options.common.actions = {false, true, false};
  options.max_path_ends = 1;
  options.max_passes = 1;
  options.batch_size = 1;
  options.max_area_growth = 0.5;
  runtime_.prepare = [&]() { ecc::legalizeRepairPlacement(block_, &dp_); };
  const auto run = [&]() {
    return ecc::SetupRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  };
  options.max_inserted_buffers = 0;
  const auto zero = run();
  ASSERT_TRUE(zero.ok) << zero.message;
  EXPECT_EQ(zero.inserted_buffers, 0u);
  EXPECT_EQ(zero.attempted_batches, 0u);
  options.max_inserted_buffers.reset();
  options.common.actions.buffer = false;
  const auto disabled = run();
  ASSERT_TRUE(disabled.ok) << disabled.message;
  EXPECT_EQ(disabled.attempted_batches, 0u);
  EXPECT_EQ(block_->getInsts().size(), instances);
  options.common.actions.buffer = true;
  const auto unlimited = run();
  ASSERT_TRUE(unlimited.ok) << unlimited.message;
  EXPECT_GT(unlimited.inserted_buffers, 0u);
  EXPECT_GT(unlimited.after.setup_wns, unlimited.before.setup_wns);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
}

TEST_F(BufferInserterStaTest, DrvAndOrdHonorZeroAndDisabledInsertion)
{
  readTestDesign("fanout_repair_sta.v");
  refreshRepairTiming();
  const auto instances = block_->getInsts().size();
  ecc::DrvRepairOptions options;
  options.common.max_fanout = 3;
  options.common.actions = {false, true, false};
  for(bool zero_limit : {true, false}) {
    options.max_inserted_buffers = zero_limit ? std::optional<unsigned>{0} : std::nullopt;
    options.common.actions.buffer = zero_limit;
    const auto drv = ecc::DrvRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
    ASSERT_TRUE(drv.ok) << drv.message;
    EXPECT_EQ(drv.inserted_buffers, 0u);
    EXPECT_FALSE(drv.converged);
    const auto ord = ecc::OrdDrvRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
    ASSERT_TRUE(ord.ok) << ord.message;
    EXPECT_EQ(ord.inserted_buffers, 0u);
    EXPECT_EQ(block_->getInsts().size(), instances);
  }
  options.common.actions.buffer = true;
  const auto unlimited = ecc::DrvRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, runtime_);
  ASSERT_TRUE(unlimited.ok) << unlimited.message;
  EXPECT_GT(unlimited.inserted_buffers, 0u);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
}

TEST_F(BufferInserterStaTest, BufferSolversHonorZeroAndDisabledInsertion)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  const auto instances = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  for(bool zero_limit : {true, false}) {
    const auto limit = zero_limit ? std::optional<unsigned>{0} : std::nullopt;
    ecc::PathSearchOptions path;
    path.max_inserted_buffers = limit;
    path.common.actions.buffer = zero_limit;
    const auto p = ecc::BufferRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, path, runtime_);
    ASSERT_TRUE(p.ok) << p.message;
    EXPECT_EQ(p.attempted_trials, 0u);
    EXPECT_EQ(p.inserted_buffers, 0u);
    ecc::VgOptions vg;
    vg.max_buffers = limit;
    vg.common.actions.buffer = zero_limit;
    const auto v = ecc::BufferRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, &resizer_, vg, runtime_);
    ASSERT_TRUE(v.ok) << v.message;
    EXPECT_FALSE((v.accepted_trials != 0));
    ecc::GobiOptions gobi;
    gobi.max_buffers = limit;
    gobi.common.actions.buffer = zero_limit;
    const auto g = ecc::BufferRepairer::run(
        block_, sta_.get(), [&]() { refreshRepairTiming(); }, &resizer_, gobi,
        runtime_);
    ASSERT_TRUE(g.ok) << g.message;
    EXPECT_FALSE((g.accepted_trials != 0));
    EXPECT_EQ(block_->getInsts().size(), instances);
    EXPECT_EQ(block_->getNets().size(), nets);
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  }
  // Net collection limits have the same zero-is-zero semantics as insertion.
  ecc::VgOptions vg;
  vg.max_nets = 0;
  vg.scope = ecc::VgScope::Setup;
  const auto v = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, &resizer_, vg, runtime_);
  ASSERT_TRUE(v.ok) << v.message;
  EXPECT_EQ(v.nets_collected, 0u);
  EXPECT_EQ(v.attempted_buffers, 0u);
  EXPECT_FALSE(v.converged); // Negative setup slack remains.
  ecc::GobiOptions gobi;
  gobi.max_nets = 0;
  const auto g = ecc::BufferRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, &resizer_, gobi, runtime_);
  ASSERT_TRUE(g.ok) << g.message;
  EXPECT_EQ(g.nets_collected, 0u);
  EXPECT_EQ(g.attempted_buffers, 0u);
  EXPECT_EQ(block_->getInsts().size(), instances);
  EXPECT_EQ(block_->getNets().size(), nets);

}

TEST_F(BufferInserterStaTest, SharedBufferForestAppliesMixedMastersAndRollsBack)
{
  readTestDesign("gate_repair_sta.v");
  refreshRepairTiming();
  auto* net = block_->findInst("load0")->findITerm("A")->getNet();
  const auto load = [&](unsigned i) {
    return block_->findInst(("load" + std::to_string(i)).c_str())->findITerm("A");
  };
  ecc::BufferTreePlan plan;
  plan.net = net;
  plan.nodes = {
      {lib_->findMaster("BUF_X1"), {2000, 1000}, {load(0), load(1)}, {}},
      {lib_->findMaster("BUF_X2"), {1000, 1000}, {load(2)}, {0}},
      {lib_->findMaster("BUF_X4"), {3000, 1000}, {load(3)}, {}}};
  const auto insts = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  {
    ecc::EcoTransaction transaction(block_);
    const auto edited = ecc::applyBufferTree(plan);
    ASSERT_EQ(edited.insertions.size(), 3u);
    EXPECT_EQ(edited.roots, (std::vector<std::size_t>{1, 2}));
    const auto& child = edited.insertions[0];
    const auto& parent = edited.insertions[1];
    EXPECT_EQ(child.buffer->getMaster(), lib_->findMaster("BUF_X1"));
    EXPECT_EQ(parent.buffer->getMaster(), lib_->findMaster("BUF_X2"));
    EXPECT_EQ(child.input_pin->getNet(), parent.output_net);
    EXPECT_EQ(load(0)->getNet(), child.output_net);
    EXPECT_EQ(load(2)->getNet(), parent.output_net);
    EXPECT_EQ(parent.input_pin->getNet(), net);
    EXPECT_EQ(edited.insertions[2].input_pin->getNet(), net);
    EXPECT_EQ(load(4)->getNet(), net);
    EXPECT_EQ(block_->getInsts().size(), insts + 3);
    refreshRepairTiming();
    db_network_->checkAxioms();
  }
  refreshRepairTiming();
  EXPECT_EQ(block_->getInsts().size(), insts);
  EXPECT_EQ(block_->getNets().size(), nets);
  for(unsigned i = 0; i < 10; ++i) EXPECT_EQ(load(i)->getNet(), net);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, SharedBufferForestRejectsInvalidPlansBeforeAnyEdit)
{
  readTestDesign("gate_repair_sta.v");
  auto* a = block_->findInst("load0")->findITerm("A");
  auto* b = block_->findInst("load1")->findITerm("A");
  auto* net = a->getNet();
  auto* master = lib_->findMaster("BUF_X1");
  ecc::BufferTreePlan valid{net, {{master, {1000, 1000}, {a}, {}},
                                {master, {1000, 1000}, {b}, {0}}}};
  std::vector<ecc::BufferTreePlan> invalid;
  auto forward = valid;
  forward.nodes[0].children = {1};
  invalid.push_back(forward);
  auto repeated = valid;
  repeated.nodes[1].children = {0, 0};
  invalid.push_back(repeated);
  auto shared = valid;
  shared.nodes.push_back({master, {}, {}, {0}});
  invalid.push_back(shared);
  auto duplicate = valid;
  duplicate.nodes[1].loads.push_back(a);
  invalid.push_back(duplicate);
  auto bad_master = valid;
  bad_master.nodes[1].master = nullptr;
  invalid.push_back(bad_master);
  auto stale = valid;
  stale.nodes[1].loads = {block_->findInst("driver")->findITerm("A1")};
  invalid.push_back(stale);
  auto empty = valid;
  empty.nodes[1].loads.clear();
  empty.nodes[1].children.clear();
  invalid.push_back(empty);
  const auto insts = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  for(const auto& plan : invalid) {
    ecc::EcoTransaction transaction(block_);
    EXPECT_THROW(ecc::applyBufferTree(plan), std::invalid_argument);
    EXPECT_EQ(block_->getInsts().size(), insts);
    EXPECT_EQ(block_->getNets().size(), nets);
    EXPECT_EQ(a->getNet(), net);
    EXPECT_EQ(b->getNet(), net);
  }
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
}

TEST_F(BufferInserterStaTest, VgSolveIsReadOnlyAndIndependentOfApplicationLimits)
{
  readTestDesign("fanout_repair_sta.v");
  ecc::VgOptions options;
  options.common.max_fanout = 3;
  options.max_buffers = 0;
  options.max_nets = 1;
  options.common.actions.buffer = false;
  refreshRepairTiming();
  ecc::detail::DesignAnalysis analysis(block_, sta_.get(), options.common,
                                       runtime_.constraints);
  ecc::DesignMetrics before;
  const auto states = analysis.analyze(before, true);
  const auto insts = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  for(auto* resizer : {static_cast<rsz::Resizer*>(nullptr), &resizer_}) {
    const auto solved = ecc::VanGinneken::solve(block_, sta_.get(), states, resizer,
                                                options, runtime_.constraints);
    ASSERT_FALSE(solved.plans.empty());
    EXPECT_EQ(block_->getInsts().size(), insts);
    EXPECT_EQ(block_->getNets().size(), nets);
    EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
    EXPECT_TRUE(sta_->search()->arrivalsValid());
    {
      ecc::EcoTransaction transaction(block_);
      const auto edits = ecc::applyBufferTree(solved.plans.front());
      EXPECT_EQ(edits.insertions.size(), solved.plans.front().nodes.size());
      EXPECT_GT(edits.insertions.size(), 0u);
    }
    refreshRepairTiming();
    EXPECT_EQ(block_->getInsts().size(), insts);
    EXPECT_EQ(block_->getNets().size(), nets);
  }
  EXPECT_THROW(ecc::VanGinneken::solve(nullptr, sta_.get(), states, &resizer_, options,
                                       runtime_.constraints),
               std::invalid_argument);
  ASSERT_FALSE(states.empty());
  const std::vector<ecc::detail::NetState> duplicate{states.front(), states.front()};
  EXPECT_THROW(ecc::VanGinneken::solve(block_, sta_.get(), duplicate, &resizer_,
                                       options, runtime_.constraints),
               std::invalid_argument);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, PathCandidateGenerationDoesNotApplyOrRefresh)
{
  readTestDesign("gate_repair_sta.v");
  addLegalRows();
  constrainSetup(true);
  refreshRepairTiming();
  const auto insts = block_->getInsts().size();
  const auto nets = block_->getNets().size();
  ecc::PathSearchOptions options;
  options.max_inserted_buffers = 0;
  options.max_trials = 1;
  const auto candidates = ecc::PathSearch::generateCandidates(
      block_, sta_.get(), options, runtime_.constraints);
  ASSERT_FALSE(candidates.empty());
  EXPECT_EQ(block_->getInsts().size(), insts);
  EXPECT_EQ(block_->getNets().size(), nets);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  {
    ecc::EcoTransaction transaction(block_);
    const auto edits = ecc::applyBufferTree(candidates.front().plan);
    EXPECT_GT(edits.insertions.size(), 0u);
  }
  refreshRepairTiming();
  EXPECT_EQ(block_->getInsts().size(), insts);
  EXPECT_EQ(block_->getNets().size(), nets);
  db_network_->checkAxioms();
}

TEST_F(BufferInserterStaTest, GobiSolveValidatesInputsBeforeCudaAndDoesNotOpenEco)
{
  readTestDesign("fanout_repair_sta.v");
  refreshRepairTiming();
  const auto instances = block_->getInsts().size();
  EXPECT_TRUE(
      ecc::Gobi::solve(block_, sta_.get(), {}, &resizer_, {}, runtime_.constraints)
          .plans.empty());
  EXPECT_THROW(
      ecc::Gobi::solve(block_, sta_.get(), {}, nullptr, {}, runtime_.constraints),
      std::invalid_argument);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  EXPECT_EQ(block_->getInsts().size(), instances);
}

TEST_F(BufferInserterStaTest, RuntimeProtectionDoesNotLeakIntoReusedConfig) {
  readTestDesign("fanout_repair_sta.v");
  refreshRepairTiming();
  ecc::RepairConfig config;
  config.set("common.max_fanout", "3");
  config.set("common.actions", "buffer");
  const auto options = config.drvOptions();
  const auto insts = block_->getInsts().size();
  ecc::RepairRuntime protected_runtime;
  for (auto *inst : block_->getInsts())
    protected_runtime.constraints.excluded_instances.push_back(inst);
  unsigned prepared = 0;
  protected_runtime.prepare = [&]() { ++prepared; };
  const auto blocked = ecc::DrvRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, options, protected_runtime);
  ASSERT_TRUE(blocked.ok) << blocked.message;
  EXPECT_EQ(blocked.inserted_buffers, 0u);
  EXPECT_EQ(prepared, 0u);
  EXPECT_EQ(block_->getInsts().size(), insts);
  const auto allowed = ecc::DrvRepairer::run(
      block_, sta_.get(), [&]() { refreshRepairTiming(); }, config.drvOptions());
  ASSERT_TRUE(allowed.ok) << allowed.message;
  EXPECT_GT(allowed.inserted_buffers, 0u);
  EXPECT_EQ(prepared, 0u);
  EXPECT_TRUE(odb::dbDatabase::ecoStackEmpty(block_));
  db_network_->checkAxioms();
}
