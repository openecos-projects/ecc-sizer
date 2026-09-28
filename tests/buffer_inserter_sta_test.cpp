// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "buffer_inserter.h"
#include "fanout_repair.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbReadVerilog.hh"
#include "est/EstimateParasitics.h"
#include "gtest/gtest.h"
#include "odb/db.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
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

}  // namespace
