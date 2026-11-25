#include "replay_capture.h"
#include "odb/db.h"
#include <thread>

int main()
{
  auto* db = odb::dbDatabase::create();
  auto* tech = odb::dbTech::create(db, "tech");
  auto* layer = odb::dbTechLayer::create(tech, "M1", odb::dbTechLayerType::ROUTING);
  auto* lib = odb::dbLib::create(db, "lib", tech, '/');
  auto* master = odb::dbMaster::create(lib, "BUF");
  master->setWidth(100);
  master->setHeight(100);
  auto* input = odb::dbMTerm::create(master, "A", odb::dbIoType::INPUT);
  auto* pin = odb::dbMPin::create(input);
  odb::dbBox::create(pin, layer, 10, 10, 20, 20);
  master->setFrozen();
  auto* chip = odb::dbChip::create(db, tech);
  auto* block = odb::dbBlock::create(chip, "test");
  odb::dbInst::create(block, master, "untouched");
  auto* moved = odb::dbInst::create(block, master, "moved");
  auto* removed = odb::dbInst::create(block, master, "removed");
  auto* reverted = odb::dbInst::create(block, master, "reverted");
  auto* renamed = odb::dbInst::create(block, master, "old_name");
  auto* original_net = odb::dbNet::create(block, "original");
  auto* other_net = odb::dbNet::create(block, "other");
  reverted->findITerm("A")->connect(original_net);
  {
    ecc::replay::Session session(block);
    odb::dbDatabase::beginEco(block);
    reverted->setLocation(500, 600);
    reverted->findITerm("A")->connect(other_net);
    odb::dbInst::create(block, master, "temporary");
    odb::dbDatabase::undoEco(block);
    session.capture("rolled_back");

    moved->setLocation(300, 400);
    moved->findITerm("A")->connect(other_net);
    odb::dbInst::destroy(removed);
    renamed->rename("new_name");
    session.capture("changed");
    moved->setLocation(0, 0);
    session.capture("next_stage");
    session.capture("unchanged");
  }
  const auto worker = [](unsigned id) {
    ecc::replay::Decisions decisions("test", id);
    for (unsigned i = 0; i < 129; ++i)
      decisions.record(
          "u\"\\\n" + std::to_string(i), "small", "large", i % 2 == 0);
  };
  std::thread first(worker, 0), second(worker, 1);
  first.join();
  second.join();
  odb::dbDatabase::destroy(db);
}
