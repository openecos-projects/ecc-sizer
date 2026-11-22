#include "replay_capture.h"
#include "odb/db.h"
#include <thread>

int main()
{
  auto* db = odb::dbDatabase::create();
  auto* tech = odb::dbTech::create(db, "tech");
  auto* lib = odb::dbLib::create(db, "lib", tech, '/');
  auto* master = odb::dbMaster::create(lib, "BUF");
  master->setWidth(100);
  master->setHeight(100);
  master->setFrozen();
  auto* chip = odb::dbChip::create(db, tech);
  auto* block = odb::dbBlock::create(chip, "test");
  odb::dbInst::create(block, master, "untouched");
  auto* moved = odb::dbInst::create(block, master, "moved");
  auto* removed = odb::dbInst::create(block, master, "removed");
  {
    ecc::replay::Session session(nullptr, block);
    moved->setLocation(300, 400);
    odb::dbInst::destroy(removed);
    session.capture("changed");
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
