#include "replay_capture.h"
#include "odb/db.h"
#include <thread>

int main()
{
  auto* db = odb::dbDatabase::create();
  auto* tech = odb::dbTech::create(db, "tech");
  auto* chip = odb::dbChip::create(db, tech);
  auto* block = odb::dbBlock::create(chip, "test");
  ecc::replay::capture(block, "input");
  const auto worker = [](unsigned id) {
    ecc::replay::Decisions decisions("test", id);
    for (unsigned i = 0; i < 129; ++i)
      decisions.record(
          "u\"\\\n" + std::to_string(i), "small", "large", i % 2 == 0);
  };
  std::thread first(worker, 0), second(worker, 1);
  first.join();
  second.join();
  ecc::replay::capture(block, "final");
  odb::dbDatabase::destroy(db);
}
