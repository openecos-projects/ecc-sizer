#pragma once
#include <string>
#include <vector>
namespace odb {
class dbBlock;
}
namespace ecc::replay {
bool enabled() noexcept;
void initialize() noexcept;
// Synchronous copy at a stable repair/commit boundary. Never throws into EDA.
void capture(odb::dbBlock* block, const char* phase) noexcept;
}  // namespace ecc::replay

namespace rsz {
class Resizer;
}
namespace ecc::replay {
class Session {
   public:
    explicit Session(rsz::Resizer* resizer, odb::dbBlock* block);
    ~Session();
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

   private:
    rsz::Resizer* resizer_ = nullptr;
};
}  // namespace ecc::replay

namespace ecc::replay {
// Bounded per-algorithm CELL decisions. These are not committed OpenDB
// geometry.
class Decisions {
   public:
    Decisions(const char* algorithm, unsigned worker = 0) noexcept;
    ~Decisions();
    Decisions(const Decisions&) = delete;
    Decisions& operator=(const Decisions&) = delete;
    bool active() const noexcept {
        return active_;
    }
    void record(const std::string& name, const std::string& before,
                const std::string& proposed, bool accepted) noexcept;

   private:
    struct Row {
        std::string name, before, proposed;
        bool accepted;
    };
    void flush() noexcept;
    bool active_;
    const char* algorithm_;
    unsigned worker_;
    std::vector< Row > rows_;
};
}  // namespace ecc::replay
