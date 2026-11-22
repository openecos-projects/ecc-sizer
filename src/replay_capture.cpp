#include "replay_capture.h"
#include "odb/db.h"
#include "odb/dbBlockCallBackObj.h"
#include "rsz/Resizer.hh"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <set>

namespace ecc::replay {
namespace {
void quote(std::ostream& stream, const std::string& text) {
    stream << '"';
    for(unsigned char c : text) {
        if(c == '"' || c == '\\')
            stream << '\\' << c;
        else if(c < 32)
            stream << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                   << unsigned(c) << std::dec;
        else
            stream << c;
    }
    stream << '"';
}
void instance(std::ostream& stream, odb::dbInst* inst) {
    const auto bbox = inst->getBBox()->getBox();
    quote(stream, "inst:" + inst->getName());
    stream << ":{\"kind\":";
    quote(stream, inst->getMaster()->isBlock() ? "macro" : "instance");
    stream << ",\"name\":";
    quote(stream, inst->getName());
    stream << ",\"master\":";
    quote(stream, inst->getMaster()->getName());
    stream << ",\"orient\":";
    quote(stream, inst->getOrient().getString());
    stream << ",\"placed\":" << (inst->isPlaced() ? "true" : "false");
    stream << ",\"fixed\":" << (inst->isFixed() ? "true" : "false");
    stream << ",\"bbox\":[" << bbox.xMin() << ',' << bbox.yMin() << ','
           << bbox.xMax() << ',' << bbox.yMax() << ']';
    stream << ",\"pins\":{";
    bool first = true;
    for(auto* term : inst->getITerms()) {
        int x = 0, y = 0;
        if(!term->getAvgXY(&x, &y))
            continue;
        if(!first)
            stream << ',';
        first = false;
        quote(stream, term->getMTerm()->getName());
        stream << ":{\"net\":";
        quote(stream, term->getNet() ? term->getNet()->getName() : "");
        stream << ",\"position\":[" << x << ',' << y << "]}";
    }
    stream << "}}";
}
}  // namespace

bool enabled() noexcept {
    const auto* directory = std::getenv("ECC_REPLAY_CAPTURE_DIR");
    return directory && *directory;
}

namespace {
struct WriterState {
    std::mutex mutex;
    std::chrono::steady_clock::time_point started =
        std::chrono::steady_clock::now();
    long long offset = [] {
        const auto* text = std::getenv("ECC_REPLAY_OFFSET_NS");
        return text ? std::stoll(text) : 0LL;
    }();
    unsigned long sequence = 0;
};
WriterState& writerState() {
    static WriterState value;
    return value;
}
template < typename Content >
void write(const char* phase, const char* domain, const char* scope,
           Content&& content) noexcept {
    if(!enabled())
        return;
    try {
        auto& state = writerState();
        std::lock_guard< std::mutex > lock(state.mutex);
        const auto started = state.started;
        const auto offset = state.offset;
        auto& sequence = state.sequence;
        const auto elapsed =
            offset + std::chrono::duration_cast< std::chrono::nanoseconds >(
                         std::chrono::steady_clock::now() - started)
                         .count();
        const auto directory =
            std::filesystem::path(std::getenv("ECC_REPLAY_CAPTURE_DIR"));
        const auto target =
            directory / ("sizer-" + std::to_string(offset) + "-" +
                         std::to_string(sequence) + ".json");
        auto temp = target;
        temp += ".tmp";
        std::ofstream stream(temp);
        stream.exceptions(std::ios::badbit | std::ios::failbit);
        stream << "{\"sequence\":" << sequence++
               << ",\"elapsed_ns\":" << elapsed;
        stream << ",\"phase\":";
        quote(stream, phase);
        stream << ",\"scope\":";
        quote(stream, scope);
        stream << ",\"subflow\":\"run sizer\",\"state_domain\":";
        quote(stream, domain);
        content(stream);
        stream << '}';
        stream.close();
        std::filesystem::rename(temp, target);
    }
    catch(const std::exception& error) {
        std::cerr << "Replay capture failed: " << error.what() << '\n';
        try {
            std::ofstream(
                std::filesystem::path(std::getenv("ECC_REPLAY_CAPTURE_DIR")) /
                "error.json")
                << "{\"error\":\"sizer capture failed\"}";
        }
        catch(...) {
        }
    }
}
}  // namespace

void initialize() noexcept {
    if(!enabled())
        return;
    try {
        (void)writerState();
    }
    catch(const std::exception& error) {
        std::cerr << "Replay clock unavailable: " << error.what() << '\n';
    }
}

Decisions::Decisions(const char* algorithm, unsigned worker) noexcept
    : active_(enabled()), algorithm_(algorithm), worker_(worker) {
}
Decisions::~Decisions() {
    flush();
}
void Decisions::record(const std::string& name, const std::string& before,
                       const std::string& proposed, bool accepted) noexcept {
    if(!active_ || before == proposed)
        return;
    try {
        rows_.push_back({name, before, proposed, accepted});
        if(rows_.size() >= 128)
            flush();
    }
    catch(const std::exception& error) {
        std::cerr << "Replay decision unavailable: " << error.what() << '\n';
    }
}
void Decisions::flush() noexcept {
    if(rows_.empty())
        return;
    write("cell_candidate_batch", "candidate", "patch",
          [this](std::ostream& stream) {
              const auto accepted =
                  std::count_if(rows_.begin(), rows_.end(),
                                [](const Row& row) { return row.accepted; });
              stream << ",\"objects\":{},\"metrics\":{\"algorithm\":";
              quote(stream, algorithm_);
              stream << ",\"worker\":" << worker_
                     << ",\"accepted\":" << accepted
                     << ",\"reverted\":" << rows_.size() - accepted
                     << ",\"decisions\":[";
              bool first = true;
              for(const auto& row : rows_) {
                  if(!first)
                      stream << ',';
                  first = false;
                  stream << "{\"instance\":";
                  quote(stream, row.name);
                  stream << ",\"before\":";
                  quote(stream, row.before);
                  stream << ",\"proposed\":";
                  quote(stream, row.proposed);
                  stream << ",\"accepted\":"
                         << (row.accepted ? "true" : "false") << '}';
              }
              stream << "]}";
          });
    rows_.clear();
}

class Session::Changes : public odb::dbBlockCallBackObj {
   public:
    explicit Changes(odb::dbBlock* block) : block_(block) { addOwner(block); }

    void capture(const char* phase) noexcept {
        if(dirty_.empty())
            return;
        auto dirty = std::move(dirty_);
        dirty_.clear();
        // Resolve names after the commit or undo, when the database is stable.
        write(phase, "native_committed", "patch", [&](std::ostream& stream) {
            stream << ",\"objects\":{";
            bool first = true;
            for(const auto& name : dirty) {
                if(auto* inst = block_->findInst(name.c_str())) {
                    if(!first)
                        stream << ',';
                    first = false;
                    instance(stream, inst);
                }
            }
            stream << "},\"remove\":[";
            first = true;
            for(const auto& name : dirty) {
                if(!block_->findInst(name.c_str())) {
                    if(!first)
                        stream << ',';
                    first = false;
                    quote(stream, "inst:" + name);
                }
            }
            stream << ']';
        });
    }

   private:
    template <typename GetName>
    void mark(GetName get_name) noexcept {
        try {
            dirty_.insert(get_name());
        } catch(const std::exception& error) {
            std::cerr << "Replay change tracking failed: " << error.what() << '\n';
        }
    }

    void mark(odb::dbInst* inst) noexcept {
        mark([&] { return inst->getName(); });
    }

    void inDbInstCreate(odb::dbInst* inst) override { mark(inst); }
    void inDbInstDestroy(odb::dbInst* inst) override { mark(inst); }
    void inDbInstPlacementStatusBefore(odb::dbInst* inst, const odb::dbPlacementStatus&) override { mark(inst); }
    void inDbInstSwapMasterAfter(odb::dbInst* inst) override { mark(inst); }
    void inDbPostMoveInst(odb::dbInst* inst) override { mark(inst); }
    void inDbPostInstRename(odb::dbInst* inst, const char* old_name) override {
        mark([&] { return old_name; });
        mark(inst);
    }
    void inDbITermPostConnect(odb::dbITerm* term) override { mark(term->getInst()); }
    void inDbITermPostDisconnect(odb::dbITerm* term, odb::dbNet*) override { mark(term->getInst()); }

    odb::dbBlock* block_;
    std::set<std::string> dirty_;
};

Session::Session(rsz::Resizer* resizer, odb::dbBlock* block) noexcept {
    if(!enabled() || !block)
        return;
    try {
        changes_ = std::make_unique<Changes>(block);
        if(resizer) {
            resizer->setCommittedStateObserver(
                [this](const char* phase) noexcept { capture(phase); });
            resizer_ = resizer;
        }
    } catch(const std::exception& error) {
        std::cerr << "Replay observer unavailable: " << error.what() << '\n';
    }
}
Session::~Session() {
    if(resizer_)
        resizer_->setCommittedStateObserver({});
}

void Session::capture(const char* phase) noexcept {
    if(changes_)
        changes_->capture(phase);
}
}  // namespace ecc::replay
