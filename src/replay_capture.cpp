#include "replay_capture.h"
#include "odb/db.h"
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>

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

void capture(odb::dbBlock* block, const char* phase) noexcept {
    if(!enabled())
        return;
    write(phase, "native_committed", "instances",
          [block](std::ostream& stream) {
              stream << ",\"objects\":{";
              bool first = true;
              for(auto* inst : block->getInsts()) {
                  if(!first)
                      stream << ',';
                  first = false;
                  instance(stream, inst);
              }
              stream << '}';
          });
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

}  // namespace ecc::replay

#include "rsz/Resizer.hh"
namespace ecc::replay {
Session::Session(rsz::Resizer* resizer, odb::dbBlock* block) {
    if(enabled()) {
        resizer_ = resizer;
        resizer_->setCommittedStateObserver(
            [block](const char* phase) noexcept { capture(block, phase); });
    }
}
Session::~Session() {
    if(resizer_)
        resizer_->setCommittedStateObserver({});
}
}  // namespace ecc::replay
