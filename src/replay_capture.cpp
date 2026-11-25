#include "replay_capture.h"
#include "odb/db.h"
#include "odb/dbBlockCallBackObj.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <map>
#include <optional>
#include <sstream>

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
std::string instance(odb::dbInst* inst, const std::string& name) {
    std::ostringstream stream;
    const auto bbox = inst->getBBox()->getBox();
    stream << "{\"kind\":";
    quote(stream, inst->getMaster()->isBlock() ? "macro" : "instance");
    stream << ",\"name\":";
    quote(stream, name);
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
    return stream.str();
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
        decltype(before_) before;
        before.swap(before_);
        try {
            std::vector<std::pair<std::string, std::string>> objects;
            std::vector<std::string> removed;
            for(const auto& [name, original] : before) {
                if(auto* inst = block_->findInst(name.c_str())) {
                    auto current = instance(inst, name);
                    if(!original || current != *original)
                        objects.emplace_back(name, std::move(current));
                } else if(original) {
                    removed.push_back(name);
                }
            }
            if(objects.empty() && removed.empty())
                return;
            write(phase, "native_committed", "patch", [&](std::ostream& stream) {
                stream << ",\"objects\":{";
                bool first = true;
                for(const auto& [name, value] : objects) {
                    if(!first)
                        stream << ',';
                    first = false;
                    quote(stream, "inst:" + name);
                    stream << ':' << value;
                }
                stream << "},\"remove\":[";
                first = true;
                for(const auto& name : removed) {
                    if(!first)
                        stream << ',';
                    first = false;
                    quote(stream, "inst:" + name);
                }
                stream << ']';
            });
        } catch(const std::exception& error) {
            std::cerr << "Replay stage capture failed: " << error.what() << '\n';
        }
    }

   private:
    // Record the first state only. A missing instance can be created and then
    // removed again by the same repair call without leaving a visible change.
    void remember(const char* name, odb::dbInst* inst) noexcept {
        try {
            if(before_.find(name) == before_.end())
                before_.emplace(name, inst ? std::optional{instance(inst, name)}
                                           : std::nullopt);
        } catch(const std::exception& error) {
            std::cerr << "Replay change tracking failed: " << error.what() << '\n';
        }
    }

    void remember(odb::dbInst* inst) noexcept {
        remember(inst->getConstName(), inst);
    }

    void inDbInstCreate(odb::dbInst* inst) override { remember(inst->getConstName(), nullptr); }
    void inDbInstDestroy(odb::dbInst* inst) override { remember(inst); }
    void inDbInstPlacementStatusBefore(odb::dbInst* inst, const odb::dbPlacementStatus&) override { remember(inst); }
    void inDbInstSwapMasterBefore(odb::dbInst* inst, odb::dbMaster*) override { remember(inst); }
    void inDbPreMoveInst(odb::dbInst* inst) override { remember(inst); }
    void inDbPostInstRename(odb::dbInst* inst, const char* old_name) override {
        remember(old_name, inst);
        remember(inst->getConstName(), nullptr);
    }
    void inDbITermPreConnect(odb::dbITerm* term, odb::dbNet*) override { remember(term->getInst()); }
    void inDbITermPreDisconnect(odb::dbITerm* term) override { remember(term->getInst()); }
    void inDbITermDestroy(odb::dbITerm* term) override { remember(term->getInst()); }

    odb::dbBlock* block_;
    std::map<std::string, std::optional<std::string>, std::less<>> before_;
};

Session::Session(odb::dbBlock* block) noexcept {
    if(!enabled() || !block)
        return;
    try {
        changes_ = std::make_unique<Changes>(block);
    } catch(const std::exception& error) {
        std::cerr << "Replay observer unavailable: " << error.what() << '\n';
    }
}
Session::~Session() = default;

void Session::capture(const char* phase) noexcept {
    if(changes_)
        changes_->capture(phase);
}
}  // namespace ecc::replay
