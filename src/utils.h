///////////////////////////////////////////////////////////////////////////////
// Authors: Hyein Lee and Jiajia Li
//          (Ph.D. advisor: Andrew B. Kahng)
//
//          Many subsequent improvements were made by Minsoo Kim
//          leading up to the initial release.
//
// BSD 3-Clause License
//
// Copyright (c) 2018, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

#ifndef __UTILS_H__
#define __UTILS_H__
#include <algorithm>
#include <string>
#include <iostream>
#include <sys/param.h>
#include <sys/time.h>
#include <sys/utsname.h>
#include <vector>
// #include "sizing_model.h"
#include <optional>
#include <sstream>
#include <utility>
#include <type_traits>
#include "spdlog/spdlog.h"
#include "fmt/printf.h"

namespace ecc {
// Stream-style diagnostics in the retained sizing search feed the same
// spdlog logger as the application. Disabled messages do not build a string.
class LogLine {
public:
    explicit LogLine(spdlog::level::level_enum level) : level_(level) {
        if(spdlog::should_log(level)) stream_.emplace();
    }
    ~LogLine() noexcept {
        if(!stream_) return;
        try {
            auto text = stream_->str();
            while(!text.empty() && text.back() == '\n') text.pop_back();
            if(!text.empty()) spdlog::log(level_, "{}", text);
        } catch(...) { /* Logging must not interrupt ECO rollback. */ }
    }
    template<class T> requires std::is_arithmetic_v<T>
    LogLine& operator<<(T value) {
        if(stream_) *stream_ << value;
        return *this;
    }
    template<class T> requires (!std::is_arithmetic_v<std::remove_cvref_t<T>>)
    LogLine& operator<<(T&& value) {
        if(stream_) *stream_ << std::forward<T>(value);
        return *this;
    }
    LogLine& operator<<(std::ostream& (*manipulator)(std::ostream&)) {
        if(stream_) manipulator(*stream_);
        return *this;
    }
private:
    spdlog::level::level_enum level_;
    std::optional<std::ostringstream> stream_;
};
inline LogLine logInfo() { return LogLine(spdlog::level::info); }
inline LogLine logDebug() { return LogLine(spdlog::level::debug); }
inline LogLine logError() { return LogLine(spdlog::level::err); }
template<class... Args>
void logPrintf(const char* format, Args&&... args) {
    if(spdlog::should_log(spdlog::level::info))
        logInfo() << fmt::sprintf(format, std::forward<Args>(args)...);
}
}  // namespace ecc

struct CELL;
typedef unsigned cell_sizes;
enum cell_vtypes { s = 0, m, f, default_vtype };
// #include "cell_sizing.h"
// #include "sizing_model.h"
#define DBL_MAX 1.7976931348623158e+308 /* max value */

inline cell_sizes getSizes(std::string size) {
    if(size == "01") {
        return 1;
    }
    if(size == "02") {
        return 2;
    }
    if(size == "03") {
        return 3;
    }
    if(size == "04") {
        return 4;
    }
    if(size == "06") {
        return 6;
    }
    if(size == "08") {
        return 8;
    }
    if(size == "10") {
        return 10;
    }
    if(size == "20") {
        return 20;
    }
    if(size == "40") {
        return 40;
    }
    if(size == "80") {
        return 80;
    }
    return 0;
}

inline cell_vtypes getVtypes(char type) {
    if(type == 's') {
        return s;
    }
    else if(type == 'f') {
        return f;
    }
    else if(type == 'm') {
        return m;
    }
    else {
        ecc::logInfo() << "ERROR\n";
        return default_vtype;
    }
}

inline int getCellSize(cell_sizes size) {
    return size;
}

inline char getCellVtype(cell_vtypes vtype) {
    switch(vtype) {
        case s:
            return 's';
        case f:
            return 'f';
        case m:
            return 'm';
        default:
            return '\0';
    }
}

bool isff(const CELL &cell);
cell_vtypes r_type(const CELL &cell);
int r_size(const CELL &cell);

bool isEqual(double a, double b);
double cpuTime(void);
void printMemoryUsage(void);
void LaunchPTBackground(std::string root, std::string benchname);
void KillPTBackground();

struct LCA {
    int vertex_num, query_num, root;  // Number of vertices, queries, and root node
    std::vector< std::vector< int > > graph;  // Vertex adjacency list
    typedef std::pair< int, int > query;
    std::vector< std::vector< query > > queries;  // Query groups
    std::vector< int > fa;
    std::vector< bool > vis;
    std::vector< int > ans;
    void init(int n, int m, int r) {
        vertex_num = n;
        root = r;
        graph.resize(n + 1);
        queries.resize(n + 1);
        ans.resize(m);
        vis.resize(n + 1);
        fa.resize(n + 1);
        for(int i = 0; i <= n; i++) {
            fa[i] = i;
        }
        std::fill(vis.begin(), vis.end(), false);
        
    }
    int find_root(int i) {
        if(i == fa[i])
            return i;
        return fa[i] = find_root(fa[i]);
    }
    // use root
    void tarjan(int cur) {
        vis[cur] = true;

        for(auto &it : graph[cur]) {
            if(!vis[it]) {
                tarjan(it);
                fa[it] = cur;
            }
        }

        for(auto &it : queries[cur]) {
            if(vis[it.first]) {
                ans[it.second] = find_root(fa[it.first]);
            }
        }
    }
};

#endif
