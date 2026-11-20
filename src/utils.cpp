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

#include "utils.h"
#include <fstream>
#include "cell_sizing.h"
#include "sizing_model.h"
using namespace std;

bool isff(const CELL &cell) {
    return cell.isFF;
}
cell_vtypes r_type(const CELL &cell) {
    return cell.c_vtype;
}
int r_size(const CELL &cell) {
    /*
    int deci = getCellSize(cell.c_size);
    if (deci == 3) return 3;
    else return (deci/10)*16+deci%10;
    */

    // TODO get size information from lib cell name
    return cell.c_size;
}

bool isEqual(double a, double b) {
    if(std::isinf(a) && std::isinf(b))
        return true;
    if(std::isinf(a) && b == DBL_MAX)
        return true;
    if(a == DBL_MAX && std::isinf(b))
        return true;
    double SMALL_NUM = 1e-6;
    return (abs(a - b) < SMALL_NUM);
}

double cpuTime(void) {
    struct timeval cputime;
    gettimeofday(&cputime, NULL);
    return static_cast< double >(cputime.tv_sec) +
           (1.e-6) * static_cast< double >(cputime.tv_usec);
}

void printMemoryUsage(void) {
    ifstream status("/proc/self/status");
    string data;

    if(status.good()) {
        for(int i = 0; i < 2; ++i)
            getline(status, data);

        // account for login and loginlinux versions
        getline(status, data);
        if(data.find("SleeAVG") == string::npos)
            for(int i = 0; i < 7; ++i)
                getline(status, data);
        else
            for(int i = 0; i < 6; ++i)
                getline(status, data);

        // vmPeak
        getline(status, data);
        ecc::logInfo() << "### VmPeak\t\t: " << data << endl;
    }
    status.close();

    status.open("/proc/self/stat");
    if(status.good()) {
        double vmsize;
        for(unsigned i = 0; i < 22; ++i) {
            status >> data;
        }
        status >> vmsize;

        ecc::logInfo() << "### Memory Usage\t: " << vmsize / 1048576. << " MB" << endl;
    }
    status.close();
}
