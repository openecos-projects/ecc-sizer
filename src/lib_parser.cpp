#include "MinMax.hh"
#include "NetworkClass.hh"
#include "Transition.hh"
#include "sizer.h"
#include "ckt.h"
#include <tcl8.6/tcl.h>
#include <tcl8.6/tclDecls.h>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <unordered_map>
#include "sta/ConcreteNetwork.hh"
#include "sta/PortExtCap.hh"
#include "sta/InputDrive.hh"
// #include "dbSta/dbNetwork.hh"
#include "db_sta/dbNetwork.hh"
#include "sta/PatternMatch.hh"
#include "sta/PortDelay.hh"
#include "sta/Sdc.hh"

#include "ord/OpenRoad.hh"
#include "odb/db.h"
#include "ord/Timing.h"
#include "dpl/Opendp.h"
#include "grt/GlobalRouter.h"
#include "db_sta/dbSta.hh"
#include "utils.h"
#define NUM_VTS 3
#define NUM_SIZES 10
#define SDC_FILE_POSTFIX ".sizer"
#define SDC_CONVERT_TCL "./SDC/convert_sdc.tcl"
#include "sta/Sta.hh"
#include "ConcreteParasitics.hh"
#include "ConcreteParasiticsPvt.hh"
#include <filesystem>
using namespace std;
using namespace sta;

void skip(istream& is) {
    bool finishedReading = false;

    int check = 1;

    std::vector< string > tokens;
    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);

        if(check == 0) {
            finishedReading = true;
        }
    }
}

int check_brac(char c) {
    if(c == '{') {
        return 1;
    }
    if(c == '}') {
        return -1;
    }
    return 0;
}

int read_line_as_tokens_chk(istream& is, vector< string >& tokens) {
    tokens.clear();
    string line;
    std::getline(is, line);

    int chkBrac = 0;

    string token = "";

    for(unsigned i = 0; i < line.size(); ++i) {
        char currChar = line[i];
        bool isSpecialChar = is_special_char(currChar);
        chkBrac += check_brac(currChar);

        if(std::isspace(currChar) || isSpecialChar) {
            if(!token.empty()) {
                // Add the current token to the list of tokens
                tokens.push_back(token);
                token.clear();
            }
        }
        else {
            // Add the char to the current token
            token.push_back(currChar);
        }
    }

    if(!token.empty())
        tokens.push_back(token);

    return chkBrac;
}

bool read_line_as_tokens(istream& is, vector< string >& tokens,
                         bool includeSpecialChars) {
    tokens.clear();

    string line;
    std::getline(is, line);

    while(is && tokens.empty()) {
        string token = "";

        for(unsigned i = 0; i < line.size(); ++i) {
            char currChar = line[i];
            bool isSpecialChar = is_special_char(currChar);

            if(std::isspace(currChar) || isSpecialChar) {
                if(!token.empty()) {
                    // Add the current token to the list of tokens
                    tokens.push_back(token);
                    token.clear();
                }

                if(includeSpecialChars && isSpecialChar) {
                    tokens.push_back(string(1, currChar));
                }
            }
            else {
                // Add the char to the current token
                token.push_back(currChar);
            }
        }

        if(!token.empty())
            tokens.push_back(token);

        if(tokens.empty()) {
            // Previous line read was empty. Read the next one.
            std::getline(is, line);
        }
    }

    return !tokens.empty();
}

void Circuit::lib_parser(string filename, unsigned corner) {
    is.open(filename.c_str());
    if(!is.good()) {
        cout << "ERROR: failed to open library " << filename << endl;
        exit(0);
    }
    cout << "Reading .lib file " << filename << " for corner " << corner
         << endl;

    LibInfo lib;

    lib.name = read_lib_name(is);
    cout << "Library name = " << lib.name << endl;

    read_head_info(is, lib, corner);
    bool ignore_cell = lib.name.find("sram") != std::string::npos ||
                       lib.name.find("SEQ") != std::string::npos;
    double tmp_trans = lib.max_transition;
    tmp_trans = tmp_trans * lib.time_unit / 1e-9;
    // if(_sizer->maxTran[corner] == 0.0 || _sizer->maxTran[corner] < tmp_trans)
    // {
    //     _sizer->maxTran[corner] = tmp_trans;
    // }
    if(_sizer->maxTran[corner] == 0.0) {
        _sizer->maxTran[corner] = 0.32;
    }
    is.seekg(0, is.beg);

    vector< string > tokens;
    vector< LibCellInfo > cells;
    while(!is.eof()) {
        read_line_as_tokens(is, tokens);
        // Read table templates
        if(tokens.size() == 2 && (tokens[0] == "power_lut_template" ||
                                  tokens[0] == "lu_table_template")) {
            LibTableTempl templ;
            templ.name = tokens[1];
            // cout << "Reading table template " << templ.name << endl;
            _begin_read_templ_info(is, templ);
            lib.templs.insert(
                std::pair< string, LibTableTempl >(templ.name, templ));
        }

        // Read cell info
        if(tokens.size() == 2 && tokens[0] == "cell") {
            LibCellInfo cell;
            // cell.dontUse = false;
            cell.dontTouch = false;
            cell.libname = lib.name;
            cell.name = tokens[1];
            string cellName = cell.name;
            cell.dontUse = isDontUse(cell.name);
            // if(ignore_cell) {
            //     _sizer->dontTouchCell.push_back(cell.name);
            //     cell.dontTouch = true;
            // }
            cell.max_tran = tmp_trans;
            // cout << "Reading cell " << cell.name << endl;
            _begin_read_cell_info(is, cell, lib);

            // Find Vt of the cell
            if(_sizer->numVt == 3) {
                if(cellName.find(_sizer->suffixLVT) != std::string::npos) {
                    cell.c_vtype = f;
                }
                else if(cellName.find(_sizer->suffixHVT) != std::string::npos) {
                    cell.c_vtype = s;
                }
                else {
                    cell.c_vtype = m;
                }
            }
            else if(_sizer->numVt == 2) {
                if(cellName.find(_sizer->suffixHVT) != std::string::npos) {
                    cell.c_vtype = s;
                }
                else {
                    cell.c_vtype = m;
                }
            }
            else {
                cell.c_vtype = s;
            }
            cells.push_back(cell);
            if(cell.name == "NAND3x2_ASAP7_75t_R") {
                report_cell(cell);
            }
            if(VERBOSE > 1) {
                report_cell(cell);
            }
        }
    }

    _sizer->LIBs.insert(pair< string, LibInfo >(lib.name, lib));

    // JLPWR
    _sizer->sw_adj = 1e-6;
    _sizer->res_unit = 1e6;
    _sizer->cap_unit = 1e-15;
    _sizer->time_unit = 1e-9;

    LibCellInfo cur_cell;

    for(unsigned i = 0; i < cells.size(); i++) {
        cur_cell = cells[i];
        _sizer->libs[corner].insert(
            pair< string, LibCellInfo >(cur_cell.name, cur_cell));
    }
    for(unsigned i = 0; i < cells.size(); i++) {
        cur_cell = cells[i];
        map< string, unsigned >::iterator temp_iter;
        // if(cur_cell.footprint == "152"){
        //     cout << "152" << endl;
        // }
        if((temp_iter = _sizer->func2id.find(cur_cell.footprint)) !=
           _sizer->func2id.end()) {
            if(!cur_cell.dontUse) {
                _sizer->func_lib_cell_list[corner][cur_cell.footprint]
                    .push_back(
                        &_sizer->libs[corner].find(cur_cell.name)->second);
            }
        }
        else {
            _sizer->func2id.insert(pair< string, unsigned >(
                cur_cell.footprint, _sizer->func2id.size()));
            if(!cur_cell.dontUse) {
                list< LibCellInfo* > cell_list;
                cell_list.push_back(
                    &_sizer->libs[corner].find(cur_cell.name)->second);
                _sizer->func_lib_cell_list[corner].insert(
                    std::pair< string, list< LibCellInfo* > >(
                        cur_cell.footprint, cell_list));
            }
        }
    }
#if 1
    int t_corner = 0;
    std::map< string, list< LibCellInfo* > >::iterator it;
    for(it = _sizer->func_lib_cell_list[t_corner].begin();
        it != _sizer->func_lib_cell_list[t_corner].end(); ++it) {
        for(LibCellInfo* lib_cell_info : it->second) {
            double partial_order = 0;
            int partial_count = 0;
            for(auto [id, pin] : lib_cell_info->pins) {
                if(pin.isInput) {
                    partial_order += pin.capacitance;
                    partial_count++;
                }
            }
            lib_cell_info->partial_order = partial_order / partial_count;
        }
        (it->second).sort([&](LibCellInfo* c1, LibCellInfo* c2) {
            return c1->leakagePower < c2->leakagePower;
        });
    }
#endif
    is.close();
}

string Circuit::read_lib_name(istream& is) {
    bool finishedReading = false;

    std::vector< string > tokens;

    while(!finishedReading) {
        read_line_as_tokens_chk(is, tokens);

        if(tokens.size() == 2 && tokens[0] == "library") {
            return tokens[1];
        }
    }
}

// Read unit, default_max_transition
void Circuit::read_head_info(istream& is, LibInfo& lib, unsigned corner) {
    unsigned cnt = 0;
    lib.max_transition = 0.0;

    std::vector< string > tokens;

    while(cnt < 7) {
        read_line_as_tokens_chk(is, tokens);

        if(tokens.size() == 0) {
            continue;
        }

        if(tokens.size() == 2 && tokens[0] == "cell") {
            break;
        }

        if(tokens.size() == 2 && tokens[0] == "default_max_transition") {
            // lib.max_transition = std::atof(tokens[1].c_str()) * lib.time_unit
            // / 1e-9;
            lib.max_transition = std::atof(tokens[1].c_str());
            // lib.max_transition = std::atof(tokens[1].c_str());
            // cout << "Default maximum transition is " << lib.max_transition <<
            // " ns" << endl;
            cout << "Default maximum transition is "
                 << lib.max_transition * lib.time_unit << endl;
            // if ( _sizer->maxTran == 0.0 || _sizer->maxTran >
            // lib.max_transition ) {
            ++cnt;
        }

        if(tokens.size() == 2 && tokens[0] == "voltage") {
            lib.volt = std::atof(tokens[1].c_str());
            cout << "Default voltage = " << lib.volt << endl;
            ++cnt;
        }

        if(tokens.size() == 2 && tokens[0] == "time_unit") {
            string temp = tokens[1];
            if(temp == "1ns") {
                lib.time_unit = 1e-9;
            }
            else if(temp == "1ps") {
                lib.time_unit = 1e-12;
            }
            else {
                lib.time_unit = 1e-9;
            }
            cout << "Time unit = " << lib.time_unit << "s" << endl;
            ++cnt;
        }

        if(tokens.size() == 2 && tokens[0] == "voltage_unit") {
            string temp = tokens[1];
            if(temp == "1V") {
                lib.voltage_unit = 1.0;
            }
            else if(temp == "1mV") {
                lib.voltage_unit = 1e-3;
            }
            else {
                lib.voltage_unit = 1.0;
            }
            cout << "Voltage unit = " << lib.voltage_unit << "V" << endl;
            ++cnt;
        }

        if(tokens.size() == 2 && tokens[0] == "current_unit") {
            string temp = tokens[1];
            if(temp == "1A") {
                lib.current_unit = 1.0;
            }
            else if(temp == "1mA") {
                lib.current_unit = 1e-3;
            }
            else {
                lib.current_unit = 1e-3;
            }
            cout << "Current unit = " << lib.current_unit << "A" << endl;
            ++cnt;
        }

        if(tokens[0] == "capacitive_load_unit") {
            string temp = "init";
            if(tokens.size() == 3) {
                temp = tokens[2];
            }
            else if(tokens.size() == 2) {
                read_line_as_tokens_chk(is, tokens);
                if(tokens.size() == 1) {
                    temp = tokens[0];
                }
            }
            if(temp == "nF" || temp == "nf") {
                lib.cap_unit = 1e-9;
            }
            else if(temp == "pF" || temp == "pf") {
                lib.cap_unit = 1e-12;
            }
            else if(temp == "fF" || temp == "ff") {
                lib.cap_unit = 1e-15;
            }
            else {
                lib.cap_unit = 1e-15;
            }
            cout << "Cap unit = " << lib.cap_unit << "F" << endl;
            if(temp == "init") {
                cout << "[Warning] There is no cap unit info !" << endl;
            }
            ++cnt;
        }

        if(tokens.size() == 2 && tokens[0] == "leakage_power_unit") {
            string temp = tokens[1];
            if(temp == "1W") {
                lib.leak_power_unit = 1.0;
            }
            else if(temp == "1mW") {
                lib.leak_power_unit = 1e-3;
            }
            else if(temp == "1uW") {
                lib.leak_power_unit = 1e-6;
            }
            else if(temp == "1nW") {
                lib.leak_power_unit = 1e-9;
            }
            else if(temp == "1pW") {
                lib.leak_power_unit = 1e-12;
            }
            else {
                lib.leak_power_unit = 1e-9;
            }
            cout << "Leakage power unit = " << lib.leak_power_unit << "W"
                 << endl;
            ++cnt;
        }
    }
    lib.int_power_unit =
        lib.voltage_unit * lib.voltage_unit * lib.cap_unit / lib.time_unit;
    cout << "Internal power unit = " << lib.int_power_unit << "W" << endl;
    lib.sw_power_unit = lib.voltage_unit * lib.voltage_unit * lib.cap_unit;
    cout << "Switching power unit = " << lib.sw_power_unit << "W" << endl;
}

void Circuit::_begin_read_templ_info(istream& is, LibTableTempl& templ) {
    bool finishedReading = false;

    int check = 1;

    vector< double > temp1;
    vector< double > temp2;

    temp1.clear();
    temp2.clear();

    std::vector< string > tokens;

    templ.loadFirst = false;
    templ.tranFirst = false;

    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);

        if(check == 0) {
            // cout << "Finish reading lut template " << templ.name << endl;
            finishedReading = true;
            if(templ.tranFirst) {
                for(unsigned i = 0; i < temp1.size(); ++i) {
                    templ.transitionIndices.push_back(temp1[i]);
                }
                for(unsigned i = 0; i < temp2.size(); ++i) {
                    templ.loadIndices.push_back(temp2[i]);
                }
            }
            if(templ.loadFirst) {
                for(unsigned i = 0; i < temp1.size(); ++i) {
                    templ.loadIndices.push_back(temp1[i]);
                }
                for(unsigned i = 0; i < temp2.size(); ++i) {
                    templ.transitionIndices.push_back(temp2[i]);
                }
            }
        }

        if(tokens.size() == 0) {
            continue;
        }

        if(tokens.size() > 2 && tokens[0] == "index_1") {
            for(unsigned i = 1; i < tokens.size(); ++i) {
                temp1.push_back(atof(tokens[i].c_str()));
            }
        }

        if(tokens.size() > 2 && tokens[0] == "index_2") {
            for(unsigned i = 1; i < tokens.size(); ++i) {
                temp2.push_back(atof(tokens[i].c_str()));
            }
        }

        if(tokens[0] == "variable_1") {
            std::size_t found;
            found = tokens[1].find("transition");
            if(found != std::string::npos) {
                templ.tranFirst = true;
            }
            found = tokens[1].find("capacitance");
            if(found != std::string::npos) {
                templ.loadFirst = true;
            }
        }
    }
}

string Circuit::_begin_read_power_info(istream& is, string toPin,
                                       LibPowerInfo& power, LibInfo lib) {
    power.toPin = toPin;

    bool finishedReading = false;

    int check = 1;
    string related_pin;
    string pg_pin;
    std::vector< string > tokens;

    power.risePower.tableVals.clear();
    power.fallPower.tableVals.clear();

    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);

        if(check == 0) {
            finishedReading = true;
        }

        if(tokens.size() == 0) {
            continue;
        }
        if(tokens[0] == "rise_power") {
            LibLUT lut;
            lut.templ = tokens[1];
            add_pg_pin(power, pg_pin);
            if(lut.templ == "scalar") {
                skip(is);
                --check;
            }
            else {
                _begin_read_lut(is, lut, "power", lib);
                update_lut(power.risePower, lut);
            }
        }
        else if(tokens[0] == "fall_power") {
            LibLUT lut;
            lut.templ = tokens[1];
            if(lut.templ == "scalar") {
                skip(is);
                --check;
            }
            else {
                _begin_read_lut(is, lut, "power", lib);
                update_lut(power.fallPower, lut);
            }
        }
        else if(tokens[0] == "related_pin") {
            related_pin = tokens[1];
        }
        else if(tokens[0] == "related_pg_pin") {
            pg_pin = tokens[1];
        }
        else if(tokens[0] == "when") {
            power.isFunc = true;
        }
    }
    power.relatedPin = related_pin;
    return related_pin;
}

void Circuit::average_lut(LibLUT& lut, int num) {
    if(num <= 0) {
        return;
    }
    for(unsigned i = 0; i < lut.loadIndices.size(); ++i) {
        for(unsigned j = 0; j < lut.transitionIndices.size(); ++j) {
            lut.tableVals[i][j] /= double(num);
        }
    }
}

// updated the old function
void Circuit::_begin_read_lut(istream& is, LibLUT& lut, string type,
                              LibInfo lib) {
    std::vector< string > tokens;

    bool flag = false;

    LibTableTempl& templ = lib.templs[lut.templ];
    // cout << "Table template name " << templ.name << endl;
    // if(!templ.loadFirst && templ.tranFirst) {
    //     printf("asdasda");
    // }
    // Read indices
    unsigned size1 = 0, size2 = 0, index_num = 0;
    double ratio = 1;
    if(type == "power") {
        // Normalize the power unit to mW
        ratio = lib.int_power_unit / 1e-3;
    }
    else if(type == "timing") {
        // Normalize the timing unit to ns
        ratio = lib.time_unit / 1e-9;
    }

    tokens.push_back("init");

    while(tokens[0] != "values") {
        read_line_as_tokens_chk(is, tokens);
        // Index 1
        if(tokens[0] == "index_1" && tokens.size() > 2) {
            size1 = tokens.size() - 1;
            if(templ.tranFirst) {
                lut.transitionIndices.resize(size1);
                for(unsigned i = 0; i < tokens.size() - 1; ++i) {
                    lut.transitionIndices[i] =
                        atof(tokens[i + 1].c_str()) * lib.time_unit / 1e-9;
                }
            }
            else if(templ.loadFirst) {
                lut.loadIndices.resize(size1);
                for(unsigned i = 0; i < tokens.size() - 1; ++i) {
                    lut.loadIndices[i] =
                        atof(tokens[i + 1].c_str()) * lib.cap_unit / 1e-15;
                }
            }
            else {
                printf("Error: LUT template is not defined correctly\n");
                // exit(0);
            }
            ++index_num;
        }
        // Index 2
        if(tokens[0] == "index_2" && tokens.size() > 2) {
            size2 = tokens.size() - 1;
            if(templ.tranFirst) {
                lut.loadIndices.resize(size2);
                for(unsigned i = 0; i < tokens.size() - 1; ++i) {
                    lut.loadIndices[i] =
                        atof(tokens[i + 1].c_str()) * lib.cap_unit / 1e-15;
                }
            }
            else if(templ.loadFirst) {
                lut.transitionIndices.resize(size2);
                for(unsigned i = 0; i < tokens.size() - 1; ++i) {
                    lut.transitionIndices[i] =
                        atof(tokens[i + 1].c_str()) * lib.time_unit / 1e-9;
                }
            }
            ++index_num;
        }
    }

    // Use indices from templates
    if(index_num == 0) {
        lut.transitionIndices.clear();
        for(unsigned i = 0; i < templ.transitionIndices.size(); ++i) {
            lut.transitionIndices.push_back(templ.transitionIndices[i]);
        }
        lut.loadIndices.clear();
        for(unsigned i = 0; i < templ.loadIndices.size(); ++i) {
            lut.loadIndices.push_back(templ.loadIndices[i]);
        }
        if(templ.loadFirst) {
            size1 = templ.loadIndices.size();
            size2 = templ.transitionIndices.size();
        }
        else if(templ.tranFirst) {
            size1 = templ.transitionIndices.size();
            size2 = templ.loadIndices.size();
        }
    }

    if(tokens.size() == 1) {
        // flag indicates the "value" takes one line
        flag = true;
    }
    else {
        // if "values" is on the same line with tableVals, revise the vector
        tokens.erase(tokens.begin());
    }

    if(lut.transitionIndices.empty() || lut.loadIndices.empty()) {
        if(flag) {
            read_line_as_tokens_chk(is, tokens);
        }
        vector< double > tmpline;
        for(unsigned i = 0; i < tokens.size(); ++i) {
            tmpline.push_back(atof(tokens[i].c_str()) * ratio);
        }
        if(lut.loadIndices.empty()) {
            lut.loadIndices.push_back(0 * lib.cap_unit / 1e-15);
            lut.loadIndices.push_back(100 * lib.cap_unit / 1e-15);
            lut.tableVals.push_back(tmpline);
            lut.tableVals.push_back(tmpline);
        }
        else {
            lut.transitionIndices.push_back(0 * lib.time_unit / 1e-9);
            lut.transitionIndices.push_back(100 * lib.time_unit / 1e-9);
            lut.tableVals.resize(tmpline.size());
            for(unsigned i = 0; i < lut.loadIndices.size(); ++i) {
                lut.tableVals[i].resize(2);
            }
            for(unsigned i = 0; i < lut.loadIndices.size(); ++i) {
                lut.tableVals[i][0] = tmpline[i];
                lut.tableVals[i][1] = tmpline[i];
            }
        }
        return;
    }

    if(templ.loadFirst) {
        lut.tableVals.resize(size1);
        for(unsigned i = 0; i < size1; ++i) {
            if(flag || i != 0) {
                read_line_as_tokens_chk(is, tokens);
            }
            lut.tableVals[i].resize(size2);
            for(unsigned j = 0; j < size2; ++j) {
                lut.tableVals[i][j] = atof(tokens[j].c_str()) * ratio;
            }
        }
    }
    else if(templ.tranFirst) {
        // Change table so that index1 = load and index2 = tran
        lut.tableVals.resize(size2);
        for(unsigned j = 0; j < size2; ++j) {
            lut.tableVals[j].resize(size1);
        }
        for(unsigned i = 0; i < size1; ++i) {
            if(flag || i != 0) {
                read_line_as_tokens_chk(is, tokens);
            }
            for(unsigned j = 0; j < size2; ++j) {
                lut.tableVals[j][i] = atof(tokens[j].c_str()) * ratio;
            }
        }
    }
    // cout << "Load index size = " << lut.loadIndices.size();
    // cout << "Tran index size = " << lut.transitionIndices.size() << endl;
    // cout << "LUT size = " << lut.tableVals.size() << endl;
}

void Circuit::add_pg_pin(LibPowerInfo& powerTables, string pg_pin) {
    for(unsigned i = 0; i < powerTables.pgPins.size(); ++i) {
        if(powerTables.pgPins[i] == pg_pin) {
            return;
        }
    }
    powerTables.pgPins.push_back(pg_pin);
}

void Circuit::update_lut(LibLUT& toLut, LibLUT fromLut) {
    if(toLut.tableVals.empty()) {
        toLut.templ = fromLut.templ;
        toLut.loadIndices = fromLut.loadIndices;
        toLut.transitionIndices = fromLut.transitionIndices;
        toLut.tableVals = fromLut.tableVals;
    }
    else {
        for(unsigned i = 0; i < toLut.loadIndices.size(); ++i) {
            for(unsigned j = 0; j < toLut.transitionIndices.size(); ++j) {
                toLut.tableVals[i][j] += fromLut.tableVals[i][j];
            }
        }
    }
}

string Circuit::_begin_read_timing_info(istream& is, string toPin,
                                        LibTimingInfo& timing, LibInfo lib) {
    timing.toPin = toPin;

    bool finishedReading = false;

    unsigned timingType = 0;

    int check = 1;
    std::vector< string > tokens;

    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);

        if(check == 0) {
            finishedReading = true;
        }

        if(tokens.size() == 0) {
            continue;
        }

        if(tokens[0] == "cell_fall" ||
           (timingType == 1 && tokens[0] == "fall_constraint")) {
            if(tokens[1] == "scalar") {
                skip(is);
                --check;
            }
            else {
                timing.fallDelay.templ = tokens[1];
                // cout << "read fall delay table" << endl;
                _begin_read_lut(is, timing.fallDelay, "timing", lib);
            }
        }
        else if(tokens[0] == "cell_rise" ||
                (timingType == 1 && tokens[0] == "rise_constraint")) {
            if(tokens[1] == "scalar") {
                skip(is);
                --check;
            }
            else {
                // cout << "read rise delay table" << endl;
                timing.riseDelay.templ = tokens[1];
                _begin_read_lut(is, timing.riseDelay, "timing", lib);
            }
        }
        else if(tokens[0] == "fall_transition" ||
                (timingType == 2 && tokens[0] == "fall_constraint")) {
            if(tokens[1] == "scalar") {
                skip(is);
                --check;
            }
            else {
                timing.fallTransition.templ = tokens[1];
                _begin_read_lut(is, timing.fallTransition, "timing", lib);
            }
        }
        else if(tokens[0] == "rise_transition" ||
                (timingType == 2 && tokens[0] == "rise_constraint")) {
            if(tokens[1] == "scalar") {
                skip(is);
                --check;
            }
            else {
                timing.riseTransition.templ = tokens[1];
                _begin_read_lut(is, timing.riseTransition, "timing", lib);
            }
        }
        else if(tokens[0] == "timing_sense") {
            if(tokens[1] == "positive_unate") {
                timing.timingSense = 'p';
            }
            else if(tokens[1] == "negative_unate") {
                timing.timingSense = 'n';
            }
            else {
                timing.timingSense = 'p';
            }
        }
        else if(tokens[0] == "related_pin") {
            assert(tokens.size() == 2);
            timing.fromPin = tokens[1];
        }
        else if(tokens[0] == "timing_type") {
            if(tokens[1] == "setup_falling" || tokens[1] == "setup_rising") {
                // cout << "setup timing" << endl;
                timing.timingSense = 'c';
                timingType = 1;
            }
            else if(tokens[1] == "hold_falling" || tokens[1] == "hold_rising") {
                timingType = 2;
                timing.timingSense = 'c';
                // cout << "hold timing" << endl;
            }
        }
        else if(tokens[0] == "when") {
            timing.isFunc = true;
        }
    }
    return timing.fromPin;
}

void Circuit::_begin_read_pin_info(istream& is, string pinName, LibPinInfo& pin,
                                   LibCellInfo& cell, LibInfo& lib) {
    // cout << "Reading pin " << pinName << endl;

    pin.name = pinName;
    pin.isClock = false;
    pin.isData = false;

    bool finishedReading = false;

    int check = 1;
    std::vector< string > tokens;
    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);
        if(check == 0) {
            finishedReading = true;
        }

        if(tokens.size() == 0) {
            continue;
        }

        if(tokens.size() == 2 && tokens[0] == "direction") {
            if(tokens[1] == "input") {
                pin.isInput = true;
                pin.isOutput = false;
            }
            else if(tokens[1] == "output") {
                pin.isInput = false;
                pin.isOutput = true;
            }
            else if(tokens[1] == "inout") {
                pin.isInput = true;
                pin.isOutput = true;
            }
            else if(tokens[1] == "internal") {
                pin.isInput = false;
                pin.isOutput = false;
            }
            else {
                assert(false);  // undefined direction
            }
        }
        else if(tokens.size() == 2 && tokens[0] == "capacitance") {
            pin.capacitance =
                std::atof(tokens[1].c_str()) * lib.cap_unit / 1e-15;
        }
        else if(tokens.size() == 2 && tokens[0] == "max_capacitance") {
            pin.maxCapacitance =
                std::atof(tokens[1].c_str()) * lib.cap_unit / 1e-15;
        }
        else if(tokens[0] == "internal_power" && pin.isOutput && !pin.isInput) {
            LibPowerInfo tmplib;

            tmplib.isFunc = false;
            tmplib.cnt = 1;

            unsigned relatedPinId, curPinId;
            string relatedPin =
                _begin_read_power_info(is, pinName, tmplib, lib);
            if(relatedPin != "") {
                // relatedPin Id
                if(cell.lib_pin2id_map.find(relatedPin) ==
                   cell.lib_pin2id_map.end()) {
                    relatedPinId = cell.lib_pin2id_map.size();
                    // cell.lib_pin2id_map[relatedPin] = relatedPinId;
                    cell.lib_pin2id_map.insert(
                        pair< string, unsigned >(relatedPin, relatedPinId));
                    // cout << "ADD PIN " << cell.name << "/" << relatedPin << "
                    // " << cell.lib_pin2id_map[relatedPin] << endl;
                }
                else {
                    relatedPinId = cell.lib_pin2id_map[relatedPin];
                }

                // curPin Id
                if(cell.lib_pin2id_map.find(pinName) ==
                   cell.lib_pin2id_map.end()) {
                    curPinId = cell.lib_pin2id_map.size();
                    // cell.lib_pin2id_map[pinName] = curPinId;
                    cell.lib_pin2id_map.insert(
                        pair< string, unsigned >(pinName, curPinId));
                    // cout << "ADD PIN " << cell.name << "/" << pinName << " "
                    // << cell.lib_pin2id_map[pinName] << endl;
                }
                else {
                    curPinId = cell.lib_pin2id_map[pinName];
                }
                // JLPWR

                if(relatedPinId != curPinId) {
                    // unsigned index = relatedPinId*100 + curPinId;
                    unsigned index = relatedPinId + curPinId * 100;
                    if(cell.powerTables.find(index) != cell.powerTables.end()) {
                        merge_powerTables(cell.powerTables[index], tmplib);
                    }
                    else {
                        cell.powerTables.insert(
                            pair< unsigned, LibPowerInfo >(index, tmplib));
                    }
                }

                // cout << "Read " << cell.powerTables.size() << " power tables"
                // << endl;
            }
            --check;
        }
        else if(tokens[0] == "timing") {
            LibTimingInfo tmplib;
            string fromPin;
            unsigned fromPinId, toPinId;

            tmplib.isFunc = false;
            tmplib.cnt0 = 1;
            tmplib.cnt1 = 1;
            tmplib.cnt2 = 1;
            tmplib.cnt3 = 1;

            // Read timing info
            fromPin = _begin_read_timing_info(is, pinName, tmplib, lib);
            // cout << "fromPin: " << fromPin << " toPin: " << pinName << endl;
            assert(tmplib.fromPin == fromPin);
            assert(tmplib.toPin == pinName);
            if(pinName == "RESET") {
                printf("hhh");
            }
            if(cell.name == "PB4W") {
                printf("hhh");
            }
            if(fromPin != "") {
                // JLPWR
                if(tmplib.timingSense == '-') {
                    if(pin.IQN)
                        tmplib.timingSense = 'n';
                    else
                        tmplib.timingSense = 'p';
                }

                if(tmplib.timingSense == 'c') {
                    pin.isData = true;
                }

                // fromPin Id
                if(cell.lib_pin2id_map.find(fromPin) ==
                   cell.lib_pin2id_map.end()) {
                    fromPinId = cell.lib_pin2id_map.size();
                    // cell.lib_pin2id_map[fromPin] = fromPinId;
                    cell.lib_pin2id_map.insert(
                        pair< string, unsigned >(fromPin, fromPinId));
                    // cout << "ADD PIN " << cell.name << "/" << fromPin << " "
                    // << cell.lib_pin2id_map[fromPin] << endl;
                }
                else {
                    fromPinId = cell.lib_pin2id_map[fromPin];
                }

                // toPin Id
                if(cell.lib_pin2id_map.find(pinName) ==
                   cell.lib_pin2id_map.end()) {
                    toPinId = cell.lib_pin2id_map.size();
                    // cell.lib_pin2id_map[pinName] = toPinId;
                    cell.lib_pin2id_map.insert(
                        pair< string, unsigned >(pinName, toPinId));
                    // cout << "ADD PIN " << cell.name << "/" << pinName << " "
                    // << cell.lib_pin2id_map[pinName] << endl;
                }
                else {
                    toPinId = cell.lib_pin2id_map[pinName];
                }
                if(fromPinId != toPinId) {
                    unsigned idx1 = fromPinId + toPinId * 100;
                    // Update existing timing info
                    auto insert_arc = [&](int index,
                                          LibTimingInfo timing_info) {
                        if(cell.timingArcs.find(index) !=
                           cell.timingArcs.end()) {
                            merge_timingArcs(cell.timingArcs[index],
                                             timing_info);
                        }
                        else {
                            if(!timing_info.riseDelay.tableVals.empty() ||
                               !timing_info.fallDelay.tableVals.empty() ||
                               !timing_info.riseTransition.tableVals.empty() ||
                               !timing_info.fallTransition.tableVals.empty()) {
                                cell.timingArcs.insert(
                                    pair< unsigned, LibTimingInfo >(
                                        index, timing_info));
                            }
                        }
                    };
                    insert_arc(idx1, tmplib);
                    if(pin.isOutput && pin.isInput && cell.name == "PB4W") {
                        std::swap(tmplib.fromPin, tmplib.toPin);
                        unsigned idx2 = fromPinId * 100 + toPinId;
                        insert_arc(idx2, tmplib);
                    }
                }
                // cout << "Read " << cell.timingArcs.size() << " timing arcs"
                // << endl;
            }
            else {
                printf("Pin name %s\n", pinName.c_str());
                assert(0);
            }
            --check;
        }
        else if(tokens[0] == "clock") {
            pin.isClock = true;
            cell.isSequential = true;
        }
        else if(tokens[0] == "clock_gate_clock_pin" && tokens[1] == "true") {
            pin.isClock = true;
            cell.isSequential = true;
        }
        else if(tokens[0] == "nextstate_type" && tokens[1] == "data") {
            pin.isData = true;

            // add EN pins
        }
        else if(tokens[0] == "clock_gate_enable_pin" && tokens[1] == "true") {
            pin.isData = true;
        }
        else if(tokens[0] == "function" && tokens[1] == "IQN") {
            // cout << "Function == IQN" << endl;
            cell.hasQN = true;
            pin.IQN = true;
        }
    }
}

void Circuit::_begin_read_cell_info(istream& is, LibCellInfo& cell,
                                    LibInfo& lib) {
    cell.isSequential = false;
    cell.dontTouch = false;

    bool finishedReading = false;
    int check = 1;

    double leak = 0;
    unsigned leak_cnt = 0;
    // true: one leakage value / false: average over state-dependent values
    bool leak_flag = false;
    string data_pin = "";

    std::vector< string > tokens;
    while(!finishedReading) {
        check += read_line_as_tokens_chk(is, tokens);
        if(check == 0) {
            finishedReading = true;
            if(cell.footprint == "" || NO_FOOTPRINT) {
                cell.footprint = cell.name;
                if(VERBOSE > 1)
                    cout << "GET FOOTPRINT FOR " << cell.footprint << endl;
                size_t start = cell.footprint.find_first_of("_");
                if(start != -1) {
                    cell.footprint.erase(start,
                                         cell.footprint.length() - start);
                }
                else
                    cell.footprint = "NA";
                if(ASAP7) {
                    string temp = cell.name;
                    cell.footprint =
                        to_string(_sizer->cellName2EquaivaID.at(cell.name));
                    // cout << "footprint " << cell.name << " " <<
                    //         cell.footprint << endl;
                }
                if(STM28) {
                    string temp = cell.name;
                    string delimiter = "_";
                    size_t pos = temp.find(delimiter);
                    size_t pos1 = temp.find(delimiter, pos + 1);
                    size_t pos2 = temp.find(delimiter, pos1 + 1);
                    temp = temp.substr(pos1 + 1, pos2);

                    size_t start = temp.find_last_of("X");
                    temp.erase(start, temp.length() - start);
                    cell.footprint = temp;
                    // cout << "footprint " << cell.name << " " <<
                    // cell.footprint << endl;
                }
                if(C40) {
                    string temp = cell.name;
                    string delimiter = "_";
                    size_t pos = temp.find(delimiter);
                    size_t pos1 = temp.find(delimiter, pos + 1);
                    temp = temp.substr(pos + 1, pos1);
                    cell.footprint = temp;
                    // cout << "footprint " << cell.name << " " <<
                    // cell.footprint << endl;
                }
            }
            // cout << "Finish reading cell " << cell.name << endl;
        }

        if(tokens.size() == 0) {
            continue;
        }

        if(tokens.size() == 1 && tokens[0] == "test_cell") {
            skip(is);
            check--;
        }

        if(tokens.size() == 2 && tokens[0] == "cell_leakage_power") {
            // Normalize the leakage power to mW
            // cell.leakagePower =
            //     atof(tokens[1].c_str()) * lib.leak_power_unit / 1e-6;
            cell.leakagePower =
                atof(tokens[1].c_str()) * lib.leak_power_unit / _sizer->sw_adj;
            leak_flag = true;
        }
        else if(tokens.size() == 1 && tokens[0] == "leakage_power") {
            read_leak(is, leak, leak_cnt);
            --check;
        }
        else if(tokens[0] == "cell_footprint") {
            assert(tokens.size() == 2);
            cell.footprint = tokens[1];
        }
        else if(tokens[0] == "area") {
            assert(tokens.size() == 2);
            cell.area = std::atof(tokens[1].c_str());
            // Missing cell height info (specify cell.width = cell.area)
            cell.width = std::atof(tokens[1].c_str());
        }
        else if(tokens[0] == "clocked_on") {
            cell.isSequential = true;
        }
        else if(tokens[0] == "dont_touch") {
            cell.dontTouch = true;
        }
        else if(tokens.size() == 2 && tokens[0] == "next_state") {
            data_pin = tokens[1];
        }
        else if(tokens.size() == 2 &&
                (tokens[0] == "pin" || tokens[0] == "bus")) {
            LibPinInfo pin;
            _begin_read_pin_info(is, tokens[1], pin, cell, lib);
            if(pin.maxCapacitance == std::numeric_limits< double >::max()) {
            }
            if(pin.isInput || pin.isOutput) {
                if(cell.lib_pin2id_map.find(pin.name) ==
                   cell.lib_pin2id_map.end()) {
                    unsigned pin_id = cell.lib_pin2id_map.size();
                    cell.lib_pin2id_map.insert(
                        pair< string, unsigned >(pin.name, pin_id));
                    // cout << "ADD PIN " << cell.name << "/" << pin.name << " "
                    // << cell.lib_pin2id_map[pin.name] << endl;
                }
                cell.pins.insert(pair< unsigned, LibPinInfo >(
                    cell.lib_pin2id_map[pin.name], pin));
            }
            --check;
        }
        else if(tokens.size() == 2 && tokens[0] == "dont_use" &&
                tokens[1] == "true") {
            cell.dontTouch = true;
        }
    }
    // assign leakage power
    if(!leak_flag) {
        if(leak_cnt != 0 && lib.leak_power_unit != 0)
            cell.leakagePower =
                leak / leak_cnt * lib.leak_power_unit / _sizer->sw_adj;
        else
            cell.leakagePower = 0.0;
    }
    std::map< unsigned, LibTimingInfo >::iterator timeItr;
    for(timeItr = cell.timingArcs.begin(); timeItr != cell.timingArcs.end();
        ++timeItr) {
        if(timeItr->second.cnt0 + timeItr->second.cnt1 + timeItr->second.cnt2 +
               timeItr->second.cnt3 >
           4) {
            average_lut(timeItr->second.fallDelay, timeItr->second.cnt0);
            average_lut(timeItr->second.riseDelay, timeItr->second.cnt1);
            average_lut(timeItr->second.fallTransition, timeItr->second.cnt2);
            average_lut(timeItr->second.riseTransition, timeItr->second.cnt3);
            timeItr->second.cnt0 = 1;
            timeItr->second.cnt1 = 1;
            timeItr->second.cnt2 = 1;
            timeItr->second.cnt3 = 1;
        }
    }
    std::map< unsigned, LibPowerInfo >::iterator powerItr;
    for(powerItr = cell.powerTables.begin(); powerItr != cell.powerTables.end();
        ++powerItr) {
        if(powerItr->second.cnt > 1) {
            int cnt = powerItr->second.cnt;
            if(powerItr->second.pgPins.size() != 0) {
                cnt = cnt / powerItr->second.pgPins.size();
            }
            if(powerItr->second.fallPower.templ != "") {
                average_lut(powerItr->second.fallPower, cnt);
            }
            if(powerItr->second.risePower.templ != "") {
                average_lut(powerItr->second.risePower, cnt);
            }
            powerItr->second.cnt = 1;
        }
    }
    if(VERBOSE > 4)
        cout << cell;

    std::map< unsigned, LibPinInfo >::iterator pinItr;
    for(pinItr = cell.pins.begin(); pinItr != cell.pins.end(); ++pinItr) {
        if(pinItr->second.isInput && pinItr->second.name == data_pin) {
            pinItr->second.isData = 1;
        }
    }
}

// Read the default max_transition defined for the library.
// Return value indicates if the last read was successful or not.
// This function must be called in the beginning before any read_cell_info
// function call.
bool Circuit::read_default_max_transition(double& maxTransition) {
    maxTransition = 0.0;
    vector< string > tokens;

    // bool valid = read_line_as_tokens (is, tokens) ;
    read_line_as_tokens(is, tokens);

    // while (valid) {
    while(!is.eof()) {
        if(tokens.size() == 2 && tokens[0] == "default_max_transition") {
            maxTransition = std::atof(tokens[1].c_str());
            cout << "MAX tran " << maxTransition << endl;
            return true;
        }

        // valid = read_line_as_tokens (is, tokens) ;
        read_line_as_tokens(is, tokens);
    }

    return false;
}

void Circuit::read_leak(istream& is, double& leak, unsigned& leak_cnt) {
    unsigned check = 1;

    vector< string > tokens;

    while(check != 0) {
        check += read_line_as_tokens_chk(is, tokens);
        if(tokens.size() == 2 && tokens[0] == "value") {
            leak += atof(tokens[1].c_str());
            ++leak_cnt;
        }
    }
}

void Circuit::merge_timingArcs(LibTimingInfo& timing1, LibTimingInfo& timing2) {
    if(timing1.isFunc && !timing2.isFunc) {
        if(timing2.fallDelay.templ == "") {
            timing1.fallDelay.tableVals.clear();
        }
        else {
            timing1.fallDelay = timing2.fallDelay;
        }
        if(timing2.riseDelay.templ == "") {
            timing1.riseDelay.tableVals.clear();
        }
        else {
            timing1.riseDelay = timing2.riseDelay;
        }
        if(timing2.fallTransition.templ == "") {
            timing1.fallTransition.tableVals.clear();
        }
        else {
            timing1.fallTransition = timing2.fallTransition;
        }
        if(timing2.riseTransition.templ == "") {
            timing1.riseTransition.tableVals.clear();
        }
        else {
            timing1.riseTransition = timing2.riseTransition;
        }
        timing1.isFunc = false;
        timing1.cnt0 = 1;
        timing1.cnt1 = 1;
        timing1.cnt2 = 1;
        timing1.cnt3 = 1;
    }
    else if(timing1.isFunc == timing2.isFunc) {
        if(timing2.fallDelay.templ != "") {
            if(timing1.fallDelay.templ != "") {
                ++timing1.cnt0;
            }
            update_lut(timing1.fallDelay, timing2.fallDelay);
        }
        if(timing2.riseDelay.templ != "") {
            if(timing1.riseDelay.templ != "") {
                ++timing1.cnt1;
            }
            update_lut(timing1.riseDelay, timing2.riseDelay);
        }
        if(timing2.fallTransition.templ != "") {
            if(timing1.fallTransition.templ != "") {
                ++timing1.cnt2;
            }
            update_lut(timing1.fallTransition, timing2.fallTransition);
        }
        if(timing2.riseTransition.templ != "") {
            if(timing1.riseTransition.templ != "") {
                ++timing1.cnt3;
            }
            update_lut(timing1.riseTransition, timing2.riseTransition);
        }
    }
}

void Circuit::merge_powerTables(LibPowerInfo& power1, LibPowerInfo& power2) {
    if(power1.isFunc && !power2.isFunc) {
        if(power2.fallPower.templ != "") {
            power1.fallPower = power2.fallPower;
        }
        if(power2.risePower.templ != "") {
            power1.risePower = power2.risePower;
        }
        power1.isFunc = false;
        power1.cnt = 1;
    }
    else if(!power1.isFunc && !power2.isFunc) {
        bool flag = false;
        for(unsigned i = 0; i < power1.pgPins.size(); ++i) {
            if(power1.pgPins[i] == power2.pgPins[0]) {
                flag = true;
                break;
            }
        }
        if(!flag) {
            if(power2.fallPower.templ != "") {
                update_lut(power1.fallPower, power2.fallPower);
            }
            if(power2.risePower.templ != "") {
                update_lut(power1.risePower, power2.risePower);
            }
            add_pg_pin(power1, power2.pgPins[0]);
        }
    }
    else if(power1.isFunc && power2.isFunc) {
        if(power2.fallPower.templ != "") {
            update_lut(power1.fallPower, power2.fallPower);
        }
        if(power2.risePower.templ != "") {
            update_lut(power1.risePower, power2.risePower);
        }
        add_pg_pin(power1, power2.pgPins[0]);
        ++power1.cnt;
    }
}
