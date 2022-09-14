#ifndef __DDR4_H
#define __DDR4_H

#include "DRAM.h"
#include "Request.h"
#include "Config.h"
#include <vector>
#include <functional>

using namespace std;

namespace ramulator
{

class DDR4
{
public:
    static string standard_name;
    enum class Org;
    enum class Speed;
    DDR4(Org org, Speed speed);
    DDR4(const string& org_str, const string& speed_str);
    DDR4(const Config& configs);
    
    static map<string, enum Org> org_map;
    static map<string, enum Speed> speed_map;
    /* Level */
    enum class Level : int
    { 
        Channel, Rank, BankGroup, Bank, Subarray, Row, Column, MAX
    };

    /* Command */
    enum class Command : int
    { 
        ACT, PRE, PREA, 
        RD,  WR,  RDA,  WRA, 
        REF, PDE, PDX,  SRE, SRX, 
        PRE_RSQ, RSQ, ACT_NACK, ACT_PARTIAL_NACK, NOP,
        MAX
    };

    string command_name[int(Command::MAX)] = {
        "ACT", "PRE", "PREA", 
        "RD",  "WR",  "RDA",  "WRA", 
        "REF", "PDE", "PDX",  "SRE", "SRX",
        "PRE_RSQ", "RSQ", "ACT_NACK", "ACT_PARTIAL_NACK", "NOP"
    };

    Level scope[int(Command::MAX)] = {
        Level::Row,    Level::Bank,   Level::Rank,   
        Level::Column, Level::Column, Level::Column, Level::Column,
        Level::Rank,   Level::Rank,   Level::Rank,   Level::Rank,   Level::Rank,
        Level::Bank,   Level::Rank,   Level::Subarray, Level::Subarray, Level::Subarray
    };

    bool is_opening(Command cmd) 
    {
        switch(int(cmd)) {
            case int(Command::ACT):
                return true;
            default:
                return false;
        }
    }

    bool is_accessing(Command cmd) 
    {
        switch(int(cmd)) {
            case int(Command::RD):
            case int(Command::WR):
            case int(Command::RDA):
            case int(Command::WRA):
                return true;
            default:
                return false;
        }
    }

    bool is_closing(Command cmd) 
    {
        switch(int(cmd)) {
            case int(Command::RDA):
            case int(Command::WRA):
            case int(Command::PRE):
            case int(Command::PREA):
            case int(Command::PRE_RSQ):
            case int(Command::ACT_NACK):
                return true;
            default:
                return false;
        }
    }

    bool is_refreshing(Command cmd) 
    {
        switch(int(cmd)) {
            case int(Command::REF):
                return true;
            default:
                return false;
        }
    }

    void mult_refresh_pb(float mult) {};

    Command add_autoprecharge (const Command cmd) {
        switch(int(cmd)) {
            case int(Command::RD):
            case int(Command::RDA):
                return Command::RDA;
            case int(Command::WR):
            case int(Command::WRA):
                return Command::WRA;
            default:
                assert(false);
                return cmd;
        }
    }

    // calculates a single bank id based on the rank ID, bank group ID, and bank ID, i.e., unifies all those individual IDs into a single unique ID
    uint32_t calc_global_bank_id(const std::vector<int> addr_vec) const {
        // uint32_t banks_per_rank = org_entry.count[uint32_t(Level::BankGroup)] * org_entry.count[uint32_t(Level::Bank)];
        uint32_t banks_per_bg = org_entry.count[uint32_t(Level::Bank)];

        // return addr_vec[uint32_t(Level::Rank)]*banks_per_rank + addr_vec[uint32_t(Level::BankGroup)]*banks_per_bg + addr_vec[uint32_t(Level::Bank)];
        return addr_vec[uint32_t(Level::BankGroup)]*banks_per_bg + addr_vec[uint32_t(Level::Bank)];
    }

    uint32_t calc_row_id_in_bank(const std::vector<int> addr_vec) const {
        return addr_vec[uint32_t(Level::Subarray)]*get_subarray_size() + addr_vec[uint32_t(Level::Row)];
    }

    uint32_t get_num_banks_per_rank() const {
        return org_entry.count[uint32_t(Level::BankGroup)] * org_entry.count[uint32_t(Level::Bank)];
    }

    uint32_t get_num_all_banks() const {
        return org_entry.count[uint32_t(Level::Rank)] * org_entry.count[uint32_t(Level::BankGroup)] * org_entry.count[uint32_t(Level::Bank)];
    }

    /* State */
    enum class State : int
    {
        Opened, Closed, PartiallyOpened, PowerUp, ActPowerDown, PrePowerDown, SelfRefresh, MAX
    } start[int(Level::MAX)] = {
        State::MAX, State::PowerUp, State::MAX, State::Closed, State::Closed, State::Closed, State::MAX
    };

    /* Translate */
    Command translate[int(Request::Type::MAX)] = {
        Command::RD,  Command::WR,
        Command::REF, Command::PDE, Command::SRE, 
        Command::RD
    };

    /* Prereq */
    function<Command(DRAM<DDR4>*, Command cmd, int)> prereq[int(Level::MAX)][int(Command::MAX)];

    // SAUGATA: added function object container for row hit status
    /* Row hit */
    function<bool(DRAM<DDR4>*, Command cmd, int)> rowhit[int(Level::MAX)][int(Command::MAX)];
    function<bool(DRAM<DDR4>*, Command cmd, int)> rowopen[int(Level::MAX)][int(Command::MAX)];

    /* Timing */
    struct TimingEntry
    {
        Command cmd;
        int dist;
        int val;
        bool sibling;
    }; 
    vector<TimingEntry> timing[int(Level::MAX)][int(Command::MAX)];

    /* Lambda */
    function<void(DRAM<DDR4>*, int)> lambda[int(Level::MAX)][int(Command::MAX)];

    /* Organization */
    enum class Org : int
    {
        DDR4_2Gb_x4,   DDR4_2Gb_x8,   DDR4_2Gb_x16,
        DDR4_4Gb_x4,   DDR4_4Gb_x8,   DDR4_4Gb_x16,
        DDR4_8Gb_x4,   DDR4_8Gb_x8,   DDR4_8Gb_x16,
        DDR4_16Gb_x8,  DDR4_32Gb_x8,
        MAX
    };

    struct OrgEntry {
        int size;
        int dq;
        int count[int(Level::MAX)];
    } org_table[int(Org::MAX)] = {
        {2<<10,  4, {0, 0, 4, 4, 1, 1<<15, 1<<10}}, {2<<10,  8, {0, 0, 4, 4, 1, 1<<14, 1<<10}}, {2<<10, 16, {0, 0, 2, 4, 1, 1<<14, 1<<10}},
        {4<<10,  4, {0, 0, 4, 4, 1, 1<<16, 1<<10}}, {4<<10,  8, {0, 0, 4, 4, 1, 1<<15, 1<<10}}, {4<<10, 16, {0, 0, 2, 4, 1, 1<<15, 1<<10}},
        {8<<10,  4, {0, 0, 4, 4, 1, 1<<17, 1<<10}}, {8<<10,  8, {0, 0, 4, 4, 1, 1<<16, 1<<10}}, {8<<10, 16, {0, 0, 2, 4, 1, 1<<16, 1<<10}},
        {16<<10, 8, {0, 0, 4, 4, 1, 1<<17, 1<<10}}, {32<<10, 8, {0, 0, 4, 4, 1, 1<<18, 1<<10}}
    }, org_entry;

    void set_channel_number(int channel);
    void set_rank_number(int rank);
    void set_subarray_size(uint32_t sa_size);
    uint32_t get_subarray_size() const {
        return _sa_size;
    }

    /* Speed */
    enum class Speed : int
    {
        DDR4_1600K, DDR4_1600L,
        DDR4_1866M, DDR4_1866N,
        DDR4_2133P, DDR4_2133R,
        DDR4_2400R, DDR4_2400U,
        DDR4_3200,
        MAX
    };

    enum class RefreshMode : int
    {
        Refresh_1X,
        Refresh_2X,
        Refresh_4X,
        MAX
    } refresh_mode = RefreshMode::Refresh_1X;

    int prefetch_size = 8; // 8n prefetch DDR
    int channel_width = 64;

    struct SpeedEntry {
        int rate;
        double freq, tCK;
        int nBL, nCCDS, nCCDL, nRTRS;
        int nCL, nRCD, nRP, nCWL;
        int nRAS, nRC;
        int nRTP, nWTRS, nWTRL, nWR;
        int nRRDS, nRRDL, nFAW;
        int nRFC, nREFI;
        int nPD, nXP, nXPDLL; // XPDLL not found in DDR4??
        int nCKESR, nXS, nXSDLL; // nXSDLL TBD (nDLLK), nXS = (tRFC+10ns)/tCK
        int nNACK_RESEND, nACTtoNACK;
    } speed_table[int(Speed::MAX)] = {
        {1600, (400.0/3)*6, (3/0.4)/6, 4, 4, 5, 2, 11, 11, 11,  9, 28, 39, 6, 2, 6, 12, 0, 0, 0, 0, 0, 4, 5, 0, 5, 0, 0},
        {1600, (400.0/3)*6, (3/0.4)/6, 4, 4, 5, 2, 12, 12, 12,  9, 28, 40, 6, 2, 6, 12, 0, 0, 0, 0, 0, 4, 5, 0, 5, 0, 0},
        {1866, (400.0/3)*7, (3/0.4)/7, 4, 4, 5, 2, 13, 13, 13, 10, 32, 45, 7, 3, 7, 14, 0, 0, 0, 0, 0, 5, 6, 0, 6, 0, 0},
        {1866, (400.0/3)*7, (3/0.4)/7, 4, 4, 5, 2, 14, 14, 14, 10, 32, 46, 7, 3, 7, 14, 0, 0, 0, 0, 0, 5, 6, 0, 6, 0, 0},
        {2133, (400.0/3)*8, (3/0.4)/8, 4, 4, 6, 2, 15, 15, 15, 11, 36, 51, 8, 3, 8, 16, 0, 0, 0, 0, 0, 6, 7, 0, 7, 0, 0},
        {2133, (400.0/3)*8, (3/0.4)/8, 4, 4, 6, 2, 16, 16, 16, 11, 36, 52, 8, 3, 8, 16, 0, 0, 0, 0, 0, 6, 7, 0, 7, 0, 0},
        {2400, (400.0/3)*9, (3/0.4)/9, 4, 4, 6, 2, 16, 16, 16, 12, 39, 55, 9, 3, 9, 18, 0, 0, 0, 0, 0, 6, 8, 0, 7, 0, 0},
        {2400, (400.0/3)*9, (3/0.4)/9, 4, 4, 6, 2, 18, 18, 18, 12, 39, 57, 9, 3, 9, 18, 0, 0, 0, 0, 0, 6, 8, 0, 7, 0, 0},
        {3200, 1600, 0.625, prefetch_size/2/*DDR*/, 4,     10,   2,    22, 22,  22, 16,  56,  78, 12,  4,    12,   24, 8,    10,   40,  0,   0,    8,  10, 0,     8,     0,  0, 0, 0}
        //rate, freq, tCK,  nBL,                  nCCDS  nCCDL nRTRS  nCL nRCD nRP nCWL nRAS nRC nRTP nWTRS nWTRL nWR nRRDS nRRDL nFAW nRFC nREFI nPD nXP nXPDLL nCKESR nXS nXSDLL
    }, speed_entry;

    int read_latency;

    float act_nack_interval_ns = 0;

    void init_speed();
    void init_timing(bool per_bank_refresh = false);
    void init_prereq();

private:
    void init_lambda();
    void init_rowhit();  // SAUGATA: added function to check for row hits
    void init_rowopen();
    

    uint32_t _sa_size = 0;

};

} /*namespace ramulator*/

#endif /*__DDR4_H*/
