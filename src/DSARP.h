/*
 * DSARP.h
 *
 * This a re-implementation of the refresh mechanisms proposed in Chang et al.,
 * "Improving DRAM Performance by Parallelizing Refreshes with Accesses", HPCA
 * 2014.
 *
 * Note: the re-implementation of DSARP has not been widely tested across
 * different benchmarks and parameters. However, timing violations of
 * SARP/DSARP have been checked.
 *
 * Usage: The "type" determines the refresh mechanisms.
 * Examples:
 * DSARP::Org test_org = DSARP::Org::DSARP_8Gb_x8;
 *
 * DSARP* dsddr3_ab = new DSARP(test_org,
 * DSARP::Speed::DSARP_1333, DSARP::Type::REFAB, 8);
 *
 * DSARP* dsddr3_pb = new DSARP(test_org,
 * DSARP::Speed::DSARP_1333, DSARP::Type::REFPB, 8);
 *
 * DSARP* dsddr3_darp = new DSARP(test_org,
 * DSARP::Speed::DSARP_1333, DSARP::Type::DARP, 8);
 *
 * DSARP* dsddr3_sarp = new DSARP(test_org,
 * DSARP::Speed::DSARP_1333, DSARP::Type::SARP, 8);
 *
 * DSARP* dsddr3_dsarp = new DSARP(test_org,
 * DSARP::Speed::DSARP_1333, DSARP::Type::DSARP, 8);
 *
 *  Created on: Mar 16, 2015
 *      Author: kevincha
 */

#ifndef DSARP_H_
#define DSARP_H_

#include <vector>
#include <functional>
#include "DRAM.h"
#include "Request.h"
#include "Config.h"

using namespace std;

namespace ramulator
{

class DSARP
{
public:
    static string standard_name;
    enum class Org;
    enum class Speed;
    enum class Type;
    DSARP(Org org, Speed speed, Type type, int n_sa);
    DSARP(const string& org_str, const string& speed_str, Type type, int n_sa);
    DSARP(const Config& configs, const Type type);



    static map<string, enum Org> org_map;
    static map<string, enum Speed> speed_map;

    enum class Type : int
    {
        REFAB, REFPB, DARP, SARP, DSARP, MAX
    } type;

    /* Level */
    // NOTE: Although there's subarray, there's no SALP at all. This is used
    // for parallelizing REF and demand accesses.
    enum class Level : int
    {
      Channel, Rank, BankGroup, Bank, Subarray, Row, Column, MAX
    };

    /* Command */
    // Rank: REF - tRFCpb | Rank: PDE - 1 | Bank: REFPB - tRFCpb | SA: ACT - tRFCpb
    enum class Command : int
    {
        ACT, PRE, PREA,
        RD,  WR,  RDA,  WRA,
        REF, REFPB, PDE, PDX, SRE, SRX,
        PRE_RSQ, RSQ, ACT_NACK, ACT_PARTIAL_NACK, NOP,
        MAX
    };

    string command_name[int(Command::MAX)] = {
        "ACT", "PRE", "PREA",
        "RD",  "WR",  "RDA",  "WRA",
        "REF", "REFPB",
        "PDE", "PDX", "SRE", "SRX",
        "PRE_RSQ", "RSQ", "ACT_NACK", "ACT_PARTIAL_NACK", "NOP"
    };

    // SubArray scope for REFPB to propagate the timings
    Level scope[int(Command::MAX)] = {
        Level::Row,    Level::Bank,   Level::Rank,
        Level::Column, Level::Column, Level::Column, Level::Column,
        Level::Rank,   Level::Bank,
        Level::Rank,   Level::Rank,   Level::Rank,   Level::Rank,
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
            case int(Command::REFPB):
                return true;
            default:
                return false;
        }
    }

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
        Command::REF, Command::PDE, Command::SRE
    };

    /* Prerequisite */
    function<Command(DRAM<DSARP>*, Command cmd, int)> prereq[int(Level::MAX)][int(Command::MAX)];

    // SAUGATA: added function object container for row hit status
    /* Row hit */
    function<bool(DRAM<DSARP>*, Command cmd, int)> rowhit[int(Level::MAX)][int(Command::MAX)];
    function<bool(DRAM<DSARP>*, Command cmd, int)> rowopen[int(Level::MAX)][int(Command::MAX)];

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
    function<void(DRAM<DSARP>*, int)> lambda[int(Level::MAX)][int(Command::MAX)];

    /* Organization */
    enum class Org : int
    {
        // These are the configurations used in the original paper, essentially DDR3
        DSARP_8Gb_x4, DSARP_8Gb_x8,
        DSARP_16Gb_x4, DSARP_16Gb_x8, // x4 is broekn
        DSARP_32Gb_x8,
        MAX
    };

    struct OrgEntry {
        int size;
        int dq;
        int count[int(Level::MAX)];
    } org_table[int(Org::MAX)] = {
        // IMPORTANT: Do not change the count for channel/rank, where is set to
        // 0 now. 0 means that this a flexible configuration that is not part
        // of the spec, but rather something to change at a higher level
        // (main.cpp).
        // TODO: @Hasan please double check sizes
        //  size        dq  channel rank    bg  bank    subarray    row     column
            {8<<10,     4,  {0,     0,      4,  4,      1,          1<<17,  1<<10}}, {8<<10,  8, {0, 0, 4, 4, 1, 1<<16, 1<<10}},
            {16<<10,    4,  {0,     0,      4,  4,      0,          1<<17,  1<<11}}, {16<<10, 8, {0, 0, 4, 4, 1, 1<<17, 1<<10}},
            {32<<10,    8,  {0,     0,      4,  4,      1,          1<<18,  1<<10}},
    }, org_entry;

    void set_channel_number(int channel);
    void set_rank_number(int rank);

    void set_subarray_size(uint32_t);

    void mult_refresh_pb(float mult);

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

    uint32_t get_subarray_size() const {
        return _sa_size;
    }

    uint32_t get_num_all_banks() const {
        return org_entry.count[uint32_t(Level::Rank)] * org_entry.count[uint32_t(Level::BankGroup)] * org_entry.count[uint32_t(Level::Bank)];
    }

    /* Speed */
    enum class Speed : int
    {
        DSARP_1600,
        DSARP_2400,
        DSARP_3200,
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
        int nCL, nRCD, nRP, nRPpb, nRPab, nCWL;
        int nRAS, nRC;
        int nRTP, nWTRS, nWTRL, nWR;
        int nRRDS, nRRDL, nFAW;
        int nRFC, nRFCab, nRFCpb, nREFI, nREFIpb;
        int nPD, nXP, nXPDLL;
        int nCKESR, nXS, nXSDLL;
        int nNACK_RESEND, nACTtoNACK;
        // 34 parameters
        //int nCKE, nXP; // CKE value n/a
        //int nSR, nXSR; // tXSR = tRFCab + 7.5ns
    } speed_table[int(Speed::MAX)] = {
        // TODO: nACK_RESEND left uninitialized intentionally?
        {1600,  (400.0/3)*6,    (3/0.4)/6,  4,  4,      5,              2,      11, 11,     11,     10,     11,     9,      28,     39,     6,      2,      6,              12,     0,      0,              0,      0,      0,      0,      0,      0,      4,  5,  0,      5,      0,      0,      0},   //  DDR4-1600K
        {2400,  (400.0/3)*9,    (3/0.4)/9,  4,  4,      6,              2,      16, 16,     16,     15,     17,     12,     39,     55,     9,      3,      9,              18,     0,      0,              0,      0,      0,      0,      0,      0,      6,  8,  0,      7,      0,      0,      0},   //  DDR4-2400R
        {3200,  1600,           0.625,      4,  4,      10,             2,      22, 22,     22,     20,     22,     16,     56,     78,     12,     4,      12,             24,     8,      10,             40,     0,      0,      0,      0,      0,      8,  10, 0,      8,      0,      0,      0, 0}    //  DDR4-3200
    //  {1333,  (400.0/3)*5,    (3/0.4)/5,  4,                  4,      2,      9,  9,              8,      9,      7,      24,     33,     5,                      5,      10,                     5,      30,             0,      0,      0,      0,      4,  4,  16,     5,      114,    512}
    //  rate    freq            tCK         nBL nCCDS   nCCDL   nCCD    nRTRS   nCL nRCD    nRP     nRPpb   nRPab   nCWL    nRAS    nRC     nRTP    nWTRS   nWTRL,  nWTR    nWR     nRRDS   nRRDL   nRRD    nFAW    nRFC    nRFCab  nRFCpb  nREFI   nREFIpb nPD nXP nXPDLL  nCKESR  nXS     nXSDLL  nNACKRESEND
    }, speed_entry;

    int read_latency;

    // Refresh rank?
    bool b_ref_rank;

    // Increase RRD b/w REF and ACT when they go to the same bank (SARP)
    double nRRD_factor = 1.138;

    float act_nack_interval_ns = 0;
    void init_speed();
    void init_prereq();
    void init_timing(bool per_bank_refresh = false);

private:
    void init_lambda();
    void init_rowhit();  // SAUGATA: added function to check for row hits
    void init_rowopen();

    uint32_t _sa_size = 0;
};

} /*namespace ramulator*/

#endif /* DSARP_H_ */
