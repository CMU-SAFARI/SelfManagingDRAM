#include "DDR4.h"
#include "DRAM.h"

#include <vector>
#include <functional>
#include <cassert>

using namespace std;
using namespace ramulator;

string DDR4::standard_name = "DDR4";

map<string, enum DDR4::Org> DDR4::org_map = {
    {"DDR4_2Gb_x4", DDR4::Org::DDR4_2Gb_x4}, {"DDR4_2Gb_x8", DDR4::Org::DDR4_2Gb_x8}, {"DDR4_2Gb_x16", DDR4::Org::DDR4_2Gb_x16},
    {"DDR4_4Gb_x4", DDR4::Org::DDR4_4Gb_x4}, {"DDR4_4Gb_x8", DDR4::Org::DDR4_4Gb_x8}, {"DDR4_4Gb_x16", DDR4::Org::DDR4_4Gb_x16},
    {"DDR4_8Gb_x4", DDR4::Org::DDR4_8Gb_x4}, {"DDR4_8Gb_x8", DDR4::Org::DDR4_8Gb_x8}, {"DDR4_8Gb_x16", DDR4::Org::DDR4_8Gb_x16},
    {"DDR4_16Gb_x8", DDR4::Org::DDR4_16Gb_x8}, {"DDR4_32Gb_x8", DDR4::Org::DDR4_32Gb_x8}
};

map<string, enum DDR4::Speed> DDR4::speed_map = {
    {"DDR4_1600K", DDR4::Speed::DDR4_1600K}, {"DDR4_1600L", DDR4::Speed::DDR4_1600L},
    {"DDR4_1866M", DDR4::Speed::DDR4_1866M}, {"DDR4_1866N", DDR4::Speed::DDR4_1866N},
    {"DDR4_2133P", DDR4::Speed::DDR4_2133P}, {"DDR4_2133R", DDR4::Speed::DDR4_2133R},
    {"DDR4_2400R", DDR4::Speed::DDR4_2400R}, {"DDR4_2400U", DDR4::Speed::DDR4_2400U},
    {"DDR4_3200", DDR4::Speed::DDR4_3200},
};


DDR4::DDR4(Org org, Speed speed)
    : org_entry(org_table[int(org)]),
    speed_entry(speed_table[int(speed)]), 
    read_latency(speed_entry.nCL + speed_entry.nBL)
{
    init_speed();
    init_prereq();
    init_rowhit(); // SAUGATA: added row hit function
    init_rowopen();
    init_lambda();
    init_timing();
}

DDR4::DDR4(const string& org_str, const string& speed_str) :
    DDR4(org_map[org_str], speed_map[speed_str]) 
{
}

DDR4::DDR4(const Config& configs) : DDR4(org_map[configs["org"]], speed_map[configs["speed"]]){
   
    /* TODO: Any side effects?
    if(configs.contains("num_banks")){
        org_entry.count[int(Level::Bank)] = configs.get_int("num_banks");
    }

    if(configs.contains("num_bank_groups")) {
        org_entry.count[int(Level::BankGroup)] = configs.get_int("num_bank_groups");
    }
    */

    //assert(partial_act_pre);
    act_nack_interval_ns = configs.get_float("smd_act_nack_resend_interval");
}

void DDR4::set_channel_number(int channel) {
  org_entry.count[int(Level::Channel)] = channel;
}

void DDR4::set_rank_number(int rank) {
  org_entry.count[int(Level::Rank)] = rank;
}

void DDR4::set_subarray_size(uint32_t sa_size) {
    uint rows_per_bank = org_entry.count[int(Level::Subarray)] * org_entry.count[int(Level::Row)];
    _sa_size = sa_size;

    assert((rows_per_bank % _sa_size == 0) && "ERROR: Rows per bank must be multiple of subarray size. Make sure you set the subarray size properly.");
    org_entry.count[int(Level::Subarray)] = rows_per_bank/_sa_size;
    org_entry.count[int(Level::Row)] = _sa_size;
}

void DDR4::init_speed()
{
    const static int RRDS_TABLE[2][5] = {
        {4, 4, 4, 4, 4},
        {5, 5, 6, 7, 9}
    };
    const static int RRDL_TABLE[2][5] = {
        {5, 5, 6, 6, 8},
        {6, 6, 7, 8, 11}
    };
    const static int FAW_TABLE[3][5] = {
        {16, 16, 16, 16, 16},
        {20, 22, 23, 26, 34},
        {28, 28, 32, 36, 48}
    };
    const static int RFC_TABLE[int(RefreshMode::MAX)][5][5] = {{
            //1600 1866 2133 2400 3200
            {128, 150, 171, 192, 256}, // 2 Gb
            {208, 243, 278, 312, 416}, // 4 Gb
            {280, 327, 374, 420, 560}, // 8 Gb
            {280, 327, 374, 420, 560}, // 16 Gb
            {280, 327, 374, 420, 560}, // 32 Gb (UPDATE: when specification for 32Gb devices is out)
        },{
            {88, 103, 118, 132,  176}, // 2 Gb
            {128, 150, 171, 192, 256}, // 4 Gb
            {208, 243, 278, 312, 416}, // 8 Gb
            {208, 243, 278, 312, 416}, // 16 Gb
            {208, 243, 278, 312, 416}, // 32 Gb (UPDATE: when specification for 32Gb devices is out)
        },{
            {72, 84, 96, 108, 144}, // 2 Gb
            {88, 103, 118, 132, 176}, // 4 Gb
            {128, 150, 171, 192, 256}, // 8 Gb
            {128, 150, 171, 192, 256}, // 16 Gb
            {128, 150, 171, 192, 256}, // 32 Gb (UPDATE: when specification for 32Gb devices is out)
        }
    };
    const static int REFI_TABLE[5] = {
        6240, 7280, 8320, 9360, 12480
    };
    const static int XS_TABLE[5][5] = {
        {136, 159, 182, 204, 272}, // 2 Gb
        {216, 252, 288, 324, 532}, // 4 Gb
        {288, 336, 384, 432, 576}, // 8 Gb
        {288, 336, 384, 432, 576}, // 16 Gb
        {288, 336, 384, 432, 576}, // 32 Gb (UPDATE: when specification for 32Gb devices is out)
    };

    int speed = 0, density = 0;
    switch (speed_entry.rate) {
        case 1600: speed = 0; break;
        case 1866: speed = 1; break;
        case 2133: speed = 2; break;
        case 2400: speed = 3; break;
        case 3200: speed = 4; break;
        default: assert(false);
    };
    switch (org_entry.size >> 10){
        case 2: density = 0; break;
        case 4: density = 1; break;
        case 8: density = 2; break;
        case 16: density = 3; break;
        case 32: density = 4; break;
        default: assert(false);
    }
    speed_entry.nRRDS = RRDS_TABLE[org_entry.dq == 16? 1: 0][speed];
    speed_entry.nRRDL = RRDL_TABLE[org_entry.dq == 16? 1: 0][speed];
    speed_entry.nFAW = FAW_TABLE[org_entry.dq == 4? 0: org_entry.dq == 8? 1: 2][speed];
    speed_entry.nRFC = RFC_TABLE[(int)refresh_mode][density][speed];
    speed_entry.nREFI = (REFI_TABLE[speed] >> int(refresh_mode));
    speed_entry.nXS = XS_TABLE[density][speed];

    speed_entry.nNACK_RESEND = ceil(act_nack_interval_ns/speed_entry.tCK);
}


void DDR4::init_prereq()
{
    // RD
    prereq[int(Level::Rank)][int(Command::RD)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp): return Command::MAX;
            case int(State::ActPowerDown): return Command::PDX;
            case int(State::PrePowerDown): return Command::PDX;
            case int(State::SelfRefresh): return Command::SRX;
            default: assert(false);
        }};


    prereq[int(Level::Bank)][int(Command::RD)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::Closed): return Command::ACT;
            case int(State::Opened):
                if (node->row_state.find(id) != node->row_state.end())
                    return Command::MAX; // the correct SA is open (or partially open) but is the correct row open? Checked in the function below
                else return Command::PRE; 
            case int(State::PartiallyOpened):
                if (node->row_state.find(id) != node->row_state.end())
                    return Command::MAX; // the correct SA is open (or partially open) but is the correct row open? Checked in the function below
                // else return Command::PRE; // Hasan: here we probably need to return PRE when using the policy that precharges a partially-open row to attempt activating another one
                else return Command::NOP; // Hasan: here we probably need to return PRE when using the policy that precharges a partially-open row to attempt activating another one
            default: assert(false);
        }};

    prereq[int(Level::Subarray)][int(Command::RD)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::Closed): return Command::PRE; // the target bank is active but not the target subarray (meaning another subarray is active) so need to precharge
            case int(State::PartiallyOpened): {
                if (node->row_state.find(id) != node->row_state.end())
                    return Command::ACT; // the target row is partially open, try fully opening it
                else return Command::PRE;
            }
            case int(State::Opened):
                if (node->row_state.find(id) != node->row_state.end())
                    return cmd;
                else return Command::PRE;
            default: assert(false);
        }};
    
    // WR
    prereq[int(Level::Rank)][int(Command::WR)] = prereq[int(Level::Rank)][int(Command::RD)];
    prereq[int(Level::Bank)][int(Command::WR)] = prereq[int(Level::Bank)][int(Command::RD)];
    prereq[int(Level::Subarray)][int(Command::WR)] = prereq[int(Level::Subarray)][int(Command::RD)];

    // REF
    prereq[int(Level::Rank)][int(Command::REF)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        for (auto bg : node->children)
            for (auto bank: bg->children) {
                if (bank->state == State::Closed)
                    continue;
                return Command::PREA;
            }
        return Command::REF;};

    // REF - used when per_bank_refresh_on
    prereq[int(Level::Bank)][int(Command::REF)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        if (node->state == State::Closed)
            return Command::REF;
        return Command::PRE;};

    // PD
    prereq[int(Level::Rank)][int(Command::PDE)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp): return Command::PDE;
            case int(State::ActPowerDown): return Command::PDE;
            case int(State::PrePowerDown): return Command::PDE;
            case int(State::SelfRefresh): return Command::SRX;
            default: assert(false);
        }};

    // SR
    prereq[int(Level::Rank)][int(Command::SRE)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp): return Command::SRE;
            case int(State::ActPowerDown): return Command::PDX;
            case int(State::PrePowerDown): return Command::PDX;
            case int(State::SelfRefresh): return Command::SRE;
            default: assert(false);
        }};
}

// SAUGATA: added row hit check functions to see if the desired location is currently open
void DDR4::init_rowhit()
{
    // RD
    rowhit[int(Level::Subarray)][int(Command::RD)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PartiallyOpened):
            case int(State::Closed): return false;
            case int(State::Opened):
                if (node->row_state.find(id) != node->row_state.end())
                    return true;
                return false;
            default: assert(false);
        }};

    // WR
    rowhit[int(Level::Subarray)][int(Command::WR)] = rowhit[int(Level::Subarray)][int(Command::RD)];
}

void DDR4::init_rowopen()
{
    // RD
    rowopen[int(Level::Bank)][int(Command::RD)] = [] (DRAM<DDR4>* node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PartiallyOpened):
            case int(State::Closed): return false;
            case int(State::Opened): return true;
            default: assert(false);
        }};

    // WR
    rowopen[int(Level::Bank)][int(Command::WR)] = rowopen[int(Level::Bank)][int(Command::RD)];
}

void DDR4::init_lambda()
{
    lambda[int(Level::Bank)][int(Command::ACT)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::Opened;
        node->just_opened = true; // Hasan
        node->row_state[id] = State::Opened;}; // Hasan: row_state is more like child_state now
    
    lambda[int(Level::Subarray)][int(Command::ACT)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::Opened;
        // node->just_opened = true; // Hasan
        node->row_state[id] = State::Opened;};


    lambda[int(Level::Bank)][int(Command::PRE)] = [] (DRAM<DDR4>* node, int id) {
        // assert(node->state != State::PartiallyOpened); // DEBUG

        assert(node->state == State::Opened || node->state == State::PartiallyOpened);
        assert(node->row_state.begin() != node->row_state.end());
        auto open_SA = node->children[node->row_state.begin()->first];
        assert(open_SA->state == State::Opened || open_SA->state == State::PartiallyOpened);

        node->collect_opened_cycles(node->cur_clk); // Hasan
        open_SA->state = State::Closed;
        node->state = State::Closed;
        open_SA->row_state.clear();
        node->row_state.clear();};


    lambda[int(Level::Bank)][int(Command::ACT_NACK)] = [] (DRAM<DDR4>* node, int id) {
        assert(node->state == State::Opened);
        assert(node->row_state.begin() != node->row_state.end());
        auto open_SA = node->children[node->row_state.begin()->first];
        assert(open_SA->state == State::Opened);

        open_SA->state = State::Closed;
        node->state = State::Closed;
        open_SA->row_state.clear();
        node->row_state.clear();};

    lambda[int(Level::Bank)][int(Command::ACT_PARTIAL_NACK)] = [] (DRAM<DDR4>* node, int id) {
        assert(node->state == State::Opened);
        assert(node->row_state.begin() != node->row_state.end());
        auto open_SA = node->children[node->row_state.begin()->first];
        assert(open_SA->state == State::Opened);

        open_SA->state = State::PartiallyOpened;
        node->state = State::PartiallyOpened;
    };

    lambda[int(Level::Bank)][int(Command::PRE_RSQ)] = [this] (DRAM<DDR4>* node, int id) {
        lambda[int(Level::Bank)][int(Command::PRE)](node, id);
        };
    lambda[int(Level::Bank)][int(Command::RSQ)] = [] (DRAM<DDR4>* node, int id) {};

    lambda[int(Level::Rank)][int(Command::PREA)] = [] (DRAM<DDR4>* node, int id) {
        for (auto bg : node->children)
            for (auto bank : bg->children) {
                if(bank->state == State::Opened) {
                    bank->collect_opened_cycles(node->cur_clk); // Hasan

                    if (bank->row_state.begin() != bank->row_state.end()) {
                        auto open_SA = bank->children[bank->row_state.begin()->first];
                        assert(open_SA->state == State::Opened);

                        open_SA->state = State::Closed;
                        open_SA->row_state.clear();
                    }       
                }
                bank->state = State::Closed;
                bank->row_state.clear();
            }};
    lambda[int(Level::Rank)][int(Command::REF)] = [] (DRAM<DDR4>* node, int id) {};
    lambda[int(Level::Bank)][int(Command::RD)] = [] (DRAM<DDR4>* node, int id) {
        node->just_opened = false; // Hasan
    };
    lambda[int(Level::Bank)][int(Command::WR)] = [] (DRAM<DDR4>* node, int id) {
        node->just_opened = false; // Hasan
    };
    lambda[int(Level::Bank)][int(Command::RDA)] = [] (DRAM<DDR4>* node, int id) {
        assert(node->state == State::Opened);
        node->just_opened = false; // Hasan
        node->collect_opened_cycles(node->cur_clk); // Hasan
        node->state = State::Closed;

        assert(node->row_state.begin() != node->row_state.end());
        auto open_SA = node->children[node->row_state.begin()->first];
        assert(open_SA->state == State::Opened);
        open_SA->state = State::Closed;
        open_SA->row_state.clear();
        node->row_state.clear();
        };
    lambda[int(Level::Bank)][int(Command::WRA)] = [] (DRAM<DDR4>* node, int id) {
        assert(node->state == State::Opened);
        node->just_opened = false; // Hasan
        node->collect_opened_cycles(node->cur_clk); // Hasan
        node->state = State::Closed;

        assert(node->row_state.begin() != node->row_state.end());
        auto open_SA = node->children[node->row_state.begin()->first];
        assert(open_SA->state == State::Opened);
        open_SA->state = State::Closed;
        open_SA->row_state.clear();
        node->row_state.clear();
        };
    lambda[int(Level::Rank)][int(Command::PDE)] = [] (DRAM<DDR4>* node, int id) {
        for (auto bg : node->children)
            for (auto bank : bg->children) {
                if (bank->state == State::Closed)
                    continue;
                node->state = State::ActPowerDown;
                return;
            }
        node->state = State::PrePowerDown;};
    lambda[int(Level::Rank)][int(Command::PDX)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::PowerUp;};
    lambda[int(Level::Rank)][int(Command::SRE)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::SelfRefresh;};
    lambda[int(Level::Rank)][int(Command::SRX)] = [] (DRAM<DDR4>* node, int id) {
        node->state = State::PowerUp;};
}


void DDR4::init_timing(bool per_bank_refresh)
{
    // clear current timing entries
    for (auto& v : timing){
        for (auto& v2 : v){
            v2.clear();
        }
    }
    
    SpeedEntry& s = speed_entry;
    vector<TimingEntry> *t;

    /*** Channel ***/ 
    t = timing[int(Level::Channel)];

    // CAS <-> CAS
    t[int(Command::RD)].push_back({Command::RD, 1, s.nBL});
    t[int(Command::RD)].push_back({Command::RDA, 1, s.nBL});
    t[int(Command::RDA)].push_back({Command::RD, 1, s.nBL});
    t[int(Command::RDA)].push_back({Command::RDA, 1, s.nBL});
    t[int(Command::WR)].push_back({Command::WR, 1, s.nBL});
    t[int(Command::WR)].push_back({Command::WRA, 1, s.nBL});
    t[int(Command::WRA)].push_back({Command::WR, 1, s.nBL});
    t[int(Command::WRA)].push_back({Command::WRA, 1, s.nBL});

    t[int(Command::PRE_RSQ)].push_back({Command::RD, 1, s.nBL});
    t[int(Command::PRE_RSQ)].push_back({Command::RDA, 1, s.nBL});
    t[int(Command::PRE_RSQ)].push_back({Command::WR, 1, s.nBL});
    t[int(Command::PRE_RSQ)].push_back({Command::WRA, 1, s.nBL});
    t[int(Command::RSQ)].push_back({Command::RD, 1, s.nBL});
    t[int(Command::RSQ)].push_back({Command::RDA, 1, s.nBL});
    t[int(Command::RSQ)].push_back({Command::WR, 1, s.nBL});
    t[int(Command::RSQ)].push_back({Command::WRA, 1, s.nBL});

    t[int(Command::RD)].push_back({Command::PRE_RSQ, 1, s.nBL});
    t[int(Command::RDA)].push_back({Command::PRE_RSQ, 1, s.nBL});
    t[int(Command::WR)].push_back({Command::PRE_RSQ, 1, s.nBL});
    t[int(Command::WRA)].push_back({Command::PRE_RSQ, 1, s.nBL});
    t[int(Command::RD)].push_back({Command::RSQ, 1, s.nBL});
    t[int(Command::RDA)].push_back({Command::RSQ, 1, s.nBL});
    t[int(Command::WR)].push_back({Command::RSQ, 1, s.nBL});
    t[int(Command::WRA)].push_back({Command::RSQ, 1, s.nBL});


    /*** Rank ***/ 
    t = timing[int(Level::Rank)];

    // CAS <-> CAS
    t[int(Command::RD)].push_back({Command::RD, 1, s.nCCDS});
    t[int(Command::RD)].push_back({Command::RDA, 1, s.nCCDS});
    t[int(Command::RDA)].push_back({Command::RD, 1, s.nCCDS});
    t[int(Command::RDA)].push_back({Command::RDA, 1, s.nCCDS});
    t[int(Command::WR)].push_back({Command::WR, 1, s.nCCDS});
    t[int(Command::WR)].push_back({Command::WRA, 1, s.nCCDS});
    t[int(Command::WRA)].push_back({Command::WR, 1, s.nCCDS});
    t[int(Command::WRA)].push_back({Command::WRA, 1, s.nCCDS});
    t[int(Command::RD)].push_back({Command::WR, 1, s.nCL + s.nCCDS + 2 - s.nCWL});
    t[int(Command::RD)].push_back({Command::WRA, 1, s.nCL + s.nCCDS + 2 - s.nCWL});
    t[int(Command::RDA)].push_back({Command::WR, 1, s.nCL + s.nCCDS + 2 - s.nCWL});
    t[int(Command::RDA)].push_back({Command::WRA, 1, s.nCL + s.nCCDS + 2 - s.nCWL});
    t[int(Command::WR)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTRS});
    t[int(Command::WR)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTRS});
    t[int(Command::WRA)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTRS});
    t[int(Command::WRA)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTRS});

    t[int(Command::PRE_RSQ)].push_back({Command::RD, 1, s.nCCDS});
    t[int(Command::PRE_RSQ)].push_back({Command::RDA, 1, s.nCCDS});
    t[int(Command::RD)].push_back({Command::PRE_RSQ, 1, s.nCCDS});
    t[int(Command::RDA)].push_back({Command::PRE_RSQ, 1, s.nCCDS});
    t[int(Command::PRE_RSQ)].push_back({Command::WR, 1, s.nCCDS});
    t[int(Command::PRE_RSQ)].push_back({Command::WRA, 1, s.nCCDS});
    t[int(Command::WR)].push_back({Command::PRE_RSQ, 1, s.nCCDS});
    t[int(Command::WRA)].push_back({Command::PRE_RSQ, 1, s.nCCDS});

    t[int(Command::RSQ)].push_back({Command::RD, 1, s.nCCDS});
    t[int(Command::RSQ)].push_back({Command::RDA, 1, s.nCCDS});
    t[int(Command::RD)].push_back({Command::RSQ, 1, s.nCCDS});
    t[int(Command::RDA)].push_back({Command::RSQ, 1, s.nCCDS});
    t[int(Command::RSQ)].push_back({Command::WR, 1, s.nCCDS});
    t[int(Command::RSQ)].push_back({Command::WRA, 1, s.nCCDS});
    t[int(Command::WR)].push_back({Command::RSQ, 1, s.nCCDS});
    t[int(Command::WRA)].push_back({Command::RSQ, 1, s.nCCDS});



    // CAS <-> CAS (between sibling ranks)
    t[int(Command::RD)].push_back({Command::RD, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::RDA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::RD, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::RDA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::WR, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::WRA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::WR, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::WRA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::WR, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::RD)].push_back({Command::WRA, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::RDA)].push_back({Command::WR, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::RDA)].push_back({Command::WRA, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::WR)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});
    t[int(Command::WR)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});
    t[int(Command::WRA)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});
    t[int(Command::WRA)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});

    t[int(Command::RD)].push_back({Command::PREA, 1, s.nRTP});
    t[int(Command::WR)].push_back({Command::PREA, 1, s.nCWL + s.nBL + s.nWR});

    t[int(Command::PRE_RSQ)].push_back({Command::RD, 1, s.nBL + s.nRTRS, true});
    t[int(Command::PRE_RSQ)].push_back({Command::RDA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::PRE_RSQ, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::PRE_RSQ, 1, s.nBL + s.nRTRS, true});
    t[int(Command::PRE_RSQ)].push_back({Command::WR, 1, s.nBL + s.nRTRS, true});
    t[int(Command::PRE_RSQ)].push_back({Command::WRA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::WR)].push_back({Command::PRE_RSQ, 1, s.nBL + s.nRTRS, true});
    t[int(Command::WRA)].push_back({Command::PRE_RSQ, 1, s.nBL + s.nRTRS, true});

    t[int(Command::RSQ)].push_back({Command::RD, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RSQ)].push_back({Command::RDA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::RSQ, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::RSQ, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RSQ)].push_back({Command::WR, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RSQ)].push_back({Command::WRA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::WR)].push_back({Command::RSQ, 1, s.nBL + s.nRTRS, true});
    t[int(Command::WRA)].push_back({Command::RSQ, 1, s.nBL + s.nRTRS, true});


    // CAS <-> PD
    t[int(Command::RD)].push_back({Command::PDE, 1, s.nCL + s.nBL + 1});
    t[int(Command::RDA)].push_back({Command::PDE, 1, s.nCL + s.nBL + 1});
    t[int(Command::WR)].push_back({Command::PDE, 1, s.nCWL + s.nBL + s.nWR});
    t[int(Command::WRA)].push_back({Command::PDE, 1, s.nCWL + s.nBL + s.nWR + 1}); // +1 for pre
    t[int(Command::PDX)].push_back({Command::RD, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::RDA, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::WR, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::WRA, 1, s.nXP});
    
    // CAS <-> SR: none (all banks have to be precharged)

    // RAS <-> RAS
    t[int(Command::ACT)].push_back({Command::ACT, 1, s.nRRDS});
    t[int(Command::ACT)].push_back({Command::ACT, 4, s.nFAW});
    t[int(Command::ACT)].push_back({Command::PREA, 1, s.nRAS});
    t[int(Command::PREA)].push_back({Command::ACT, 1, s.nRP});

    t[int(Command::ACT_PARTIAL_NACK)].push_back({Command::PREA, 1, s.nRAS - s.nACTtoNACK});

    // RAS <-> REF
    if(!per_bank_refresh) {
        t[int(Command::PRE)].push_back({Command::REF, 1, s.nRP});
        t[int(Command::PREA)].push_back({Command::REF, 1, s.nRP});
        t[int(Command::REF)].push_back({Command::ACT, 1, s.nRFC});

        t[int(Command::PRE_RSQ)].push_back({Command::REF, 1, s.nRP});
    }

    // RAS <-> PD
    t[int(Command::ACT)].push_back({Command::PDE, 1, 1});
    t[int(Command::PDX)].push_back({Command::ACT, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::PRE, 1, s.nXP});
    t[int(Command::PDX)].push_back({Command::PREA, 1, s.nXP});

    // RAS <-> SR
    t[int(Command::PRE)].push_back({Command::SRE, 1, s.nRP});
    t[int(Command::PREA)].push_back({Command::SRE, 1, s.nRP});
    t[int(Command::SRX)].push_back({Command::ACT, 1, s.nXS});

    // REF <-> REF
    if(!per_bank_refresh)
        t[int(Command::REF)].push_back({Command::REF, 1, s.nRFC});

    // REF <-> PD
    t[int(Command::REF)].push_back({Command::PDE, 1, 1});
    t[int(Command::PDX)].push_back({Command::REF, 1, s.nXP});

    // REF <-> SR
    t[int(Command::SRX)].push_back({Command::REF, 1, s.nXS});
    
    // PD <-> PD
    t[int(Command::PDE)].push_back({Command::PDX, 1, s.nPD});
    t[int(Command::PDX)].push_back({Command::PDE, 1, s.nXP});

    // PD <-> SR
    t[int(Command::PDX)].push_back({Command::SRE, 1, s.nXP});
    t[int(Command::SRX)].push_back({Command::PDE, 1, s.nXS});
    
    // SR <-> SR
    t[int(Command::SRE)].push_back({Command::SRX, 1, s.nCKESR});
    t[int(Command::SRX)].push_back({Command::SRE, 1, s.nXS});

    /*** Bank Group ***/ 
    t = timing[int(Level::BankGroup)];
    // CAS <-> CAS
    t[int(Command::RD)].push_back({Command::RD, 1, s.nCCDL});
    t[int(Command::RD)].push_back({Command::RDA, 1, s.nCCDL});
    t[int(Command::RDA)].push_back({Command::RD, 1, s.nCCDL});
    t[int(Command::RDA)].push_back({Command::RDA, 1, s.nCCDL});
    t[int(Command::WR)].push_back({Command::WR, 1, s.nCCDL});
    t[int(Command::WR)].push_back({Command::WRA, 1, s.nCCDL});
    t[int(Command::WRA)].push_back({Command::WR, 1, s.nCCDL});
    t[int(Command::WRA)].push_back({Command::WRA, 1, s.nCCDL});
    t[int(Command::WR)].push_back({Command::WR, 1, s.nCCDL});
    t[int(Command::WR)].push_back({Command::WRA, 1, s.nCCDL});
    t[int(Command::WRA)].push_back({Command::WR, 1, s.nCCDL});
    t[int(Command::WRA)].push_back({Command::WRA, 1, s.nCCDL});
    t[int(Command::WR)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTRL});
    t[int(Command::WR)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTRL});
    t[int(Command::WRA)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTRL});
    t[int(Command::WRA)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTRL});


    // RAS <-> RAS
    t[int(Command::ACT)].push_back({Command::ACT, 1, s.nRRDL});

    /*** Bank ***/ 
    t = timing[int(Level::Bank)];

    // CAS <-> RAS
    t[int(Command::ACT)].push_back({Command::RD, 1, s.nRCD});
    t[int(Command::ACT)].push_back({Command::RDA, 1, s.nRCD});
    t[int(Command::ACT)].push_back({Command::WR, 1, s.nRCD});
    t[int(Command::ACT)].push_back({Command::WRA, 1, s.nRCD});

    t[int(Command::RD)].push_back({Command::PRE, 1, s.nRTP});
    t[int(Command::WR)].push_back({Command::PRE, 1, s.nCWL + s.nBL + s.nWR});

    t[int(Command::RDA)].push_back({Command::ACT, 1, s.nRTP + s.nRP});
    t[int(Command::WRA)].push_back({Command::ACT, 1, s.nCWL + s.nBL + s.nWR + s.nRP});

    // RAS <-> RAS
    // t[int(Command::ACT)].push_back({Command::ACT, 1, s.nRC}); // Hasan: This is not really needed as nRAS and nPRE enforce this timing anyway. Removing this as it creates problems with ACT_NACK commands in SMD
    t[int(Command::ACT)].push_back({Command::PRE, 1, s.nRAS});
    t[int(Command::PRE)].push_back({Command::ACT, 1, s.nRP});

    t[int(Command::ACT)].push_back({Command::PRE_RSQ, 1, s.nRAS});
    t[int(Command::PRE_RSQ)].push_back({Command::ACT, 1, s.nRP});

    t[int(Command::ACT_PARTIAL_NACK)].push_back({Command::PRE, 1, s.nRAS - s.nACTtoNACK});

    if(per_bank_refresh) {
        t[int(Command::PRE)].push_back({Command::REF, 1, s.nRP});
        t[int(Command::PREA)].push_back({Command::REF, 1, s.nRP});
        t[int(Command::REF)].push_back({Command::ACT, 1, s.nRFC});

        t[int(Command::PRE_RSQ)].push_back({Command::REF, 1, s.nRP});

        t[int(Command::REF)].push_back({Command::REF, 1, s.nRFC});
    }


    /*** Subarray ***/ 
    t = timing[int(Level::Subarray)];

    t[int(Command::ACT_NACK)].push_back({Command::ACT, 1, s.nNACK_RESEND});
    t[int(Command::ACT_PARTIAL_NACK)].push_back({Command::ACT, 1, s.nNACK_RESEND});
}
