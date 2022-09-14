/*
 * DSARP.cpp
 *
 * This a re-implementation of the refresh mechanisms proposed in Chang et al.,
 * "Improving DRAM Performance by Parallelizing Refreshes with Accesses", HPCA
 * 2014.
 *
 *  Created on: Mar 16, 2015
 *      Author: kevincha
 */

#include <vector>
#include <functional>
#include <cassert>
#include <math.h>
#include "DSARP.h"
#include "DRAM.h"

using namespace std;
using namespace ramulator;

string DSARP::standard_name = "DSARP";

map<string, enum DSARP::Org> DSARP::org_map = {
  {"DSARP_8Gb_x4", DSARP::Org::DSARP_8Gb_x4}, {"DSARP_8Gb_x8", DSARP::Org::DSARP_8Gb_x8},
  {"DSARP_16Gb_x4", DSARP::Org::DSARP_16Gb_x4}, {"DSARP_16Gb_x8", DSARP::Org::DSARP_16Gb_x8},
  {"DSARP_32Gb_x8", DSARP::Org::DSARP_32Gb_x8},
};

map<string, enum DSARP::Speed> DSARP::speed_map = {
  {"DSARP_1600", DSARP::Speed::DSARP_1600},
  {"DSARP_2400", DSARP::Speed::DSARP_2400},
  {"DSARP_3200", DSARP::Speed::DSARP_3200},
};

DSARP::DSARP(Org org, Speed speed, Type type, int n_sa) :
  type(type),
  org_entry(org_table[int(org)]),
  speed_entry(speed_table[int(speed)]),
  read_latency(speed_entry.nCL + speed_entry.nBL),
  _sa_size(n_sa)
{
  init_speed();
  init_prereq();
  init_rowhit(); // SAUGATA: added row hit function
  init_rowopen();
  init_lambda();
  init_timing();

  // All mechanisms are built on top of REFpb, except for REFab
  b_ref_rank = false;
  switch(int(type)){
    case int(Type::REFAB):
      standard_name = "REFAB";
      b_ref_rank = true;
      break;
    case int(Type::REFPB): standard_name = "REFPB"; break;
    case int(Type::DARP):  standard_name = "DARP"; break;
    case int(Type::SARP):  standard_name = "SARP"; break;
    case int(Type::DSARP): standard_name = "DSARP"; break;
  }

  // Update the SA count (is power of 2, within [1, 128]) and row count
  // TODO: it should be okay to comment these out... but is it?
  //assert(n_sa && n_sa <= 128 && (n_sa & (n_sa-1)) == 0);
  //org_entry.count[int(Level::Subarray)] = n_sa;
  //long tmp = long(org_entry.dq) * org_entry.count[int(Level::Bank)] *
  //  n_sa * org_entry.count[int(Level::Column)];
  //org_entry.count[int(Level::Row)] = long(org_entry.size) * (1<<20) / tmp;

  // Change the translation for refresh requests
  if (!b_ref_rank)
    translate[int(Request::Type::REFRESH)] = Command::REFPB;
}

DSARP::DSARP(const string& org_str, const string& speed_str, Type type, int n_sa) :
  DSARP(org_map[org_str], speed_map[speed_str], type, n_sa) {}

DSARP::DSARP(const Config& configs, const Type type) : DSARP(org_map[configs["org"]], speed_map[configs["speed"]], type, 0)
{
    if(configs.contains("num_banks")){
        org_entry.count[int(Level::Bank)] = configs.get_int("num_banks");
    }

    if(configs.contains("num_bank_groups")) {
        org_entry.count[int(Level::BankGroup)] = configs.get_int("num_bank_groups");
    }

    act_nack_interval_ns = configs.get_float("smd_act_nack_resend_interval");
}


void DSARP::set_channel_number(int channel) {
  org_entry.count[int(Level::Channel)] = channel;
}

void DSARP::set_rank_number(int rank) {
  org_entry.count[int(Level::Rank)] = rank;
}

void DSARP::set_subarray_size(uint32_t sa_size) {
    uint rows_per_bank = org_entry.count[int(Level::Subarray)] * org_entry.count[int(Level::Row)];
    _sa_size = sa_size;

    assert((rows_per_bank % _sa_size == 0) && "ERROR: Rows per bank must be multiple of subarray size. Make sure you set the subarray size properly.");
    org_entry.count[int(Level::Subarray)] = rows_per_bank/_sa_size;
    org_entry.count[int(Level::Row)] = _sa_size;
}

void DSARP::mult_refresh_pb(float mult)
{
  speed_entry.nREFIpb *= mult;
  //speed_entry.nRFCpb /= 2;
}

void DSARP::init_speed()
{
  /* Numbers are in DRAM cycles */

  // The numbers for RFCab are extrapolated based on past and current DRAM
  // generation since they are not available yet. Details on the extrapolation
  // are in the paper.
  // TODO: Check if these are the values we'd want to use
  const static int RFCAB_TABLE[int(RefreshMode::MAX)][int(Org::MAX)][int(Speed::MAX)] = {
    {
      {280, 420, 560}, 
      {280, 420, 560}, 
      {280, 420, 560}, 
      {280, 420, 560}, 
      {280, 420, 560}
    },
    {
      {208, 312, 416}, 
      {208, 312, 416}, 
      {208, 312, 416}, 
      {208, 312, 416}, 
      {208, 312, 416}
    },
    {
      {128, 192, 256}, 
      {128, 192, 256}, 
      {128, 192, 256}, 
      {128, 192, 256}, 
      {128, 192, 256}
    },
  };

  // ratio between RFCAB/RFCPB 0.465811965811966

  // These are extrapolated using the RFCab/REFpb ratio from the LPDDR standard, which is 2.16.
  const static int RFCPB_TABLE[int(RefreshMode::MAX)][int(Org::MAX)][int(Speed::MAX)] = {
    {
      {131, 196, 261}, 
      {131, 196, 261}, 
      {131, 196, 261}, 
      {131, 196, 261}, 
      {131, 196, 261}
    },
    {
      {97, 146, 194}, 
      {97, 146, 194}, 
      {97, 146, 194}, 
      {97, 146, 194}, 
      {97, 146, 194}
    },
    {
      {60, 90, 120}, 
      {60, 90, 120}, 
      {60, 90, 120}, 
      {60, 90, 120}, 
      {60, 90, 120}
    },
  };

  // TODO: double check if these need to change at all
  // High temperature mode (32ms retention time)
  const static int REFI_TABLE[int(Speed::MAX)] = 
    {6240, 9360, 12480};

  const static int REFIPB_TABLE[int(Speed::MAX)] = 
    {390, 585, 780};

  const static int RRDS_TABLE[2][5] = {
      {4, 4, 4},
      {5, 7, 9}
  };
  const static int RRDL_TABLE[2][5] = {
      {5, 6, 8},
      {6, 8, 11}
  };
  const static int FAW_TABLE[3][5] = {
      {16, 16, 16},
      {20, 26, 34},
      {28, 36, 48}
  };

  int speed = 0, density = 0;
  switch (speed_entry.rate) {
    case 1600: speed = 0; break;
    case 2400: speed = 1; break;
    case 3200: speed = 2; break;
    default: assert(false);
  };
  switch (org_entry.size >> 10){
    case 8: density = 0; break;
    case 16: density = 1; break;
    case 32: density = 2; break;
    default: assert(false && "Unknown density");
  }

  // TODO: Make sure to fix nRFC
  speed_entry.nRRDS = RRDS_TABLE[org_entry.dq == 16? 1: 0][speed];
  speed_entry.nRRDL = RRDL_TABLE[org_entry.dq == 16? 1: 0][speed];
  speed_entry.nFAW = FAW_TABLE[org_entry.dq == 4? 0: org_entry.dq == 8? 1: 2][speed];
  speed_entry.nRFC = RFCPB_TABLE[(int)refresh_mode][density][speed];
  speed_entry.nRFCpb = RFCPB_TABLE[(int)refresh_mode][density][speed];
  speed_entry.nRFCab = RFCAB_TABLE[(int)refresh_mode][density][speed];
  speed_entry.nREFI = REFI_TABLE[speed] >> int(refresh_mode);
  speed_entry.nREFIpb = REFIPB_TABLE[speed] >> int(refresh_mode);
}

void DSARP::init_prereq()
{
  // RD
  prereq[int(Level::Rank)][int(Command::RD)] = [] (DRAM<DSARP>* node, Command cmd, int id) {
    switch (int(node->state)) {
      case int(State::PowerUp): return Command::MAX;
      case int(State::ActPowerDown): return Command::PDX;
      case int(State::PrePowerDown): return Command::PDX;
      case int(State::SelfRefresh): return Command::SRX;
      default: assert(false);
    }};
  // Rank transitions to Bank
  prereq[int(Level::Bank)][int(Command::RD)] = [] (DRAM<DSARP>* node, Command cmd, int id) {
    switch (int(node->state)) {
      case int(State::Closed): return Command::ACT;
      case int(State::Opened):
        // Really is the subarray state. If the subarray matches, check the row ID
        if (node->row_state.find(id) != node->row_state.end())
          return Command::MAX;
        return Command::PRE;
      default: assert(false);
    }};
  // Bank transitions to Subarray
  prereq[int(Level::Subarray)][int(Command::RD)] = [] (DRAM<DSARP>* node, Command cmd, int id) {
    switch (int(node->state)) {
      case int(State::Closed): return Command::ACT;
      case int(State::Opened):
        // Actual row state
        if (node->row_state.find(id) != node->row_state.end())
          return cmd;
        return Command::PRE;
      default: assert(false);
    }};

  // WR
  prereq[int(Level::Rank)][int(Command::WR)] = prereq[int(Level::Rank)][int(Command::RD)];
  prereq[int(Level::Bank)][int(Command::WR)] = prereq[int(Level::Bank)][int(Command::RD)];
  prereq[int(Level::Subarray)][int(Command::WR)] = prereq[int(Level::Subarray)][int(Command::RD)];

  // REF -- on all banks
  prereq[int(Level::Rank)][int(Command::REF)] = [] (DRAM<DSARP>* node, Command cmd, int id) {
    for (auto bg : node->children) {
      for (auto bank : bg->children) {
        if (bank->state == State::Closed)
          continue;
        return Command::PREA;
      }
    }
    return Command::REF;};

  // REF -- per bank
  prereq[int(Level::Bank)][int(Command::REFPB)] = [] (DRAM<DSARP>* node, Command cmd, int id) {
    if (node->state == State::Closed) return Command::REFPB;
    else return Command::PRE;};

  // PD
  prereq[int(Level::Rank)][int(Command::PDE)] = [] (DRAM<DSARP>* node, Command cmd, int id) {
    switch (int(node->state)) {
      case int(State::PowerUp): return Command::PDE;
      case int(State::ActPowerDown): return Command::PDE;
      case int(State::PrePowerDown): return Command::PDE;
      case int(State::SelfRefresh): return Command::SRX;
      default: assert(false);
    }};

  // SR
  prereq[int(Level::Rank)][int(Command::SRE)] = [] (DRAM<DSARP>* node, Command cmd, int id) {
    switch (int(node->state)) {
      case int(State::PowerUp): return Command::SRE;
      case int(State::ActPowerDown): return Command::PDX;
      case int(State::PrePowerDown): return Command::PDX;
      case int(State::SelfRefresh): return Command::SRE;
      default: assert(false);
    }};
}

// SAUGATA: added row hit check functions to see if the desired location is currently open
void DSARP::init_rowhit()
{
  // RD
  rowhit[int(Level::Subarray)][int(Command::RD)] = [] (DRAM<DSARP>* node, Command cmd, int id) {
      switch (int(node->state)){
          case int(State::Closed): return false;
          case int(State::Opened):
              if (node->row_state.find(id) != node->row_state.end()) return true;
              else return false;
          default: assert(false);
      }};
  // WR
  rowhit[int(Level::Subarray)][int(Command::WR)] = rowhit[int(Level::Subarray)][int(Command::RD)];
}

void DSARP::init_rowopen()
{
  // RD
  rowopen[int(Level::Subarray)][int(Command::RD)] = [] (DRAM<DSARP>* node, Command cmd, int id) {
      switch (int(node->state)){
          case int(State::Closed): return false;
          case int(State::Opened): return true;
          default: assert(false);
      }};
  // WR
  rowopen[int(Level::Subarray)][int(Command::WR)] = rowopen[int(Level::Subarray)][int(Command::RD)];
}

void DSARP::init_lambda()
{
  // RANK
  lambda[int(Level::Rank)][int(Command::PREA)] = [] (DRAM<DSARP>* node, int id) {
    node->row_state.clear();
    for (auto bg: node->children)
    {
      for (auto bank : bg->children) {
        bank->state = State::Closed;
        bank->row_state.clear();
        for (auto sa : bank->children){
          sa->state = State::Closed;
          sa->row_state.clear();}}}};
  lambda[int(Level::Rank)][int(Command::REF)] = [] (DRAM<DSARP>* node, int id) {};

  // Power down related commands
  lambda[int(Level::Rank)][int(Command::PDE)] = [] (DRAM<DSARP>* node, int id) {
    for (auto bank : node->children) {
      if (bank->state == State::Closed)
        continue;
      node->state = State::ActPowerDown;
      return;
    }
    node->state = State::PrePowerDown;};
  lambda[int(Level::Rank)][int(Command::PDX)] = [] (DRAM<DSARP>* node, int id) {
    node->state = State::PowerUp;};
  lambda[int(Level::Rank)][int(Command::SRE)] = [] (DRAM<DSARP>* node, int id) {
    node->state = State::SelfRefresh;};
  lambda[int(Level::Rank)][int(Command::SRX)] = [] (DRAM<DSARP>* node, int id) {
    node->state = State::PowerUp;};

  // Open a row
  lambda[int(Level::Bank)][int(Command::ACT)] = [] (DRAM<DSARP>* node, int id) {
    node->state = State::Opened;
    node->row_state[id] = State::Opened;};
  lambda[int(Level::Subarray)][int(Command::ACT)] = [] (DRAM<DSARP>* node, int id) {
    node->state = State::Opened;
    node->row_state[id] = State::Opened;};

  // Close a bank
  lambda[int(Level::Bank)][int(Command::PRE)] = [] (DRAM<DSARP>* node, int id) {
    node->state = State::Closed;
    node->row_state.clear();
    for (auto sa : node->children){
      sa->state = State::Closed;
      sa->row_state.clear();}};

  lambda[int(Level::Bank)][int(Command::RD)] = [] (DRAM<DSARP>* node, int id) {};
  lambda[int(Level::Bank)][int(Command::WR)] = [] (DRAM<DSARP>* node, int id) {};

  // Make sure the bank is closed after the column command
  lambda[int(Level::Bank)][int(Command::RDA)] = [] (DRAM<DSARP>* node, int id) {
    node->state = State::Closed;
    node->row_state.clear();};

  lambda[int(Level::Bank)][int(Command::WRA)] = [] (DRAM<DSARP>* node, int id) {
    node->state = State::Closed;
    node->row_state.clear();};

  // Nothing much, just make sure the bank is closed
  lambda[int(Level::Bank)][int(Command::REFPB)] = [] (DRAM<DSARP>* node, int id) {
    assert(node->state == State::Closed);
    node->row_state.clear();};

  // COL
  lambda[int(Level::Subarray)][int(Command::RDA)] = [] (DRAM<DSARP>* node, int id) {
    node->state = State::Closed;
    node->row_state.clear();};
  lambda[int(Level::Subarray)][int(Command::WRA)] = [] (DRAM<DSARP>* node, int id) {
    node->state = State::Closed;
    node->row_state.clear();};

  // PowerDown -- this has not been tested
  lambda[int(Level::Rank)][int(Command::PDE)] = [] (DRAM<DSARP>* node, int id) {
    for (auto bank : node->children)
      for (auto sa : bank->children) {
        if (sa->state == State::Closed)
          continue;
        node->state = State::ActPowerDown;
        return;
      }
    node->state = State::PrePowerDown;};
}

void DSARP::init_timing(bool per_bank_refresh)
{
  // TODO: What is the purpose of this?
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

  // READ to WRITE
  t[int(Command::RD)].push_back({Command::WR, 1, s.nCL + s.nCCDS + 2 - s.nCWL});
  t[int(Command::RD)].push_back({Command::WRA, 1, s.nCL + s.nCCDS + 2 - s.nCWL});
  t[int(Command::RDA)].push_back({Command::WR, 1, s.nCL + s.nCCDS + 2 - s.nCWL});
  t[int(Command::RDA)].push_back({Command::WRA, 1, s.nCL + s.nCCDS + 2 - s.nCWL});

  // WRITE to READ
  t[int(Command::WR)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTRS});
  t[int(Command::WR)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTRS});
  t[int(Command::WRA)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTRS});
  t[int(Command::WRA)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTRS});

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

  // CAS <-> PREA
  t[int(Command::RD)].push_back({Command::PREA, 1, s.nRTP});
  t[int(Command::WR)].push_back({Command::PREA, 1, s.nCWL + s.nBL + s.nWR});

  // CAS <-> PD
  t[int(Command::RD)].push_back({Command::PDE, 1, s.nCL + s.nBL + 1});
  t[int(Command::RDA)].push_back({Command::PDE, 1, s.nCL + s.nBL + 1});
  t[int(Command::WR)].push_back({Command::PDE, 1, s.nCWL + s.nBL + s.nWR});
  t[int(Command::WRA)].push_back({Command::PDE, 1, s.nCWL + s.nBL + s.nWR + 1}); // +1 for pre
  t[int(Command::PDX)].push_back({Command::RD, 1, s.nXP});
  t[int(Command::PDX)].push_back({Command::RDA, 1, s.nXP});
  t[int(Command::PDX)].push_back({Command::WR, 1, s.nXP});
  t[int(Command::PDX)].push_back({Command::WRA, 1, s.nXP});

  // RAS <-> RAS
  t[int(Command::ACT)].push_back({Command::ACT, 1, s.nRRDS});
  t[int(Command::ACT)].push_back({Command::ACT, 4, s.nFAW});
  t[int(Command::ACT)].push_back({Command::PREA, 1, s.nRAS});
  t[int(Command::PREA)].push_back({Command::ACT, 1, s.nRPab});

  // RAS <-> REF
  t[int(Command::PRE)].push_back({Command::REF, 1, s.nRPab});
  t[int(Command::PREA)].push_back({Command::REF, 1, s.nRPab});
  t[int(Command::REF)].push_back({Command::ACT, 1, s.nRFCab});

  // RAS <-> PD
  t[int(Command::ACT)].push_back({Command::PDE, 1, 1});
  t[int(Command::PDX)].push_back({Command::ACT, 1, s.nXP});
  t[int(Command::PDX)].push_back({Command::PRE, 1, s.nXP});
  t[int(Command::PDX)].push_back({Command::PREA, 1, s.nXP});

  // RAS <-> SR
  t[int(Command::PRE)].push_back({Command::SRE, 1, s.nRPpb});
  t[int(Command::PREA)].push_back({Command::SRE, 1, s.nRPab});
  t[int(Command::SRX)].push_back({Command::ACT, 1, s.nXS});

  // REF <-> REF
  t[int(Command::REF)].push_back({Command::REF, 1, s.nRFCab});
  // double check if we are issuing refs
  t[int(Command::REF)].push_back({Command::REFPB, 1, s.nRFCab});
  t[int(Command::REFPB)].push_back({Command::REF, 1, s.nRFCpb});

  // REF <-> PD
  t[int(Command::REF)].push_back({Command::PDE, 1, 1});
  t[int(Command::REFPB)].push_back({Command::PDE, 1, 1});
  t[int(Command::PDX)].push_back({Command::REF, 1, s.nXP});
  t[int(Command::PDX)].push_back({Command::REFPB, 1, s.nXP});

  // REF <-> SR
  t[int(Command::SRX)].push_back({Command::REF, 1, s.nXS});
  t[int(Command::SRX)].push_back({Command::REFPB, 1, s.nXS});

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

  // REFPB
  // TODO: this should not exist
  // t[int(Command::REFPB)].push_back( { Command::REFPB, 1, s.nRFCpb });

  /*** Bank ***/
  t = timing[int(Level::Bank)];

  // CAS <-> RAS
  t[int(Command::ACT)].push_back({Command::RD, 1, s.nRCD});
  t[int(Command::ACT)].push_back({Command::RDA, 1, s.nRCD});
  t[int(Command::ACT)].push_back({Command::WR, 1, s.nRCD});
  t[int(Command::ACT)].push_back({Command::WRA, 1, s.nRCD});

  t[int(Command::RD)].push_back({Command::PRE, 1, s.nRTP});
  t[int(Command::WR)].push_back({Command::PRE, 1, s.nCWL + s.nBL + s.nWR});

  t[int(Command::RDA)].push_back({Command::ACT, 1, s.nRTP + s.nRPpb});
  t[int(Command::WRA)].push_back({Command::ACT, 1, s.nCWL + s.nBL + s.nWR + s.nRPpb});

  // RAS <-> RAS
  t[int(Command::ACT)].push_back({Command::ACT, 1, s.nRC});
  t[int(Command::ACT)].push_back({Command::PRE, 1, s.nRAS});
  t[int(Command::PRE)].push_back({Command::ACT, 1, s.nRPab});
  t[int(Command::PRE)].push_back({Command::REFPB, 1, s.nRPpb});

  // Cannot overlap REFPB
  t[int(Command::REFPB)].push_back( { Command::REFPB, 1, s.nRFCpb });

  // B/w banks
  // TODO: shouldn't this be RRDL???
  t[int(Command::REFPB)].push_back( { Command::REFPB, 1, s.nRRDL, true});
  t[int(Command::ACT)].push_back({Command::REFPB, 1, s.nRRDL, true});
  t[int(Command::REFPB)].push_back({Command::ACT, 1, s.nRRDL, true});

  // REFPB -- these are used when SARP is not enabled
  if (!(type == Type::DSARP || type == Type::SARP)) {
    t[int(Command::REFPB)].push_back( { Command::ACT, 1, s.nRFCpb });
    t[int(Command::REFPB)].push_back( { Command::RD, 1, s.nRFCpb });
    t[int(Command::REFPB)].push_back( { Command::RDA, 1, s.nRFCpb });
    t[int(Command::REFPB)].push_back( { Command::WR, 1, s.nRFCpb });
    t[int(Command::REFPB)].push_back( { Command::WRA, 1, s.nRFCpb });
    t[int(Command::REFPB)].push_back( { Command::PRE, 1, s.nRFCpb });
    t[int(Command::REFPB)].push_back( { Command::PREA, 1, s.nRFCpb });
  }

  /*** Subarray ***/
  if (type == Type::DSARP || type == Type::SARP) {
    t = timing[int(Level::Subarray)];

    // between different subarrays -> Increase RRD
    t[int(Command::ACT)].push_back({Command::REFPB, 1,
        (int)ceil(((double)s.nRRDL)*nRRD_factor), true});
    t[int(Command::REFPB)].push_back({Command::ACT, 1,
        (int)ceil(((double)s.nRRDL)*nRRD_factor), true});

    // Same subarray
    t[int(Command::REF)].push_back( { Command::ACT, 1, s.nRFCab });
    t[int(Command::REFPB)].push_back( { Command::ACT, 1, s.nRFCpb });

    // CAS <-> RAS
    t[int(Command::ACT)].push_back( { Command::RD, 1, s.nRCD });
    t[int(Command::ACT)].push_back( { Command::RDA, 1, s.nRCD });
    t[int(Command::ACT)].push_back( { Command::WR, 1, s.nRCD });
    t[int(Command::ACT)].push_back( { Command::WRA, 1, s.nRCD });

    t[int(Command::RD)].push_back( { Command::PRE, 1, s.nRTP });
    t[int(Command::WR)].push_back(
        { Command::PRE, 1, s.nCWL + s.nBL + s.nWR });

    t[int(Command::RDA)].push_back( { Command::ACT, 1, s.nRTP + s.nRPpb });
    t[int(Command::WRA)].push_back(
        { Command::ACT, 1, s.nCWL + s.nBL + s.nWR + s.nRPpb });

    // RAS <-> RAS
    t[int(Command::ACT)].push_back( { Command::ACT, 1, s.nRC });
    t[int(Command::ACT)].push_back( { Command::PRE, 1, s.nRAS });
    t[int(Command::PRE)].push_back( { Command::ACT, 1, s.nRPpb });
    t[int(Command::PRE)].push_back( { Command::REFPB, 1, s.nRPpb });
    t[int(Command::PRE)].push_back( { Command::REF, 1, s.nRPpb });

    // Enforcing timings such that there's no subarray parallelism
    // between sibling subarrays for demand requests
    t[int(Command::ACT)].push_back( { Command::ACT, 1, s.nRC, true });
    t[int(Command::PRE)].push_back( { Command::ACT, 1, s.nRPpb, true });
    t[int(Command::RDA)].push_back( { Command::ACT, 1, s.nRTP + s.nRPpb,
        true });
    t[int(Command::WRA)].push_back(
        { Command::ACT, 1, s.nCWL + s.nBL + s.nWR + s.nRPpb, true });
  }
}
