#ifndef __CONTROLLER_H
#define __CONTROLLER_H

//#define PRINT_CMD_TRACE

#include <cassert>
#include <cstdio>
#include <deque>
#include <fstream>
#include <list>
#include <string>
#include <vector>
#include <set>
#include <memory>
#include <unordered_set>
#include <random>

#include "Config.h"
#include "DRAM.h"
#include "Refresh.h"
#include "RAIDR.h"
#include "Request.h"
#include "Scheduler.h"
#include "Statistics.h"
#include "Graphene.h"

// #include "ALDRAM.h"
// #include "SALP.h"
//#include "TLDRAM.h"

#include "CROWTable.h"
#include "SMD.h"

#include "MemoryScrubber.h"

#include "libdrampower/LibDRAMPower.h"
#include "xmlparser/MemSpecParser.h"

using namespace std;

namespace ramulator
{

    extern bool warmup_complete;

template <typename T>
class Controller
{
protected:
    // For counting bandwidth
    ScalarStat read_transaction_bytes;
    ScalarStat write_transaction_bytes;

    ScalarStat row_hits;
    ScalarStat row_misses;
    ScalarStat row_conflicts;
    VectorStat read_row_hits;
    VectorStat read_row_misses;
    VectorStat read_row_conflicts;
    VectorStat write_row_hits;
    VectorStat write_row_misses;
    VectorStat write_row_conflicts;
    ScalarStat useless_activates;

    ScalarStat num_act;
    ScalarStat num_pre;
    ScalarStat num_prea;
    ScalarStat num_rd;
    ScalarStat num_rda;
    ScalarStat num_wr;
    ScalarStat num_wra;
    ScalarStat num_ref;
    ScalarStat num_speculative_precharges;

    ScalarStat read_latency_avg;
    ScalarStat read_latency_sum;
    VectorStat read_latency_sum_per_core;
    VectorStat read_latency_avg_per_core;

    ScalarStat req_queue_length_avg;
    ScalarStat req_queue_length_sum;
    ScalarStat read_req_queue_length_avg;
    ScalarStat read_req_queue_length_sum;
    ScalarStat write_req_queue_length_avg;
    ScalarStat write_req_queue_length_sum;

    ScalarStat num_para_acts;
    ScalarStat num_raidr_acts;

    // CROW
    ScalarStat crow_invPRE;
    ScalarStat crow_invACT;
    ScalarStat crow_full_restore;
    ScalarStat crow_skip_full_restore;
    ScalarStat crow_num_hits;
    ScalarStat crow_num_all_hits;
    ScalarStat crow_num_misses;
    ScalarStat crow_num_copies;

    ScalarStat crow_num_fr_set;
    ScalarStat crow_num_fr_notset;
    ScalarStat crow_num_fr_ref;
    ScalarStat crow_num_fr_restore;
    ScalarStat crow_num_hits_with_fr;
    ScalarStat crow_bypass_copying;

    ScalarStat crow_idle_cycle_due_trcd;
    ScalarStat crow_idle_cycle_due_tras;

    ScalarStat tl_dram_invalidate_due_to_write;
    ScalarStat tl_dram_precharge_cached_row_due_to_write;
    ScalarStat tl_dram_precharge_failed_due_to_timing;
    // END - CROW

    // SMD
    ScalarStat smd_force_long_active_row_precharge;
    ScalarStat smd_ref_status_responses;
    ScalarStat smd_opportunistic_ref_status_queries;
    ScalarStat smd_ready_but_timed_out_req;
    ScalarStat smd_ready_but_SA_conflict_req;
    ScalarStat smd_ready_but_SA_really_refreshing;

    VectorStat smd_ref_status_timing_failures;
    VectorStat smd_no_bank_for_ref_status_update;
    VectorStat smd_total_ref_status_queries;

    ScalarStat smd_act_nack_cnt;
    ScalarStat smd_act_partial_nack_cnt;
    // END - SMD

    // DRAMPower
    VectorStat dpower_act_energy, dpower_pre_energy, dpower_rd_energy, dpower_wr_energy, dpower_ref_energy, dpower_refpb_energy;
    VectorStat dpower_act_stdby_energy, dpower_pre_stdby_energy;
    VectorStat dpower_ecc_enc_energy, dpower_ecc_dec_energy;
    VectorStat dpower_io_term_energy;
    VectorStat dpower_total_energy, dpower_avg_power;
    

    // END - DRAMPower

#ifndef INTEGRATED_WITH_GEM5
    VectorStat record_read_hits;
    VectorStat record_read_misses;
    VectorStat record_read_conflicts;
    VectorStat record_write_hits;
    VectorStat record_write_misses;
    VectorStat record_write_conflicts;
    VectorStat record_read_latency_avg_per_core;
#endif

public:
    /* Member Variables */
    long clk = 0;
    DRAM<T>* channel;

    Scheduler<T>* scheduler;  // determines the highest priority request whose commands will be issued
    RowPolicy<T>* rowpolicy;  // determines the row-policy (e.g., closed-row vs. open-row)
    RowTable<T>* rowtable;  // tracks metadata about rows (e.g., which are open and for how long)
    Refresh<T>* refresh;
    RAIDR<T> raidr;
    Graphene<T> graphene;
    MemoryScrubber<T>* scrubber;

    struct Queue {
        list<Request> q;
        unsigned int max = 64;
        unsigned int size() const {return q.size();}
    };

    typedef enum class RegionBusyResponse : int {
        NO_CHIPS_BUSY,
        SOME_CHIPS_BUSY,
        ALL_CHIPS_BUSY
    } RegionBusyResponse;

    struct SMD_ALERT_MANAGER {
        std::set<int> alerts_set;
        std::deque<int> alerts_queue;

        void push_back(const int key) {
            if (alerts_set.find(key) == alerts_set.end()) {
                alerts_set.insert(key);
                alerts_queue.push_back(key);
            }
        }

        int front() {
            return alerts_queue.front();
        }

        void process_front() {
            // pops the front element of the queue but keeps the key in the set preventing new alerts from being registered
            alerts_queue.pop_front();
        }

        uint32_t count() {
            return alerts_queue.size();
        }

        void remove(const int key) {
            alerts_set.erase(key);
        }

    } smd_alerts;

    Queue readq;  // queue for read requests
    Queue writeq;  // queue for write requests
    Queue actq; // read and write requests for which activate was issued are moved to 
                   // actq, which has higher priority than readq and writeq.
                   // This is an optimization
                   // for avoiding useless activations (i.e., PRECHARGE
                   // after ACTIVATE w/o READ of WRITE command)
    Queue otherq;  // queue for all "other" requests (e.g., refresh)

    deque<Request> pending;  // read requests that are about to receive data from DRAM
    deque<Request> pending_act_nack;  // for SMD in ACT_NACK mode, contains ACT_NACK responses of DRAM chips
    bool write_mode = false;  // whether write requests should be prioritized over reads
    //long refreshed = 0;  // last time refresh requests were generated

    /* Command trace for DRAMPower 3.1 */
    string cmd_trace_prefix = "cmd-trace-";
    vector<ofstream> cmd_trace_files;
    bool record_cmd_trace = false;
    /* Commands to stdout */
    bool print_cmd_trace = false;

    bool enable_crow = false;
    bool enable_crow_upperbound = false;
    bool enable_tl_dram = false;
    int crow_evict_threshold = 0;
    int crow_half_life = 0;
    float crow_to_mru_frac = 0.0f;
    uint crow_table_grouped_SAs = 1;
    uint copy_rows_per_SA = 0;
    uint weak_rows_per_SA = 0;
    float refresh_mult = 1.0f;
    bool prioritize_evict_fully_restored = false;
    bool collect_row_act_histogram = false;
    int num_SAs = 0;
    uint32_t chips_per_rank = 0;

    bool enable_para = false;
    // raidr variables
    bool enable_raidr = false;
    bool enable_graphene = false;

    // PARA random number generation
    std::mt19937 para_gen;
    std::discrete_distribution<uint64_t> para_dist;
    std::function<bool()> flip_PARA_coin;

    bool enable_scrubbing = false;

    bool is_DDR4 = false, is_LPDDR4 = false; // Hasan

    CROWTable<T>* crow_table = nullptr;
    int* ref_counters;
    //ulong crow_table_inv_interval;
    //vector<int> crow_table_inv_index;
    //bool inv_next_copy_row = false;

    std::vector<libDRAMPower> dpower;
    
    bool refresh_disabled = false;
    bool per_bank_refresh_on = false;
    bool smd_enabled = false;
    bool smd_ecc_scrubbing_enabled = false;
    bool smd_rh_protection_enabled = false;
    SMD_MODE smd_mode;
    uint32_t smd_max_row_open_intervals = 8;
    uint32_t smd_act_to_nack_cycles = 0;

    int smd_partial_nack_combined_threshold = 0;
    int smd_partial_nack_resend_interval = 0;

    SMDTracker<T> smd_ref_tracker;
    SMDTracker<T> smd_scrub_tracker;
    std::vector<std::unique_ptr<MaintenancePolicy<T>>> smd_refreshers;
    std::vector<std::unique_ptr<MaintenancePolicy<T>>> smd_scrubbers;
    std::vector<std::unique_ptr<SMDRowHammerProtection<T>>> smd_rh_protectors;
    std::vector<uint32_t> alerted_ranks;

    /* Constructor */
    Controller(const Config& configs, DRAM<T>* channel) :
        channel(channel),
        scheduler(new Scheduler<T>(this, configs)),
        rowpolicy(new RowPolicy<T>(this, configs)),
        rowtable(new RowTable<T>(this)),
        refresh(new Refresh<T>(this)),
        raidr(RAIDR<T>(configs, this)),
        graphene(Graphene<T>(configs, this)),
        scrubber(new MemoryScrubber<T>(configs, this)),
        cmd_trace_files(channel->children.size()),
        smd_ref_tracker(SMDTracker<T>(configs, *this)),
        smd_scrub_tracker(SMDTracker<T>(configs, *this))
    {
        record_cmd_trace = configs.get_bool("record_cmd_trace");
        print_cmd_trace = configs.get_bool("print_cmd_trace");
        if (record_cmd_trace){
            if (configs["cmd_trace_prefix"] != "") {
              cmd_trace_prefix = configs["cmd_trace_prefix"];
            }
            string prefix = cmd_trace_prefix + "chan-" + to_string(channel->id) + "-rank-";
            string suffix = ".cmdtrace";
            for (unsigned int i = 0; i < channel->children.size(); i++)
                cmd_trace_files[i].open(prefix + to_string(i) + suffix);
        }

        per_bank_refresh_on = configs.get_bool("per_bank_refresh");

        uint32_t banks_per_rank = channel->spec->get_num_banks_per_rank();
        uint32_t num_rows = channel->spec->org_entry.count[uint32_t(T::Level::Row)];
        num_SAs = channel->spec->org_entry.count[uint32_t(T::Level::Subarray)];

        is_DDR4 = channel->spec->standard_name == "DDR4";
        is_LPDDR4 = channel->spec->standard_name == "LPDDR4";

        // Initialize memory controller level PARA
        enable_para = configs.get_bool("enable_para");
        para_gen = std::mt19937{uint32_t(channel->id)}; // using fixed seed for repeatable simulations
        double neighbor_row_refresh_pct = (double)configs.get_float("para_neighbor_refresh_pct");
        para_dist = std::discrete_distribution<uint64_t>({100 - neighbor_row_refresh_pct, neighbor_row_refresh_pct});
        flip_PARA_coin = [&]() {return para_dist(para_gen) == 1;};

        // Initialize memory controller based scrubbing
        enable_scrubbing = configs.get_bool("enable_scrubbing");

        // initialize a SMD refresh policy for each chip connected to this channel
        chips_per_rank = channel->spec->channel_width/channel->spec->org_entry.dq;

        smd_mode = str_to_smd_mode[configs.get_str("smd_mode")];
        smd_act_to_nack_cycles = configs.get_int("smd_act_to_nack_cycles");

        smd_max_row_open_intervals = configs.get_uint("smd_max_row_open_intervals");

        std::string smd_ref_policy = configs.get_str("smd_ref_policy");
        if (smd_ref_policy == "FixedRate"){
            for (uint32_t rank_id = 0; rank_id < (uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)]; rank_id++) {
                for (uint32_t chip_id = 0; chip_id < chips_per_rank; chip_id++) {
                    smd_refreshers.emplace_back(new SMDFixedRateRefresh<T>(configs, this, rank_id, chip_id, banks_per_rank, num_SAs, num_rows));
                }
            }
        } else if (smd_ref_policy == "VariableRefresh"){
            for (uint32_t rank_id = 0; rank_id < (uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)]; rank_id++) {
                for (uint32_t chip_id = 0; chip_id < chips_per_rank; chip_id++) {
                    smd_refreshers.emplace_back(new SMDVariableRefresh<T>(configs, this, rank_id, chip_id, banks_per_rank, num_SAs, num_rows));
                }
            }
        } else if (smd_ref_policy == "NoRefresh"){
            for (uint32_t rank_id = 0; rank_id < (uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)]; rank_id++) {
                for (uint32_t chip_id = 0; chip_id < chips_per_rank; chip_id++) {
                    smd_refreshers.emplace_back(new SMDNoRefresh<T>(configs, this, rank_id, chip_id, banks_per_rank, num_SAs, num_rows));
                }
            }
        }
        else {
            assert(false && "[Controller] ERROR: Undefined SMD Refresh Policy.");
        }

        smd_ecc_scrubbing_enabled = configs.get_bool("smd_ecc_scrubbing_enabled");
        for (uint32_t rank_id = 0; rank_id < (uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)]; rank_id++) {
            for (uint32_t chip_id = 0; chip_id < chips_per_rank; chip_id++) {
                smd_scrubbers.emplace_back(new SMDECCScrubbing<T>(configs, this, rank_id, chip_id, banks_per_rank, num_SAs, num_rows));
            }
        }

        smd_rh_protection_enabled = configs.get_bool("smd_rh_protection_enabled");
        for (uint32_t rank_id = 0; rank_id < (uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)]; rank_id++) {
            for (uint32_t chip_id = 0; chip_id < chips_per_rank; chip_id++) {
                smd_rh_protectors.emplace_back(new SMDRowHammerProtection<T>(configs, this, rank_id, chip_id, banks_per_rank, num_SAs, num_rows));
            }
        }

        smd_partial_nack_combined_threshold = configs.get_int("smd_combined_policy_threshold");
        smd_partial_nack_resend_interval = ceil(configs.get_float("smd_act_nack_resend_interval")/channel->spec->speed_entry.tCK);;

        // printf("nNACK_RESEND %d\n", smd_partial_nack_resend_interval);

        // Initialize DRAMPower
        DRAMPower::MemorySpecification memSpec(DRAMPower::MemSpecParser::getMemSpecFromXML(configs.get_str("dpower_memspec_path")));

        // a separate DRAMPower object per rank
        dpower.reserve((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)]);
        for (uint32_t rank_id = 0; rank_id < (uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)]; rank_id++)
            dpower.emplace_back(memSpec, configs.get_bool("dpower_include_io_and_termination"));
        

        // regStats

        row_hits
            .name("row_hits_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row hits per channel per core")
            .precision(0)
            ;
        row_misses
            .name("row_misses_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row misses per channel per core")
            .precision(0)
            ;
        row_conflicts
            .name("row_conflicts_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row conflicts per channel per core")
            .precision(0)
            ;

        read_row_hits
            .init(configs.get_int("cores"))
            .name("read_row_hits_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row hits for read requests per channel per core")
            .precision(0)
            ;
        read_row_misses
            .init(configs.get_int("cores"))
            .name("read_row_misses_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row misses for read requests per channel per core")
            .precision(0)
            ;
        read_row_conflicts
            .init(configs.get_int("cores"))
            .name("read_row_conflicts_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row conflicts for read requests per channel per core")
            .precision(0)
            ;

        write_row_hits
            .init(configs.get_int("cores"))
            .name("write_row_hits_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row hits for write requests per channel per core")
            .precision(0)
            ;
        write_row_misses
            .init(configs.get_int("cores"))
            .name("write_row_misses_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row misses for write requests per channel per core")
            .precision(0)
            ;
        write_row_conflicts
            .init(configs.get_int("cores"))
            .name("write_row_conflicts_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row conflicts for write requests per channel per core")
            .precision(0)
            ;

        useless_activates
            .name("useless_activates_"+to_string(channel->id)+ "_core")
            .desc("Number of useless activations. E.g, ACT -> PRE w/o RD or WR")
            .precision(0)
            ;

        read_transaction_bytes
            .name("read_transaction_bytes_"+to_string(channel->id))
            .desc("The total byte of read transaction per channel")
            .precision(0)
            ;
        write_transaction_bytes
            .name("write_transaction_bytes_"+to_string(channel->id))
            .desc("The total byte of write transaction per channel")
            .precision(0)
            ;

        read_latency_sum
            .name("read_latency_sum_"+to_string(channel->id))
            .desc("The memory latency cycles (in memory time domain) sum for all read requests in this channel")
            .precision(0)
            ;
        read_latency_sum_per_core
            .init(configs.get_int("cores"))
            .name("read_latency_sum_per_core"+to_string(channel->id))
            .desc("The memory latency cycles (in memory time domain) sum for all read requests per core")
            .precision(0)
            ;
        read_latency_avg
            .name("read_latency_avg_"+to_string(channel->id))
            .desc("The average memory latency cycles (in memory time domain) per request for all read requests in this channel")
            .precision(6)
            ;
        read_latency_avg_per_core
            .init(configs.get_int("cores"))
            .name("read_latency_avg_per_core"+to_string(channel->id))
            .desc("The average memory latency cycles (in memory time domain) per request per each core")
            .precision(6)
            ;
        req_queue_length_sum
            .name("req_queue_length_sum_"+to_string(channel->id))
            .desc("Sum of read and write queue length per memory cycle per channel.")
            .precision(0)
            ;
        req_queue_length_avg
            .name("req_queue_length_avg_"+to_string(channel->id))
            .desc("Average of read and write queue length per memory cycle per channel.")
            .precision(6)
            ;

        read_req_queue_length_sum
            .name("read_req_queue_length_sum_"+to_string(channel->id))
            .desc("Read queue length sum per memory cycle per channel.")
            .precision(0)
            ;
        read_req_queue_length_avg
            .name("read_req_queue_length_avg_"+to_string(channel->id))
            .desc("Read queue length average per memory cycle per channel.")
            .precision(6)
            ;

        write_req_queue_length_sum
            .name("write_req_queue_length_sum_"+to_string(channel->id))
            .desc("Write queue length sum per memory cycle per channel.")
            .precision(0)
            ;
        write_req_queue_length_avg
            .name("write_req_queue_length_avg_"+to_string(channel->id))
            .desc("Write queue length average per memory cycle per channel.")
            .precision(6)
            ;

        // Counting the number of commands issued of each type
        num_act
            .name("num_act_"+to_string(channel->id))
            .desc("The number of ACT commands issued to the channel.")
            .precision(0)
            ;
        num_pre
            .name("num_pre_"+to_string(channel->id))
            .desc("The number of PRE commands issued to the channel.")
            .precision(0)
            ;
        num_prea
            .name("num_prea_"+to_string(channel->id))
            .desc("The number of PREA commands issued to the channel.")
            .precision(0)
            ;
        num_rd
            .name("num_rd_"+to_string(channel->id))
            .desc("The number of RD commands issued to the channel.")
            .precision(0)
            ;

        num_rda
            .name("num_rda_"+to_string(channel->id))
            .desc("The number of RDA commands issued to the channel.")
            .precision(0)
            ;
        num_wr
            .name("num_wr_"+to_string(channel->id))
            .desc("The number of WR commands issued to the channel.")
            .precision(0)
            ;

        num_wra
            .name("num_wra_"+to_string(channel->id))
            .desc("The number of WRA commands issued to the channel.")
            .precision(0)
            ;
        num_ref
            .name("num_ref_"+to_string(channel->id))
            .desc("The number of REF commands issued to the channel.")
            .precision(0)
            ;
        num_speculative_precharges
            .name("num_speculative_precharges"+to_string(channel->id) + "_core")
            .desc("Total number of precharge commands issued for speculatively closing a row.")
            .precision(0)
            ;

        // Memory Controller level maintenance operation stats
        num_para_acts
            .name("num_para_activates"+to_string(channel->id) + "_core")
            .desc("The total number of activations performed by PARA to refresh neighbor rows")
            .precision(0)
            ;

        num_raidr_acts
            .name("num_raidr_activates"+to_string(channel->id) + "_core")
            .desc("The total number of activations performed by RAIDR")
            .precision(0)
            ;

        // CROW
        crow_invPRE
            .name("crow_invPRE_channel_"+to_string(channel->id) + "_core")
            .desc("Number of Precharge commands issued to be able to activate an entry in the CROW table.")
            .precision(0)
            ;

        crow_invACT
            .name("crow_invACT_channel_"+to_string(channel->id) + "_core")
            .desc("Number of Activate command issued to fully activate the entry to invalidate from the CROW table.")
            .precision(0)
            ;

        crow_full_restore
            .name("crow_full_restore_channel_"+to_string(channel->id) + "_core")
            .desc("Number of Activate commands issued to fully restore an entry that is being discarded due to inserting a new entry.")
            .precision(0)
            ;

        crow_skip_full_restore
            .name("crow_skip_full_restore_channel_"+to_string(channel->id) + "_core")
            .desc("Number of times full restore was not needed (FR bit not set) when discarding an entry due to inserting a new one.")
            .precision(0)
            ;

        crow_num_hits
            .name("crow_num_hits_channel_"+to_string(channel->id) + "_core")
            .desc("Number of hits to the CROW table (without additional activations needed for full restoration).")
            .precision(0)
            ;

        crow_num_all_hits
            .name("crow_num_all_hits_channel_"+to_string(channel->id) + "_core")
            .desc("Number of hits to the CROW table.")
            .precision(0)
            ;

        crow_num_misses
            .name("crow_num_misses_channel_"+to_string(channel->id) + "_core")
            .desc("Number of misses to the CROW table.")
            .precision(0)
            ;

        crow_num_copies
            .name("crow_num_copies_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row copy operation CROW performed.")
            .precision(0)
            ;

        crow_num_fr_set
            .name("crow_num_fr_set_channel_"+to_string(channel->id) + "_core")
            .desc("Number of times FR bit is set when precharging.")
            .precision(0)
            ;
        crow_num_fr_notset
            .name("crow_num_fr_notset_channel_"+to_string(channel->id) + "_core")
            .desc("Number of times FR bit is not set when precharging.")
            .precision(0)
            ;
        crow_num_fr_ref
            .name("crow_num_fr_ref_channel_"+to_string(channel->id) + "_core")
            .desc("Number of times FR bit is set since the row won't be refreshed soon.")
            .precision(0)
            ;
        crow_num_fr_restore
            .name("crow_num_fr_restore_channel_"+to_string(channel->id) + "_core")
            .desc("Number of times FR bit is set since the row is not fully restored.")
            .precision(0)
            ;

        crow_num_hits_with_fr
            .name("crow_num_hits_with_fr_channel_"+to_string(channel->id) + "_core")
            .desc("Number of CROWTable hits to FR bit set entries.")
            .precision(0)
            ;

        crow_idle_cycle_due_trcd
            .name("crow_cycles_trcd_stall_channel_"+to_string(channel->id) + "_core")
            .desc("Number of cycles for which the command bus was idle but there was a request waiting for tRCD.")
            .precision(0)
            ;

        crow_idle_cycle_due_tras
            .name("crow_cycles_tras_stall_channel_"+to_string(channel->id) + "_core")
            .desc("Number of cycles for which the command bus was idle but there was a request waiting for tRAS.")
            .precision(0)
            ;

        crow_bypass_copying
            .name("crow_bypass_copying_channel_"+to_string(channel->id) + "_core")
            .desc("Number of rows not copied to a spare row due to having only rows above the hit threshold already cached.")
            .precision(0)
            ;

        tl_dram_invalidate_due_to_write
            .name("tl_dram_invalidate_due_to_write_channel_"+to_string(channel->id) + "_core")
            .desc("Number of TL-DRAM cached rows invalidated during activation due to pending writes.")
            .precision(0)
            ;

        tl_dram_precharge_cached_row_due_to_write
            .name("tl_dram_precharge_cached_row_due_to_write_channel_"+to_string(channel->id) + "_core")
            .desc("Number of TL-DRAM cached rows precharged to write data.")
            .precision(0)
            ;

        tl_dram_precharge_failed_due_to_timing
            .name("tl_dram_precharge_failed_due_to_timing_channel_"+to_string(channel->id) + "_core")
            .desc("Number of cycles failed to issue a PRE command to TL-DRAM cache row.")
            .precision(0)
            ;
        // END - CROW

        // SMD
        smd_force_long_active_row_precharge
            .name("smd_force_precharge_"+to_string(channel->id) + "_core")
            .desc("Number of auto-precharges performed to rows that stayed open for too long.")
            .precision(0)
            ;

        smd_ref_status_responses
            .name("smd_ref_status_responses_"+to_string(channel->id) + "_core")
            .desc("Number of responses received for REF_STATUS_QUERY commands.")
            .precision(0)
            ;

        smd_opportunistic_ref_status_queries
            .name("smd_opportunistic_ref_status_queries_"+to_string(channel->id) + "_core")
            .desc("Number of opportunistic REF_STATUS_QUERY commands sent when no memory request is ready.")
            .precision(0)
            ;

        smd_ref_status_timing_failures
            .init(channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("smd_ref_status_timing_failures_"+to_string(channel->id) + "_core")
            .desc("Number of ref status query failures due to timing violations.")
            .precision(0)
            ;

        smd_no_bank_for_ref_status_update
            .init(channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("smd_no_bank_for_ref_status_update_"+to_string(channel->id) + "_core")
            .desc("Number of ref status queries not performed due to having no bank that requires an update.")
            .precision(0)
            ;

        smd_total_ref_status_queries
            .init(channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("smd_total_ref_status_queries_"+to_string(channel->id) + "_core")
            .desc("Number of total issued ref status queries.")
            .precision(0)
            ;

        smd_ready_but_timed_out_req
            .name("smd_ready_but_timed_out_req_"+to_string(channel->id) + "_core")
            .desc("Number of cycles no request was ready but a req might have been ready if the corresponding ref status wasn't timed out.")
            .precision(0)
            ;

        smd_ready_but_SA_conflict_req
            .name("smd_ready_but_SA_conflict_req_"+to_string(channel->id) + "_core")
            .desc("Number of cycles no request was ready but a req might have been ready if the target SA was not locked in the RefStatus table.")
            .precision(0)
            ;
        
        smd_ready_but_SA_really_refreshing
            .name("smd_ready_but_SA_really_refreshing_"+to_string(channel->id) + "_core")
            .desc("Number of cycles no request was ready but a req might have been ready if the target SA was not being refreshed really refreshed.")
            .precision(0)
            ;

        smd_act_nack_cnt
            .name("smd_act_nack_cnt"+to_string(channel->id) + "_core")
            .desc("Total number of ACT_NACK responses.")
            .precision(0)
            ;

        smd_act_partial_nack_cnt
            .name("smd_act_partial_nack_cnt"+to_string(channel->id) + "_core")
            .desc("Total number of ACT_PARTIAL_NACK responses. This happens when some chips return ACT_NACK some don't.")
            .precision(0)
            ;

        // END - SMD

        // DRAMPower
        dpower_act_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_act_energy"+to_string(channel->id) + "_core")
            .desc("ACT command energy (per rank) in mJ.")
            .precision(3);

        dpower_pre_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_pre_energy"+to_string(channel->id) + "_core")
            .desc("PRE command energy (per rank) in mJ.")
            .precision(3);

        dpower_rd_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_rd_energy"+to_string(channel->id) + "_core")
            .desc("READ command energy (per rank) in mJ.")
            .precision(3);

        dpower_wr_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_wr_energy"+to_string(channel->id) + "_core")
            .desc("WRITE command energy (per rank) in mJ.")
            .precision(3);

        dpower_ref_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_ref_energy"+to_string(channel->id) + "_core")
            .desc("REFRESH command energy (per rank) in mJ.")
            .precision(3);

        dpower_refpb_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_refpb_energy"+to_string(channel->id) + "_core")
            .desc("REFRESHpb command energy (per rank) in mJ.")
            .precision(3);

        dpower_act_stdby_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_act_stdby_energy"+to_string(channel->id) + "_core")
            .desc("ACT standby energy (per rank) in mJ.")
            .precision(3);

        dpower_pre_stdby_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_pre_stdby_energy"+to_string(channel->id) + "_core")
            .desc("PRE standby energy (per rank) in mJ.")
            .precision(3);

        dpower_io_term_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_io_term_energy"+to_string(channel->id) + "_core")
            .desc("Total IO/termination energy (per rank) in mJ.")
            .precision(3);

        dpower_ecc_enc_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_ecc_enc_energy"+to_string(channel->id) + "_core")
            .desc("Total ECC encoding energy (per rank) in mJ.")
            .precision(3);

        dpower_ecc_dec_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_ecc_dec_energy"+to_string(channel->id) + "_core")
            .desc("Total ECC decoding energy (per rank) in mJ.")
            .precision(3);
        
        dpower_total_energy
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_total_energy"+to_string(channel->id) + "_core")
            .desc("Total DRAM energy (per rank) in mJ.")
            .precision(3);

        dpower_avg_power
            .init((uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)])
            .name("dpower_avg_power"+to_string(channel->id) + "_core")
            .desc("Average DRAM power (per rank) in mW.")
            .precision(3);
        

        // END - DRAMPower

#ifndef INTEGRATED_WITH_GEM5
        record_read_hits
            .init(configs.get_int("cores"))
            .name("record_read_hits")
            .desc("record read hit count for this core when it reaches request limit or to the end")
            ;

        record_read_misses
            .init(configs.get_int("cores"))
            .name("record_read_misses")
            .desc("record_read_miss count for this core when it reaches request limit or to the end")
            ;

        record_read_conflicts
            .init(configs.get_int("cores"))
            .name("record_read_conflicts")
            .desc("record read conflict count for this core when it reaches request limit or to the end")
            ;

        record_write_hits
            .init(configs.get_int("cores"))
            .name("record_write_hits")
            .desc("record write hit count for this core when it reaches request limit or to the end")
            ;

        record_write_misses
            .init(configs.get_int("cores"))
            .name("record_write_misses")
            .desc("record write miss count for this core when it reaches request limit or to the end")
            ;

        record_write_conflicts
            .init(configs.get_int("cores"))
            .name("record_write_conflicts")
            .desc("record write conflict for this core when it reaches request limit or to the end")
            ;
        record_read_latency_avg_per_core
            .init(configs.get_int("cores"))
            .name("record_read_latency_avg_"+to_string(channel->id))
            .desc("The memory latency cycles (in memory time domain) average per core in this channel")
            .precision(6)
            ;

#endif
    }

    ~Controller(){
        delete scheduler;
        delete rowpolicy;
        delete rowtable;
        delete channel;
        delete refresh;
        for (auto& file : cmd_trace_files)
            file.close();
        cmd_trace_files.clear();

        delete crow_table;
        delete[] ref_counters;
    }

    void finish(long read_req, long dram_cycles) {
        read_latency_avg = read_latency_sum.value() / read_req;
        req_queue_length_avg = req_queue_length_sum.value() / dram_cycles;
        read_req_queue_length_avg = read_req_queue_length_sum.value() / dram_cycles;
        write_req_queue_length_avg = write_req_queue_length_sum.value() / dram_cycles;

        for(uint32_t coreid = 0; coreid < read_latency_avg_per_core.size() ; coreid++){
            read_latency_avg_per_core[coreid] = read_latency_sum_per_core[coreid].value()/
                                            (float)(read_row_hits[coreid].value() + read_row_misses[coreid].value() +
                                                    read_row_conflicts[coreid].value());
        }

        // call finish function of each channel
        channel->finish(dram_cycles);

        // print out the row_act_hist
        if(collect_row_act_histogram) {
            printf("Printing Row Activation Histogram\n");
            printf("Format: row_id, access_count\n");
            printf("=================================\n");
            for(int bank_id = 0; bank_id < 8; bank_id++) {
                for(int sa_id = 0; sa_id < num_SAs; sa_id++) {
                    printf("----- Bank %d, Subarray %d\n", bank_id, sa_id);
                    auto& cur_hist = row_act_hist[bank_id][sa_id];

                    for(auto it = cur_hist.begin(); it != cur_hist.end(); it++) {
                        printf("%d, %d\n", it->first, it->second);           
                    }
                }

            }
        }

        update_DPower(true);
        
    }

    void update_DPower(const bool finish = false) {
        for (uint32_t rank_id = 0; rank_id < (uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)]; rank_id++) {
            dpower[rank_id].calcWindowEnergy(clk);

            dpower_act_energy[rank_id] += dpower[rank_id].getEnergy().act_energy/1000000000; // converting pJ to mJ
            dpower_pre_energy[rank_id] += dpower[rank_id].getEnergy().pre_energy/1000000000;
            dpower_rd_energy[rank_id] += dpower[rank_id].getEnergy().read_energy/1000000000;
            dpower_wr_energy[rank_id] += dpower[rank_id].getEnergy().write_energy/1000000000;
            dpower_ref_energy[rank_id] += dpower[rank_id].getEnergy().ref_energy/1000000000;

            auto& refpb_energy = dpower[rank_id].getEnergy().refb_energy_banks;
            dpower_refpb_energy[rank_id] += std::accumulate(refpb_energy.begin(), refpb_energy.end(), 0)/1000000000;

            dpower_act_stdby_energy[rank_id] += dpower[rank_id].getEnergy().act_stdby_energy/1000000000;
            dpower_pre_stdby_energy[rank_id] += dpower[rank_id].getEnergy().pre_stdby_energy/1000000000;

            dpower_ecc_enc_energy[rank_id] += dpower[rank_id].getEnergy().ecc_enc/1000000000;
            dpower_ecc_dec_energy[rank_id] += dpower[rank_id].getEnergy().ecc_dec/1000000000;

            dpower_io_term_energy[rank_id] += dpower[rank_id].getEnergy().io_term_energy/1000000000;

            dpower_total_energy[rank_id] += dpower[rank_id].getEnergy().window_energy/1000000000;

            if (finish) {
                dpower[rank_id].calcEnergy();
                dpower_avg_power[rank_id] = dpower[rank_id].getPower().average_power;
            }
        }
    }

    void discard_DPowerWindow() {
        for (uint32_t rank_id = 0; rank_id < (uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)]; rank_id++)
            dpower[rank_id].calcWindowEnergy(clk);
    }

    /* Member Functions */
    Queue& get_queue(Request::Type type)
    {
        switch (int(type)) {
            case int(Request::Type::READ): 
            case int(Request::Type::RAIDR_REFRESH):
            case int(Request::Type::PREFETCH): return readq;
            case int(Request::Type::WRITE): return writeq;
            default: return otherq;
        }
    }

    bool enqueue(Request& req)
    {
        Queue& queue = get_queue(req.type);
        if (queue.max == queue.size())
        {
            //printf("Q is not empty\n");
            return false;
        }
        req.arrive = clk;
        queue.q.push_back(req);
        // shortcut for read requests, if a write to same addr exists
        // necessary for coherence
        if ((req.type == Request::Type::READ || req.type == Request::Type::PREFETCH) && find_if(writeq.q.begin(), writeq.q.end(),
                [req](Request& wreq){ return req.addr == wreq.addr;}) != writeq.q.end()){
            req.depart = clk + 1;
            pending.push_back(req);
            readq.q.pop_back();
        }

        //printf("Ctrl: Enqueue req type %d, addr %ld\n", req.type, req.addr);
        return true;
    }

    bool upgrade_prefetch_req(const Request& req) {
        assert(req.type == Request::Type::READ);

        Queue& queue = get_queue(req.type);

        // DEBUG
        //printf("=========== Upgrading to a READ request.\n");
        //printf("--- readq ---\n");
        //for(auto it = queue.q.begin(); it != queue.q.end(); it++)
        //    printf("%ld\n", it->addr);

        //printf("--- actq ---\n");
        //for(auto it = actq.q.begin(); it != actq.q.end(); it++)
        //    printf("%ld\n", it->addr);
    
        //printf("--- pending ---\n");
        //for(auto it = pending.begin(); it != pending.end(); it++)
        //    printf("%ld\n", it->addr);
        //printf("===========\n\n");


        // the prefetch request could be in readq, actq, or pending
        if (upgrade_prefetch_req(queue, req))
            return true;

        if (upgrade_prefetch_req(actq, req))
            return true;

        if (upgrade_prefetch_req(pending, req))
            return true;

        return false;
    } 

    void tick()
    {
        clk++;
        req_queue_length_sum += readq.size() + writeq.size() + pending.size();
        read_req_queue_length_sum += readq.size() + pending.size();
        write_req_queue_length_sum += writeq.size();

        if(warmup_complete && !dpower_is_reset) {
            discard_DPowerWindow(); // discarding the last window results collected during warmup
            dpower_is_reset = true;
        }

        const uint32_t DPOWER_UPDATE_PERIOD = 50000000;
        if (clk % DPOWER_UPDATE_PERIOD == (DPOWER_UPDATE_PERIOD - 1)){
            if (warmup_complete)
                update_DPower();
            else
                discard_DPowerWindow();
        }

        // CROW
        //if (enable_crow) {
        //    if ((clk % crow_table_inv_interval) == 0) {
        //        assert(!inv_next_copy_row);
        //        inv_next_copy_row = true;
        //    }
        //}
        
        // END - CROW

        /*** 1. Serve completed reads ***/
        if (pending.size()) {
            Request& req = pending[0];
            if (req.depart <= clk) {
                switch(req.type) {
                    case Request::Type::READ: {
                        if (req.depart - req.arrive > 1) { // this request really accessed a row
                            read_latency_sum += req.depart - req.arrive;
                            read_latency_sum_per_core[req.coreid] += (req.depart - req.arrive);
                            channel->update_serving_requests(
                                req.addr_vec.data(), -1, clk);
                            }
                        //printf("Ctrl: Finishing req type %d, addr %ld\n", req.type, req.addr); // debug
                        if (req.callback != nullptr){
                            req.callback(req);}

                        #ifdef PRINT_CMD_TRACE
                            printf("req_uid:%lld\tCompleting READ req. arrival:%ld, total latency:%ld\n", req.req_unique_id, req.arrive, req.depart - req.arrive);
                        #endif
                        
                        break;
                    }
                    case Request::Type::REF_STATUS_QUERY: {
                        assert(smd_mode == SMD_MODE::RSQ || smd_mode == SMD_MODE::ALERT);
                        smd_ref_status_responses++;

                        for(uint32_t chip_id = 0; chip_id < chips_per_rank; chip_id++) {
                            smd_ref_tracker.update(chip_id, req.addr_vec[2], get_smd_refresher(req.addr_vec[1], chip_id)->communicate_locked_SAs(req.addr_vec[2]));
                            if(smd_ecc_scrubbing_enabled)
                                smd_scrub_tracker.update(chip_id, req.addr_vec[2], get_smd_scrubber(req.addr_vec[1], chip_id)->communicate_locked_SAs(req.addr_vec[2]));
                        }

                        if(smd_mode == SMD_MODE::ALERT)
                            smd_alerts.remove(req.addr_vec[1]);

                        smd_ref_tracker.unmark_inflight_req(req.addr_vec[2]);
                        if(smd_ecc_scrubbing_enabled)
                            smd_scrub_tracker.unmark_inflight_req(req.addr_vec[2]);
                        break;
                    }

                    default:
                        assert(false && "[Controller] ERROR: Completing an unexpected request type.");
                }

                pending.pop_front();
            }
        }

        if (pending_act_nack.size() > 0) {
            Request& req = pending_act_nack[0];

            if (req.depart <= clk) {
                switch (req.type){
                    case Request::Type::ACT_NACK: {
                        // We should revert the row buffer and timing states at this point
                        //      a) the row should be closed
                        //      b) the timing should be updated such that ACT is possible after ACT_NACK_RESEND_INTERVAL

                        #ifdef PRINT_CMD_TRACE
                        printf("ACT_NACK: Canceling row activate: ch: %u, r: %u, bg: %u, b: %u, r: %u\n", req.addr_vec[0], req.addr_vec[1],
                            req.addr_vec[2], req.addr_vec[3], req.addr_vec[4]);
                        #endif
                        channel->close_row(req.addr_vec, clk);
                        rowtable->update(T::Command::ACT_NACK, req.addr_vec, clk);
                        break;
                    }

                    case Request::Type::ACT_PARTIAL_NACK: {

                        // some chips have the requested row activated. The memory controller has to send 
                        // a precharge command to this bank prior to activating another row from the same bank

                        // TODO: Instead of proactively precharging, it would be better to precharge 
                        // if there is a request in the queue that target a different row from the same bank

                        #ifdef PRINT_CMD_TRACE
                        printf("ACT_PARTIAL_NACK: Canceling row activate: ch: %u, r: %u, bg: %u, b: %u, r: %u\n", req.addr_vec[0], req.addr_vec[1],
                            req.addr_vec[2], req.addr_vec[3], req.addr_vec[4]);
                        #endif

                        channel->mark_partial_act_nack(req.addr_vec, clk);

                        break;
                    }

                    default:
                        assert(false && "[Controller] ERROR: Completing an unexpected request type.");
                }

                pending_act_nack.pop_front();
            }
        }

        /*** 2. Refresh scheduler ***/
        if (!(refresh_disabled || smd_enabled))
            refresh->tick_ref();

        if (enable_raidr)
            raidr.tick_ref();

        if (enable_graphene)
            graphene.tick();

        /*** 2.5 SMD Refresh ***/
        if (smd_enabled) {
            
            /* 
            A chip has asserted alert, updated the MR, and started refreshing the target SA that was found to be precharged.
            While the memory controller was processing the alert: waiting for the maint_status query response or for the command bus to be ready
                the same DRAM chip asserts the alert signal again, updates the MR, and starts refreshing a new SA
                OR a different DRAM chip asserts its alert signal, updates the MR, and starts refreshing an SA
                    in simulation, the SMDTracker will get the latest (i.e., updated MR) locked SA when the query response arrives
                    the memory controller need to update the alert_clk based on the last clk alert is signalled
                    last_alert_clk is shared among all chips in a rank (i.e., using per rank last_alert_clk)
            */

            if(smd_mode == SMD_MODE::ALERT) {
                uint32_t num_ranks = (uint32_t) channel->spec->org_entry.count[int(T::Level::Rank)];
                for(uint32_t rank_id = 0; rank_id < num_ranks; rank_id++) {
                    bool is_ref_alert_set = false;
                    bool is_scrub_alert_set = false;
                    for(uint32_t chip_id = 0; chip_id < chips_per_rank; chip_id++) {
                        is_ref_alert_set |= get_smd_refresher(rank_id, chip_id)->get_and_clear_ref_alert();
                        if(smd_ecc_scrubbing_enabled)
                            is_scrub_alert_set |= get_smd_scrubber(rank_id, chip_id)->get_and_clear_ref_alert();
                    }

                    if (is_ref_alert_set) {
                        smd_alerts.push_back(rank_id);
                        smd_ref_tracker.update_alert_clk(rank_id);
                    }

                    if (is_scrub_alert_set) {
                        smd_alerts.push_back(rank_id);
                        if(smd_ecc_scrubbing_enabled)
                            smd_scrub_tracker.update_alert_clk(rank_id);
                    }

                    // TODO_hasan: For especially SMD_MODE::ALERT, we need to modify the address mapping scheme such that sequential addresses target different subarrays in different banks.
                    // For example, after covering a row from SA0 in bank0, the consecutive accesses should target SAX in bankY, where X and Y are not 0.
                }
            }

            if (smd_alerts.count() > 0) {
                // got a REF alert. Immediately issue a command to query the ref status
                typename T::Command _cmd = T::Command::RSQ;
                if(smd_query_ref_status((uint32_t)smd_alerts.front(), _cmd) == 0){
                    // RSQ is issued successfully
                    smd_alerts.process_front();
                }
                // if issuing RSQ fails, the controller will try again until all ranks in alerted_ranks are cleared
                
                return;
            }

            
            for (auto& smd_ref : smd_refreshers) {
                smd_ref->tick();
            }

            if(smd_ecc_scrubbing_enabled){
                for (auto& smd_scrub : smd_scrubbers) {
                    smd_scrub->tick();
                }
            }

            if(smd_rh_protection_enabled) {
                for (auto& rh_protector : smd_rh_protectors) {
                    rh_protector->tick();
                }
            }
        }

        if (enable_scrubbing)
            scrubber->tick();
        

        /*** 3. Should we schedule writes? ***/
        if (!write_mode) {
            // yes -- write queue is almost full or read queue is empty
            if (writeq.size() >= int(0.8 * writeq.max) /*|| readq.size() == 0*/){
                //printf("Switched to WRITE mode \n"); // DEBUG
                write_mode = true;
            }
        }
        else {
            // no -- write queue is almost empty and read queue is not empty
            if (writeq.size() <= int(0.2 * writeq.max) && readq.size() != 0) {
                //printf("Switched to READ mode \n"); // DEBUG
                write_mode = false;
            }
        }

        /*** 4. Find the best command to schedule, if any ***/
        // Second check the actq (which has higher priority) to see if there
        // are requests available to service in this cycle
        Queue* queue = &actq;
        typename T::Command cmd;
        auto req = scheduler->get_head(queue->q);

        bool is_valid_req = (req != queue->q.end());

        // FIX: Hasan: this seems redundant since get_head() already calls get_first_cmd() and is_ready() and returns a req that is ready
        // However, get_head() returns the first req in the queue (regardless of whether it is ready or not) and this is why we check readiness again
        // It will be better if get_head() returns q.end() when no request is ready
        // UPDATE: changed get_head() to return q.end() when there is no ready request
        if(is_valid_req) {
             cmd = get_first_cmd(req);
            //printf("ACTQueue -- will issue cmd type: %d to bg: %d ba: %d sa: %d row: %d fc:%d pn:%d\n",
            //    req->type, req->addr_vec[int(T::Level::BankGroup)], req->addr_vec[int(T::Level::Bank)],
            //    req->addr_vec[int(T::Level::Subarray)], req->addr_vec[int(T::Level::Row)], req->is_first_command,
            //    req->partially_nacked);

        //     is_valid_req = is_ready(cmd, req->addr_vec);
        //     //printf("Ctrl: Found a actq req type %d, addr %ld\n", req->type, req->addr);
        }

        bool perform_precharge = false;

        // "this block of code" decides whether or not to perform a PREcharge operation
        // in case the COMBINED policy is being used to deal with partial NACKs
        // note that it applies the WAIT policy when nack_threshold is set to something very large


        // TODO: another bug: when smd state changes, we will forget about scheduling the precharge
        // command even if previously an act got nacked, which is not realistic.
        // Redesign the code, (1) store NACK information for requests when they are NACKed
        // (2) based on NACK information, decide to perform PRE or WAIT depending on policy
        // (3) do not forget to deprioritize NACKed and then PREcharged requests

        // NOTE: now we only check if there is a request in the actq, but do not wait
        // until it becomes valid to try to schedule a PRE command
        list<Request>::iterator request_to_erase;
        if(smd_enabled && (smd_partial_nack_combined_threshold < max(readq.max, writeq.max)) && actq.size())
        {
            // Traverse the actq to find requests that were partially nacked 
            for (auto actq_req = actq.q.begin() ; actq_req != actq.q.end() ; actq_req++)
            {
                if (actq_req->partially_nacked)
                {
                    // find if the nacked request is accessing a precharged bank
                    int ra = actq_req->addr_vec[1];
                    int bg = actq_req->addr_vec[2];
                    int ba = actq_req->addr_vec[3];
                    bool bank_precharged = channel->children[ra]->children[bg]->children[ba]->state == T::State::Closed;

                    typename T::Command my_cmd = get_first_cmd(actq_req);

                    // if the nacked request is accessing an open bank
                    // and its trying to ACT the bank (i.e., nacked request 
                    // for the next chance)
                    if (my_cmd == T::Command::ACT && !bank_precharged)
                    {
                        Queue* temp_queue = !write_mode ? &readq : &writeq;

                        // find the subarray the partially nacked request is accessing
                        int partially_nacked_subarray = actq_req->addr_vec[int(T::Level::Subarray)];
                        int partially_nacked_bank = actq_req->addr_vec[int(T::Level::Bank)] + actq_req->addr_vec[int(T::Level::BankGroup)] * 4;

                        // count the number of requests that access the same bank but other subarrays
                        // (the requests that can be serviced if we precharge the bank)
                        int requests_to_other_subarrays = 0;
                        
                        for (auto &queue_req : temp_queue->q)
                        {
                            int this_requests_subarray = queue_req.addr_vec[int(T::Level::Subarray)];
                            int this_requests_bank = queue_req.addr_vec[int(T::Level::Bank)] + queue_req.addr_vec[int(T::Level::BankGroup)] * 4;
                            
                            if ((this_requests_bank == partially_nacked_bank) && (this_requests_subarray != partially_nacked_subarray))
                                requests_to_other_subarrays++;
                        }

                        // As per combined policy dictates, try to precharge the bank now that
                        // there are requests that can benefit from the bank being closed
                        if (requests_to_other_subarrays >= smd_partial_nack_combined_threshold)
                        {
                            //printf("Try to perform precharge...\n");
                            perform_precharge = true;
                            // We will try to perform a precharge operation ASAP
                            
                            if(is_ready(T::Command::PRE, actq_req->addr_vec))
                            {
                                my_cmd = T::Command::PRE;
                                // Demote the request back to the queue it belongs
                                Queue* new_queue = actq_req->type == Request::Type::READ ? &readq : &writeq;
                                actq_req->is_first_command = true;
                                // Does not delay if there is only one request
                                // TODO: but isn't that OK? Why do we want to delay further than nACK resend?
                                // assuming it will be delayed by at least nack_resend interval because of timings
                                actq_req->arrive += smd_partial_nack_resend_interval; //TODO: is this OK? Check if this is good enough
                                std::vector<int> addr_vec = get_addr_vec(my_cmd,actq_req);
                                issue_cmd(my_cmd, addr_vec, actq_req, false, false);
                                new_queue->q.push_back(*actq_req);
                                request_to_erase = actq_req;
                                //printf("Successfully perform a precharge!\n");
                                break; // We will schedule one precharge anyways, so stop iterating over other actq entries
                            }
                            else
                            {
                                //printf("... Couldn't because timings do not hold\n");
                                perform_precharge = false;
                            }

                        }

                    }
                }
            }

            if (perform_precharge)
            {
                actq.q.erase(request_to_erase);
                return;
            }

        }


        //if (req == queue->q.end() || !is_ready(cmd, req->addr_vec)) {
        if (!is_valid_req) {
            //printf("Ctrl: Couldnt issue a actq req type %d, addr %ld\n", req->type, req->addr);

            queue = !write_mode ? &readq : &writeq;

            if (otherq.size())
            {
                //printf("Select otherq, size: %d\n", otherq.size());
                queue = &otherq;  // "other" requests are rare, so we give them precedence over reads/writes
            }

            req = scheduler->get_head(queue->q);

            is_valid_req = (req != queue->q.end());

            // FIX: Hasan: this seems redundant since get_head() already calls get_first_cmd() and is_ready() and returns a req that is ready
            // However, get_head() returns the first req in the queue (regardless of whether it is ready or not) and this is why we check readiness again
            // It will be better if get_head() returns q.end() when no request is ready
            if(is_valid_req){

                //printf("Ctrl: Found a otherq req type %d, addr %ld\n", req->type, req->addr);
                cmd = get_first_cmd(req);
                //is_valid_req = is_ready(cmd, req->addr_vec);

                //printf("WRQueue -- will issue cmd type: %d to bg: %d ba: %d sa: %d row: %d fc: %d pn: %d\n",
                //req->type, req->addr_vec[int(T::Level::BankGroup)], req->addr_vec[int(T::Level::Bank)],
                //req->addr_vec[int(T::Level::Subarray)], req->addr_vec[int(T::Level::Row)], req->is_first_command,
                //req->partially_nacked);

                //if(!is_valid_req)
                    //printf("Ctrl: Couldnt issue a otherq req type %d, addr %ld\n", req->type, req->addr);
            
            }

            if(smd_enabled && smd_mode == SMD_MODE::RSQ && !is_valid_req) {
                bool has_timed_out_req, has_SA_conflict;
                const Request* inv_req = smd_has_ready_but_timed_out_req(queue->q, has_timed_out_req, has_SA_conflict);
                
                if(has_timed_out_req)
                    smd_ready_but_timed_out_req++;

                if (has_SA_conflict) {
                    smd_ready_but_SA_conflict_req++;
                    // std::cout << "[Controller] clk: " << clk << " SMD ready but SA conflict req: ch: " << inv_req->addr_vec[0];
                    // std::cout << " global_bank: " << channel->spec->calc_global_bank_id(req->addr_vec) << " SA: ";
                    // std::cout << req->addr_vec[int(T::Level::Row)]/channel->spec->get_subarray_size() << std::endl;

                    // printf("MST Conflict Req - bg:%d b:%d gb:%d row:%d, sa:%d \n", inv_req->addr_vec[2], inv_req->addr_vec[3], inv_req->addr_vec[2]*channel->spec->org_entry.count[int(T::Level::Bank)] + inv_req->addr_vec[3], inv_req->addr_vec[4], inv_req->addr_vec[4]/channel->spec->get_subarray_size());

                    // printf("=== clk: %ld Queue %s === \n", clk, queue == &readq ? "(READ)" : "(WRITE)");
                    // for(auto req : queue->q) {
                    //     printf("bg:%d b:%d gb:%d row:%d, sa:%d \n", req.addr_vec[2], req.addr_vec[3], req.addr_vec[2]*channel->spec->org_entry.count[int(T::Level::Bank)] + req.addr_vec[3], req.addr_vec[4], req.addr_vec[4]/channel->spec->get_subarray_size());
                    // }

                    // printf("=== Refresh Status Table \n");
                    // smd_ref_tracker.print();
                    // printf("=== ===\n");

                    for(uint32_t chip_id = 0; chip_id < chips_per_rank; chip_id++) {
                        if(get_smd_refresher(inv_req->addr_vec[1], chip_id)->is_SA_under_maintenance(channel->spec->calc_global_bank_id(inv_req->addr_vec), inv_req->addr_vec[int(T::Level::Subarray)])) {
                            smd_ready_but_SA_really_refreshing++;
                            break;
                        }
                    }
                }
            }

            // implement here the code to check if a request was not
            // ready due to waiting for tRCD or tRAS (separate stats for each)
            // HASAN: disabling it since those checks are expensive
            // if (req == queue->q.end() || !is_ready(req)) {
            //    if(req_hold_due_trcd(queue->q))
            //        crow_idle_cycle_due_trcd++;

            //    if(req_hold_due_tras(queue->q))
            //        crow_idle_cycle_due_tras++;
            // }
        }
        

        // CROW
        //bool inv_skipped = true;
        //vector<int> crow_addr_vec(int(T::Level::MAX), -1);
        //if(enable_crow && inv_next_copy_row) {
        //    typename T::Command cmd;

        //    CROWEntry* cur_entry = crow_table->get_entry(crow_table_inv_index, 
        //            crow_table_inv_index[int(T::Level::Row) + 1]);
        //    uint row_id = cur_entry->row_addr;
        //    bool valid_bit = cur_entry->valid;

        //    // activate the row that needs to be invalidated if the FR bit
        //    // is set. Activation is not required if that row is already
        //    // activated.
        //    int active_row_id = rowtable->get_open_row(crow_table_inv_index);

        //    if ((active_row_id == (int)row_id) || !valid_bit) {
        //        // already activated or there is no valid row in the
        //        // corresponding entry in the crow_table
        //        inv_next_copy_row = false;
        //        update_crow_table_inv_index();
        //    } else if (active_row_id != -1) {
        //        // if another row is open, need to precharge it first, then
        //        // fully activate the row to discard
        //        
        //        auto crow_cmd = T::Command::PRE;

        //        for(uint i = 0; i < int(T::Level::Row); i++)
        //            crow_addr_vec[i] = crow_table_inv_index[i];
        //        crow_addr_vec[int(T::Level::Row)] = active_row_id;

        //        if(channel->check_iteratively(crow_cmd, crow_addr_vec.data(), clk)) {
        //            cmd = crow_cmd;
        //            inv_skipped = false;
        //            crow_invPRE++;
        //        }
        //    } else {
        //        // if the bank is closed, we issue an activate with full
        //        // restoration

        //        auto crow_cmd = T::Command::ACT;

        //        for(uint i = 0; i < int(T::Level::Row); i++)
        //            crow_addr_vec[i] = crow_table_inv_index[i];

        //        crow_addr_vec[int(T::Level::Row)] = row_id;

        //        if(channel->check_iteratively(crow_cmd, crow_addr_vec.data(), clk)) {
        //            cmd = crow_cmd;
        //            inv_skipped = false;
        //            crow_invACT++;

        //            inv_next_copy_row = false;

        //            //crow_table->invalidate(crow_table_inv_index, 
        //            //        crow_table_inv_index[int(T::Level::Row) + 1]);
        //            update_crow_table_inv_index();
        //        }
        //    }
        //
        //    if (!inv_skipped) {
        //        issue_cmd(cmd, crow_addr_vec, (cmd == T::Command::ACT) ? true : false);
        //        return;
        //    }
        //    // END - CROW
        //}

        //if (req == queue->q.end() || !is_ready(req)) {
        if (!is_valid_req) {
            // we couldn't find a command to schedule -- let's try to be speculative
            auto cmd = T::Command::PRE;
            vector<int> victim = rowpolicy->get_victim(cmd);
            if (!victim.empty()){
                #ifdef PRINT_CMD_TRACE
                std::cout << "[TimeoutPolicy] Precharge by the timeout policy!" << std::endl;
                #endif
                issue_cmd(cmd, victim, readq.q.end());
                num_speculative_precharges++;
            }
            else if (smd_enabled && smd_mode == SMD_MODE::RSQ){
                // no command to issue, can't even perform a PRE
                // use this cycle to update the SMD REF status (if needed)
                smd_query_ref_status();
                smd_opportunistic_ref_status_queries++;
            }

            return;  // nothing more to be done this cycle
        }

        if (req->is_first_command) {
            req->is_first_command = false;
            int coreid = req->coreid;
            if (req->type == Request::Type::READ || req->type == Request::Type::WRITE || req->type == Request::Type::PREFETCH) {
              channel->update_serving_requests(req->addr_vec.data(), 1, clk);
            }
            int tx = (channel->spec->prefetch_size * channel->spec->channel_width / 8);
            if (req->type == Request::Type::READ || req->type == Request::Type::PREFETCH) {
                if (is_row_hit(req)) {
                    ++read_row_hits[coreid];
                    ++row_hits;
                } else if (is_row_open(req)) {
                    ++read_row_conflicts[coreid];
                    ++row_conflicts;
                } else {
                    ++read_row_misses[coreid];
                    ++row_misses;
                    // printf("Read row miss: %d, coreid: %d \n", (int)read_row_misses[coreid].value(), coreid); // Hasan: DEBUG
                }
              read_transaction_bytes += tx;
            } else if (req->type == Request::Type::WRITE) {
              if (is_row_hit(req)) {
                  ++write_row_hits[coreid];
                  ++row_hits;
              } else if (is_row_open(req)) {
                  ++write_row_conflicts[coreid];
                  ++row_conflicts;
              } else {
                  ++write_row_misses[coreid];
                  ++row_misses;
              }
              write_transaction_bytes += tx;
            }
        }

        // issue command on behalf of request
        // auto cmd = get_first_cmd(req);

        // CROW

        // if going to activate a new row which will discard an entry from
        // the CROW table, we may need to fully restore the discarding row
        // first

        bool make_crow_copy = true;
        if (enable_crow && channel->spec->is_opening(cmd)) {
            vector<int> target_addr_vec = get_addr_vec(cmd, req);
            if(!crow_table->is_hit(target_addr_vec) && crow_table->is_full(target_addr_vec)) {
                bool discard_next = true;

                if(prioritize_evict_fully_restored) {
                    assert(false && "Error: Unimplemented functionality!");
                    //int not_FR_ind = crow_table->find_not_FR(target_addr_vec);
                    
                    //if(not_FR_ind != -1) {
                    //    discard_next = false;
                    //    crow_table->invalidate(target_addr_vec, not_FR_ind);
                    //}

                }

                if(discard_next) {
                    CROWEntry* cur_entry = crow_table->get_LRU_entry(target_addr_vec, crow_evict_threshold);
                    //CROWEntry* cur_entry = get_discarding_entry(target_addr_vec);
                    
                    if(cur_entry == nullptr) {
                        assert(!enable_tl_dram && "Error: It should always be possible to discard an entry with TL-DRAM.");
                        make_crow_copy = false;
                        crow_bypass_copying++;
                    }
                    else {
                        if(cur_entry->FR && !enable_tl_dram) {
                            // We first need to activate the discarding row to fully
                            // restore it
                            
                            //printf("CROW: clk: %lu, fully restoring %d, %d, %d, %d, %d \n", clk, target_addr_vec[0],
                              //      target_addr_vec[1], target_addr_vec[2], target_addr_vec[3], row_id);
                            target_addr_vec[int(T::Level::Row)] = cur_entry->row_addr;

                            //crow_table->invalidate(target_addr_vec, discard_ind);

                            issue_cmd(cmd, target_addr_vec, req, true); 
                            crow_full_restore++;
                            return;
                        } else {
                            // move to LRU
                            target_addr_vec[int(T::Level::Row)] = cur_entry->row_addr;
                            crow_table->make_LRU(target_addr_vec, cur_entry);
                            crow_skip_full_restore++;
                        }
                    }
                }
            }
            else if (crow_table->is_hit(target_addr_vec) && enable_tl_dram) {
                if(req->type == Request::Type::WRITE){
                    crow_table->invalidate(target_addr_vec);
                    tl_dram_invalidate_due_to_write++;
                }
            }

        }

        if((enable_crow && enable_tl_dram) && ((cmd == T::Command::WR) || (cmd == T::Command::WRA))) {
            vector<int> target_addr_vec = get_addr_vec(cmd, req);
            target_addr_vec[int(T::Level::Row)] = rowtable->get_open_row(target_addr_vec);
            //assert(crow_table->get_hit_entry(target_addr_vec) && "Error: The currently open row in a bank should always hit in CROWTable!");
            // this assertion may not be true right after warmup finishes

            CROWEntry* cur_entry = crow_table->get_hit_entry(target_addr_vec);
            if(cur_entry != nullptr && cur_entry->total_hits != 0) {
                // convert the command to precharge
                cmd = T::Command::PRE;
                if(is_ready(T::Command::PRE, target_addr_vec)){
                    issue_cmd(cmd, target_addr_vec, req, true);
                    tl_dram_precharge_cached_row_due_to_write++;
                } else {
                    tl_dram_precharge_failed_due_to_timing++;
                }
                return;
            }
        }
        // END - CROW


        // the memory controller may keep a row open for too long preventing the rows in the same subarray from being refreshed by SMD
        // this may happen because of a long stream of accesses to the same row
        // Therefore, we force the memory controller to precharge rows kept open for a long time
        if(/*smd_enabled && (smd_mode == SMD_MODE::RSQ || smd_mode == SMD_MODE::ACT_NACK) &&*/ channel->spec->is_accessing(cmd)) {
            // check if the target row remained open for a long time
            // If so, convert the column command to RDA/WRA to precharge the row following the column command
            // this is to prevent the rows in the subarray from being refreshed by SMD for a long time

            // printf("active_time: %d, max_active_time: %d\n", rowtable->get_row_open_interval(req->addr_vec), ref_tracker_timeout_period*smd_max_row_open_intervals); // DEBUG

            assert(!channel->spec->is_closing(cmd));
            if(rowtable->get_row_open_interval(req->addr_vec) > ref_tracker_timeout_period*smd_max_row_open_intervals) {
                cmd = channel->spec->add_autoprecharge(cmd);
                // Note: RD/WR and RDA/WRA have the same timings to is_ready() should be still true after adding an autoprecharge

                smd_force_long_active_row_precharge++;
            }
            
        }

        std::vector<int> addr_vec = get_addr_vec(cmd,req);

        #ifdef PRINT_CMD_TRACE
            printf("req_uid:%lld\t", req->req_unique_id);
        #endif
        issue_cmd(cmd, addr_vec, req, false, make_crow_copy);


        if(channel->spec->is_opening(cmd)) {
            // promote the request that caused issuing activation to actq
            if(req->type != Request::Type::PARA_REFRESH)
                actq.q.push_back(*req);

            if(req->type == Request::Type::PARA_REFRESH)
                num_para_acts++;
                
            if(req->type == Request::Type::RAIDR_REFRESH)
                num_raidr_acts++;

            queue->q.erase(req);
            return;
        }

        // nothing else to do for precharges (e.g., PRE and PREA)
        if (channel->spec->is_closing(cmd) && !channel->spec->is_accessing(cmd)) {
            return;
        }

        assert(channel->spec->is_accessing(cmd) || channel->spec->is_refreshing(cmd));

        // set a future completion time for read requests
        if (req->type == Request::Type::READ || req->type == Request::Type::PREFETCH) {
            req->depart = clk + channel->spec->read_latency;
            pending.push_back(*req);
        }

        if (req->type == Request::Type::WRITE) {
            channel->update_serving_requests(req->addr_vec.data(), -1, clk);
        }

        // remove request from queue
        queue->q.erase(req);
    }

    // CROW
    bool req_hold_due_trcd(list<Request>& q) {
        // go over all requests in 'q' and check if there is one that could
        // have been issued if tRCD was 0.
        
        for (auto it = q.begin(); it != q.end(); it++) {
            if(is_ready_no_trcd(it))
                return true;
        }

        return false;
    }

    bool req_hold_due_tras(list<Request>& q) {
        // go over all requests in 'q' and check if there is one that could
        // have been issued if tRAS was 0.
        
        for (auto it = q.begin(); it != q.end(); it++) {
            if(is_ready_no_tras(it))
                return true;
        }

        return false;
    }

    bool is_ready_no_trcd(list<Request>::iterator req) {
        typename T::Command cmd = get_first_cmd(req);

        if(!channel->spec->is_accessing(cmd))
            return false;

        return channel->check_no_trcd(cmd, req->addr_vec.data(), clk);
    }

    bool is_ready_no_tras(list<Request>::iterator req) {
        typename T::Command cmd = get_first_cmd(req);

        if(cmd != T::Command::PRE)
            return false;

        return channel->check_no_tras(cmd, req->addr_vec.data(), clk);
    }
    // END - CROW

    bool is_ready(list<Request>::iterator req)
    {
        typename T::Command cmd = get_first_cmd(req);
        // return channel->check_iteratively(cmd, req->addr_vec.data(), clk);
        // return channel->check(cmd, req->addr_vec.data(), clk);

        if (cmd == T::Command::NOP)
            return false;

        return is_ready(cmd, req->addr_vec);
    }

    bool is_ready(typename T::Command cmd, const vector<int>& addr_vec)
    {
        // return channel->check_iteratively(cmd, addr_vec.data(), clk);
        // return channel->check(cmd, addr_vec.data(), clk);

        bool check_status = channel->check(cmd, addr_vec.data(), clk);

        if (check_status && smd_enabled && smd_mode == SMD_MODE::RSQ && cmd == T::Command::ACT)
            return (smd_ref_tracker.can_open(addr_vec) == 1) && (!smd_ecc_scrubbing_enabled || smd_scrub_tracker.can_open(addr_vec) == 1);

        return check_status;
    }

    int smd_is_ready (const Request& req) {
        typename T::Command cmd = get_first_cmd(req);

        if (cmd == T::Command::NOP)
            return false;

        bool timing_check = channel->check(cmd, req.addr_vec.data(), clk);

        if (timing_check && cmd == T::Command::ACT)
            return (smd_ref_tracker.can_open(req.addr_vec) == 1) && (!smd_ecc_scrubbing_enabled || smd_scrub_tracker.can_open(req.addr_vec) == 1);

        return timing_check;
    }

    bool is_row_hit(list<Request>::iterator req)
    {
        // cmd must be decided by the request type, not the first cmd
        typename T::Command cmd = channel->spec->translate[int(req->type)];
        return channel->check_row_hit(cmd, req->addr_vec.data());
    }

    bool is_row_hit(typename T::Command cmd, const vector<int>& addr_vec)
    {
        return channel->check_row_hit(cmd, addr_vec.data());
    }

    bool is_row_open(list<Request>::iterator req)
    {
        // cmd must be decided by the request type, not the first cmd
        typename T::Command cmd = channel->spec->translate[int(req->type)];
        return channel->check_row_open(cmd, req->addr_vec.data());
    }

    bool is_row_open(typename T::Command cmd, const vector<int>& addr_vec)
    {
        return channel->check_row_open(cmd, addr_vec.data());
    }

    bool is_bank_precharged(typename T::Command cmd, const vector<int>& addr_vec)
    {
        return !channel->check_row_open(cmd, addr_vec.data());
    }

    // void update_temp(ALDRAM::Temp current_temperature)
    // {
    // }

    // For telling whether this channel is busying in processing read or write
    bool is_active() {
      return (channel->cur_serving_requests > 0);
    }

    // For telling whether this channel is under refresh
    bool is_refresh() {
      return clk <= channel->end_of_refreshing;
    }

    void record_core(int coreid) {
#ifndef INTEGRATED_WITH_GEM5
      record_read_hits[coreid] = read_row_hits[coreid];
      record_read_misses[coreid] = read_row_misses[coreid];
      record_read_conflicts[coreid] = read_row_conflicts[coreid];
      record_write_hits[coreid] = write_row_hits[coreid];
      record_write_misses[coreid] = write_row_misses[coreid];
      record_write_conflicts[coreid] = write_row_conflicts[coreid];
      record_read_latency_avg_per_core[coreid] = read_latency_sum_per_core[coreid].value()/
                                        (float)(read_row_hits[coreid].value() + read_row_misses[coreid].value() + 
                                                read_row_conflicts[coreid].value());
#endif
    }

    void reload_options(const Config& configs) {

        if((channel->spec->standard_name == "SALP-MASA") || (channel->spec->standard_name == "SALP-1") || 
                (channel->spec->standard_name == "SALP-2"))
            channel->update_num_subarrays(channel->spec->org_entry.count[int(T::Level::Row)]/channel->spec->get_subarray_size());

        // prioritize_evict_fully_restored = configs.get_bool("crow_evict_fully_restored");
        // collect_row_act_histogram = configs.get_bool("collect_row_activation_histogram");
        // copy_rows_per_SA = configs.get_int("copy_rows_per_SA");
        // weak_rows_per_SA = configs.get_int("weak_rows_per_SA");
        refresh_mult = configs.get_float("refresh_mult");
        // crow_evict_threshold = configs.get_int("crow_entry_evict_hit_threshold");
        // crow_half_life = configs.get_int("crow_half_life");
        // crow_to_mru_frac = configs.get_float("crow_to_mru_frac");
        // crow_table_grouped_SAs = configs.get_int("crow_table_grouped_SAs");

        // Adjust tREFI
        
        smd_partial_nack_combined_threshold = configs.get_int("smd_combined_policy_threshold");
        channel->spec->act_nack_interval_ns = configs.get_float("smd_act_nack_resend_interval");

        per_bank_refresh_on = configs.get_bool("per_bank_refresh");
        
        channel->spec->init_speed(); // init_speed() sets nREFI to its original value. So the order between
        channel->spec->speed_entry.nREFI *= refresh_mult; // these two lines must be preserved
        channel->spec->mult_refresh_pb(refresh_mult); // Atb: to multiply DSARP's tREFIpb tREFCpb

        channel->spec->init_prereq();
        if(per_bank_refresh_on){
            // making REF work as REFpb (per-bank REF)
            channel->spec->speed_entry.nREFI = ceil(channel->spec->speed_entry.nREFI/(float)channel->spec->get_num_banks_per_rank());
            channel->spec->speed_entry.nRFC = ceil(channel->spec->speed_entry.nRFC/2.0f);

            channel->spec->scope[int(T::Command::REF)] = T::Level::Bank;
            channel->spec->prereq[int(T::Level::Rank)][int(T::Command::REF)] = nullptr;

            otherq.q.clear(); // if there are pending rank-level REFs, they will have invalid bank address and break the simulation.
                            // Discarding such REFs and restarting from clean refresh state. This shouldn't lead to significant inaccuracy since we discard at most 1-2 REFs here.
            
            // init per bank ref command history
            channel->init_REFpb_history();
            

        } else {
            channel->spec->scope[int(T::Command::REF)] = T::Level::Rank;
        }

        smd_act_to_nack_cycles = configs.get_int("smd_act_to_nack_cycles");
        channel->spec->speed_entry.nACTtoNACK = smd_act_to_nack_cycles;

        channel->spec->init_timing(per_bank_refresh_on);

        refresh_disabled = configs.get_bool("disable_refresh"); 
        smd_enabled = configs.get_bool("smd");
        smd_mode = str_to_smd_mode[configs.get_str("smd_mode")];

        // std::cout << "Is SMD enabled: " << smd_enabled << std::endl; 

        if (copy_rows_per_SA > 0)
            enable_crow = true;

        if (configs.get_bool("enable_crow_upperbound")) {
            enable_crow_upperbound = true;
            enable_crow = false;
        }

        enable_tl_dram = configs.get_bool("enable_tl_dram");

        if(enable_crow || enable_crow_upperbound) {
            
            trcd_crow_partial_hit = configs.get_float("trcd_crow_partial_hit");
            trcd_crow_full_hit = configs.get_float("trcd_crow_full_hit");

            tras_crow_partial_hit_partial_restore = configs.get_float("tras_crow_partial_hit_partial_restore");
            tras_crow_partial_hit_full_restore = configs.get_float("tras_crow_partial_hit_full_restore");
            tras_crow_full_hit_partial_restore = configs.get_float("tras_crow_full_hit_partial_restore");
            tras_crow_full_hit_full_restore = configs.get_float("tras_crow_full_hit_full_restore");
            tras_crow_copy_partial_restore = configs.get_float("tras_crow_copy_partial_restore");
            tras_crow_copy_full_restore = configs.get_float("tras_crow_copy_full_restore");

            twr_partial_restore = configs.get_float("twr_partial_restore");
            twr_full_restore = configs.get_float("twr_full_restore");

            initialize_crow();

        }

        if(collect_row_act_histogram)
            assert(num_SAs <= 128);


        enable_para = configs.get_bool("enable_para");
        enable_raidr = configs.get_bool("enable_raidr");
        enable_graphene = configs.get_bool("enable_graphene");

        enable_scrubbing = configs.get_bool("enable_scrubbing");
        scrubber->reload_options(configs);
    }

    std::unordered_set<uint32_t> get_reqbuffer_banks(const uint32_t rank_id) const {
        std::unordered_set<uint32_t> target_banks;

        for(auto it = actq.q.begin(); it != actq.q.end(); it++) {
            if(it->addr_vec[int(T::Level::Rank)] == (int)rank_id)
                target_banks.insert(channel->spec->calc_global_bank_id(it->addr_vec));
        }

        for(auto it = readq.q.begin(); it != readq.q.end(); it++) {
            if(it->addr_vec[int(T::Level::Rank)] == (int)rank_id)
                target_banks.insert(channel->spec->calc_global_bank_id(it->addr_vec));
        }

        for(auto it = writeq.q.begin(); it != writeq.q.end(); it++) {
            if(it->addr_vec[int(T::Level::Rank)] == (int)rank_id)
                target_banks.insert(channel->spec->calc_global_bank_id(it->addr_vec));
        }


        return target_banks;
    }

    void issueDPowerCommand(const typename T::Command cmd, const uint32_t rank_id, const uint32_t gbid) {

        DRAMPower::MemCommand::cmds dpower_cmd = DRAMPower::MemCommand::NOP;
        switch(cmd) {
            case T::Command::ACT: {
                    dpower_cmd = DRAMPower::MemCommand::ACT;
                break;
            }
            case T::Command::PRE: {
                dpower_cmd = DRAMPower::MemCommand::PRE;
                break;
            }
            case T::Command::PREA: {
                dpower_cmd = DRAMPower::MemCommand::PREA;
                break;
            }
            case T::Command::RD: {
                dpower_cmd = DRAMPower::MemCommand::RD;
                break;
            }
            case T::Command::RDA: {
                dpower_cmd = DRAMPower::MemCommand::RDA;
                break;
            }
            case T::Command::WR: {
                dpower_cmd = DRAMPower::MemCommand::WR;
                break;
            }
            case T::Command::WRA: {
                dpower_cmd = DRAMPower::MemCommand::WRA;
                break;
            }
            case T::Command::REF: {
                if (per_bank_refresh_on)
                    dpower_cmd = DRAMPower::MemCommand::REFB;
                else
                    dpower_cmd = DRAMPower::MemCommand::REF;
                break;
            }
            // TODO: implement ACT_NACK and NACK'ed ACT commands

            default: {

                // assert(false && "ERROR: Unimplemented DRAMPower command!");
            }
        }

        dpower[rank_id].doCommand(dpower_cmd, gbid, clk);
    }

    void issueDPowerSMDREF(const uint32_t num_rows_refreshed, const uint32_t rank_id, const uint32_t gbid) {
        dpower[rank_id].doSMDRefCommand(num_rows_refreshed, gbid, clk);
    }

    void issueDPowerRowScrubbing(const uint32_t num_rows_scrubbed, const uint32_t rank_id, const uint32_t gbid) {
        dpower[rank_id].doRowScrubCommand(num_rows_scrubbed, gbid, clk);
    }


private:
    typename T::Command get_first_cmd(const Request& req) const {
        typename T::Command cmd = channel->spec->translate[int(req.type)];
        return channel->decode(cmd, req.addr_vec.data());
        // return channel->decode_iteratively(cmd, req.addr_vec.data());
    }

    typename T::Command get_first_cmd(list<Request>::iterator req) const {
        return get_first_cmd(*req);
    }
    

    unsigned long last_clk = 0; // DEBUG
    unsigned long num_cas_cmds = 0;
    void issue_cmd(typename T::Command cmd, vector<int>& addr_vec, list<Request>::iterator req, bool do_full_restore = false, bool make_crow_copy = true)
    {
        //assert(!is_full_restore && "Full restoration feature is not needed anymore. The corresponding code pieces in Controller.h are commented out.");

        // ===== Print the requests in all request queues =====
        // if(warmup_complete){
            // printf("Is write mode? : %s \n", write_mode ? "Yes" : "No");
            // printf("=== ActQ === \n");
            // for(auto req : actq.q) {
            //     printf("bg:%d b:%d row:%d \n", req.addr_vec[2], req.addr_vec[3], req.addr_vec[4]);
            // }
            // printf("=== ===\n");
            
            // printf("=== ReadQ === \n");
            // for(auto req : readq.q) {
            //     printf("bg:%d b:%d row:%d \n", req.addr_vec[2], req.addr_vec[3], req.addr_vec[4]);
            // }
            // printf("=== ===\n");

            
            // printf("=== WriteQ === \n");
            // for(auto req : writeq.q) {
            //     printf("bg:%d b:%d row:%d \n", req.addr_vec[2], req.addr_vec[3], req.addr_vec[4]);
            // }
            // printf("=== ===\n");
            // // ===================================

            // if(cmd == T::Command::ACT)
            //    printf("Controller (%lu): Cmd: %s, addr: %d %d %d \n", clk, channel->spec->command_name[int(cmd)].c_str(), 
            //        addr_vec[2], addr_vec[3], addr_vec[4]);

            // if(cmd == T::Command::ACT) {
            //     int SA_size = channel->spec->org_entry.count[int(T::Level::Row)]/num_SAs;
            //     printf("Controller %d: clk: %lu, ACT to c: %d r:%d bg:%d b:%d sa:%d row:%d \n", 
            //             channel->id, clk, addr_vec[0], addr_vec[1], addr_vec[2], addr_vec[3], 
            //                 addr_vec[4]/SA_size, addr_vec[4]);


            // }
        // }
        
        // ===== CMD Issue Timeline ===== //
        // assumes a single-channel, single-rank configuration
        // and DDR4 with bank-groups
        #ifdef PRINT_CMD_TRACE
        if(warmup_complete) {
            printf("%lu \t%lu:\t %lu %d\t|", clk, clk - last_clk, num_cas_cmds, (int)read_row_conflicts[0].value());
            last_clk = clk;

            if(cmd == T::Command::PREA){ 
                // rank-level precharge
                for(int bg = 0; bg < channel->spec->org_entry.count[int(T::Level::Bank) - 1]; bg++) {
                    for(int b = 0; b < channel->spec->org_entry.count[int(T::Level::Bank)]; b++) {
                        printf(" P ");
                    }
                    printf("|");
                }
            }
            else if (channel->spec->is_refreshing(cmd)) {
                // TODO: is this assertion still required?
                //assert(cmd == T::Command::REF || cmd == T::Command::REFpb);

                for(int bg = 0; bg < channel->spec->org_entry.count[int(T::Level::Bank) - 1]; bg++) {
                    for(int b = 0; b < channel->spec->org_entry.count[int(T::Level::Bank)]; b++) {
                        if (!per_bank_refresh_on /*&& smd_enabled*/) {
                            printf(" + ");
                            continue;
                        }

                        if((bg == addr_vec[int(T::Level::Bank) - 1]) && 
                                b == addr_vec[int(T::Level::Bank)]) {
                                    printf(" + ");
                        } else {
                            printf(" - ");
                        }
                    }
                    printf("|");
                }
            }
            else {
                for(int bg = 0; bg < channel->spec->org_entry.count[int(T::Level::Bank) - 1]; bg++) {
                    for(int b = 0; b < channel->spec->org_entry.count[int(T::Level::Bank)]; b++) {
                    if((bg == addr_vec[int(T::Level::Bank) - 1]) && 
                            b == addr_vec[int(T::Level::Bank)]) {
                            switch(cmd) {
                                case T::Command::ACT:
                                    // if(do_full_restore)
                                    //     printf(" A ");
                                    // else { 
                                    //     if(enable_crow && crow_table->is_hit(addr_vec))
                                    //         printf(" H ");
                                    //     else
                                    //         printf(" a ");
                                    // }

                                    if (is_smd_region_busy(addr_vec) != RegionBusyResponse::NO_CHIPS_BUSY)
                                        printf(" a ");
                                    else
                                        printf(" A ");
                                    break;
                                case T::Command::PRE:
                                    printf(" p ");
                                    break;
                                case T::Command::RD:
                                    printf(" r ");
                                    num_cas_cmds++;
                                    break;
                                case T::Command::RDA:
                                    num_cas_cmds++;
                                    printf(" R ");
                                    break;
                                case T::Command::WR:
                                    num_cas_cmds++;
                                    printf(" w ");
                                    break;
                                case T::Command::WRA:
                                    num_cas_cmds++;
                                    printf(" W ");
                                    break;
                                default:
                                    printf(" %d ", int(cmd));

                            }
                    } else {
                            printf(" - ");
                    }
                    }
                    printf("|");
                }
            }
            //if (cmd != T::Command::PRE)
                printf("\t\tSA: %d,\tRow: %d,\tCol %d, \tRank %d \tChannel %d", addr_vec[int(T::Level::Subarray)], addr_vec[int(T::Level::Row)], 
                    addr_vec[int(T::Level::Column)], addr_vec[int(T::Level::Rank)], addr_vec[int(T::Level::Channel)]);
            printf("\n");
            fflush(stdout);
        } //warmup_complete


        #endif // PRINT_CMD_TRACE

        // ===== END - CMD Issue Timeline ===== //

        assert(is_ready(cmd, addr_vec));
        

        if(warmup_complete && collect_row_act_histogram) {
            if(channel->spec->is_opening(cmd)) {
                int row_id = addr_vec[int(T::Level::Row)];
                int sa_id = addr_vec[int(T::Level::Subarray)];
                
                auto& cur_hist = row_act_hist[addr_vec[int(T::Level::Bank)]][sa_id];

                if(cur_hist.find(row_id) == cur_hist.end())
                    cur_hist[row_id] = 1;
                else
                    cur_hist[row_id]++;
            }
        }

        // DEBUG
        // if (channel->spec->is_opening(cmd)) {
        //     std::cout << "[Controller] clk: " << clk << " Activating bank: " << addr_vec[int(T::Level::Bank) - 1] << " " << addr_vec[int(T::Level::Bank)];
        //     std::cout << " row: " << addr_vec[int(T::Level::Row)] << " SA: " << addr_vec[int(T::Level::Row)]/channel->spec->get_subarray_size() << std::endl;
        // }
        // END - DEBUG



        // CROW
        int crow_hit = 0; // 0 -> no hit, 1 -> hit to partially restored row, 2 -> hit to fully restored row
        bool crow_copy = false;

        //if(cmd == T::Command::ACT){
        //    printf("Controller: clk: %lu Issuing command %d to %d %d %d %d %d \n", clk, int(cmd), 
        //        addr_vec[0], addr_vec[1], addr_vec[2], addr_vec[3], addr_vec[4]);
        //if(cmd == T::Command::ACT){
        //    crow_table->print();
        //}

        if(enable_crow || enable_crow_upperbound) {
            if(channel->spec->is_opening(cmd)) {
                if(enable_crow_upperbound || crow_table->is_hit(addr_vec)) {
                    assert(make_crow_copy && "Error: A row activation without copying should not hit on CROWTable!");

                    bool require_full_restore = false;
                   
                    if(!enable_crow_upperbound)
                        require_full_restore = crow_table->get_hit_entry(addr_vec)->FR;

                    if(require_full_restore)
                        crow_hit = 1;
                    else
                        crow_hit = 2;

                    //printf("Controller: crow_table HIT! \n");

                    if(!do_full_restore) {
                        crow_num_hits++;
                        if(enable_crow)
                            crow_table->access(addr_vec); // we shouldn't access it if we do additional activation to fully restore
                    } else {
                        crow_table->access(addr_vec, true); // move the fully restored row to LRU position
                    }
                    //else
                    //    printf("Controller: Activation for full restore! \n");

                    crow_num_all_hits++;

                    if(!enable_crow_upperbound && require_full_restore){
                        crow_num_hits_with_fr++;

                        //printf("CROW (%lu): Hit with FR bit set to: %d %d %d \n", clk, addr_vec[2], 
                        //        addr_vec[3], addr_vec[4]);

                        //if(crow_num_hits_with_fr.value() > crow_num_fr_set.value())
                        //    clk = clk;

                    }
                } else {
                    assert(!do_full_restore && "Full restoration should be issued only when the row is in the CROW table.");
                    assert((!enable_tl_dram || make_crow_copy) && "Error: ACT command should always copy when TL-DRAM is used.");
                    // crow miss
                    if(make_crow_copy) {
                        crow_copy = true;
                        crow_table->add_entry(addr_vec, false);
                        crow_num_copies++;
                    } else {
                        crow_table->access(addr_vec); // we know it is a miss but we access to update the hit_count of the LRU entry
                    }
                    
                    crow_num_misses++;

                    //printf("Controller: crow_table MISS! \n");
                }
            }

            if((cmd == T::Command::WR) || (cmd == T::Command::WRA)) {
               if(enable_crow_upperbound)
                  crow_hit = 2;
               else if(crow_table->is_hit(addr_vec)) {
                   if(crow_table->get_hit_entry(addr_vec)->FR)
                       crow_hit = 1;
                   else
                       crow_hit = 2;
               }
               else
                   // assert(false && "Can this happen?"); // Hasan:
                   // happens right after warmup when a row was opened
                   // before CROWTable initialization. Printing the message
                   // below to log how many times this happens to make sure
                   // it does not happen many times due to a different
                   // reason
                   //printf("Warning: Ramulator: Writing on a CROWTable miss!\n");
                   crow_hit = -1;
            }

            if((!enable_crow_upperbound && !enable_tl_dram) && channel->spec->is_closing(cmd)) {
                crow_set_FR_on_PRE(cmd, addr_vec);
            }

            
            if(!enable_crow_upperbound && channel->spec->is_refreshing(cmd)) {
                                
                int nREFI = channel->spec->speed_entry.nREFI;
                float tCK = channel->spec->speed_entry.tCK;
                int base_refw = is_LPDDR4 ? 32*refresh_mult : 64*refresh_mult;
                ulong ticks_in_ref_int = base_refw*1000000/tCK;
                int num_refs = ticks_in_ref_int/nREFI;

               
                int num_rows_refreshed_at_once;
                int num_ref_limit;
               
               if(is_DDR4 || is_LPDDR4) {
                  num_ref_limit = channel->spec->org_entry.count[int(T::Level::Row)];
                  num_rows_refreshed_at_once = ceil(channel->spec->org_entry
                                                    .count[int(T::Level::Row)]/(float)num_refs);
               }
               else { // For SALP
                   num_ref_limit = channel->spec->org_entry.count[int(T::Level::Row)] * 
                                    channel->spec->org_entry.count[int(T::Level::Row) - 1];
                   num_rows_refreshed_at_once = ceil(channel->spec->org_entry
                                                    .count[int(T::Level::Row)] * channel->spec->org_entry.count[int(T::Level::Row) - 1]/(float)num_refs);
               }

                //printf("Controller: Refreshing! \n");
                ref_counters[addr_vec[int(T::Level::Rank)]] += num_rows_refreshed_at_once;

                if(ref_counters[addr_vec[int(T::Level::Rank)]] >= num_ref_limit)
                    ref_counters[addr_vec[int(T::Level::Rank)]] = 0;
            }

            switch(crow_hit) {
                case 0:
                    if(crow_copy){
                        assert(make_crow_copy && "Error: Using copy timing parameters for regular access!");
                        load_timing(channel, crow_copy_timing);
                    }
                    break;
                case 1: // partial hit
                    assert(!crow_copy && "Error: A row should not be copied if is already duplicated!");

                    if(do_full_restore){
                        assert(cmd == T::Command::ACT);
                        assert(!enable_crow_upperbound);
                        load_timing(channel, partial_crow_hit_full_restore_timing); 
                    }
                    else
                        load_timing(channel, partial_crow_hit_partial_restore_timing);
                    break;
                case 2: // hit to a fully restored row
                    assert(!crow_copy && "Error: A row should not be copied if is already duplicated!");

                    if(do_full_restore){
                        assert(cmd == T::Command::ACT);
                        assert(!enable_crow_upperbound);
                        load_timing(channel, full_crow_hit_full_restore_timing);
                    }
                    else
                        load_timing(channel, full_crow_hit_partial_restore_timing);
            }
            
        }

        // END - CROW

        RegionBusyResponse smd_region_busy_resp = RegionBusyResponse::NO_CHIPS_BUSY;
        if (smd_enabled && smd_mode == SMD_MODE::ACT_NACK){
            if (cmd == T::Command::ACT){
                // schedule a ACT_NACK response if the target DRAM region is busy with some maintenance operation
                smd_region_busy_resp = is_smd_region_busy(addr_vec);
                switch(smd_region_busy_resp) {
                    case RegionBusyResponse::NO_CHIPS_BUSY: {

                        if(smd_rh_protection_enabled){
                            for(uint32_t chip_id = 0; chip_id < chips_per_rank; chip_id++) {
                                get_smd_rh_protector(addr_vec[uint32_t(T::Level::Rank)], chip_id)->process_row_activation(addr_vec);
                            }
                        }
                        break;
                    }
                    case RegionBusyResponse::ALL_CHIPS_BUSY: {
                        Request act_nack_resp(addr_vec, Request::Type::ACT_NACK, nullptr);
                        act_nack_resp.depart = clk + smd_act_to_nack_cycles;
                        pending_act_nack.push_back(act_nack_resp);
                        smd_act_nack_cnt++;
                        break;
                    }
                    case RegionBusyResponse::SOME_CHIPS_BUSY: {
                        Request act_nack_resp(addr_vec, Request::Type::ACT_PARTIAL_NACK, nullptr);
                        act_nack_resp.depart = clk + smd_act_to_nack_cycles;
                        pending_act_nack.push_back(act_nack_resp);
                        smd_act_partial_nack_cnt++;
                        req->partially_nacked = true;
                        break;
                    }
                    default: {
                        assert(false && "ERROR: Unhandled is_smd_region_busy() return value!");
                    }
                }
            }
        }

        if(cmd == T::Command::PRE){
            // replace subarray address in the addr_vec with the currently open subarray
            addr_vec.resize(int(T::Level::MAX) - 1);
            auto tmp_addr_vec = addr_vec;
            tmp_addr_vec[int(T::Level::Subarray)] = channel->get_open_SA(addr_vec);
            if(rowtable->get_hits(tmp_addr_vec, true) == 0){
                useless_activates++;
                //printf("Useless Activate \n"); // DEBUG

                //if(warmup_complete)
                //    assert(false); // DEBUG
            }
        }
        
        channel->update(cmd, addr_vec.data(), clk);


        // update command counting stats
        switch(cmd) {
            case T::Command::ACT: {
                if(smd_region_busy_resp == RegionBusyResponse::NO_CHIPS_BUSY)
                    num_act++;
                break;
            }
            case T::Command::PRE: {
                num_pre++;
                break;
            }
            case T::Command::PREA: {
                num_prea++;
                break;
            }
            case T::Command::RD: {
                num_rd++;
                break;
            }
            case T::Command::RDA: {
                num_rda++;
                break;
            }
            case T::Command::WR: {
                num_wr++;
                break;
            }
            case T::Command::WRA: {
                num_wra++;
                break;
            }
            case T::Command::REF: {
                num_ref++;
                break;
            }
            default: {
                // std::cerr << "ERROR: No counter stat for command type: " << int(cmd) << std::endl;
                // assert(false);
            }

        }

        // DRAMPower
        // do not send ACT commands that will be NACK'ed to DRAMPower 
        if((cmd != T::Command::ACT) || (smd_region_busy_resp == RegionBusyResponse::NO_CHIPS_BUSY)) {

            if(req != readq.q.end())
                req->partially_nacked = false;
            issueDPowerCommand(cmd, addr_vec[uint32_t(T::Level::Rank)], channel->spec->calc_global_bank_id(addr_vec));
        }
        // END - DRAMPower

        // Issue Neighbor row refreshes if memory controller level PARA is enabled
        if(enable_para && (cmd == T::Command::ACT)){
            if(flip_PARA_coin()) {
                auto neighbor_row_addr_vec = addr_vec;
                if (addr_vec[int(T::Level::Row)] > 0) {
                    neighbor_row_addr_vec[int(T::Level::Row)]--;
                    auto req_neighbor_ref = Request(neighbor_row_addr_vec, Request::Type::PARA_REFRESH, nullptr);
                    this->enqueue(req_neighbor_ref);
                }

                neighbor_row_addr_vec = addr_vec;
                if (addr_vec[int(T::Level::Row)] < (channel->spec->org_entry.count[int(T::Level::Row)] - 1)) {
                    neighbor_row_addr_vec[int(T::Level::Row)]++;
                    auto req_neighbor_ref = Request(neighbor_row_addr_vec, Request::Type::PARA_REFRESH, nullptr);
                    this->enqueue(req_neighbor_ref);
                }
            }
        } 

        if(enable_crow && do_full_restore && (cmd == T::Command::ACT)) {
            // clean just_opened state
            if(is_LPDDR4) {
                channel->children[addr_vec[int(T::Level::Rank)]]->
                    children[addr_vec[int(T::Level::Bank)]]->just_opened = false;
            }
            else {
                assert(false && "Not implemented for the current DRAM standard.");
            }
        }

        // CROW
        if(enable_crow)
            load_default_timing(channel);
        // END - CROW

        // SMD
        if (smd_enabled && (cmd == T::Command::PRE) && smd_mode == SMD_MODE::RSQ)
            smd_query_ref_status(addr_vec[int(T::Level::Rank)], cmd);

        // SMD -- END
        
        // SMD
        // weird way to implement now, when an ACT is sent
        if (enable_graphene && (cmd == T::Command::ACT))
            graphene.update(cmd, addr_vec, 36/*random*/);

        // SMD -- END

        rowtable->update(cmd, addr_vec, clk);
        
        if (record_cmd_trace){
            // select rank
            auto& file = cmd_trace_files[addr_vec[1]];
            string& cmd_name = channel->spec->command_name[int(cmd)];

            if(cmd_name != "SASEL") {
                if((cmd_name == "ACT") && (crow_copy || (crow_hit > 0)) && (!enable_crow_upperbound))
                    file << clk << ',' << "ACTD";
                else
                    file << clk << ',' << cmd_name;
                // TODO bad coding here
                if (cmd_name == "PREA" || cmd_name == "REF")
                    file<<endl;
                else{
                    int bank_id = 0;
                    if(channel->spec->standard_name == "SALP-MASA")
                        bank_id = addr_vec[int(T::Level::Bank)]*channel->spec->org_entry.count[int(T::Level::Bank) + 1] + addr_vec[int(T::Level::Bank) + 1];
                    else
                        bank_id = addr_vec[int(T::Level::Bank)];
                    if (channel->spec->standard_name == "DDR4" || channel->spec->standard_name == "GDDR5")
                        bank_id += addr_vec[int(T::Level::Bank) - 1] * channel->spec->org_entry.count[int(T::Level::Bank)];
                    file<<','<<bank_id<<endl;
                }
            }
        }
        if (print_cmd_trace){
            printf("%5s %10ld:", channel->spec->command_name[int(cmd)].c_str(), clk);
            for (int lev = 0; lev < int(T::Level::MAX); lev++)
                printf(" %5d", addr_vec[lev]);
            printf("\n");
        }
    }
    vector<int> get_addr_vec(typename T::Command cmd, list<Request>::iterator req){
        return req->addr_vec;
    }

    void load_timing(DRAM<T>* node, vector<typename T::TimingEntry> timing[][int(T::Command::MAX)]) {
        node->set_timing(timing[int(node->level)]);

        for(auto child : node->children)
            load_timing(child, timing);
    }

    void load_default_timing(DRAM<T>* node) {
        node->set_timing(node->spec->timing[int(node->level)]);

        for(auto child : node->children)
            load_default_timing(child);
    }

    void initialize_crow() {

        // 1. timing for CROW table hit on partially restored row when
        // intended to partially restore
        initialize_crow_timing(partial_crow_hit_partial_restore_timing, trcd_crow_partial_hit, 
                tras_crow_partial_hit_partial_restore, twr_partial_restore, 1.0f /*crow_hit_tfaw*/);

        // 2. timing for CROW table hit on partially restored row when
        // intended to fully restore
        initialize_crow_timing(partial_crow_hit_full_restore_timing, trcd_crow_partial_hit, 
                tras_crow_partial_hit_full_restore, twr_full_restore, 1.0f);

        // 3. timing for CROW table hit on fully restored row when
        // intended to partially restore
        initialize_crow_timing(full_crow_hit_partial_restore_timing, trcd_crow_full_hit, 
                tras_crow_full_hit_partial_restore, twr_partial_restore, 1.0f);

        // 4. timing for CROW table hit on fully restored row when
        // intended to fully restore
        initialize_crow_timing(full_crow_hit_full_restore_timing, trcd_crow_full_hit, 
                tras_crow_full_hit_full_restore, twr_full_restore, 1.0f);

        // 5. timing for CROW copy
        initialize_crow_timing(crow_copy_timing, 1.0f, tras_crow_copy_full_restore, twr_full_restore, 1.0f);
                
        if(enable_crow_upperbound)
            return;

        if(crow_table != nullptr)
            delete crow_table;

        crow_table = new CROWTable<T>(channel->spec, channel->id, num_SAs, copy_rows_per_SA, 
                weak_rows_per_SA, crow_evict_threshold, crow_half_life, crow_to_mru_frac,
                crow_table_grouped_SAs);

        // Hasan: CROW does not support refreshing at 2x or 4x mode yet
        //int refresh_mult = 1; //int(channel->spec->refresh_mode)*2; 
        //assert(int(T::RefreshMode::MAX) == 3); // TODO_hasan: fix the line above if adding more refresh modes
        ref_counters = new int[channel->spec->org_entry.count[int(T::Level::Rank)]];
        for(int i = 0; i < channel->spec->org_entry.count[int(T::Level::Rank)]; i++)
            ref_counters[i] = 0;
        
        //int num_all_nodes = 1;
        //for(int i = int(T::Level::Rank); i <= int(T::Level::Bank); i++) {
        //    num_all_nodes *= channel->spec->org_entry.count[i];
        //}
        
        // Hasan: Assuming all rows (including SpareRows) are being refreshed
        //double crow_table_inv_interval_ms = (double) (64/refresh_mult)/
        //            (num_all_nodes*num_SAs*copy_rows_per_SA);
        //crow_table_inv_interval = (ulong) ((crow_table_inv_interval_ms * 1000000)/
        //                            (channel->spec->speed_entry.tCK));

        //if(is_DDR4)
        //    crow_table_inv_index.resize(int(T::Level::Row) + 1 + 1 /*+1 for copy_row id*/, 0);
        //else
        //    crow_table_inv_index.resize(int(T::Level::Row) + 1 /*+1 for copy_row id*/, 0); // For SALP, we don't have bank-groups

        // crow_table_inv_index[int(T::Level::Channel)] = channel->id;
    }

    void initialize_crow_timing(vector<typename T::TimingEntry> timing[][int(T::Command::MAX)], const float trcd_factor, 
                    const float tras_factor, const float twr_factor, const float tfaw_factor) {

        if(timing == partial_crow_hit_partial_restore_timing)
            printf("Initializing Partial CROW Hit to Partial Restoration Timing... \n");
        else if(timing == partial_crow_hit_full_restore_timing)
            printf("Initializing Partial CROW Hit to Full Restoration Timing... \n");
        else if(timing == full_crow_hit_partial_restore_timing)
            printf("Initializing Full CROW Hit to Partial Restoration Timing... \n");
        else if(timing == full_crow_hit_full_restore_timing)
            printf("Initializing Full CROW Hit to Full Restoration Timing... \n");
        else if(timing == crow_copy_timing)
            printf("Initializing CROW Copy Timing... \n");
        else
            assert(false && "Initializing unknown CROW timing.");


        // copy the default timing parameters
        for(uint l = 0; l < int(T::Level::MAX); l++) {
            for(uint c = 0; c < int(T::Command::MAX); c++) {
                timing[l][c] = channel->spec->timing[l][c];
            }
        }

        vector<typename T::TimingEntry>* t;
        int trcd = 0, tras = 0;

        // apply trcd_factor to the related timing params
        t = timing[int(T::Level::Bank)];

        for (auto& t : t[int(T::Command::ACT)]) {
            if((t.cmd == T::Command::RD) || (t.cmd == T::Command::RDA)){
                printf("Default ACT-to-RD cycles: %d\n", t.val);
                t.val = (int)ceil(t.val * trcd_factor);
                trcd = t.val;
                printf("New ACT-to-RD cycles: %d\n", t.val);
            }

            if((t.cmd == T::Command::WR) || (t.cmd == T::Command::WRA)) {
                printf("Default ACT-to-WR cycles: %d\n", t.val);
                t.val = (int)ceil(t.val * trcd_factor);
                printf("New ACT-to-WR cycles: %d\n", t.val);
            }
        }

        
        
        // apply tras_factor to the related timing parameters
        t = timing[int(T::Level::Rank)];

        for (auto& t : t[int(T::Command::ACT)]) {
            if(t.cmd == T::Command::PREA){
               printf("Default ACT-to-PREA cycles: %d\n", t.val);
               t.val = (int)ceil(t.val * tras_factor);
               tras = t.val;
               printf("New ACT-to-PREA cycles: %d\n", t.val);
            }
        }

        t = timing[int(T::Level::Bank)];

        for (auto& t : t[int(T::Command::ACT)]) {
            if(t.cmd == T::Command::PRE) {
                printf("Default ACT-to-PRE cycles: %d\n", t.val);
                t.val = (int)ceil(t.val * tras_factor);
                printf("New ACT-to-PRE cycles: %d\n", t.val);
            }
        }

        // apply both trcd_factor and tras_factor to tRC
        assert(trcd != 0 && tras !=0 && "tRCD or tRAS was not set.");
        t = timing[int(T::Level::Bank)];

        for (auto& t : t[int(T::Command::ACT)]) {
            if(t.cmd == T::Command::ACT) {
                printf("Default ACT-to-ACT cycles: %d\n", t.val);
                t.val = trcd + tras; 
                printf("New ACT-to-ACT cycles: %d\n", t.val);
            }
        }

        // apply twr_factor to the related timing parameters
        t = timing[int(T::Level::Rank)];

        for (auto& t : t[int(T::Command::WR)]) {
            if(t.cmd == T::Command::PREA) {
                printf("Default WR-to-PREA cycles: %d\n", t.val);
                t.val = channel->spec->speed_entry.nCWL + channel->spec->speed_entry.nBL +
                            (int)ceil(channel->spec->speed_entry.nWR*twr_factor);
                printf("New WR-to-PREA cycles: %d\n", t.val);
            }
        }


        t = timing[int(T::Level::Bank)];

        for (auto& t : t[int(T::Command::WR)]) {
            if(t.cmd == T::Command::PRE) {
                printf("Default WR-to-PRE cycles: %d\n", t.val);
                t.val = channel->spec->speed_entry.nCWL + channel->spec->speed_entry.nBL +
                            (int)ceil(channel->spec->speed_entry.nWR*twr_factor);
                printf("New WR-to-PRE cycles: %d\n", t.val);
            }
        }

        for (auto& t : t[int(T::Command::WRA)]) {
            if(t.cmd == T::Command::ACT) {
                printf("Default WRA-to-ACT cycles: %d\n", t.val);
                t.val = channel->spec->speed_entry.nCWL + channel->spec->speed_entry.nBL +
                            (int)ceil(channel->spec->speed_entry.nWR*twr_factor) +
                            channel->spec->speed_entry.nRP;
                printf("New WRA-to-ACT cycles: %d\n", t.val);
            }

        }

        // apply tfaw_factor to the related timing parameters
        t = timing[int(T::Level::Rank)];

        for (auto& t : t[int(T::Command::ACT)]) {
            if(t.cmd == T::Command::ACT && (t.dist == 4)) {
                printf("Default ACT-to-ACT (tFAW) cycles: %d\n", t.val);
                t.val = (int)ceil(t.val*tfaw_factor);
                printf("New ACT-to-ACT (tFAW) cycles: %d\n", t.val);
            }

        }
    }

    //void update_crow_table_inv_index() {

    //    int SA_size = channel->spec->org_entry.count[int(T::Level::Row)]/num_SAs;
    //    // We should never change the channel id (as we have a separate
    //    // controller for each channel), thats why below we
    //    // start from 1
    //    for(uint i = 1; i < crow_table_inv_index.size(); i++) {
    //        if(i < int(T::Level::Row)) {
    //            crow_table_inv_index[i] = (crow_table_inv_index[i] + 1) % channel->spec->org_entry.count[i];
    //        } else if (i == int(T::Level::Row)){
    //            // row id
    //            crow_table_inv_index[i] = (crow_table_inv_index[i] + SA_size) % channel->spec->org_entry.count[i];
    //        } else {
    //            // copy_row id
    //            crow_table_inv_index[i] = (crow_table_inv_index[i] + 1) % copy_rows_per_SA;
    //        }

    //        if(crow_table_inv_index[i]) // if not zero, we are done
    //            return;
    //    }

    //}

    void crow_set_FR_on_PRE(typename T::Command cmd, const vector<int>& addr_vec) {
        
        if(cmd != T::Command::PRE) {
            vector<int> cur_addr_vec = addr_vec;

            int bank_levels = int(T::Level::Bank) - int(T::Level::Rank);

            switch(bank_levels) {
                case 1:
                    for(int i = 0; i < channel->spec->org_entry.count[int(T::Level::Bank)]; i++) {
                        cur_addr_vec[int(T::Level::Bank)] = i;
                        crow_set_FR_on_PRE_single_bank(cur_addr_vec);
                    }
                    break;
                case 2:
                    for(int i = 0; i < channel->spec->org_entry.count[int(T::Level::Bank) - 1]; 
                            i++) {
                        cur_addr_vec[int(T::Level::Bank) - 1] = i;
                        for(int j = 0; j < channel->spec->org_entry.count[int(T::Level::Bank)]; 
                                j++) {
                            cur_addr_vec[int(T::Level::Bank)] = j;
                            crow_set_FR_on_PRE_single_bank(cur_addr_vec);
                        }
                    }

                    break;
                default:
                    assert(false && "Not implemented!");
            }
        } else {
            crow_set_FR_on_PRE_single_bank(addr_vec);
        }

    }

    void crow_set_FR_on_PRE_single_bank(const vector<int>& addr_vec) {
        
        // get the id of the row to be precharged
        int pre_row = rowtable->get_open_row(addr_vec);

        if(pre_row == -1)
            return;

        auto crow_addr_vec = addr_vec;
        crow_addr_vec[int(T::Level::Row)] = pre_row;

        if(!crow_table->is_hit(crow_addr_vec)) // An active row may not be in crow_table right after the warmup period finished
            return;


        //printf("clk: %lu, Precharge target: %d %d %d %d \n", clk, addr_vec[0], addr_vec[1], addr_vec[2], addr_vec[3]);
        //printf("pre_row: %d \n", pre_row);
        // get the next SA to be refreshed
        const float ref_period_threshold = 0.4f; // TODO_hasan: make this a config option
        
        int nREFI = channel->spec->speed_entry.nREFI;
        float tCK = channel->spec->speed_entry.tCK;
        int SA_size;
        int next_SA_to_ref;
        
        
        if(is_DDR4 || is_LPDDR4) {
           SA_size = channel->spec->org_entry.count[int(T::Level::Row)]/num_SAs;
        }
        else { // for SALP
            SA_size = channel->spec->org_entry.count[int(T::Level::Row)];
        }
        
        next_SA_to_ref = ref_counters[addr_vec[int(T::Level::Rank)]]/SA_size;
        
        float trefi_in_ns = nREFI * tCK;

        int SA_id;
        if(is_DDR4 || is_LPDDR4)
            SA_id = pre_row/SA_size;
        else // for SALP
            SA_id = addr_vec[int(T::Level::Row) - 1];
        //printf("SA_size: %d SA_id: %d \n", SA_size, SA_id);

               
        int base_refw = is_LPDDR4 ? 32*refresh_mult : 64*refresh_mult; 
        ulong ticks_in_ref_int = (base_refw)*1000000/tCK;
        //printf("tCK: %f, ticks_in_ref_int: %lu \n", tCK, ticks_in_ref_int);
        int num_refs = ticks_in_ref_int/nREFI;
        
        int rows_per_bank;

        if(is_DDR4 || is_LPDDR4)
            rows_per_bank = channel->spec->org_entry.count[int(T::Level::Row)];
        else // for SALP
            rows_per_bank = channel->spec->org_entry.count[int(T::Level::Row)] * channel->spec->org_entry.count[int(T::Level::Row) - 1];

        int num_rows_refreshed_at_once = ceil(rows_per_bank/(float)num_refs);

        int SA_diff = (next_SA_to_ref > SA_id) ? (num_SAs - (next_SA_to_ref - SA_id)) : SA_id - next_SA_to_ref;
        //printf("Controller: next_SA_to_ref: %d, SA_id: %d, num_SAs: %d \n", next_SA_to_ref, SA_id, num_SAs);
        //printf("trefi_in_ns: %f, SA_size: %d, num_rows_refreshed_at_once: %d \n", trefi_in_ns, SA_size, num_rows_refreshed_at_once);

        long time_diff = (long)(SA_diff * (trefi_in_ns*(SA_size/(float)num_rows_refreshed_at_once)));
        bool refresh_check = time_diff > ((base_refw*1000000*refresh_mult) * ref_period_threshold);

        //printf("Controller: SA_diff: %d, time_diff: %ld, comp: %ld \n", SA_diff, time_diff, (long)((64*1000000/ref_mult) * ref_period_threshold));


        // get the restoration time applied
        long applied_restoration = channel->cycles_since_last_act(addr_vec, clk);
        bool restore_check = (applied_restoration < channel->spec->speed_entry.nRAS);

        if(refresh_check && restore_check)
            crow_num_fr_set++;
        else
            crow_num_fr_notset++;

        if(refresh_check)
            crow_num_fr_ref++;

        if(restore_check)
            crow_num_fr_restore++;

        
        // DEBUG
        //if(refresh_check && restore_check)
            //printf("CROW (%lu): Setting FR bit for: %d %d %d \n", clk, crow_addr_vec[2],
            //                    crow_addr_vec[3], crow_addr_vec[4]);
        // END - DEBUG

        crow_table->set_FR(crow_addr_vec, refresh_check && restore_check);

    }

    bool upgrade_prefetch_req (Queue& q, const Request& req) {
        if(q.size() == 0)
            return false;

        auto pref_req = find_if(q.q.begin(), q.q.end(), [req](Request& preq) {
                                                return req.addr == preq.addr;});

        if (pref_req != q.q.end()) {
            pref_req->type = Request::Type::READ;
            pref_req->callback = pref_req->proc_callback; // FIXME: proc_callback is an ugly workaround
            return true;
        }
            
        return false;
    }

    // FIXME: ugly
    bool upgrade_prefetch_req (deque<Request>& p, const Request& req) {
        if (p.size() == 0)
            return false;

        auto pref_req = find_if(p.begin(), p.end(), [req](Request& preq) {
                                                return req.addr == preq.addr;});

        if (pref_req != p.end()) {
            pref_req->type = Request::Type::READ;
            pref_req->callback = pref_req->proc_callback; // FIXME: proc_callback is an ugly workaround
            return true;
        }
            
        return false;
    }

    const std::unique_ptr<MaintenancePolicy<T>>& get_smd_refresher(const uint32_t rank_id, const uint32_t chip_id) const {
        return smd_refreshers[rank_id*chips_per_rank + chip_id];
    }

    const std::unique_ptr<MaintenancePolicy<T>>& get_smd_scrubber(const uint32_t rank_id, const uint32_t chip_id) const {
        return smd_scrubbers[rank_id*chips_per_rank + chip_id];
    }

    const std::unique_ptr<SMDRowHammerProtection<T>>& get_smd_rh_protector(const uint32_t rank_id, const uint32_t chip_id) const {
        return smd_rh_protectors[rank_id*chips_per_rank + chip_id];
    }

    void smd_query_ref_status() {
        typename T::Command cmd = T::Command::RSQ;
        for (uint32_t rank_id = 0; rank_id < (uint32_t)channel->spec->org_entry.count[int(T::Level::Rank)]; rank_id++) {
            switch (smd_query_ref_status(rank_id, cmd)) {
                case 0: {// success
                    int rsq_addr[2] = {channel->id, (int)rank_id};
                    channel->update(T::Command::RSQ, rsq_addr, clk);
                    return;
                }
                case -1: // failed timing
                    return;
                // -2 indicates there is no bank that needs an update in the rank. So trying the other ranks
            }
        }
    }

    int smd_query_ref_status(const uint32_t rank_id, typename T::Command& cmd) {

        // check timing to see if the data bus allows querying the refresh status
        int rsq_addr[2] = {channel->id, (int)rank_id};
        if (!channel->check(T::Command::RSQ, rsq_addr, clk)) {
            smd_ref_status_timing_failures[rank_id]++;
            return -1; // timing failed
        }

        // // we need to check channel level timing only
        // if(!channel->check_only_this_level(T::Command::RD, clk)) {
        //     smd_ref_status_timing_failures[rank_id]++;
        //     return -1; // timing failed
        // }

        int ref_bank_gid = rank_id;
        int scrub_bank_gid = rank_id;
        if (smd_mode == SMD_MODE::RSQ) {
            ref_bank_gid = smd_ref_tracker.find_bank_to_query(rank_id) ;
            if (smd_ecc_scrubbing_enabled)
             scrub_bank_gid = smd_scrub_tracker.find_bank_to_query(rank_id) ;
            if (ref_bank_gid < 0 && (!smd_ecc_scrubbing_enabled || (scrub_bank_gid < 0))) {
                smd_no_bank_for_ref_status_update[rank_id]++;
                return -2; // no bank in this rank requires an update
            }
        }

        
        // PRE commands that query the refresh state read data from DRAM similar to READ commands
        // reading the refresh status should have lower latency compared to READs since it does not access the row buffers
        // UPDATE: assuming that this request takes the same latency as reads but this is to be updated later
        if (ref_bank_gid >= 0)
        {
            std::vector<int> req_addr = {channel->id, (int)rank_id, ref_bank_gid};
            Request req_ref_status(req_addr, Request::Type::REF_STATUS_QUERY, nullptr);
            req_ref_status.depart = clk + channel->spec->read_latency;
            pending.push_back(req_ref_status);


            smd_ref_tracker.mark_inflight_req(ref_bank_gid);

            smd_total_ref_status_queries[rank_id]++;

            // convert PRE to PRE_RSQ
            if (cmd == T::Command::PRE)  
                cmd = T::Command::PRE_RSQ;
        }

        if (smd_ecc_scrubbing_enabled && (scrub_bank_gid >= 0))
        {
            std::vector<int> req_addr = {channel->id, (int)rank_id, scrub_bank_gid};
            Request req_ref_status(req_addr, Request::Type::REF_STATUS_QUERY, nullptr);
            req_ref_status.depart = clk + channel->spec->read_latency;
            pending.push_back(req_ref_status);

            if(smd_ecc_scrubbing_enabled)
                smd_scrub_tracker.mark_inflight_req(scrub_bank_gid);

            smd_total_ref_status_queries[rank_id]++;

            // convert PRE to PRE_RSQ
            if (cmd == T::Command::PRE)  
                cmd = T::Command::PRE_RSQ;
        }

        return 0;
    }

    

    const Request* smd_has_ready_but_timed_out_req(const list<Request>& queue, bool& has_timed_out_req, bool& has_SA_conflict) {
        has_timed_out_req = false;
        has_SA_conflict = false;

        const Request* last_timed_out_req = nullptr;

        for (auto& req : queue) {

            switch(smd_is_ready(req)) {
                case -1:
                    has_timed_out_req = true;
                    last_timed_out_req = &req;
                    break;
                case -2:
                    has_timed_out_req = true;
                    has_SA_conflict = true;
                    return &req;
            }
        }

        return last_timed_out_req;
    }

    // Returns RegionBusyResponse
    RegionBusyResponse is_smd_region_busy(const std::vector<int>& addr_vec) const {
        uint32_t gbid = channel->spec->calc_global_bank_id(addr_vec);
        // uint32_t sa_id = addr_vec[int(T::Level::Row)]/channel->spec->get_subarray_size();
        uint32_t sa_id = addr_vec[int(T::Level::Subarray)];

        uint num_busy_chips = 0;

        for(uint32_t chip_id = 0; chip_id < chips_per_rank; chip_id++) {
            if(get_smd_refresher(addr_vec[int(T::Level::Rank)], chip_id)->is_SA_under_maintenance(gbid, sa_id))
            // || (smd_ecc_scrubbing_enabled && get_smd_scrubber(addr_vec[int(T::Level::Rank)], chip_id)->is_SA_under_maintenance(gbid, sa_id))) 
            // Hasan: no need to check the scrubber policy separately as all policies share the same var for managing locked subarrays. 
            // So, is_SA_under_maintenance() returns true if any active maintenance has locked the region
                num_busy_chips++;

            if (num_busy_chips > 0 && num_busy_chips <= chip_id)
                return RegionBusyResponse::SOME_CHIPS_BUSY;
        }

        if (num_busy_chips == chips_per_rank)
        return RegionBusyResponse::ALL_CHIPS_BUSY;

        return RegionBusyResponse::NO_CHIPS_BUSY;
    }

    

    vector<typename T::TimingEntry> partial_crow_hit_partial_restore_timing[int(T::Level::MAX)][int(T::Command::MAX)];
    vector<typename T::TimingEntry> partial_crow_hit_full_restore_timing[int(T::Level::MAX)][int(T::Command::MAX)];
    vector<typename T::TimingEntry> full_crow_hit_partial_restore_timing[int(T::Level::MAX)][int(T::Command::MAX)];
    vector<typename T::TimingEntry> full_crow_hit_full_restore_timing[int(T::Level::MAX)][int(T::Command::MAX)];
    vector<typename T::TimingEntry> crow_copy_timing[int(T::Level::MAX)][int(T::Command::MAX)];

	float trcd_crow_partial_hit = 1.0f, trcd_crow_full_hit = 1.0f;
	
	float tras_crow_partial_hit_partial_restore = 1.0f, tras_crow_partial_hit_full_restore = 1.0f;
	float tras_crow_full_hit_partial_restore = 1.0f, tras_crow_full_hit_full_restore = 1.0f;
	float tras_crow_copy_partial_restore = 1.0f, tras_crow_copy_full_restore = 1.0f; 
	
	float twr_partial_restore = 1.0f, twr_full_restore = 1.0f;

    map<int, int> row_act_hist[8][128]; // hardcoded for 8 bank and 128 SAs per bank (assuming 512-row SAs and 64K rows per bank)

    bool dpower_is_reset = false;
    
};

// template <>
// vector<int> Controller<SALP>::get_addr_vec(
//     SALP::Command cmd, list<Request>::iterator req);

// template <>
// bool Controller<SALP>::is_ready(list<Request>::iterator req);

// template <>
// void Controller<SALP>::initialize_crow_timing(vector<SALP::TimingEntry> timing[]
//         [int(SALP::Command::MAX)], const float trcd_factor, const float tras_factor, 
//         const float twr_factor, const float tfaw_factor);

//template <>
//void Controller<SALP>::update_crow_table_inv_index();

// template <>
// void Controller<ALDRAM>::update_temp(ALDRAM::Temp current_temperature);

//template <>
//void Controller<TLDRAM>::tick();

} /*namespace ramulator*/

#endif /*__CONTROLLER_H*/
