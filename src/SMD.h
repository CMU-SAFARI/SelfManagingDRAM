#ifndef __SMD_H
#define __SMD_H

#include <vector>
#include <cstdint>
#include <numeric>
#include <algorithm>
#include <random>
#include "Config.h"
#include "Controller.h"
#include "BloomFilter.h"

#include "Statistics.h"

// #define SMD_DEBUG

namespace ramulator
{


static uint32_t ref_tracker_timeout_period = 0; // After the SMDTracker queries a bank's refresh status, 
                                                // the corresponding MaintenanceMachine cannot lock another SA in the same bank until this period has passed since the query
                                                // At the same time, this defines the time interval for which a SMDTracker entry remains active and after
                                                // which the entry becomes outdated 

static uint32_t scrub_tracker_timeout_period = 0;

typedef enum {
    RSQ,
    ALERT,
    ACT_NACK
} SMD_MODE;

static std::map<std::string, SMD_MODE> str_to_smd_mode = {{"RSQ", SMD_MODE::RSQ}, {"ALERT", SMD_MODE::ALERT}, {"ACT_NACK", SMD_MODE::ACT_NACK}};

typedef struct SubarrayAddr {
    uint32_t bank_gid;
    uint32_t sa_id;
    SubarrayAddr(const uint32_t bank_gid, const uint32_t sa_id) : bank_gid(bank_gid), sa_id(sa_id) {}
    SubarrayAddr() : SubarrayAddr(0, 0) {}
    bool operator==(const SubarrayAddr& other) const {
        return (bank_gid == other.bank_gid) && (sa_id == other.sa_id);
    }
} SubarrayAddr;

typedef struct RowAddr {
    uint32_t bank_gid;
    uint32_t sa_id;
    uint32_t row_id;
    RowAddr(const uint32_t bank_gid, const uint32_t sa_id, const uint32_t row_id) : bank_gid(bank_gid), sa_id(sa_id), row_id(row_id) {}
    RowAddr() : RowAddr(0, 0, 0) {}
    bool operator==(const RowAddr& other) const {
        return (bank_gid == other.bank_gid) && (sa_id == other.sa_id) && (row_id == other.row_id);
    }
} RowAddr;

class GrapheneCounterTable {
public:
    GrapheneCounterTable(const uint32_t num_counters, const uint32_t act_threshold) : 
        num_counters(num_counters), act_threshold(act_threshold) {}

    bool increment (const uint32_t row_id) {

        assert(counters.size() <= num_counters);

        if (counters.find(row_id) != counters.end()) {
            // row_id is already in the counter table
            counters[row_id]++;

            return (counters[row_id] % act_threshold) == 0; // return true if counter value is multiples of act_threshold
        }

        // row_id is not in the counter table
        if (counters.size() < num_counters) {
            // there are unused counters. Insert the row
            counters[row_id] = 1;
            return false;
        }

        std::map<uint32_t, uint32_t>::iterator min_counter = get_min_counter();

        if(min_counter->second != spillover_counter) {
            // incrementing the spillover counter since it is smaller than the smallest entry in the counter table
            spillover_counter++;
            return false;
        }

        assert(min_counter->second == spillover_counter);
        // replace min_counter with the new row
        counters.erase(min_counter);
        counters[row_id] = spillover_counter + 1;

        return (counters[row_id] % act_threshold) == 0; // return true if counter value is multiples of act_threshold
    }

    void reset () {
        spillover_counter = 0;
        counters.clear();
    }

private:
    uint32_t spillover_counter = 0;
    std::map<uint32_t, uint32_t> counters;
    const uint32_t num_counters;
    const uint32_t act_threshold;

    std::map<uint32_t, uint32_t>::iterator get_min_counter() {
        return std::min_element(counters.begin(), counters.end());
    }
};

template <typename T>
class SMDTracker {

    typedef struct MSTEntry {
        std::vector<uint32_t> busy_SAs;
        MSTEntry() {}
    } MSTEntry; // MST -> Maintenance Status Table

    typedef struct MST {
        std::vector<MSTEntry> entries;
        MST(const uint32_t num_banks) {
            entries = std::vector<MSTEntry>(num_banks, MSTEntry());
        }
    } MST; // MST -> Maintenance Status Table


    public:
    SMDTracker(const Config& configs, const Controller<T>& ctrl) : ctrl(ctrl) {

        smd_mode = str_to_smd_mode[configs.get_str("smd_mode")];

        const T& spec = *(ctrl.channel->spec);

        auto total_num_banks = spec.get_num_all_banks();

        chips_per_rank = spec.channel_width/spec.org_entry.dq;

        // initialize an MST for each chip. A single MST stores refresh information of all banks in the chip
        maint_status = std::vector<MST>(chips_per_rank, MST(total_num_banks));
        inflight_reqs = std::vector<bool>(total_num_banks, false);
        last_update_clk = std::vector<long>(total_num_banks, -1);
    }

    void print() {

        auto total_num_banks = ctrl.channel->spec->get_num_all_banks();

        for (uint bank_id = 0; bank_id < total_num_banks; bank_id++) {
            std::cout << "Bank " << bank_id << ": ";
            for(auto& ms : maint_status) {

                if(is_bank_timed_out(bank_id))
                    std::cout << "*";

                if (ms.entries[bank_id].busy_SAs.size() == 1) {
                    std::cout << ms.entries[bank_id].busy_SAs[0] << " ";
                    continue;
                }
                
                if (ms.entries[bank_id].busy_SAs.size() == 0)
                    std::cout << "- ";
                else
                    std::cout << "+ ";
            }

            std::cout << std::endl;
        }
    }
    
    int can_open(const std::vector<int>& addr_vec) const {

        uint32_t global_bank_id = ctrl.channel->spec->calc_global_bank_id(addr_vec);
        // uint32_t sa_id = addr_vec[uint32_t(T::Level::Row)]/ctrl.channel->spec->get_subarray_size();
        uint32_t sa_id = addr_vec[uint32_t(T::Level::Subarray)];

        return can_open(global_bank_id, sa_id);
    }

    int can_open(const uint32_t global_bank_id, const uint32_t sa_id) const {

        // std::cout << "[SMD] The target subarray is available for access!" << std::endl;

        return can_access_SA(ctrl.clk, SubarrayAddr(global_bank_id, sa_id));
    }

    void update(const uint32_t chip_id, const uint32_t global_bank_id, const std::vector<uint32_t>& busy_SAs) {

        if(smd_mode == SMD_MODE::RSQ){
            MSTEntry& entry = maint_status[chip_id].entries[global_bank_id];
            entry.busy_SAs = busy_SAs;

            last_update_clk[global_bank_id] = ctrl.clk;

            // std::cout << "[SMDTracker] clk: " << ctrl.clk << " Updating SMD Ref Status. global_bank: " << global_bank_id;
            // if (busy_SAs.size() > 0) {
            //     std::cout << ". Busy SAs: ";
            //     for (auto& busy_sa : busy_SAs)
            //         std::cout << busy_sa << " ";

            //     std::cout << std::endl;
            // } else {
            //     std::cout << ". No busy SAs" << std::endl;
            // }
        } else if (smd_mode == SMD_MODE::ALERT) {
            assert(false && "ERROR: Incomplete implementation!");
            // ref_status_alert[chip_id].busy_SAs = busy_SAs;

        } else 
            assert(false && "ERROR: Unimplemented SMD_MODE!");
    }

    void update_alert_clk(const uint32_t rank_id) {
        assert(false && "ERROR: Incomplete implementation!");
        //last_alert_clk[rank_id] = ctrl.clk;
    }


    int find_bank_to_query(const uint32_t rank_id) {

        // prioritize banks that are targeted by requests in the request buffer
        auto req_banks = ctrl.get_reqbuffer_banks(rank_id);

        for (auto& bank_gid : req_banks) {
            if (is_bank_timed_out(bank_gid) && !exists_inflight_req(bank_gid))
                return bank_gid;
        }

        // return the first bank that is timed out and thus requires an update
        // for (uint32_t bank_gid = 0; bank_gid < last_update_clk.size(); bank_gid++) {
        //     if (is_bank_timed_out(bank_gid) && !exists_inflight_req(bank_gid)) {
        //         return bank_gid;
        //     }
        // }

        return -1; // return -1 if no bank needs a ref status update
    }

    void mark_inflight_req(const uint32_t bank_gid) {
        inflight_reqs[bank_gid] = true;
    }

    void unmark_inflight_req(const uint32_t bank_gid) {
        inflight_reqs[bank_gid] = false;
    }

    
    private:

        bool exists_inflight_req(const uint32_t bank_gid) const {
            return inflight_reqs[bank_gid];
        }

        bool is_bank_timed_out(const uint32_t bank_gid) const {
            return last_update_clk[bank_gid] < 0 || (std::abs(ctrl.clk - last_update_clk[bank_gid]) > ref_tracker_timeout_period);
        }

        int can_access_SA(const long clk, const SubarrayAddr& sa_addr) const {
            
            // the entries should not be timed out
            if(is_bank_timed_out(sa_addr.bank_gid))
                return -1;


            // no chip should be refreshing the target SA
            for (const auto& ms : maint_status) {
                const auto& entry = ms.entries[sa_addr.bank_gid];
                if(std::find(entry.busy_SAs.cbegin(), entry.busy_SAs.cend(), sa_addr.sa_id) != entry.busy_SAs.cend())
                    return -2; // found a chip that refreshes the target subarray
            }

            return 1;
        }

        SMD_MODE smd_mode;
        const Controller<T>& ctrl;
        std::vector<MST> maint_status; // per chip refresh status tables
        std::vector<bool> inflight_reqs;
        std::vector<long> last_update_clk;

        uint32_t chips_per_rank;
};

template <typename T>
class MaintenanceMachine;

template <typename T>
class VariableRefreshMachine;

template <typename T>
class NeighborRowRefreshMachine;

template <typename T>
class ECCScrubbingMachine;

typedef struct SALock {
    bool locked = false;
    long cooldown_exp = -1;
    uint32_t sa_id;
    bool bank_locked = false;
} SALock;

// this is a base class. Every CR-DRAM chip implements a MaintenancePolicy
template <typename T>
class MaintenancePolicy {

    friend class MaintenanceMachine<T>;
    friend class VariableRefreshMachine<T>;
    friend class NeighborRowRefreshMachine<T>;
    friend class ECCScrubbingMachine<T>;
    
    public:
        MaintenancePolicy(const Config& configs, Controller<T>* ctrl, const uint32_t rank_id, const uint32_t chip_id, 
            const uint32_t num_banks_in_chip, const uint32_t SAs_per_bank, const uint32_t num_rows) : ctrl(ctrl) {

            pending_maint_limit = configs.get_uint("smd_pending_ref_limit");
            smd_mode = str_to_smd_mode[configs.get_str("smd_mode")];
            _num_banks_in_chip = num_banks_in_chip;
            _num_SAs_per_bank = SAs_per_bank;
            _num_rows = num_rows;
            _rank_id = rank_id;
            _chip_id = chip_id;
            channel = ctrl->channel;

            // a chip has its own locked_SAs structure shared by all maintenance mechanisms
            ind_locked_SAs = ctrl->channel->id*ctrl->channel->spec->org_entry.count[uint32_t(T::Level::Rank)] + rank_id;

            while (ind_locked_SAs >= locked_SAs.size())
                locked_SAs.push_back(std::vector<std::vector<SALock>>());

            if (chip_id == locked_SAs[ind_locked_SAs].size()){
                // adding locked_SAs for a new chip
                locked_SAs[ind_locked_SAs].push_back(std::vector<SALock>());
                get_locked_SAs().resize(_num_banks_in_chip);
            }

            
        }
        virtual ~MaintenancePolicy(){};

        virtual void tick() = 0;

        std::vector<uint32_t> communicate_locked_SAs (const uint32_t bank_id) {
            std::vector<uint32_t> SAs;

            auto& le = get_locked_SAs()[bank_id];
            if (le.locked)
                SAs.push_back(le.sa_id);

            assert(le.cooldown_exp <= clk && "[MaintenancePolicy] ERROR: Communicating the refresh status for the second time before the expiration of the cooldown period.");

            le.cooldown_exp = clk + ref_tracker_timeout_period;

            // std::cout << "[MaintenancePolicy] Communicating locked SAs of bank " << bank_id << std::endl;

            return SAs;
        }

        bool is_SA_under_maintenance(const uint32_t global_bank_id, const uint32_t sa_id) const {
            const auto& le = get_locked_SAs()[global_bank_id];

            if ((le.locked && le.sa_id == sa_id) || le.bank_locked)
                return true;

            return false;
        }

        bool contains_locked_SA(const uint32_t global_bank_id) const {
            const auto& le = get_locked_SAs()[global_bank_id];

            return le.locked;
        }

        long get_clk() const {
            return clk;
        }

        uint32_t get_chip_id() const {
            return _chip_id;
        }

        void set_ref_alert() {
            ref_alert_on = true;
        }

        bool get_and_clear_ref_alert() {
            bool cur_alert = ref_alert_on;
            ref_alert_on = false;

            return cur_alert;
        }

    protected:

        void lockSA(const uint32_t bank_id, const uint32_t sa_id) {

            #ifdef SMD_DEBUG
                uint bgid = bank_id/channel->spec->org_entry.count[int(T::Level::Bank)];
                uint bid = bank_id % channel->spec->org_entry.count[int(T::Level::Bank)];
                printf("%lu\t[%s]\t[Chip %d] Locking r: %d bg: %d b: %d, sa: %d \n", clk, _policy_name.c_str(), _chip_id, _rank_id, bgid, bid, sa_id);
            #endif // SMD_DEBUG

            auto& le = get_locked_SAs()[bank_id];

            assert(!le.locked && "[MaintenancePolicy] ERROR: Already locked a subarray from the target bank. The policy has to release a subarray first.");
            assert(le.cooldown_exp <= clk && "[MaintenancePolicy] ERROR: Cannot lock a subarray from a bank before its cooldown period expires.");

            le.sa_id = sa_id;
            le.locked = true;
            le.bank_locked = lock_entire_bank;
        }

        void releaseSA(const uint32_t bank_id, const uint32_t sa_id) {
            auto& le = get_locked_SAs()[bank_id];

            // std::cout << "[MaintenancePolicy] Releasing locked SAs in bank  " << bank_id << std::endl;

            if (le.sa_id == sa_id) {
                assert(le.locked && "[MaintenancePolicy] ERROR: Trying to release an already released subarray.");

                #ifdef SMD_DEBUG
                    uint bgid = bank_id/channel->spec->org_entry.count[int(T::Level::Bank)];
                    uint bid = bank_id % channel->spec->org_entry.count[int(T::Level::Bank)];
                    printf("%lu\t[%s]\t[Chip %d] Releasing r: %d bg: %d b: %d, sa: %d \n", clk, _policy_name.c_str(), _chip_id, _rank_id, bgid, bid, sa_id);
                #endif // SMD_DEBUG

                le.locked = false;
                le.bank_locked = false;

                // std::cout << "[MaintenancePolicy] clk: " << clk << ", Releasing bank: " << sa_addr.bank_id << ", SA: " << sa_addr.sa_id << std::endl;
                return;
            } else {
                std::cout << "[MaintenancePolicy] ERROR: Trying to release SA " << sa_id << " but SA " << le.sa_id << " is the currently locked SA." << std::endl;
                assert(false);
            }

            assert(false && "[MaintenancePolicy] ERROR: Trying to release a subarray that is not locked.");
        }

        bool is_SA_active(const uint32_t bank_id, const uint32_t sa_id) {
            int open_sa_id = channel->get_open_SA(_rank_id, bank_id);

            if (open_sa_id == (int)sa_id)
                return true;

            return false;
        }

        bool is_on_cooldown(const uint32_t bank_id) {
            return get_locked_SAs()[bank_id].cooldown_exp > clk;
        }

        std::vector<SALock>& get_locked_SAs() const {
            return locked_SAs[ind_locked_SAs][_chip_id];
        }

        long clk = 0;

        uint32_t _num_banks_in_chip;
        uint32_t _num_SAs_per_bank;
        uint32_t _num_rows;

        uint32_t maint_latency = 50;
        uint32_t pending_maint_limit = 9;
        uint32_t row_maint_granularity = 1;

        bool lock_entire_bank = false;

        uint32_t num_maint_machines;

        uint32_t _rank_id;
        uint32_t _chip_id;
        uint32_t ind_locked_SAs;
        std::string _policy_name = "";

        SMD_MODE smd_mode;

        bool ref_alert_on = false;

        Controller<T>* ctrl;
        DRAM<T>* channel;

    private:
        static std::vector<std::vector<std::vector<SALock>>> locked_SAs;
};

template <typename T> 
std::vector<std::vector<std::vector<SALock>>> MaintenancePolicy<T>::locked_SAs;


typedef struct MaintenanceCounter {
    uint32_t bank_id; // each bank has one refresh counter
    uint32_t row_counter; // the row to refresh with the next REF operation
    uint32_t sa_counter; // the SA to refresh with the next REF operation
    uint64_t rollbacks; // the number of times this counter started over
    uint32_t pending_maint; // the number REFs that are awaiting turn to be performed on the bank the MaintenanceCounter is responsible for
    MaintenanceCounter(const uint32_t bank_id) : bank_id(bank_id) {
        row_counter = 0;
        sa_counter = 0;
        rollbacks = 0;
        pending_maint = 0;
    }

    // We'd probably like to start ecc scrubbing operations at a different subarray than refresh operations
    // so they are not overlapped in case the refresh and ecc intervals are the same?
    MaintenanceCounter(const uint32_t bank_id, const uint32_t sa_offset) : bank_id(bank_id), sa_counter(sa_offset) {
        row_counter = 0;
        rollbacks = 0;
        pending_maint = 0;
    }

    MaintenanceCounter() : MaintenanceCounter(0){}
    MaintenanceCounter& operator=(const MaintenanceCounter& other) {
        bank_id = other.bank_id;
        row_counter = other.row_counter;
        sa_counter = other.sa_counter;
        rollbacks = other.rollbacks;
        pending_maint = other.pending_maint;

        return *this;
    }

    void increment(const uint32_t num_SAs, const uint32_t num_rows, const uint32_t incr_val) {
        if(sa_counter == (num_SAs - 1)) {
            sa_counter = 0;

            auto old_row_counter = row_counter;
            row_counter = (row_counter + incr_val) % num_rows;

            if (old_row_counter > row_counter)
                rollbacks++;

            return;
        }
        sa_counter++;
    }

    void set(const RowAddr& ra) {
        bank_id = ra.bank_gid;
        sa_counter = ra.sa_id;
        row_counter = ra.row_id;
    }
} MaintenanceCounter;

// a MaintenanceMachine is responsible for performing refresh operation as needed by its RefreshCounters
template <typename T>
class MaintenanceMachine {

    public:
        MaintenanceMachine(const uint32_t id, const uint32_t num_counters, MaintenancePolicy<T>& maint_policy) : id(id), maint_policy(maint_policy) {
            _num_counters = num_counters;
            maint_counters.reserve(num_counters);
        }

        virtual ~MaintenanceMachine(){};

        void add_pending_maint(){
            for(auto& rc : maint_counters) {
                rc.pending_maint++;
                if (rc.pending_maint > maint_policy.pending_maint_limit) {
                    std::cout << "[MaintenanceMachine] ERROR: pending maintenance limit exceeded for bank " << rc.bank_id << ". Current pending maintenances: " << rc.pending_maint << std::endl;
                    std::cout << "[MaintenanceMachine] hitting this most likely because the current MaintenanceMachines cannot keep up with the refresh demand. Consider increasing the number of MaintenanceMachines" << std::endl;
                    assert(false);
                }

                #ifdef SMD_DEBUG
                    printf("%lu\t[Chip %d] Added new pending maintenance op. %d pending maintenance ops in r: %d gbid: %d \n", maint_policy.clk, maint_policy._chip_id, rc.pending_maint, maint_policy._rank_id, rc.bank_id);
                #endif // SMD_DEBUG
            }
        }

        virtual void tick() {

            if(maint_completion_clk > maint_policy.get_clk())
                return; // the MaintenanceMachine is busy refreshing. Nothing to do

            if(maint_completion_clk == maint_policy.get_clk()) // just finished refreshing. Release the SA
                maint_policy.releaseSA(last_locked_SA.bank_id, last_locked_SA.sa_counter);

            // check if the counter pointed to by ctr_index has a pending refresh. If so, try to issue a REF. If not (or cannot issue a REF), just increment the ctr_index.
            MaintenanceCounter& cur_mc = maint_counters[ctr_index];
            ctr_index = (ctr_index + 1) % _num_counters;

            if (cur_mc.pending_maint == 0) {
                // std::cout << "[MaintenanceMachine] clk: " <<  maint_policy.get_clk() << " No pending REFs - bank: " << cur_mc.bank_id << " SA: " << cur_mc.sa_counter << std::endl;
                return;
            }

            bool maint_initiated = false;
            switch(maint_policy.smd_mode) {
                case SMD_MODE::RSQ: {
                    maint_initiated = process_ref_rsq(cur_mc);
                    break;
                }
                case SMD_MODE::ALERT: {
                    maint_initiated = process_ref_alert(cur_mc);
                    break;
                }
                case SMD_MODE::ACT_NACK: {
                    maint_initiated = process_maint_act_nack(cur_mc);
                    break;
                }
                default:
                    assert(false && "ERROR: Undefined SMD_MODE!");
            }

            if (!maint_initiated)
                return;

            // increment the maintenance address counter
            cur_mc.increment(maint_policy._num_SAs_per_bank, maint_policy._num_rows, maint_policy.row_maint_granularity);

            // decrement the pending maintenance count
            assert(cur_mc.pending_maint > 0);
            cur_mc.pending_maint--;

            #ifdef SMD_DEBUG
                printf("%lu\t[Chip %d] Maintenance issued. %d pending maintenance ops in r: %d gbid: %d \n", maint_policy.clk, maint_policy._chip_id, cur_mc.pending_maint, maint_policy._rank_id, cur_mc.bank_id);
            #endif // SMD_DEBUG
        }

        std::vector<MaintenanceCounter> maint_counters;

    protected:

        bool process_ref_rsq (const MaintenanceCounter& mc) {
            if(maint_policy.is_on_cooldown(mc.bank_id)) {
                // std::cout << "[MaintenanceMachine] clk: " <<  maint_policy.get_clk() << " On cooldown - bank: " << mc.bank_id << " SA: " << mc.sa_counter << std::endl;
                return false;
            }

            if(maint_policy.is_SA_active(mc.bank_id, mc.sa_counter)){
                // std::cout << "[MaintenanceMachine] clk: " <<  maint_policy.get_clk() << " Open row in the target SA - bank: " << mc.bank_id << " SA: " << mc.sa_counter << std::endl;
                return false;
            }
               
            // the subarray is not being currently accessed and it is not on cooldown. The MaintenanceMachine can lock it
            maint_policy.lockSA(mc.bank_id, mc.sa_counter);
            last_locked_SA = mc;

            // set the ref completion cycle
            maint_completion_clk = maint_policy.get_clk() + maint_policy.maint_latency*maint_policy.row_maint_granularity;

            // if(maint_policy.get_chip_id() == 0) {
            //     std::cout << "[MaintenanceMachine] clk: " << maint_policy.get_clk() << " Refreshing bank: " << mc.bank_id << " SA: " << mc.sa_counter;
            //     std::cout << " until clk: " << maint_completion_clk << std::endl;
            // }

            return true;
        }

        bool process_ref_alert(const MaintenanceCounter& mc) {
            maint_policy.lockSA(mc.bank_id, mc.sa_counter);
            last_locked_SA = mc;

            if(!retry_ref(mc))
                SA_conflict = true;

            set_alert_status();
            
            return true;
        }

        virtual bool process_maint_act_nack(const MaintenanceCounter& mc) {
            
            if (retry_ref(mc)){
                maint_policy.lockSA(mc.bank_id, mc.sa_counter);
                last_locked_SA = mc;
                return true;
            }

            #ifdef SMD_DEBUG
                uint bgid = mc.bank_id/maint_policy.channel->spec->org_entry.count[int(T::Level::Bank)];
                uint bid = mc.bank_id % maint_policy.channel->spec->org_entry.count[int(T::Level::Bank)];
                printf("%lu\t[Chip %d] Could not lock subarray. Detected an active row in r: %d bg: %d b: %d, sa: %d \n", maint_policy.clk, maint_policy._chip_id, maint_policy._rank_id, bgid, bid, mc.sa_counter);
            #endif // SMD_DEBUG

            return false;
        }

        virtual bool retry_ref(const MaintenanceCounter& mc) {
            if(!maint_policy.is_SA_active(mc.bank_id, mc.sa_counter) && !maint_policy.contains_locked_SA(mc.bank_id)) {
                maint_completion_clk = maint_policy.get_clk() + maint_policy.maint_latency*maint_policy.row_maint_granularity;

                // DRAMPower estimates DRAM energy for a single DRAM chip. The refresh power is calculated incorrectly (for all chips) if 
                // issueDPowerSMDREF() is called by every single chip. Instead, we call issueDPowerSMDREF() only from chip0 and estimating the energy of that chip.
                if (maint_policy._chip_id == 0)
                    maint_policy.ctrl->issueDPowerSMDREF(maint_policy.row_maint_granularity, maint_policy._rank_id, mc.bank_id);
                return true;
            }

            maint_completion_clk = 0;
            return false;
        }

        void set_alert_status() {
            maint_policy.set_ref_alert();
        }

        uint32_t id;
        uint32_t _num_counters;
        uint32_t ctr_index = 0;
        bool SA_conflict = false; // used only in ref_alert mode

        MaintenancePolicy<T>& maint_policy;
        long maint_completion_clk = -1;
        MaintenanceCounter last_locked_SA;
};

template <typename T>
class SMDFixedRateRefresh : public MaintenancePolicy<T> {

    public:
        SMDFixedRateRefresh(const Config& configs, Controller<T>* ctrl, const uint32_t rank_id, 
            const uint32_t chip_id, const uint32_t num_banks_in_chip, const uint32_t SAs_per_bank, const uint32_t num_rows) : 
            MaintenancePolicy<T>(configs, ctrl, rank_id, chip_id, num_banks_in_chip, SAs_per_bank, num_rows) {

            this->_policy_name = "FixedRateRefresh";

            if (configs.get_str("smd_single_ref_latency") == "auto" || configs.get_str("smd_single_ref_latency") == "AUTO" ||
                configs.get_str("smd_single_ref_latency") == "Auto") {
                uint NUM_REF_CMDS_PER_REF_WINDOW = 8192;
                uint num_rows_per_bank = this->channel->spec->org_entry.count[int(T::Level::Subarray)]*this->channel->spec->org_entry.count[int(T::Level::Row)];
                this->maint_latency = ceil((this->channel->spec->speed_entry.nRFC*NUM_REF_CMDS_PER_REF_WINDOW)/(float)num_rows_per_bank);
                std::cout << "[INFO] Automatically setting SMD ref latency to " << this->maint_latency << " cycles.\n";

            } else
                this->maint_latency = configs.get_uint("smd_single_ref_latency");

            ref_interval = std::floor(configs.get_uint("smd_refresh_period")/(this->_num_rows*this->_num_SAs_per_bank));

            assert(ref_interval > this->maint_latency && "[SMDFixedRateRefresh] ERROR: the latency of a refresh operation should not be longer than the interval for refreshing a new row.");

            this->row_maint_granularity = configs.get_uint("smd_row_refresh_granularity");

            // ref_interval is currently calculated based on each ref operation refreshes one row
            // we multiply the ref_interval by row_maint_granularity since that many rows will be refreshed within a single refresh operation, i.e., when an SA is locked
            ref_interval *= this->row_maint_granularity;

            if(configs.get_bool("smd_worst_case_ref_distribution"))
                ref_interval_offset = chip_id*(this->maint_latency*this->row_maint_granularity);

            // std::cout << "[SMDFixedRateRefresh] Refreshing a new row once every " << ref_interval << " cycles."  << std::endl;
            // std::cout << "[SMDFixedRateRefresh] The current refresh latency: " << this->maint_latency << " cycles."  << std::endl;
            // std::cout << "[SMDFixedRateRefresh] The number of refresh machines: " << this->num_maint_machines << ""  << std::endl;

            this->num_maint_machines = configs.get_uint("smd_num_ref_machines");
            assert((this->_num_banks_in_chip % this->num_maint_machines) == 0 && "[SMDFixedRateRefresh] ERROR: num_maint_machines should divide the total number of banks with no remainder.");

            // ref_tracker_timeout_period can be arbitrary
            // setting it smaller than the ref_interval is good so that a refresh machine does not need to wait for another pending ref before refreshing again
            // the downside is, the lower the ref_tracker_timeout_period, the more ref status updates needed by the memory controller
            ref_tracker_timeout_period = std::floor(ref_interval*configs.get_float("smd_timeout_to_ref_interval_ratio"));

            for(uint rm_id = 0; rm_id < this->num_maint_machines; rm_id++)
                ref_machines.emplace_back(rm_id, this->_num_banks_in_chip/this->num_maint_machines, *this);

            // create a MaintenanceCounter for each bank and distribute them evenly across the ref_machines
            for (uint32_t bank_id = 0; bank_id < this->_num_banks_in_chip; bank_id++) {
                if(configs.get_bool("smd_worst_case_ref_distribution"))
                    ref_machines[bank_id % this->num_maint_machines].maint_counters.emplace_back(bank_id, chip_id % SAs_per_bank);
                else
                    ref_machines[bank_id % this->num_maint_machines].maint_counters.emplace_back(bank_id);
            }
        }

        void tick() {
            this->clk++;

            for (auto& rm : ref_machines)
                rm.tick();

            if(((this->clk + 1 + ref_interval_offset) % ref_interval) == 0) {
                // time to refresh new rows
                for (auto& rm : ref_machines)
                    rm.add_pending_maint();
            }
        }

    private:
        uint32_t ref_interval; // the DRAM chip refreshes a different row from each bank at this interval

        std::vector<MaintenanceMachine<T>> ref_machines;

        uint32_t ref_interval_offset = 0;
};

template <typename T>
class SMDNoRefresh : public MaintenancePolicy<T> {

    public:
        SMDNoRefresh(const Config& configs, Controller<T>* ctrl, const uint32_t rank_id, 
            const uint32_t chip_id, const uint32_t num_banks_in_chip, const uint32_t SAs_per_bank, const uint32_t num_rows) : 
            MaintenancePolicy<T>(configs, ctrl, rank_id, chip_id, num_banks_in_chip, SAs_per_bank, num_rows) {

            this->_policy_name = "NoRefresh";
        }

        void tick() {
            this->clk++;
        }
};

template <typename T>
class SMDECCScrubbing : public MaintenancePolicy<T> {

    friend class ECCScrubbingMachine<T>;

    public:
        SMDECCScrubbing(const Config& configs, Controller<T>* ctrl, const uint32_t rank_id, 
            const uint32_t chip_id, const uint32_t num_banks_in_chip, const uint32_t SAs_per_bank, const uint32_t num_rows) : 
            MaintenancePolicy<T>(configs, ctrl, rank_id, chip_id, num_banks_in_chip, SAs_per_bank, num_rows) {

            this->_policy_name = "ECCScrubbing";

            if (configs.get_str("smd_scrubbing_lock_region") == "bank")
                this->lock_entire_bank = true;

            scrub_interval = std::floor(configs.get_ulong("smd_ecc_scrubbing_period")/(this->_num_rows*this->_num_SAs_per_bank));

            this->row_maint_granularity = configs.get_uint("smd_scrubbing_granularity");
            this->num_maint_machines = configs.get_uint("smd_num_scrubbing_machines");
            this->maint_latency = configs.get_uint("smd_single_scrubbing_latency");

            assert(scrub_interval > this->maint_latency && "[SMDECCScrubbing] ERROR: the latency of a scrub operation should not be longer than the interval for scrubing a new row.");

            scrub_interval *= this->row_maint_granularity;

            // if (rank_id == 0 && chip_id == 0){
            //     std::cout << "[SMDECCScrubbing] Scrubbing a new row once every " << scrub_interval << " cycles."  << std::endl;
            //     std::cout << "[SMDECCScrubbing] The current scrub latency: " << this->maint_latency << " cycles."  << std::endl;
            //     std::cout << "[SMDECCScrubbing] The number of scrub machines: " << this->num_maint_machines << ""  << std::endl;
            // }

            assert((this->_num_banks_in_chip % this->num_maint_machines) == 0 && "[SMDECCScrubbing] ERROR: num_maint_machines should divide the total number of banks with no remainder.");

            for(uint rm_id = 0; rm_id < this->num_maint_machines; rm_id++)
                scrub_machines.emplace_back(rm_id, this->_num_banks_in_chip/this->num_maint_machines, *this);

            // create a MaintenanceCounter for each bank and distribute them evenly across the scrub_machines
            for (uint32_t bank_id = 0; bank_id < this->_num_banks_in_chip; bank_id++) {
                scrub_machines[bank_id % this->num_maint_machines].maint_counters.emplace_back(bank_id, SAs_per_bank/2);
            }
        }

        void tick() {
            this->clk++;

            for (auto& sm : scrub_machines)
                sm.tick();

            if(((this->clk + 1) % scrub_interval) == 0) {
                // time to refresh new rows
                //printf("SCRUBBER TICK\n");
                for (auto& sm : scrub_machines)
                    sm.add_pending_maint();
            }
        }

    private:
        uint64_t scrub_interval;

        std::vector<ECCScrubbingMachine<T>> scrub_machines;
};

template <typename T>
class ECCScrubbingMachine : public MaintenanceMachine<T> {
    public:
        ECCScrubbingMachine(const uint32_t id, const uint32_t num_counters, SMDECCScrubbing<T>& scrubbing_policy) : 
            MaintenanceMachine<T>(id, num_counters, scrubbing_policy){}

    protected:
        bool retry_ref(const MaintenanceCounter& mc) {
            if(!this->maint_policy.is_SA_active(mc.bank_id, mc.sa_counter) && !this->maint_policy.contains_locked_SA(mc.bank_id)) {
                this->maint_completion_clk = this->maint_policy.get_clk() + this->maint_policy.maint_latency*this->maint_policy.row_maint_granularity;

                // DRAMPower estimates DRAM energy for a single DRAM chip. The refresh power is calculated incorrectly (for all chips) if 
                // issueDPowerRowScrubbing() is called by every single chip. Instead, we call issueDPowerRowScrubbing() only from chip0 and estimating the energy of that chip.
                if (this->maint_policy._chip_id == 0)
                    this->maint_policy.ctrl->issueDPowerRowScrubbing(this->maint_policy.row_maint_granularity, this->maint_policy._rank_id, mc.bank_id);
                return true;
            }

            this->maint_completion_clk = 0;
            return false;
        }
};


template <typename T>
class SMDVariableRefresh : public MaintenancePolicy<T> {

    friend class VariableRefreshMachine<T>;

    public:
        SMDVariableRefresh(const Config& configs, Controller<T>* ctrl, const uint32_t rank_id, 
            const uint32_t chip_id, const uint32_t num_banks_in_chip, const uint32_t SAs_per_bank, const uint32_t num_rows) : 
            MaintenancePolicy<T>(configs, ctrl, rank_id, chip_id, num_banks_in_chip, SAs_per_bank, num_rows),
            bloom_filter(configs.get_uint("smd_variable_refresh_bloom_filter_size"), configs.get_uint("smd_variable_refresh_bloom_filter_hashes"), rank_id, chip_id) {

            this->_policy_name = "VariableRefresh";

            uint32_t ref_period_ms = std::ceil(configs.get_ulong("smd_refresh_period")*this->channel->spec->speed_entry.tCK/1000000);
            uint32_t relaxed_ref_period_ms = ref_period_ms * REFRESH_RELAXING_FACTOR;

            // sample normal distribution to estimate rows retention times
            // std::random_device rd{};
            uint32_t num_chips = (64/this->channel->spec->org_entry.dq);
            std::mt19937 gen{rank_id*num_chips + chip_id}; // using fixed seed for repeatable simulations
            std::normal_distribution<> normal_dist{400, 70};

            double weak_row_percentage = (double)configs.get_float("smd_variable_refresh_weak_row_percentage");
            std::discrete_distribution<uint64_t> disc_dist({100 - weak_row_percentage, weak_row_percentage});

            std::function<uint64_t()> sample_ret_time;

            if (configs.get_str("smd_variable_refresh_distribution") == "discrete") {
                sample_ret_time = [&]() {return disc_dist(gen);}; // using discrete distribution
            } else {
                sample_ret_time = [&]() { // using normal distribution
                    if (normal_dist(gen) < relaxed_ref_period_ms) 
                        return 1; 
                    return 0;}; 
            }

            uint32_t num_weaks = 0;
            for (uint32_t bank_id = 0; bank_id < num_banks_in_chip; bank_id++){
                for (uint32_t row_id = 0; row_id < num_rows*SAs_per_bank; row_id++) {
                    bool is_weak = sample_ret_time() == 1;

                    if (is_weak) {
                        bloom_filter.insert(get_bloom_filter_address(bank_id, row_id));
                        num_weaks++;
                    }
                }                
            }

            // printf("[SMD] Num zero entries remaining in the Bloom Filter: %u\n", bloom_filter.num_zero_entries());

            // printf("[VariableRefresh] r:%u c:%u, Number of weak rows: %u\n", rank_id, chip_id, num_weaks);

            if (configs.get_str("smd_single_ref_latency") == "auto" || configs.get_str("smd_single_ref_latency") == "AUTO" ||
                configs.get_str("smd_single_ref_latency") == "Auto") {
                uint NUM_REF_CMDS_PER_REF_WINDOW = 8192;
                uint num_rows_per_bank = this->channel->spec->org_entry.count[int(T::Level::Subarray)]*this->channel->spec->org_entry.count[int(T::Level::Row)];
                this->maint_latency = ceil((this->channel->spec->speed_entry.nRFC*NUM_REF_CMDS_PER_REF_WINDOW)/(float)num_rows_per_bank);
                // std::cout << "[INFO] Automatically setting SMD ref latency to " << this->maint_latency << " cycles.\n";

            } else
                this->maint_latency = configs.get_uint("smd_single_ref_latency");

            ref_interval = std::floor(configs.get_uint("smd_refresh_period")/(this->_num_rows*this->_num_SAs_per_bank));

            assert(ref_interval > this->maint_latency && "[SMDFixedRateRefresh] ERROR: the latency of a refresh operation should not be longer than the interval for refreshing a new row.");

            this->row_maint_granularity = configs.get_uint("smd_row_refresh_granularity");

            // ref_interval is currently calculated based on each ref operation refreshes one row
            // we multiply the ref_interval by row_maint_granularity since that many rows will be refreshed within a single refresh operation, i.e., when an SA is locked
            ref_interval *= this->row_maint_granularity;

            ref_tracker_timeout_period = std::floor(ref_interval*configs.get_float("smd_timeout_to_ref_interval_ratio"));

            this->num_maint_machines = configs.get_uint("smd_num_ref_machines");
            assert((this->_num_banks_in_chip % this->num_maint_machines) == 0 && "[SMDFixedRateRefresh] ERROR: num_maint_machines should divide the total number of banks with no remainder.");

            if(configs.get_bool("smd_worst_case_ref_distribution"))
                ref_interval_offset = chip_id*(this->maint_latency*this->row_maint_granularity);

            ref_machines.reserve(this->num_maint_machines);
            for(uint rm_id = 0; rm_id < this->num_maint_machines; rm_id++)
                ref_machines.emplace_back(rm_id, this->_num_banks_in_chip/this->num_maint_machines, *this);

            // create a MaintenanceCounter for each bank and distribute them evenly across the ref_machines
            for (uint32_t bank_id = 0; bank_id < this->_num_banks_in_chip; bank_id++) {
                if(configs.get_bool("smd_worst_case_ref_distribution"))
                    ref_machines[bank_id % this->num_maint_machines].maint_counters.emplace_back(bank_id, chip_id % SAs_per_bank);
                else
                    ref_machines[bank_id % this->num_maint_machines].maint_counters.emplace_back(bank_id);
            }

        }

        void tick() {
            this->clk++;

            for (auto& rm : ref_machines)
                rm.tick();

            if(((this->clk + 1 + ref_interval_offset) % ref_interval) == 0) {
                // time to refresh new rows
                for (auto& rm : ref_machines)
                    rm.add_pending_maint();
            }
        }

    protected:
        const uint32_t REFRESH_RELAXING_FACTOR = 4; // binning rows as rows requiring refresh at default and 4x refresh period

        bool is_weak_row(const uint32_t bank_id, const uint32_t row_id) {
            uint32_t bf_addr = get_bloom_filter_address(bank_id, row_id);

            return bloom_filter.test(bf_addr);
        }

    private:
        uint32_t ref_interval; // the DRAM chip refreshes a different row from each bank at this interval
        std::vector<VariableRefreshMachine<T>> ref_machines;
        BloomFilter bloom_filter;

        uint32_t ref_interval_offset = 0;

        uint32_t get_bloom_filter_address(const uint32_t bank_id, const uint32_t row_id) const {
            // there can be at most 16 banks, so shifting row_id by 4 and adding bank_id
            return (row_id << 4) + bank_id;
        }
};


template <typename T>
class VariableRefreshMachine : public MaintenanceMachine<T> {
    public:
        VariableRefreshMachine(const uint32_t id, const uint32_t num_counters, SMDVariableRefresh<T>& variable_refresh_policy) : 
            MaintenanceMachine<T>(id, num_counters, variable_refresh_policy), variable_refresh_policy(variable_refresh_policy){

            num_vr_refs
                .name("num_vr_refs_" + to_string(variable_refresh_policy.channel->id) + "_" + to_string(variable_refresh_policy._chip_id) + "_" + to_string(variable_refresh_policy._rank_id) + "_" + to_string(id))
                .desc("The number of rows refreshed by the VariableRefresh mechanism.")
                .precision(0)
            ;

        }

        // void tick () {
        //     MaintenanceMachine<T>::tick();
        // }

    private:

        SMDVariableRefresh<T>& variable_refresh_policy; // it is ugly to store this here again since the inherited class also stores a reference to the same object

        bool process_maint_act_nack(const MaintenanceCounter& mc) {

            // check how many rows need refresh in the chunk of rows to be refreshed
            uint32_t rows_to_refresh = 0;
            if((mc.rollbacks % variable_refresh_policy.REFRESH_RELAXING_FACTOR) == (variable_refresh_policy.REFRESH_RELAXING_FACTOR - 1))
                rows_to_refresh = variable_refresh_policy.row_maint_granularity;
            else {
                for (uint32_t cur_row = mc.row_counter; cur_row < mc.row_counter + variable_refresh_policy.row_maint_granularity; cur_row++) {
                    uint32_t row_to_query = mc.sa_counter*variable_refresh_policy._num_rows + cur_row;
                    if (variable_refresh_policy.is_weak_row(mc.bank_id, row_to_query)) {
                        rows_to_refresh++;
                    }

                }
            }

            if (rows_to_refresh == 0)
                return true;
            
            if (retry_var_ref(mc, rows_to_refresh)){
                variable_refresh_policy.lockSA(mc.bank_id, mc.sa_counter);
                this->last_locked_SA = mc;
                num_vr_refs += rows_to_refresh;
                return true;
            }

            #ifdef SMD_DEBUG
                uint bgid = mc.bank_id/this->maint_policy.channel->spec->org_entry.count[int(T::Level::Bank)];
                uint bid = mc.bank_id % this->maint_policy.channel->spec->org_entry.count[int(T::Level::Bank)];
                printf("%lu\t[Chip %d] Could not lock subarray. Detected an active row in r: %d bg: %d b: %d, sa: %d \n", this->maint_policy.clk, this->maint_policy._chip_id, this->maint_policy._rank_id, bgid, bid, mc.sa_counter);
            #endif // SMD_DEBUG

            return false;
        }

        bool retry_var_ref(const MaintenanceCounter& mc, const uint32_t rows_to_refresh) {
            if(!this->variable_refresh_policy.is_SA_active(mc.bank_id, mc.sa_counter) && !this->variable_refresh_policy.contains_locked_SA(mc.bank_id)) {
                this->maint_completion_clk = this->variable_refresh_policy.get_clk() + this->variable_refresh_policy.maint_latency*rows_to_refresh;
                // DRAMPower estimates DRAM energy for a single DRAM chip. The refresh power is calculated incorrectly (for all chips) if 
                // issueDPowerSMDREF() is called by every single chip. Instead, we call issueDPowerSMDREF() only from chip0 and estimating the energy of that chip.
                if (this->variable_refresh_policy._chip_id == 0)
                    this->variable_refresh_policy.ctrl->issueDPowerSMDREF(rows_to_refresh, this->variable_refresh_policy._rank_id, mc.bank_id);
                return true;
            }

            this->maint_completion_clk = 0;
            return false;
        }

    protected:
        ScalarStat num_vr_refs;
};

template <typename T>
class NeighborRowRefreshMachine : public MaintenanceMachine<T> {

public:
    NeighborRowRefreshMachine(MaintenancePolicy<T>& maint_policy) : MaintenanceMachine<T>(0, 1, maint_policy){}


    void add_pending_maint(const RowAddr ra){
        assert(this->maint_counters.size() == 1);
        assert(pending_neighbor_refs.size() == this->maint_counters[0].pending_maint);

        if (std::find(pending_neighbor_refs.begin(), pending_neighbor_refs.end(), ra) != pending_neighbor_refs.end()) {
            // printf("[NeighborRowRefreshMachine] Neighbor row REF to bank: %u, sa: %u, row: %u is already issued.\n", ra.bank_gid, ra.sa_id, ra.row_id); // DEBUG
            return; //this row is already in the queue of rows to refresh the neighbors of
        }

        MaintenanceMachine<T>::add_pending_maint();
        pending_neighbor_refs.push_back(ra);
    }

    virtual void tick() {

        if(this->maint_completion_clk > this->maint_policy.get_clk())
            return; // the MaintenanceMachine is busy refreshing. Nothing to do

        if(this->maint_completion_clk == this->maint_policy.get_clk()) // just finished refreshing. Release the SA
            this->maint_policy.releaseSA(this->last_locked_SA.bank_id, this->last_locked_SA.sa_counter);

        // check if the counter pointed to by ctr_index has a pending refresh. If so, try to issue a REF. If not (or cannot issue a REF), just increment the ctr_index.
        MaintenanceCounter& cur_mc = this->maint_counters[0];

        if (cur_mc.pending_maint == 0) {
            // std::cout << "[MaintenanceMachine] clk: " <<  maint_policy.get_clk() << " No pending REFs - bank: " << cur_mc.bank_id << " SA: " << cur_mc.sa_counter << std::endl;
            return;
        }

        // set the maintenance counter to the next SA in which to perform neighbor row refresh
        cur_mc.set(pending_neighbor_refs[0]);

        bool maint_initiated = false;
        switch(this->maint_policy.smd_mode) {
            case SMD_MODE::RSQ: {
                maint_initiated = this->process_ref_rsq(cur_mc);
                break;
            }
            case SMD_MODE::ALERT: {
                maint_initiated = this->process_ref_alert(cur_mc);
                break;
            }
            case SMD_MODE::ACT_NACK: {
                maint_initiated = this->process_maint_act_nack(cur_mc);
                break;
            }
            default:
                assert(false && "ERROR: Undefined SMD_MODE!");
        }

        if (!maint_initiated)
            return;

        // decrement the pending maintenance count
        cur_mc.pending_maint--;
        pending_neighbor_refs.erase(pending_neighbor_refs.begin());
    }

private:
    std::vector<RowAddr> pending_neighbor_refs;

};


template <typename T>
class SMDRowHammerProtection : public MaintenancePolicy<T> {

    typedef enum {
        CBF,
        PARA,
        GRAPHENE
    } RHProtectionMode;

    public:
        SMDRowHammerProtection(const Config& configs, Controller<T>* ctrl, const uint32_t rank_id, 
            const uint32_t chip_id, const uint32_t num_banks_in_chip, const uint32_t SAs_per_bank, const uint32_t num_rows) : 
            MaintenancePolicy<T>(configs, ctrl, rank_id, chip_id, num_banks_in_chip, SAs_per_bank, num_rows), rh_machine(*this),
            bf(configs.get_uint("smd_rh_protection_bloom_filter_size"), configs.get_uint("smd_rh_protection_bloom_filter_hashes"),
                                    configs.get_uint("smd_rh_protection_mac"), configs.get_str("smd_rh_protection_bloom_filter_type") == "space_efficient", 
                                    rank_id, chip_id) {

            this->_policy_name = "RowHammerProtection";

            if (configs.get_str("smd_single_ref_latency") == "auto" || configs.get_str("smd_single_ref_latency") == "AUTO" ||
                configs.get_str("smd_single_ref_latency") == "Auto") {
                uint NUM_REF_CMDS_PER_REF_WINDOW = 8192;
                uint num_rows_per_bank = this->channel->spec->org_entry.count[int(T::Level::Subarray)]*this->channel->spec->org_entry.count[int(T::Level::Row)];
                this->maint_latency = ceil((this->channel->spec->speed_entry.nRFC*NUM_REF_CMDS_PER_REF_WINDOW)/(float)num_rows_per_bank);
                std::cout << "[INFO] Automatically setting SMD ref latency to " << this->maint_latency << " cycles.\n";

            } else
                this->maint_latency = configs.get_uint("smd_single_ref_latency");

            if (configs.get_str("smd_rh_protection_mode") == "CBF")
                rh_mode = RHProtectionMode::CBF;

            if (configs.get_str("smd_rh_protection_mode") == "PARA")
                rh_mode = RHProtectionMode::PARA;

            if (configs.get_str("smd_rh_protection_mode") == "Graphene") {
                rh_mode = RHProtectionMode::GRAPHENE;

                uint32_t rfc = this->channel->spec->speed_entry.nRFC;
                uint32_t refi = this->channel->spec->speed_entry.nREFI;
                uint32_t rc = this->channel->spec->speed_entry.nRC;
                refw = configs.get_uint("smd_refresh_period");
                uint32_t num_acts_per_ref_window = std::floor((refw*(1-rfc/(float)refi))/rc);
                // std::cout << "[INFO] Max ACTs per refresh window: " << num_acts_per_ref_window << ".\n";

                uint32_t act_threshold = configs.get_uint("smd_rh_protection_mac")/2;
                uint32_t num_counters = num_acts_per_ref_window/act_threshold;
                // std::cout << "[INFO] Initializing Graphene with " << num_counters << " counters per bank.\n";
                counter_tables = std::vector<GrapheneCounterTable>(this->channel->spec->get_num_all_banks(), GrapheneCounterTable(num_counters, act_threshold));
            }


            this->pending_maint_limit = configs.get_uint("smd_rh_protection_neighbor_ref_queue_size");

            this->row_maint_granularity = configs.get_uint("smd_rh_blast_radius")*2; // multiplying by since refreshing "smd_rh_blast_radius" number of victim from each side of an aggressor

            bf_epoch = configs.get_ulong("smd_rh_protection_bloom_filter_epoch");

            rh_machine.maint_counters.emplace_back(0);

            uint32_t num_chips = (64/this->channel->spec->org_entry.dq);
            gen = std::mt19937{rank_id*num_chips + chip_id}; // using fixed seed for repeatable simulations
            double neighbor_row_refresh_pct = (double)configs.get_float("smd_rh_neighbor_refresh_pct");
            disc_dist = std::discrete_distribution<uint64_t>({100 - neighbor_row_refresh_pct, neighbor_row_refresh_pct});

            issued_neighbor_refs
                .name("issued_neighbor_refs_r" + to_string(rank_id) + "_c" + to_string(chip_id))
                .desc("The number of neighbor refresh operations that has been issued by SMD.")
                .precision(0)
            ;
        }

        void tick() {
            this->clk++;

            if (rh_mode == RHProtectionMode::CBF)
                if((this->clk % bf_epoch) == (bf_epoch - 1))
                    bf.swap_filters();

            if (rh_mode == RHProtectionMode::GRAPHENE)
                if ((this->clk % refw) == 0)
                    for(auto& ct : counter_tables)
                        ct.reset();

            rh_machine.tick();
        }

        void process_row_activation(const std::vector<int>& addr_vec) {

            uint32_t bank_id = this->channel->spec->calc_global_bank_id(addr_vec);
            uint32_t row_id = this->channel->spec->calc_row_id_in_bank(addr_vec);

            if (rh_mode == RHProtectionMode::PARA) {
                if(disc_dist(gen)){
                    rh_machine.add_pending_maint(RowAddr(bank_id, addr_vec[uint32_t(T::Level::Subarray)], addr_vec[uint32_t(T::Level::Row)]));
                    issued_neighbor_refs++;

                    #ifdef SMD_DEBUG
                        printf("%lu\t[%s]\t[Chip %d] Issuing neighbor row refresh for r: %d gbid: %d, grid: %d \n", this->clk, this->_policy_name.c_str(), this->_chip_id, this->_rank_id, bank_id, row_id);
                    #endif // SMD_DEBUG
                }
            } else if (rh_mode == RHProtectionMode::CBF) {
                uint32_t bf_addr = get_bloom_filter_address(bank_id, row_id);
                bf.insert(bf_addr);

                if(bf.test(bf_addr)){
                    // perform neighbor row refresh with a low probability
                    if(disc_dist(gen)){
                        rh_machine.add_pending_maint(RowAddr(bank_id, addr_vec[uint32_t(T::Level::Subarray)], addr_vec[uint32_t(T::Level::Row)]));
                        issued_neighbor_refs++;

                        #ifdef SMD_DEBUG
                            printf("%lu\t[%s]\t[Chip %d] Issuing neighbor row refresh for r: %d gbid: %d, grid: %d \n", this->clk, this->_policy_name.c_str(), this->_chip_id, this->_rank_id, bank_id, row_id);
                        #endif // SMD_DEBUG
                    }
                }
            } else if (rh_mode == RHProtectionMode::GRAPHENE) {
                bool issue_neighbor_ref = counter_tables[bank_id].increment(row_id);

                if (issue_neighbor_ref) {
                    rh_machine.add_pending_maint(RowAddr(bank_id, addr_vec[uint32_t(T::Level::Subarray)], addr_vec[uint32_t(T::Level::Row)]));
                    issued_neighbor_refs++;

                    #ifdef SMD_DEBUG
                        printf("%lu\t[%s]\t[Chip %d] Issuing neighbor row refresh for r: %d gbid: %d, grid: %d \n", this->clk, this->_policy_name.c_str(), this->_chip_id, this->_rank_id, bank_id, row_id);
                    #endif // SMD_DEBUG
                }
            }
            else {
                assert(false && "ERROR: Undefined RHProtectionMode!");
            }
        }

    protected:
        ScalarStat issued_neighbor_refs;

    private:
        NeighborRowRefreshMachine<T> rh_machine;
        DualCountingBloomFilter bf;
        uint64_t bf_epoch = 0;

        RHProtectionMode rh_mode = RHProtectionMode::CBF;

        std::vector<GrapheneCounterTable> counter_tables;
        uint32_t refw = 0;

        // random number generation
        std::mt19937 gen;
        std::discrete_distribution<uint64_t> disc_dist;

        uint32_t get_bloom_filter_address(const uint32_t bank_id, const uint32_t row_id) const {
            // there can be at most 16 banks, so shifting row_id by 4 and adding bank_id
            return (row_id << 4) + bank_id;
        }
};

}

#endif /* __SMD_H */
