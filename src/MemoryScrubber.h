#pragma once

#include "Config.h"

namespace ramulator
{

class ScrubCounter {
    public:
    
    ScrubCounter(uint32_t _num_ranks, uint32_t _num_banks, uint32_t _num_sas, 
        uint32_t _num_rows, uint32_t _num_cols) : num_ranks(_num_ranks), num_banks(_num_banks), 
        num_sas(_num_sas), num_rows(_num_rows), num_cols(_num_cols) {}

    
    bool is_row_scrub_in_progress() {
        return scrub_in_progress;
    }

    bool is_row_scrub_complete() {
        return !is_row_scrub_in_progress();
    }

    void begin_row_scrub() {
        assert(is_row_scrub_complete());
        scrub_in_progress = true;
    }

    void advance() {
        assert(scrub_in_progress);
        col_id++;

        if (col_id == num_cols) {
            scrub_in_progress = false;
            col_id = 0;
            rank_id++;

            if (rank_id == num_ranks) {
                rank_id = 0;
                bank_id++;

                if (bank_id == num_banks) {
                    bank_id = 0;
                    row_id++;

                    if (row_id == num_rows) {
                        row_id = 0;
                        sa_id++;

                        if (sa_id == num_sas) {
                            sa_id = 0;
                        }
                    }
                }
            }

        }
    }

    uint32_t get_rank_id() { return rank_id; }
    uint32_t get_bank_id() { return bank_id; }
    uint32_t get_sa_id() { return sa_id; }
    uint32_t get_row_id() { return row_id; }
    uint32_t get_col_id() { return col_id; }

    private:
    uint32_t rank_id = 0;
    uint32_t bank_id = 0;
    uint32_t sa_id = 0;
    uint32_t row_id = 0;
    uint32_t col_id = 0;

    uint32_t num_ranks, num_banks, num_sas, num_rows, num_cols;

    bool scrub_in_progress = false;
};

template <typename T>
class MemoryScrubber {

    public:
        MemoryScrubber(const Config& configs, Controller<T>* ctrl) : ctrl(ctrl) {
            reload_options(configs);

            uint32_t num_ranks = ctrl->channel->spec->org_entry.count[int(T::Level::Rank)];
            uint32_t num_banks = ctrl->channel->spec->get_num_banks_per_rank();
            uint32_t num_SAs_per_bank = ctrl->channel->spec->org_entry.count[int(T::Level::Subarray)];
            uint32_t num_rows = ctrl->channel->spec->org_entry.count[int(T::Level::Row)];
            uint32_t num_cols = ctrl->channel->spec->org_entry.count[int(T::Level::Column)]/8; // we divide by 8 to treat each column as 64-byte data chunk

            scrub_counter = new ScrubCounter(num_ranks, num_banks, num_SAs_per_bank, num_rows, num_cols);
        }

        ~MemoryScrubber() {
            delete scrub_counter;
        }

        void tick() {
            if ((ctrl->clk % scrub_interval) == (scrub_interval - 1)){
                pending_row_scrubs++;
                
                assert(pending_row_scrubs <= 8 && "ERROR: Memory Scrubber exceeded the max pending row scrub count.");
            }

            if (pending_row_scrubs > 0) {
                if (scrub_counter->is_row_scrub_complete()) {
                    scrub_counter->begin_row_scrub();
                    pending_row_scrubs--;
                }
            }

            if (scrub_counter->is_row_scrub_in_progress()) {
                
                vector<int> addr_vec;
                addr_vec.resize(int(T::Level::MAX));
                addr_vec[int(T::Level::Channel)] = ctrl->channel->id;
                addr_vec[int(T::Level::Rank)] = scrub_counter->get_rank_id();
                uint32_t bg_id = scrub_counter->get_bank_id()/ctrl->channel->spec->org_entry.count[int(T::Level::Bank)];
                addr_vec[int(T::Level::BankGroup)] = bg_id;
                uint32_t bank_id = scrub_counter->get_bank_id() % ctrl->channel->spec->org_entry.count[int(T::Level::Bank)];
                addr_vec[int(T::Level::Bank)] = bank_id;
                addr_vec[int(T::Level::Subarray)] = scrub_counter->get_sa_id();
                addr_vec[int(T::Level::Row)] = scrub_counter->get_row_id();
                addr_vec[int(T::Level::Column)] = scrub_counter->get_col_id() * 8; // we multiply by 8 to treat each column as 64-byte data chunk

                Request scrub_req(addr_vec, Request::Type::READ, nullptr);

                if(ctrl->enqueue(scrub_req))
                    scrub_counter->advance();
            }
        }

        void reload_options(const Config& configs) {
            uint32_t num_banks = ctrl->channel->spec->get_num_all_banks();
            uint32_t num_SAs_per_bank = ctrl->channel->spec->org_entry.count[int(T::Level::Subarray)];
            uint32_t num_rows = ctrl->channel->spec->org_entry.count[int(T::Level::Row)];

            scrub_interval = std::floor(configs.get_ulong("scrubbing_period")/(num_rows*num_SAs_per_bank*num_banks));
        }

    private:
    Controller<T>* ctrl;
    uint32_t scrub_interval;
    uint32_t pending_row_scrubs = 0;
    ScrubCounter* scrub_counter;
};

}