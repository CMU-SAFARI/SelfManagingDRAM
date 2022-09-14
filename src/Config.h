#ifndef __CONFIG_H
#define __CONFIG_H

#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <iostream>
#include <cassert>
#include <typeinfo>

namespace ramulator
{

class Config {

private:
    std::map<std::string, std::string> options;
    std::map<std::string, std::string> sim_options;
    std::vector<std::string> trace_files;
    bool use_sim_options = false;

    // Hasan: these should not be needed anymore. Remove after being sure
    //int channels;
    //int ranks;
    //int subarrays;
    //int cpu_tick;
    //int mem_tick;
    //int core_num = 0;
    //long expected_limit_insts = 0;
    //long warmup_insts = 0;

    //// CROW
    //int copy_rows_per_SA = 0;
    //float crow_hit_trcd = 1.0f, crow_hit_tras = 1.0f, crow_hit_twr = 1.0f, crow_hit_tfaw = 1.0f;
    //float crow_copy_trcd = 1.0f, crow_copy_tras = 1.0f, crow_copy_twr = 1.0f, crow_copy_tfaw = 1.0f;
    //float crow_full_restore_trcd = 1.0f, crow_full_restore_tras = 1.0f, 
    //                crow_full_restore_twr = 1.0f, crow_full_restore_tfaw = 1.0f;
    // END - CROW
    
    std::map<std::string, std::string> defaults = {
        // DRAM
        {"standard", "LPDDR4"},
        {"speed", "LPDDR4_3200"},
        {"org", "LPDDR4_8Gb_x16"},
        {"channels", "1"},
        {"ranks", "1"},
        {"subarray_size", "512"},

        // Memory Controller
        {"row_policy", "opened"},
        {"timeout_row_policy_threshold", "120"},
        {"disable_refresh", "false"},
        {"refresh_mult", "1.0f"},
        {"per_bank_refresh", "false"},

        // CPU
        {"cores", "1"},
        {"cpu_tick", "5"},
        {"mem_tick", "2"},
        {"early_exit", "off"},
        {"expected_limit_insts", "200000000"},
        {"warmup_insts", "100000000"},
        {"translation", "Random"},

        // Cache
        {"cache", "L3"},
        {"l3_size", "4194304"},
        {"prefetcher", "off"}, // "off" or "stride"

        // Stride Prefetcher
        {"stride_pref_entries", "1024"},
        {"stride_pref_mode", "1"}, // 0 -> single stride mode, 1 -> multi stride mode
        {"stride_pref_single_stride_tresh", "6"},
        {"stride_pref_multi_stride_tresh", "6"},
        {"stride_pref_stride_start_dist", "1"},
        {"stride_pref_stride_degree", "4"},
        {"stride_pref_stride_dist", "16"},

        // Other
        {"record_cmd_trace", "off"},
        {"print_cmd_trace", "off"},
        {"collect_row_activation_histogram", "off"},

        // DDR4 Maintenance Operations
        {"enable_para", "off"},
        {"para_neighbor_refresh_pct", "1"}, // a number between 0-100 (%) indicating the probability to trigger neighbor row refresh

        {"enable_scrubbing", "off"},
        {"scrubbing_period", "1600000000"}, // scrub at 1 second period

        // Self-Managing DRAM (SMD)
        {"smd", "off"},
        {"smd_mode", "RSQ"}, // RSQ (refresh status query) or ALERT or ACT_NACK
        {"smd_ref_policy", "FixedRate"},
        {"smd_num_ref_machines", "4"},
        {"smd_refresh_period", "102400000"}, // 102400000 cycles = 64ms at 1600Mhz (i.e., 3200 data rate)
        {"smd_row_refresh_granularity", "8"}, // means 8 rows are refreshed from one subarray once locked
        {"smd_timeout_to_ref_interval_ratio", "0.5"},
        {"smd_single_ref_latency", "80"}, // 80 cycles = 50ns at 1600Mhz (i.e., 3200 data rate) When 'auto', smd_single_ref_latency is overwritten based on average refresh latency per row of the regular DRAM refresh, i.e., (8192*tRFC)/NUM_ROWS_PER_BANK
        {"smd_pending_ref_limit", "9"},
        {"smd_max_row_open_intervals", "8"},
        {"smd_act_to_nack_cycles", "4"},
        // to read out an entire row, the row needs to remain open for about 355ns (from the cycle ACT is issued until the PRE is issued). 
        // So, this time does not include tRP, but tRCD and 128*4(cycles between consecutive READ commands) 
        {"smd_act_nack_resend_interval", "100.0f"},
        {"smd_worst_case_ref_distribution", "false"},
        // What happens when not all chips return a NACK:
        //      PRE: PREcharge the bank, try to find new requests to serve while NACKing chips get out of maintenance state
        //      WAIT: Wait for a while until NACKing chips get out of maintenance state
        //      COMB: Dynamically switch between PRE or WAIT policies based on a heuristic
        {"smd_combined_policy_threshold", "32767"}, // set 0 for the PRE policy, a large integer for the WAIT policy
        
        // SMD Variable Refresh Rate
        {"smd_variable_refresh_distribution", "discrete"},
        {"smd_variable_refresh_weak_row_percentage", "0.05"},
        {"smd_variable_refresh_bloom_filter_size", "8192"},
        {"smd_variable_refresh_bloom_filter_hashes", "6"},

        // SMD Memory Scrubbing
        {"smd_ecc_scrubbing_enabled", "false"},
        {"smd_scrubbing_lock_region", "bank"}, // valid entries: 'bank' and 'region'. When bank, scrubbing puts an entire bank under maintenance whereas region operates as in smd refresh
        {"smd_num_scrubbing_machines", "1"}, // scrub one bank at a time
        {"smd_pending_scrub_limit", "9"},
        {"smd_scrubbing_granularity", "1"}, // number of rows scrubbed within a single smd region lock
        {"smd_single_scrubbing_latency", "552"}, // tRCD + tRP + 128*4 (cycles to read an entire row, not adding writing back since errors are assumed to be rare and mostly a write back won't be needed)
        {"smd_ecc_scrubbing_period", "1600000000"}, // scrub at 1 second period
        {"smd_timeout_to_ecc_scrubbing_interval_ratio", "0.5"},

        // SMD RowHammer Protection
        {"smd_rh_protection_enabled", "false"},
        {"smd_rh_protection_mode", "CBF"}, // CBF or PARA or Graphene. CBF is Counting Bloom Filter, similar to BlockHammer's aggressor detection logic
        {"smd_rh_blast_radius", "1"}, // Indicates how many neighbor rows from each side of an aggressor will be refreshed
        {"smd_rh_neighbor_refresh_pct", "1"}, // a number between 0-100 (%) indicating the probability to trigger neighbor row refresh
        {"smd_rh_protection_bloom_filter_size", "8192"},
        {"smd_rh_protection_bloom_filter_hashes", "4"},
        {"smd_rh_protection_bloom_filter_epoch", "51200000"}, // 32ms
        {"smd_rh_protection_bloom_filter_type", "blockhammer"}, // space_efficient or blockhammer
        {"smd_rh_protection_mac", "8192"},
        {"smd_rh_protection_neighbor_ref_queue_size", "1024"},

        // DRAMPower
        {"dpower_memspec_path", "./configs/SMD_configs/16Gb_DDR4_3200_8bit.xml"},
        {"dpower_include_io_and_termination", "true"},


        // CROW
        {"crow_entry_evict_hit_threshold", "0"},
        {"crow_half_life", "1000"},
        {"crow_to_mru_frac", "0.0f"},
        {"enable_crow_upperbound", "false"},
        {"enable_tl_dram", "false"},
        {"copy_rows_per_SA", "0"},
        {"weak_rows_per_SA", "0"},
        {"crow_hit_trcd", "1.0f"}, // obsolete
        {"crow_hit_tras", "1.0f"}, // obsolete
        {"crow_hit_twr", "1.0f"}, // obsolete
        {"crow_hit_tfaw", "1.0f"}, // obsolete
        {"crow_copy_trcd", "1.0f"}, // obsolete
        {"crow_copy_tras", "1.0f"}, // obsolete
        {"crow_copy_twr", "1.0f"}, // obsolete
        {"crow_copy_tfaw", "1.0f"},  // obsolete
        {"crow_table_grouped_SAs", "1"} // the number of SAs to be grouped in CROW table to share the entries. 
                                        // This is an optimization to reduce the storage requirement of the CROW table
    };

    template<typename T>
    T get(const std::string& param_name, T (*cast_func)(const std::string&)) const {

        std::string param = this->operator[](param_name);

        try {
            if(param == "") {
                param = defaults.at(param_name); // get the default param, if exists

                if(param == "") {
                    std::cerr << "ERROR: All options should have their default values in Config.h!" << std::endl;
                    std::cerr << "No default value found for: " << param_name << std::endl;
                    exit(-1);
                }
            }
            return (*cast_func)(param); 
        } catch (const std::invalid_argument& ia) {
            std::cerr << "Invalid argument: " << ia.what() << std::endl;
            exit(-1);
        } catch (const std::out_of_range& oor) {
            std::cerr << "Out of Range error: " << oor.what() << ". Param name: " << param_name << " is undefined." << std::endl;
            exit(-1);
        } catch (...) {
            std::cerr << "Error! Unhandled exception." << std::endl;
            std::exit(-1);
        }

        return T();
    }

    static bool param_to_bool(const std::string& s) {
        if(s == "true")
            return true;

        if(s == "on")
            return true;

        return false;
    }

    bool sim_contains(const std::string& name) const {
        if(sim_options.find(name) != sim_options.end()) {
            return true;
        } else {
            return false;
        }
    }

public:
    Config() {}
    Config(const std::string& fname);
    void parse(const std::string& fname);
    void parse_cmdline(const int argc, char** argv);
    std::string operator [] (const std::string& name) const {
       if(use_sim_options && sim_contains(name))
           return (sim_options.find(name))->second;

       if (contains(name)) {
         return (options.find(name))->second;
       } else {
         return "";
       }
    }

    int get_int(const std::string& param_name) const {
        
        return get<int>(param_name, [](const std::string& s){ return std::stoi(s); }); // Hasan: the lambda function trick helps ignoring the optional argument of stoi
    }

    uint32_t get_uint(const std::string& param_name) const {
        
        return get<uint32_t>(param_name, [](const std::string& s){ return (uint32_t)std::strtoul(s.c_str(), nullptr, 10); }); // Hasan: the lambda function trick helps ignoring the optional argument of stoi
    }

    uint64_t get_ulong(const std::string& param_name) const {
        
        return get<uint64_t>(param_name, [](const std::string& s){ return std::strtoul(s.c_str(), nullptr, 10); }); // Hasan: the lambda function trick helps ignoring the optional argument of stoi
    }
    
    long get_long(const std::string& param_name) const {
        
        return get<long>(param_name, [](const std::string& s){ return std::stol(s); });
    }
    
    float get_float(const std::string& param_name) const {

        return get<float>(param_name, [](const std::string& s){ return std::stof(s); });
    }

    bool get_bool(const std::string& param_name) const {
        return get<bool>(param_name, param_to_bool);
    }

    std::string get_str(const std::string& param_name) const {
        return get<std::string>(param_name, [](const std::string& s){ return s; });
    }

    bool contains(const std::string& name) const {

      if(use_sim_options && sim_contains(name))
          return true;

      if (options.find(name) != options.end()) {
        return true;
      } else {
        return false;
      }
    } 

    void add (const std::string& name, const std::string& value) {

      if(use_sim_options) {
        if(!sim_contains(name))
            sim_options.insert(make_pair(name, value));
        else
            printf("ramulator::Config::add options[%s] already set.\n", name.c_str());

        return;
      }

      if (!contains(name)) {
        options.insert(make_pair(name, value));
      } else {
        printf("ramulator::Config::add options[%s] already set.\n", name.c_str());
      }
    }

    template<typename T>
    void update (const std::string& name, const T& value) {
        if(use_sim_options)
            sim_options[name] = std::to_string(value);
        else
            options[name] = std::to_string(value);
    }

    void enable_sim_options () {
        use_sim_options = true;
    }

    void disable_sim_options () {
        use_sim_options = false;
    }

    // void set_core_num(int _core_num) {core_num = _core_num;}

    // int get_channels() const {return channels;}
    // int get_subarrays() const {return subarrays;}
    // int get_ranks() const {return ranks;}
    // int get_cpu_tick() const {return cpu_tick;}
    // int get_mem_tick() const {return mem_tick;}
    // int get_core_num() const {return core_num;}
    // long get_expected_limit_insts() const {return expected_limit_insts;}
    // long get_warmup_insts() const {return warmup_insts;}
    
    // int get_copy_rows_per_SA() const {return copy_rows_per_SA;}
    // float get_crow_hit_trcd() const {return crow_hit_trcd;}
    // float get_crow_hit_tras() const {return crow_hit_tras;}
    // float get_crow_hit_twr() const {return crow_hit_twr;}
    // float get_crow_hit_tfaw() const {return crow_hit_tfaw;}
    // float get_crow_copy_trcd() const {return crow_copy_trcd;}
    // float get_crow_copy_tras() const {return crow_copy_tras;}
    // float get_crow_copy_twr() const {return crow_copy_twr;}
    // float get_crow_copy_tfaw() const {return crow_copy_tfaw;}
    // float get_crow_full_restore_trcd() const {return crow_full_restore_trcd;}
    // float get_crow_full_restore_tras() const {return crow_full_restore_tras;}
    // float get_crow_full_restore_twr() const {return crow_full_restore_twr;}
    // float get_crow_full_restore_tfaw() const {return crow_full_restore_tfaw;}
    
    bool has_l3_cache() const {
      const std::string& cache_type = get_str("cache");   
      return (cache_type == "all") || (cache_type == "L3");
    }

    bool has_core_caches() const {
      const std::string& cache_type = get_str("cache");   
      return (cache_type == "all" || cache_type == "L1L2");
    }
    
    //bool is_early_exit() const {
    //  // the default value is true
    //  if (options.find("early_exit") != options.end()) {
    //    if ((options.find("early_exit"))->second == "off") {
    //      return false;
    //    }
    //    return true;
    //  }
    //  return true;
    //}

    bool calc_weighted_speedup() const {
      return (get_long("expected_limit_insts") != 0);
    }

    // bool record_cmd_trace() const {
    //   // the default value is false
    //   if (options.find("record_cmd_trace") != options.end()) {
    //     if ((options.find("record_cmd_trace"))->second == "on") {
    //       return true;
    //     }
    //     return false;
    //   }
    //   return false;
    // }
    // bool print_cmd_trace() const {
    //   // the default value is false
    //   if (options.find("print_cmd_trace") != options.end()) {
    //     if ((options.find("print_cmd_trace"))->second == "on") {
    //       return true;
    //     }
    //     return false;
    //   }
    //   return false;
    // }

    const std::vector<std::string>& get_trace_files() const {
        return trace_files;
    }

    void print_configs() const {

        std::cout << "==========================================" << std::endl;
        std::cout << "===== Configuration parameters =====" << std::endl;

        std::cout << "Default parameters:" << std::endl;
        for (auto& param : defaults)
            std::cout << param.first << ": " << param.second << std::endl;

        std::cout << std::endl;
        std::cout << "Baseline parameters:" << std::endl;
        for (auto& param : options)
            std::cout << param.first << ": " << param.second << std::endl;

        std::cout << std::endl;
        std::cout << "Simulation parameters (used after warmup):" << std::endl;
        for (auto& param : sim_options)
            std::cout << param.first << ": " << param.second << std::endl;

        std::cout << "===== END - Configuration parameters =====" << std::endl;
        std::cout << "==========================================" << std::endl;
    }
};


} /* namespace ramulator */

#endif /* _CONFIG_H */

