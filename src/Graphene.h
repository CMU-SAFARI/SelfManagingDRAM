#ifndef GRAPHENE_H
#define GRAPHENE_H

#include "Config.h"
#include "Request.h"

#include <vector>
#include <unordered_map>
#include <random>

namespace ramulator
{  
  template <typename T>
  class Controller;

  template <class T>
  class Graphene
  {
  public:
    Graphene(const Config& configs, Controller<T>* ctrl);
    ~Graphene() = default;
    /**
     * @brief: Schedule preventive refresh to victims of the aggressor row at addr_vec
     * TODO: Think about moving this to the parent class
     */
    void schedule_preventive_refresh(const std::vector<int> addr_vec);
    void tick();
    void update(typename T::Command cmd, const std::vector<int> &addr_vec, uint64_t open_for_nclocks);
    
    std::string to_string()
    {
    return "Refresh-based RowHammer Defense\n"
                        "  └  "
                        "Graphene\n";
    }

  private:
    Controller<T>* ctrl;
    int clk;
    int no_table_entries;
    int activation_threshold;
    int reset_period; // in nanoseconds
    int reset_period_clk; // in clock cycles
    bool debug = false;
    bool debug_verbose = false;
    int no_rows_per_bank;
    int no_rows_per_subarray;
    int no_banks; // b per bg
    int no_bank_groups; // bg per rank
    int no_ranks;
    bool pending_preventive_refresh = false;
    std::vector<int> last_addr_vec;
    // per bank activation count table
    // indexed using rank id, bank id
    // e.g., if rank 0, bank 4, index is 4
    // if rank 1, bank 5, index is 16 (assuming 16 banks/rank) + 5
    std::vector<std::unordered_map<int, int>> activation_count_table;
    // spillover counter per bank
    std::vector<int> spillover_counter;

    // take rowpress into account
    bool rowpress = false;
    int rowpress_increment_nticks = 0;
    int nRAS = 0;
  };

  template <class T>
  Graphene<T>::Graphene(const Config& config, Controller<T>* ctrl) : ctrl(ctrl)
  {
    no_table_entries = config.get_uint("graphene_no_table_entries");
    activation_threshold = config.get_uint("graphene_activation_threshold");
    reset_period = config.get_uint("graphene_reset_period");
    debug = config.get_bool("graphene_debug");
    debug_verbose = config.get_bool("graphene_debug_verbose");
    rowpress = config.get_bool("graphene_rowpress");
    rowpress_increment_nticks = config.get_uint("graphene_rowpress_increment_nticks");
    nRAS = 0;
    reset_period_clk = (int) (reset_period / ((float) ctrl->channel->spec->speed_entry.tCK));

    assert(!rowpress);

    // Get organization configuration
    no_rows_per_bank = ctrl->channel->spec->org_entry.count[int(T::Level::Row)] * ctrl->channel->spec->org_entry.count[int(T::Level::Subarray)];
    no_rows_per_subarray = ctrl->channel->spec->org_entry.count[int(T::Level::Row)];
    no_bank_groups = ctrl->channel->spec->org_entry.count[int(T::Level::BankGroup)];
    no_banks = ctrl->channel->spec->org_entry.count[int(T::Level::Bank)];
    no_ranks = ctrl->channel->spec->org_entry.count[int(T::Level::Rank)];

    if (debug)
    {
      std::cout << "Graphene: no_table_entries: " << no_table_entries << std::endl;
      std::cout << "Graphene: activation_threshold: " << activation_threshold << std::endl;
      std::cout << "Graphene: reset_period: " << reset_period << std::endl;
      std::cout << "Graphene: reset_period_clk: " << reset_period_clk << std::endl;
      std::cout << "  └  tCK: " << ((float) ctrl->channel->spec->speed_entry.tCK) << std::endl;
      std::cout << "Graphene: no_rows_per_bank: " << no_rows_per_bank << std::endl;
      std::cout << "Graphene: no_rows_per_subarray: " << no_rows_per_subarray << std::endl;
      std::cout << "Graphene: no_bank_groups: " << no_bank_groups << std::endl;
      std::cout << "Graphene: no_banks: " << no_banks << std::endl;
      std::cout << "Graphene: no_ranks: " << no_ranks << std::endl;
    }

    // Initialize activation count table
    // each table has no_table_entries entries
    for (int i = 0; i < no_banks * no_bank_groups * no_ranks; i++)
    {
      std::unordered_map<int, int> table;
      for (int j = -150000; j < -150000 + no_table_entries; j++)
        table.insert(std::make_pair(j, 0));
      activation_count_table.push_back(table);
    }

    // Initialize spillover counter
    spillover_counter = std::vector<int>(no_banks * no_bank_groups * no_ranks, 0);
  }
  
  template <class T>
  void Graphene<T>::schedule_preventive_refresh(const std::vector<int> addr_vec)
  {
    // create two new preventive refreshes targeting addr_vec
    std::vector<int> m1_addr_vec = addr_vec;
    std::vector<int> m2_addr_vec = addr_vec;
    m1_addr_vec[int(T::Level::Row)] = (m1_addr_vec[int(T::Level::Row)] + 1) % no_rows_per_subarray;
    m2_addr_vec[int(T::Level::Row)] = (m2_addr_vec[int(T::Level::Row)] - 1) % no_rows_per_subarray;

    if (debug)
    {
      std::cout << "Scheduling preventive refreshes for row " << addr_vec[int(T::Level::Row)] << std::endl;
      std::cout << "  └  " << "m1: " << m1_addr_vec[int(T::Level::Row)] << std::endl;
      std::cout << "  └  " << "m2: " << m2_addr_vec[int(T::Level::Row)] << std::endl;
    }

    Request m1(m1_addr_vec, Request::Type::PARA_REFRESH, nullptr);
    Request m2(m2_addr_vec, Request::Type::PARA_REFRESH, nullptr);
    
    // schedule the requests
    this->ctrl->enqueue(m1);
    this->ctrl->enqueue(m2);
  }

  template <class T>
  void Graphene<T>::tick()
  {
    if (pending_preventive_refresh)
    {
      schedule_preventive_refresh(last_addr_vec);
      pending_preventive_refresh = false;
    }

    // reset activation count table every reset_period
    // by setting every element to 0
    // and reset spillover counter
    if (clk % reset_period_clk == 0)
    {
      for (int i = 0; i < no_banks * no_bank_groups * no_ranks; i++)
      {
        for (auto it = activation_count_table[i].begin(); it != activation_count_table[i].end(); it++)
          it->second = 0;
        spillover_counter[i] = 0;
      }
    }

    clk++;
  }

  template <class T>
  void Graphene<T>::update(typename T::Command cmd, const std::vector<int> &addr_vec, uint64_t open_for_nclocks)
  {
    if (cmd != T::Command::ACT)
      return;
    
    int bank_group_id = addr_vec[int(T::Level::BankGroup)];
    int bank_id = addr_vec[int(T::Level::Bank)];
    int rank_id = addr_vec[int(T::Level::Rank)];
    int row_id = (addr_vec[int(T::Level::Subarray)] * no_rows_per_subarray) + addr_vec[int(T::Level::Row)];

    int index = rank_id * no_banks * no_bank_groups + bank_group_id * no_banks + bank_id;

    if (debug_verbose)
    {
      std::cout << "Graphene: ACT on row " << row_id << std::endl;
      std::cout << "  └  " << "rank: " << rank_id << std::endl;
      std::cout << "  └  " << "bank_group: " << bank_group_id << std::endl;
      std::cout << "  └  " << "bank: " << bank_id << std::endl;
      std::cout << "  └  " << "index: " << index << std::endl;
    }

    // check if the row is already in the table
    if (activation_count_table[index].find(row_id) == activation_count_table[index].end())
    {
      // if row is not in the table, find an entry 
      // with a count equal to that of the spillover counter
      bool found = false;
      int to_remove = -1;
      int spillover_value = -1;

      for (auto it = activation_count_table[index].begin(); it != activation_count_table[index].end(); it++)
      {
        if (debug_verbose)
          std::cout << "  └  " << "checking row " << it->first << " with count " << it->second << std::endl;

        if (it->second == spillover_counter[index])
        {
          // if we find an entry, record it
          spillover_value = it->second;
          to_remove = it->first;
          found = true;
          break;
        }
      }
      if (found)
      {
        // for debug
        if (debug_verbose)
        {
          // print the row that is being removed
          std::cout << "Removing row " << to_remove << " from table " << index << std::endl;
          // print the row that is being added
          std::cout << "Adding row " << row_id << " to table " << index << std::endl;
          std::cout << "  └  " << "spillover counter: " << spillover_counter[index] << std::endl;
        }
        // remove to_remove from the table
        activation_count_table[index].erase(to_remove);
        // add row_id to the table
        activation_count_table[index][row_id] = spillover_value;
      }
      // if we did not find such an entry, increment spillover counter by one
      else
      {
        int increment = rowpress ? ((open_for_nclocks - nRAS + rowpress_increment_nticks)/rowpress_increment_nticks) + 1 
                                  : 1;
        spillover_counter[index] += increment;
      }
    }
    else
    {
      // if row in table, increment its activation count
      activation_count_table[index][row_id] += rowpress ? ((open_for_nclocks - nRAS + rowpress_increment_nticks)/rowpress_increment_nticks) + 1 
                                                        : 1;
      
      if (debug_verbose)
      {
        std::cout << "Row " << row_id << " in table[" << index << "]" << std::endl;
        std::cout << "  └  " << "threshold: " << activation_threshold << std::endl;
        std::cout << "  └  " << "count: " << activation_count_table[index][row_id] << std::endl;
      }

      // check if the count exceeds the threshold
      if (activation_count_table[index][row_id] >= activation_threshold)
      {
        if (debug)
          std::cout << "Row " << row_id << " in table " << index << " has exceeded the threshold!" << std::endl;
        // if yes, schedule preventive refreshes
        pending_preventive_refresh = true;
        last_addr_vec = addr_vec;
        activation_count_table[index][row_id] = spillover_counter[index];
      }
    }
  }
}
#endif