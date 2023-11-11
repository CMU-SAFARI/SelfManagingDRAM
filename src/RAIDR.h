#ifndef RAIDR_H
#define RAIDR_H

#include <functional>

#include "Request.h"
#include "Config.h"
#include "BloomFilter.h"

namespace ramulator
{

template <typename T>
class Controller;

template <typename T>
class RAIDR
{
public:
  // constructor
  RAIDR(const Config& configs, Controller<T>* ctrl);
  void tick_ref();

private:
  Controller<T>* ctrl;
  uint64_t clk = 0;
  uint64_t refresh_row_counter = 0;
  int period_counter = 0;
  uint64_t ticked_refresh = 0;
  BloomFilter** bloom_filters;
  // std::vector<BloomFilter> bloom_filters;

  int num_ranks;
  int num_banks;
  int num_bank_groups;
  int num_rows;
  int num_subarrays;
  int num_rows_per_subarray;

  int nREFI_internal;

  float refresh_mult;
  
  bool is_weak_row(const uint32_t rank_id, const uint32_t bank_id, const uint32_t row_id) {
      uint32_t bf_addr = get_bloom_filter_address(bank_id, row_id);
      return bloom_filters[rank_id]->test(bf_addr);
  }

  uint32_t get_bloom_filter_address(const uint32_t bank_id, const uint32_t row_id) const {
      // there can be at most 16 banks, so shifting row_id by 4 and adding bank_id
      return (row_id << 4) + bank_id;
  }

  std::vector<int> addr_vec_from_refresh_counter()
  {
    // assumes channel + rank + bg + bank + row + col
    std::vector<int> addr_vec;
    addr_vec.resize(7);
    addr_vec[0] = ctrl->channel->id;

    uint64_t _refresh_row_counter = refresh_row_counter;

    // select the first bank group bits from the refresh counter
    int bg_bits = _refresh_row_counter & ((1 << ((int) log2(num_bank_groups))) - 1);
    _refresh_row_counter >>= ((int) log2(num_bank_groups));
    addr_vec[2] = bg_bits;
    // select the first bank bits from the refresh counter
    int bank_bits = _refresh_row_counter & ((1 << ((int) log2(num_banks))) - 1);
    _refresh_row_counter >>= ((int) log2(num_banks));
    addr_vec[3] = bank_bits;

    // select the first rank bits from the refresh counter
    if (num_ranks > 1)
    {
      int rank_bits = _refresh_row_counter & ((1 << ((int) log2(num_ranks))) - 1);
      _refresh_row_counter >>= ((int) log2(num_ranks));
      addr_vec[1] = rank_bits;
    }
    else
    {
      addr_vec[1] = 0;
    }
    // select the first subarray bits from the refresh counter
    int subarray_bits = _refresh_row_counter & ((1 << ((int) log2(num_subarrays))) - 1);
    _refresh_row_counter >>= ((int) log2(num_subarrays));
    addr_vec[4] = subarray_bits;

    // select the first row bits from the refresh counter
    int row_bits = _refresh_row_counter & ((1 << ((int) log2(num_rows_per_subarray))) - 1);
    _refresh_row_counter >>= ((int) log2(num_rows_per_subarray));
    addr_vec[5] = row_bits;

    addr_vec[6] = 0;

    // print refresh row counter
    // std::cout << "refresh_row_counter: " << refresh_row_counter << std::endl;

    // std::cout << "bg_bits: " << bg_bits << std::endl;
    // std::cout << "bank_bits: " << bank_bits << std::endl;
    // std::cout << "subarray_bits: " << subarray_bits << std::endl;
    // std::cout << "row_bits: " << row_bits << std::endl;


    return addr_vec;
  }
};

template <class T>
RAIDR<T>::RAIDR(const Config& configs, Controller<T>* ctrl) : ctrl(ctrl) {
  // get number of ranks
  num_ranks = ctrl->channel->children.size();

  // get number of banks, bank groups, and rows
  num_banks = ctrl->channel->spec->org_entry.count[(int)T::Level::Bank];
  num_bank_groups = ctrl->channel->spec->org_entry.count[(int)T::Level::BankGroup];
  num_rows_per_subarray = ctrl->channel->spec->org_entry.count[(int)T::Level::Row];
  num_subarrays = ctrl->channel->spec->org_entry.count[(int)T::Level::Subarray];
  num_rows = num_rows_per_subarray * num_subarrays;

  // print number of ranks, banks, bank groups, and rows
  std::cout << "num_ranks: " << num_ranks << std::endl;
  std::cout << "num_banks: " << num_banks << std::endl;
  std::cout << "num_bank_groups: " << num_bank_groups << std::endl;
  std::cout << "num_rows: " << num_rows << std::endl;

  refresh_mult = configs.get_float("refresh_mult");

  bloom_filters = new BloomFilter*[num_ranks];
  for (int i = 0; i < num_ranks; i++)
  {
    BloomFilter* bf = new BloomFilter(configs.get_uint("raidr_variable_refresh_bloom_filter_size"), configs.get_uint("raidr_variable_refresh_bloom_filter_hashes"), i);
    bloom_filters[i] = bf;
  }

  uint32_t ref_period_ms = std::ceil(configs.get_ulong("raidr_refresh_period")*ctrl->channel->spec->speed_entry.tCK/1000000);
  uint32_t relaxed_ref_period_ms = ref_period_ms * 4;

  std::mt19937 gen{1337}; // using fixed seed for repeatable simulations
  std::normal_distribution<> normal_dist{400, 70};

  double weak_row_percentage = (double)configs.get_float("raidr_variable_refresh_weak_row_percentage");
  std::discrete_distribution<uint64_t> disc_dist({100 - weak_row_percentage, weak_row_percentage});

  std::function<uint64_t()> sample_ret_time;

  if (configs.get_str("raidr_variable_refresh_distribution") == "discrete")
  {
    sample_ret_time = [&]()
    { return disc_dist(gen); }; // using discrete distribution
  }
  else
  {
    sample_ret_time = [&]() { // using normal distribution
      if (normal_dist(gen) < relaxed_ref_period_ms)
        return 1;
      return 0;
    };
  }

  // TODO: Generate a bloom filter for each rank
  uint32_t num_weaks = 0;
  for (uint32_t rank_id = 0; rank_id < num_ranks; rank_id++)
  {
    for (uint32_t bank_id = 0; bank_id < num_banks * num_bank_groups; bank_id++)
    {
      for (uint32_t row_id = 0; row_id < num_rows; row_id++)
      {
        bool is_weak = sample_ret_time() == 1;

        if (is_weak)
        {
          bloom_filters[rank_id]->insert(get_bloom_filter_address(bank_id, row_id));
          num_weaks++;
        }
      }
    }
  }

  // we refresh one row from one bank every refresh_interval
  nREFI_internal = ((64*1000*1000 / ctrl->channel->spec->speed_entry.tCK) * refresh_mult) / (num_ranks * num_banks * num_bank_groups * num_rows);

  // priunt nREFI_internal
  std::cout << "===============RAIDR===============" << std::endl;
  std::cout << "nREFI_internal: " << nREFI_internal << std::endl;
}

// Basic refresh scheduling for all bank refresh that is applicable to all DRAM types
template <class T>
void RAIDR<T>::tick_ref() {
  clk++;

  // how many refreshes should we do in a refresh period
  int refreshes_per_period = num_ranks * num_banks * num_bank_groups * num_rows;

  // Time to check if we need to schedule a refresh
  if ((clk - ticked_refresh) >= nREFI_internal) {
    std::vector<int> addr_vec = addr_vec_from_refresh_counter();

    bool is_weak = is_weak_row(addr_vec[1], (addr_vec[2]*num_banks) + addr_vec[3], (addr_vec[4]*num_rows_per_subarray)+addr_vec[5]);

    if (is_weak)
    {
      auto refresh_req = Request(addr_vec, Request::Type::RAIDR_REFRESH, nullptr);
      //print where we are refreshing
      // std::cout << "WEAK refreshing rank: " << addr_vec[1] << " bank: " << (addr_vec[2]*num_banks) + addr_vec[3]  << " row: " << (addr_vec[4]*num_rows_per_subarray)+addr_vec[5] << std::endl;
      ctrl->enqueue(refresh_req);
    }
    else
    {
      if (period_counter % 4 == 3)
      {
        auto refresh_req = Request(addr_vec, Request::Type::RAIDR_REFRESH, nullptr);
        // print where we are refreshing
        // std::cout << "STRONG refreshing rank: " << addr_vec[1] << " bank: " << (addr_vec[2]*num_banks) + addr_vec[3]  << " row: " << (addr_vec[4]*num_rows_per_subarray)+addr_vec[5] << std::endl;
        ctrl->enqueue(refresh_req);
      }
    }

    if (refresh_row_counter == refreshes_per_period)
    {
      refresh_row_counter = 0;
      period_counter++;
    }

    refresh_row_counter++;

    ticked_refresh = clk;
  }

}
  
} // namespace Ramulator


#endif