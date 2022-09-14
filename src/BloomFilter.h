#ifndef __BLOOM_FILTER_H
#define __BLOOM_FILTER_H

#include <cstdint>
#include <cassert>
#include <vector>
#include <random>
#include <algorithm>
#include <set>
#include <memory>

#include "Statistics.h"

using namespace ramulator;

// A simple Bloom Filter implementation that uses area-efficient H3 class hash functions
class BloomFilter {

public:
    // size indicates the number of BloomFilter entries
    // size must be power of two
    BloomFilter(const uint32_t size, const uint32_t num_hash_funcs, const uint32_t rank_id, const uint32_t chip_id, const uint32_t bf_id = 0) {

        // check if size is power of two
        assert((size & (size - 1)) == 0 && "ERROR: Bloom filter size must be a power-of-two number!");
        assert(num_hash_funcs > 0 && "ERROR: A Bloom Filter requires at least one hash function!");

        // randomly initialize the Q matrix
        gen = std::mt19937(bf_id); //Standard mersenne_twister_engine seeded with bf_id
        uint32_t max_q_val = size - 1;
        distrib = std::uniform_int_distribution<uint32_t>(0, max_q_val);

        for (uint32_t i = 0; i < num_hash_funcs; i++){
            Q.push_back(std::vector<uint32_t>(QLENGTH));
        }
        reset_hashes();

        init_bf_storage(size, num_hash_funcs);

        // initializing the statistics
        bf_positives
            .name("bf_positives_r" + to_string(rank_id) + "_c" + to_string(chip_id))
            .desc("The number of times the bloom filter returns true when queried.")
            .precision(0)
            ;

        bf_negatives
            .name("bf_negatives_r" + to_string(rank_id) + "_c" + to_string(chip_id))
            .desc("The number of times the bloom filter returns false when queried.")
            .precision(0)
            ;

        bf_false_positives
            .name("bf_false_positives_r" + to_string(rank_id) + "_c" + to_string(chip_id))
            .desc("The number of times the bloom filter returns true incorrectly when queried.")
            .precision(0)
            ;
        
    }

    virtual ~BloomFilter(){}

    virtual void init_bf_storage(const uint32_t size, const uint32_t num_hash_funcs) {
        // initialize the entries vector
        entries.resize(size);
        clear();
    }

    virtual void clear() {
        std::fill(entries.begin(), entries.end(), false);
        shadow.clear();
    }

    void reset_hashes() {
        for (uint32_t i = 0; i < Q.size(); i++){
            for (uint32_t j = 0; j < QLENGTH; j++)
                Q[i][j] = distrib(gen);
        }
    }

    virtual void insert(const uint32_t key) {

        // std::cout << "[BloomFilter] Inserting element: " << key << std::endl; // DEBUG
        for (uint32_t i = 0; i < Q.size(); i++) {
            uint32_t entry_addr = hash(key, i);
            // printf("[BloomFilter] Hash function %u entry address: %u\n", i, entry_addr); // DEBUG
            entries[entry_addr] = true;
        }

        shadow.insert(key);
    }

    virtual bool test(const uint32_t key) {

        for (uint32_t i = 0; i < Q.size(); i++) {
            uint32_t entry_addr = hash(key, i);

            if (!entries[entry_addr]) {
                bf_negatives++;
                return false;
            }
        }

        bf_positives++;

        if (test_shadow(key) != true)
            bf_false_positives++;

        return true;
    }

    bool test_shadow(const uint32_t key) const {
        return shadow.count(key) == 1;
    }

    uint32_t num_zero_entries() const {
        uint32_t num_zeros = 0;

        for (const bool entry : entries)
            if (!entry)
                num_zeros++;
            
        return num_zeros;
    }

private:

    // bloom filter entries
    std::vector<bool> entries;

    std::mt19937 gen;
    std::uniform_int_distribution<uint32_t> distrib;

protected:
    ScalarStat bf_positives;
    ScalarStat bf_negatives;
    ScalarStat bf_false_positives;

    // define the Q matrices
    const uint32_t QLENGTH = 32;
    std::vector<std::vector<uint32_t>> Q;

    // shadow set to track false positives
    std::set<uint32_t> shadow;

    uint32_t hash(uint32_t key, const uint32_t hash_func_ind) const {
        
        uint32_t entry_addr = 0;

        const auto& q = Q[hash_func_ind];
        for (const auto& qval : q) {
            if (key & 0x1)
                entry_addr ^= qval;
            
            key >>= 1;
        }

        return entry_addr;
    }

};


class CountingBloomFilter : public BloomFilter {

public:

    CountingBloomFilter(const uint32_t size, const uint32_t num_hash_funcs, const uint32_t saturation_point, const uint32_t rank_id, const uint32_t chip_id, const uint32_t bf_id = 0) :
        BloomFilter(size, num_hash_funcs, rank_id, chip_id, bf_id) {

        max_counter_value = saturation_point;

        init_bf_storage(size, num_hash_funcs);
    }

    
    void init_bf_storage(const uint32_t size, const uint32_t num_hash_funcs) {
        // initialize the entries vector
        entries.resize(size);
        clear();
    }

    void clear() {
        std::fill(entries.begin(), entries.end(), 0);
        this->shadow.clear();
    }

    void insert(const uint32_t key) {

        // std::cout << "[BloomFilter] Inserting element: " << key << std::endl; // DEBUG
        for (uint32_t i = 0; i < Q.size(); i++) {
            uint32_t entry_addr = hash(key, i);
            // printf("[BloomFilter] Hash function %u entry address: %u\n", i, entry_addr); // DEBUG
            entries[entry_addr] = std::min(max_counter_value, entries[entry_addr] + 1);
        }

        shadow.insert(key);
    }

    bool test(const uint32_t key) {

        for (uint32_t i = 0; i < Q.size(); i++) {
            uint32_t entry_addr = hash(key, i);

            if (entries[entry_addr] != max_counter_value) {
                bf_negatives++;
                return false;
            }
        }

        bf_positives++;

        if (test_shadow(key) != true)
            bf_false_positives++;

        return true;
    }


protected:

private:
    std::vector<uint32_t> entries;
    uint32_t max_counter_value = 0;

};


class DualCountingBloomFilter : public BloomFilter {
    public:

        DualCountingBloomFilter(const uint32_t size, const uint32_t num_hash_funcs, const uint32_t saturation_point, const bool space_efficient_scbf,
            const uint32_t rank_id, const uint32_t chip_id) : BloomFilter(size, num_hash_funcs, rank_id, chip_id) {

            active_bf = std::unique_ptr<CountingBloomFilter>(new CountingBloomFilter(size, num_hash_funcs, saturation_point, rank_id, chip_id, 0));
            passive_bf = std::unique_ptr<CountingBloomFilter>(new CountingBloomFilter(size, num_hash_funcs, saturation_point, rank_id, chip_id, 1));

            _space_efficient_scbf = space_efficient_scbf;
        }

        void insert(const uint32_t key) {
            if (_space_efficient_scbf){
                passive_bf->insert(key);
            }
            else {
                active_bf->insert(key);
                passive_bf->insert(key);
            }
        }

        bool test(const uint32_t key) {
            if (_space_efficient_scbf)
                return active_bf->test(key) + passive_bf->test(key);
            else
                return active_bf->test(key);
        }

        void swap_filters(){
            std::swap(active_bf, passive_bf);

            passive_bf->clear();
            passive_bf->reset_hashes();
        }

    private:
        std::unique_ptr<CountingBloomFilter> active_bf, passive_bf;
        bool _space_efficient_scbf = false;
};

#endif /* __BLOOM_FILTER_H */