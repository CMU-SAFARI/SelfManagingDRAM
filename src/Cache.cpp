#include "Cache.h"

#ifndef DEBUG_CACHE
#define debug(...)
#else
#define debug(...) do { \
          printf("\033[36m[DEBUG] %s ", __FUNCTION__); \
          printf(__VA_ARGS__); \
          printf("\033[0m\n"); \
      } while (0)
#endif

namespace ramulator
{

Cache::Cache(int size, int assoc, int block_size,
    int mshr_entry_num, Level level,
    std::shared_ptr<CacheSystem> cachesys):
    level(level), cachesys(cachesys), higher_cache(0),
    lower_cache(nullptr), size(size), assoc(assoc),
    block_size(block_size), mshr_entry_num(mshr_entry_num) {

  debug("level %d size %d assoc %d block_size %d\n",
      int(level), size, assoc, block_size);

  if (level == Level::L1) {
    level_string = "L1";
  } else if (level == Level::L2) {
    level_string = "L2";
  } else if (level == Level::L3) {
    level_string = "L3";
  }

  is_first_level = (level == cachesys->first_level);
  is_last_level = (level == cachesys->last_level);

  // Check size, block size and assoc are 2^N
  assert((size & (size - 1)) == 0);
  assert((block_size & (block_size - 1)) == 0);
  assert((assoc & (assoc - 1)) == 0);
  assert(size >= block_size);

  // Initialize cache configuration
  block_num = size / (block_size * assoc);
  index_mask = block_num - 1;
  index_offset = calc_log2(block_size);
  tag_offset = calc_log2(block_num) + index_offset;

  prefetcher = nullptr;

  debug("index_offset %d", index_offset);
  debug("index_mask 0x%x", index_mask);
  debug("tag_offset %d", tag_offset);

  // regStats
  cache_read_miss.name(level_string + string("_cache_read_miss"))
                 .desc("cache read miss count")
                 .precision(0)
                 ;

  cache_write_miss.name(level_string + string("_cache_write_miss"))
                  .desc("cache write miss count")
                  .precision(0)
                  ;
  cache_prefetch_miss.name(level_string + string("_cache_prefetch_miss"))
                  .desc("cache prefetch miss count")
                  .precision(0)
                  ;

  cache_prefetch_hit.name(level_string + string("_cache_prefetch_hit"))
                    .desc("prefetch requests that were already in the cache")
                    .precision(0)
                    ;

  cache_total_miss.name(level_string + string("_cache_total_miss"))
                  .desc("cache total miss count")
                  .precision(0)
                  ;

  cache_eviction.name(level_string + string("_cache_eviction"))
                .desc("number of evict from this level to lower level")
                .precision(0)
                ;

  cache_read_access.name(level_string + string("_cache_read_access"))
                  .desc("cache read access count")
                  .precision(0)
                  ;

  cache_write_access.name(level_string + string("_cache_write_access"))
                    .desc("cache write access count")
                    .precision(0)
                    ;

  cache_prefetch_access.name(level_string + string("_cache_prefetch_access"))
                       .desc("cache prefetch access count")
                       .precision(0)
                       ;

  cache_total_access.name(level_string + string("_cache_total_access"))
                    .desc("cache total access count")
                    .precision(0)
                    ;

  cache_mshr_hit.name(level_string + string("_cache_mshr_hit"))
                .desc("cache mshr hit count")
                .precision(0)
                ;
  cache_mshr_unavailable.name(level_string + string("_cache_mshr_unavailable"))
                         .desc("cache mshr not available count")
                         .precision(0)
                         ;
  cache_set_unavailable.name(level_string + string("_cache_set_unavailable"))
                         .desc("cache set not available")
                         .precision(0)
                         ;
}

bool Cache::send(Request req) {
  debug("level %d req.addr %lx req.type %d, index %d, tag %ld",
      int(level), req.addr, int(req.type), get_index(req.addr),
      get_tag(req.addr));

  cache_total_access++;
  if (req.type == Request::Type::WRITE) {
    cache_write_access++;
  } else if (req.type == Request::Type::READ) {
    cache_read_access++;
  } else {
    assert(req.type == Request::Type::PREFETCH);
    cache_prefetch_access++;
  }


  // debug
  //if(is_last_level){
  //    if(req.addr == 881276160)
  //        req.addr = req.addr;
    //printf("LLC: Received a %d request to addr: %ld\n", int(req.type), req.addr);
  //}
  // END - debug


  // If there isn't a set, create it.
  auto& lines = get_lines(req.addr);
  std::list<Line>::iterator line;

  if (is_hit(lines, req.addr, &line)) {
      if(req.type == Request::Type::PREFETCH) {
          cache_prefetch_hit++;
          return true;;
      }

    lines.push_back(Line(req.addr, get_tag(req.addr), false,
        line->dirty || (req.type == Request::Type::WRITE)));
    lines.erase(line);
    cachesys->hit_list.push_back(
        make_pair(cachesys->clk + latency[int(level)], req));

    debug("hit, update timestamp %ld", cachesys->clk);
    debug("hit finish time %ld",
        cachesys->clk + latency[int(level)]);

    if(prefetcher)
        prefetcher->hit(req.addr, cachesys->clk);

    return true;

  } else {
    debug("miss @level %d", int(level));
    cache_total_miss++;
    if (req.type == Request::Type::WRITE) {
      cache_write_miss++;
    } else if (req.type == Request::Type::READ) {
      cache_read_miss++;
    } else {
      assert(req.type == Request::Type::PREFETCH);
      cache_prefetch_miss++;
    }

    // The dirty bit will be set if this is a write request and @L1
    bool dirty = (req.type == Request::Type::WRITE);

    // Modify the type of the request to lower level
    if (req.type == Request::Type::WRITE) {
      req.type = Request::Type::READ;
    }

    // Look it up in MSHR entries
    assert((req.type == Request::Type::READ) || (req.type == Request::Type::PREFETCH));
    auto mshr = hit_mshr(req.addr);
    if (mshr != mshr_entries.end()) {
      debug("hit mshr");
      cache_mshr_hit++;
      mshr->second->dirty = dirty || mshr->second->dirty;
      // FIXME: Shall we train the prefetcher on MSHR hit???
      // if(prefetcher)
      //    prefetcher->miss(req.addr, cachesys->clk);
      
      // upgrade the previous prefetch request to demand request so the
      // processor will be informed on completion of the request
      if (prefetcher && (req.type == Request::Type::READ) && mshr->second->is_prefetch){
        bool is_upgraded = cachesys->upgrade_prefetch_req(align(req.addr));
        if(!is_upgraded)
            printf("Address of the request failed to upgrade: %ld\n", align(req.addr));
        assert(is_upgraded && "ERROR: Failed to upgrade a PREFETCH request to READ!");
        mshr->second->is_prefetch = false;
      }

      return true;
    }

    // All requests come to this stage will be READ, so they
    // should be recorded in MSHR entries.
    if (mshr_entries.size() == mshr_entry_num) {
      // When no MSHR entries available, the miss request
      // is stalling.
      cache_mshr_unavailable++;
      debug("no mshr entry available");
      return false;
    }

    // Check whether there is a line available
    if (all_sets_locked(lines)) {
      cache_set_unavailable++;
      return false;
    }

    auto newline = allocate_line(lines, req.addr);
    if (newline == lines.end()) {
      return false;
    }

    newline->dirty = dirty;

    // Add to MSHR entries
    mshr_entries.push_back(make_pair(req.addr, newline));

    // Send the request to next level;
    if (!is_last_level) {
      lower_cache->send(req);
    } else {
      cachesys->wait_list.push_back(
          make_pair(cachesys->clk + latency[int(level)], req));
    }

    if (prefetcher) {
        if (req.type == Request::Type::READ) {
            prefetcher->miss(req.addr, cachesys->clk);
        } else {
            assert(req.type == Request::Type::PREFETCH);
            newline->is_prefetch = true;
        }
    }

    return true;
  }
}

void Cache::evictline(long addr, bool dirty) {

  auto it = cache_lines.find(get_index(addr));
  assert(it != cache_lines.end()); // check inclusive cache
  auto& lines = it->second;
  auto line = find_if(lines.begin(), lines.end(),
      [addr, this](Line l){return (l.tag == get_tag(addr));});

  assert(line != lines.end());
  // Update LRU queue. The dirty bit will be set if the dirty
  // bit inherited from higher level(s) is set.
  lines.push_back(Line(addr, get_tag(addr), false,
      dirty || line->dirty));
  lines.erase(line);
}

std::pair<long, bool> Cache::invalidate(long addr) {
  long delay = latency_each[int(level)];
  bool dirty = false;

  auto& lines = get_lines(addr);
  if (lines.size() == 0) {
    // The line of this address doesn't exist.
    return make_pair(0, false);
  }
  auto line = find_if(lines.begin(), lines.end(),
      [addr, this](Line l){return (l.tag == get_tag(addr));});

  // If the line is in this level cache, then erase it from
  // the buffer.
  if (line != lines.end()) {
    assert(!line->lock);
    debug("invalidate %lx @ level %d", addr, int(level));
    lines.erase(line);
  } else {
    // If it's not in current level, then no need to go up.
    return make_pair(delay, false);
  }

  if (higher_cache.size()) {
    long max_delay = delay;
    for (auto hc : higher_cache) {
      auto result = hc->invalidate(addr);
      if (result.second) {
        max_delay = max(max_delay, delay + result.first * 2);
      } else {
        max_delay = max(max_delay, delay + result.first);
      }
      dirty = dirty || line->dirty || result.second;
    }
    delay = max_delay;
  } else {
    dirty = line->dirty;
  }
  return make_pair(delay, dirty);
}


void Cache::evict(std::list<Line>* lines,
    std::list<Line>::iterator victim) {
  debug("level %d miss evict victim %lx", int(level), victim->addr);
  cache_eviction++;

  long addr = victim->addr;
  long invalidate_time = 0;
  bool dirty = victim->dirty;

  // First invalidate the victim line in higher level.
  if (higher_cache.size()) {
    for (auto hc : higher_cache) {
      auto result = hc->invalidate(addr);
      invalidate_time = max(invalidate_time,
          result.first + (result.second ? latency_each[int(level)] : 0));
      dirty = dirty || result.second || victim->dirty;
    }
  }

  debug("invalidate delay: %ld, dirty: %s", invalidate_time,
      dirty ? "true" : "false");

  if (!is_last_level) {
    // not LLC eviction
    assert(lower_cache != nullptr);
    lower_cache->evictline(addr, dirty);
  } else {
    // LLC eviction
    if (dirty) {
      Request write_req(addr, Request::Type::WRITE);
      cachesys->wait_list.push_back(make_pair(
          cachesys->clk + invalidate_time + latency[int(level)],
          write_req));

      debug("inject one write request to memory system "
          "addr %lx, invalidate time %ld, issue time %ld",
          write_req.addr, invalidate_time,
          cachesys->clk + invalidate_time + latency[int(level)]);
    }
  }

  lines->erase(victim);
}

std::list<Cache::Line>::iterator Cache::allocate_line(
    std::list<Line>& lines, long addr) {
  // See if an eviction is needed
  if (need_eviction(lines, addr)) {
    // Get victim.
    // The first one might still be locked due to reorder in MC
    auto victim = find_if(lines.begin(), lines.end(),
        [this](Line line) {
          bool check = !line.lock;
          if (!is_first_level) {
            for (auto hc : higher_cache) {
              if(!check) {
                return check;
              }
              check = check && hc->check_unlock(line.addr);
            }
          }
          return check;
        });
    if (victim == lines.end()) {
      return victim;  // doesn't exist a line that's already unlocked in each level
    }
    assert(victim != lines.end());
    evict(&lines, victim);
  }

  // Allocate newline, with lock bit on and dirty bit off
  lines.push_back(Line(addr, get_tag(addr)));
  auto last_element = lines.end();
  --last_element;
  return last_element;
}

bool Cache::is_hit(std::list<Line>& lines, long addr,
    std::list<Line>::iterator* pos_ptr) {
  auto pos = find_if(lines.begin(), lines.end(),
      [addr, this](Line l){return (l.tag == get_tag(addr));});
  *pos_ptr = pos;
  if (pos == lines.end()) {
    return false;
  }
  return !pos->lock;
}

void Cache::concatlower(Cache* lower) {
  lower_cache = lower;
  assert(lower != nullptr);
  lower->higher_cache.push_back(this);
};

bool Cache::need_eviction(const std::list<Line>& lines, long addr) {
  if (find_if(lines.begin(), lines.end(),
      [addr, this](Line l){
        return (get_tag(addr) == l.tag);})
      != lines.end()) {
    // Due to MSHR, the program can't reach here. Just for checking
    assert(false);
  } else {
    if (lines.size() < assoc) {
      return false;
    } else {
      return true;
    }
  }
}

void Cache::callback(Request& req) {
  debug("level %d", int(level));

  //debug
  //if(is_last_level)
      //printf("LLC: callback req type %d, addr %ld\n", req.type, req.addr);

  auto it = find_if(mshr_entries.begin(), mshr_entries.end(),
      [&req, this](std::pair<long, std::list<Line>::iterator> mshr_entry) {
        return (align(mshr_entry.first) == align(req.addr));
      });

  if (it != mshr_entries.end()) {
    it->second->lock = false;
    mshr_entries.erase(it);
  }

  if (req.type != Request::Type::PREFETCH) // to prefetch only to LLC
    if (higher_cache.size()) {
        for (auto hc : higher_cache) {
        hc->callback(req);
    }
  }
}

void CacheSystem::tick() {
  debug("clk %ld", clk);

  ++clk;

  // Sends ready waiting request to memory
  auto it = wait_list.begin();
  while (it != wait_list.end() && clk >= it->first) {
    if (!send_memory(it->second)) {
      ++it;
    } else {

      debug("complete req: addr %lx", (it->second).addr);

      it = wait_list.erase(it);
    }
  }

  // hit request callback
  it = hit_list.begin();
  while (it != hit_list.end()) {
    if (clk >= it->first) {
      it->second.callback(it->second);

      debug("finish hit: addr %lx", (it->second).addr);

      it = hit_list.erase(it);
    } else {
      ++it;
    }
  }
}

bool CacheSystem::upgrade_prefetch_req(long addr) {

    //printf("=== Wait list req\n");
    //for(auto it = wait_list.begin(); it != wait_list.end(); it++) {
    //    printf("%ld\n", (it->second).addr);
    //}
    //printf("---\n\n");

    if(wait_list.size() != 0){

        auto pref_req = find_if(wait_list.begin(), wait_list.end(), 
                                                [addr](pair<long, Request>& preq) {
                        return (addr == preq.second.addr) && preq.second.type == Request::Type::PREFETCH;});

        if (pref_req != wait_list.end()) {
            (pref_req->second).type = Request::Type::READ;
            (pref_req->second).callback = (pref_req->second).proc_callback; // FIXME: proc_callback is an ugly workaround
            return true;
        }
     }

    return upgrade_prefetch_req_in_mem(addr);
}

} // namespace ramulator
