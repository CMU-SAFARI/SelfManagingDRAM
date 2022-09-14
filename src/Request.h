#ifndef __REQUEST_H
#define __REQUEST_H

#include <vector>
#include <functional>

using namespace std;

namespace ramulator
{

class Request
{
public:
    bool is_first_command;
    long addr;
    // long addr_row;
    vector<int> addr_vec;
    // specify which core this request sent from, for virtual address translation
    int coreid;
    bool marked; // a flag that is used by some schedulers, e.g., PARBS
                 // TODO: find a better name for this flag, or implement
                 // the functionality in a better way

    int64_t req_unique_id = -1; // a unique ID for tracking individual requests for debugging purposes
    bool partially_nacked = false; // a flag to indicate if a request has been partially nacked before

    enum class Type
    {
        READ,
        WRITE,
        REFRESH,
        POWERDOWN,
        SELFREFRESH,
        PARA_REFRESH,
        EXTENSION,
        PREFETCH,
        REF_STATUS_QUERY,
        ACT_NACK,
        ACT_PARTIAL_NACK,
        MAX
    } type;

    long arrive = -1;
    long depart;
    function<void(Request&)> callback; // call back with more info
    function<void(Request&)> proc_callback; // FIXME: ugly workaround

    Request(long addr, Type type, int coreid = 0)
        : is_first_command(true), addr(addr), coreid(coreid), marked(false), type(type),
      callback([](Request& req){}) {}

    Request(long addr, Type type, function<void(Request&)> callback, int coreid = 0)
        : is_first_command(true), addr(addr), coreid(coreid), marked(false), type(type), callback(callback) {}

    Request(const vector<int>& addr_vec, Type type, function<void(Request&)> callback, int coreid = 0)
        : is_first_command(true), addr_vec(addr_vec), coreid(coreid), marked(false), type(type), callback(callback) {}

    Request()
        : is_first_command(true), coreid(0), marked(false) {}
};

} /*namespace ramulator*/

#endif /*__REQUEST_H*/

