// Static device capability tables (see tools/gen_devtables.py).
#pragma once

#include <cstddef>
#include <cstring>

namespace vmopen {

struct DevTable {
    const char* name;
    const unsigned char* data;
    size_t size;
};

extern const DevTable kDevTables[];
const DevTable* FindDevTable(const char* name);

} // namespace vmopen
