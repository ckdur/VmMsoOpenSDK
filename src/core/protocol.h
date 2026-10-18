// Wire protocol of the Vimu MSO family.
//
// Every request is sent on the control OUT endpoint (0x01):
//
//     [module][channel][cmd][value u32][payload ...]      (payload absent: one 0x00 byte)
//
// Multi byte fields are big endian when the device protocol version is > 9
// (all current devices). Depending on the request the host then either
//   * reads a reply on the control IN endpoint (0x81), usually
//     [module][channel][cmd][status][...], status 1 = ok,
//   * reads bulk data on the data IN endpoint (0x82), or
//   * writes bulk data on the data OUT endpoint (0x06) and then reads a reply.
//
// Modules: 0x00 system, 0x01 DDS, 0x02 oscilloscope, 0x10 IO.
#pragma once

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

namespace vmopen {

enum class Ask : uint8_t {
    Reply = 1,    // read one control reply
    DataIn = 2,   // read data_len bytes from the data IN endpoint
    DataOut = 3,  // write data_len bytes to the data OUT endpoint, then read a reply
};

enum Module : uint8_t {
    MODULE_SYS = 0x00,
    MODULE_DDS = 0x01,
    MODULE_OSC = 0x02,
    MODULE_IO = 0x10,
};

class Payload {
public:
    explicit Payload(bool be = true) : be_(be) {}
    Payload& u8(uint8_t v)
    {
        b_.push_back(v);
        return *this;
    }
    Payload& u16(uint16_t v) { return put(v, 2); }
    Payload& u32(uint32_t v) { return put(v, 4); }
    Payload& u64(uint64_t v) { return put(v, 8); }
    const std::vector<uint8_t>& bytes() const { return b_; }

private:
    Payload& put(uint64_t v, int n)
    {
        for (int i = 0; i < n; i++) {
            int sh = be_ ? (n - 1 - i) * 8 : i * 8;
            b_.push_back((uint8_t)(v >> sh));
        }
        return *this;
    }
    bool be_;
    std::vector<uint8_t> b_;
};

struct Command {
    std::vector<uint8_t> pkt;
    Ask ask = Ask::Reply;
    unsigned pre_delay_ms = 0;
    unsigned post_delay_ms = 0;
    uint8_t* data = nullptr;     // DataIn destination / DataOut source
    size_t data_len = 0;
    std::shared_ptr<std::vector<uint8_t>> keep;  // owns the DataOut buffer

    uint8_t module() const { return pkt.empty() ? 0xff : pkt[0]; }
};

inline Command MakeCommand(bool be, uint8_t mod, uint8_t ch, uint8_t cmd, uint32_t value,
                           const std::vector<uint8_t>* payload = nullptr)
{
    Command c;
    c.pkt.reserve(8 + (payload ? payload->size() : 0));
    c.pkt.push_back(mod);
    c.pkt.push_back(ch);
    c.pkt.push_back(cmd);
    for (int i = 0; i < 4; i++) {
        int sh = be ? (3 - i) * 8 : i * 8;
        c.pkt.push_back((uint8_t)(value >> sh));
    }
    if (payload && !payload->empty())
        c.pkt.insert(c.pkt.end(), payload->begin(), payload->end());
    else
        c.pkt.push_back(0);
    return c;
}

inline uint32_t RdBe32(const uint8_t* p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
inline uint16_t RdBe16(const uint8_t* p) { return (uint16_t)(p[0] << 8 | p[1]); }

} // namespace vmopen
