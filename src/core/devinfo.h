// Device description: what the host knows about a connected instrument.
//
// It is assembled from two sources:
//  * the identification reply of the device (model name, hardware version,
//    memory size, unique id) - see ParseIdentify()
//  * a static capability table selected from the model name (devtables.cpp),
//    parsed by ParseCapabilities()
// Calibration replies sent by the device during the handshake then refine
// some of the values (Apply*Calibration()).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace vmopen {

enum ModuleMask : uint8_t {
    MOD_DDS = 0x01,
    MOD_OSC = 0x02,
    MOD_LOGIC = 0x08,
    MOD_IO = 0x10,
    MOD_METER = 0x20,
    MOD_POWER = 0x40,
};

// One programmable range of an analog input channel.
struct OscZoom {
    int16_t code = 0;     // gain code, value = code/zoom_div (negative: 1/(-code/zoom_div))
    int16_t dac = 0;      // offset DAC code used to centre the range
};

struct OscChannelDesc {
    std::vector<int32_t> inzoom;  // input attenuator codes
    std::vector<OscZoom> zoom;    // programmable gain stages
    std::vector<double> jiaozhun; // offset calibration [zoom][inzoom]
    std::vector<uint16_t> adc_adjust;      // ADC interleave adjust codes
    std::vector<double> adc_adjust_zoom;   // ADC interleave gain adjust

    double Jiaozhun(unsigned in, unsigned zm) const
    {
        if (in >= inzoom.size() || zm >= zoom.size())
            return 0;
        return jiaozhun[zm * inzoom.size() + in];
    }
};

struct OscDesc {
    uint8_t type = 0;
    std::vector<uint8_t> channel_ids;
    std::vector<uint32_t> sample_rates;
    uint8_t adc_bits = 8;
    uint8_t f117 = 0;
    double adc_min_v = -0.5;   // ADC full scale expressed in "zoom units"
    double adc_max_v = 0.5;
    uint16_t f128 = 0;
    uint8_t zoom_is_dac = 0;   // calibration overrides the DAC code instead of the gain code
    uint16_t zoom_div = 1000;
    std::vector<OscChannelDesc> ch;
    uint8_t adc_adjust_num = 0;
    uint32_t memory_kb = 0;    // per device memory (all channels)
    uint16_t f14a = 0;
    int32_t range_min_mv = 0;
    int32_t range_max_mv = 0;
    uint8_t f156 = 0;
    uint32_t trigger_styles = 0;    // TRIGGER_STYLE_* mask
    uint64_t trigger_sources = 0;   // 1<<TRIGGER_SOURCE_* mask
    uint8_t support_front_percent = 0;
    uint8_t support_trigger_level = 0;
    uint8_t f165 = 0;
    uint8_t support_force = 0;
    uint8_t support_holdoff = 0;
    uint8_t f16c = 0, f16d = 0;
    uint8_t support_acdc = 0;

    unsigned ChannelNum() const { return (unsigned)ch.size(); }
    double InzoomValue(unsigned c, unsigned i) const;
    double ZoomValue(unsigned c, unsigned i) const;
};

struct DdsBoxing {
    uint16_t style = 0;
    int32_t vol_min_mv = 0, vol_max_mv = 0;  // amplitude = max - min
    uint16_t f10 = 0;
    double min_freq = 0, max_freq = 0, freq_step = 0;
    int32_t bias_min_mv = 0, bias_max_mv = 0;
    uint16_t f44 = 0;
    uint8_t sweep = 0;
    uint32_t sweep_a = 0, sweep_b = 0, sweep_c = 0;
    uint8_t burst = 0;
    uint32_t f60 = 0;
};

struct DdsFreqZoom {
    uint32_t freq = 0;
    double zoom = 1.0;
};

struct DdsDesc {
    uint8_t type = 0;
    std::vector<uint8_t> channel_ids;
    uint32_t clock = 0;          // DDS clock (Hz)
    uint8_t depth_kb = 0;        // waveform buffer depth (K samples)
    uint8_t freq_bits = 0;       // phase accumulator width
    uint8_t dac_bits = 0;
    std::vector<int32_t> vol_min_mv, vol_max_mv;  // per channel DAC output range
    uint8_t zoom_num = 0;
    std::vector<DdsFreqZoom> zoom;   // [channel][zoom_num] amplitude/frequency correction
    std::vector<DdsBoxing> boxing;
    uint8_t trigger_sources = 0;
    uint8_t gate_enges = 0;
    uint8_t burst_large = 0;

    unsigned ChannelNum() const { return (unsigned)channel_ids.size(); }
    const DdsBoxing* Boxing(unsigned style) const;
};

struct LogicDesc {
    uint8_t type = 0;
    uint8_t channel_num = 0;
    std::vector<uint32_t> sample_rates;
    uint32_t memory = 0;
};

struct IoDac {
    uint8_t bits = 0;
    int32_t vol_min_mv = 0, vol_max_mv = 0;
};

struct IoDesc {
    uint8_t type = 0;
    uint8_t io_num = 0;
    uint8_t out_states = 0;
    std::vector<IoDac> dac;
};

struct PowerDesc {
    uint8_t type = 0;
    std::vector<uint32_t> cal;  // 3 values per channel
    unsigned channel_num = 0;
};

struct DevInfo {
    // identification
    uint8_t proto_ver = 0;
    std::string name;      // e.g. "MSO20BL(2.0)"
    uint8_t hw_ver = 0;
    uint8_t hw_sub = 0;
    uint8_t ram_index = 0;
    uint32_t only_id0 = 0;
    uint32_t only_id1 = 0;
    const char* table = nullptr;

    uint8_t f5c = 0;
    uint8_t modules = 0;   // ModuleMask

    OscDesc osc;
    DdsDesc dds;
    LogicDesc logic;
    IoDesc io;
    PowerDesc power;
    uint8_t meter_type = 0, meter_num = 0;

    bool BigEndian() const { return proto_ver > 9; }

    bool ParseIdentify(const uint8_t* p, size_t n);
    void ParseCapabilities(const uint8_t* blob, size_t n, bool logic, bool dds, bool meter, bool power);

    // handshake calibration replies (full reply including the 4 byte header)
    bool ApplyAdcAdjust(const uint8_t* p, size_t n);
    bool ApplyInputAdjust(const uint8_t* p, size_t n);
    void ApplyZoomCalibration(const uint8_t* buf, size_t n);
    bool ApplyDdsDacVol(const uint8_t* p, size_t n);
    bool ApplyDdsDacZoom(const uint8_t* p, size_t n);
    bool ApplyIoDac(const uint8_t* p, size_t n);
    bool ApplyPower(const uint8_t* p, size_t n);

    std::string Describe() const;

private:
    int ParseOsc(const uint8_t* p, size_t n);
    int ParseDds(const uint8_t* p, size_t n);
    int ParseLogic(const uint8_t* p, size_t n);
    int ParseIo(const uint8_t* p, size_t n);
    int ParsePower(const uint8_t* p, size_t n);
};

} // namespace vmopen
