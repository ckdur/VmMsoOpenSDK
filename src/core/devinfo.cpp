#include "devinfo.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "devtables.h"
#include "log.h"

namespace vmopen {

namespace {

// All multi byte values in descriptors and replies are big endian.
inline uint16_t be16(const uint8_t* p) { return (uint16_t)(p[0] << 8 | p[1]); }
inline uint32_t be32(const uint8_t* p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
inline uint64_t be64(const uint8_t* p) { return (uint64_t)be32(p) << 32 | be32(p + 4); }

// Memory size table indexed by the identification reply (bits).
const uint64_t kRamSize[8] = {
    1ull << 24, 1ull << 25, 1ull << 26, 1ull << 27, 1ull << 28, 1ull << 29, 1ull << 30, 1ull << 31,
};

} // namespace

double OscDesc::InzoomValue(unsigned c, unsigned i) const
{
    int code = zoom_div;
    if (c < ch.size() && i < ch[c].inzoom.size())
        code = ch[c].inzoom[i];
    if (code >= 0)
        return (double)code / (double)(int)zoom_div;
    return 1.0 / (-(double)code / (double)(int)zoom_div);
}

double OscDesc::ZoomValue(unsigned c, unsigned i) const
{
    int16_t code = (int16_t)zoom_div;
    if (c < ch.size() && i < ch[c].zoom.size())
        code = ch[c].zoom[i].code;
    if (code >= 0)
        return (double)(int)code / (double)(int)zoom_div;
    return 1.0 / (-(double)(int)code / (double)(int)zoom_div);
}

const DdsBoxing* DdsDesc::Boxing(unsigned style) const
{
    for (const DdsBoxing& b : boxing)
        if (b.style == style)
            return &b;
    return nullptr;
}

bool DevInfo::ParseIdentify(const uint8_t* p, size_t n)
{
    if (n < 0x1c || p[0] != 0 || p[1] != 0 || p[2] != 0xfd || p[3] != 1)
        return false;
    proto_ver = p[4];
    char nm[17] = {0};
    std::memcpy(nm, p + 5, 16);
    name = nm;
    hw_ver = p[0x15];
    hw_sub = p[0x16];
    ram_index = p[0x17];
    only_id0 = be32(p + 0x18);

    // Select the capability table the same way the vendor library does.
    auto starts = [&](const char* s) { return std::strncmp(nm, s, std::strlen(s)) == 0; };
    auto at = [&](int i) { return nm[i]; };
    bool logic = true, dds = false, meter = false, pwr = false;
    const char* t = nullptr;
    bool fix14 = false;
    if (starts("MSO2") && at(4) == '0' && at(5) == 'L' && at(6) == 'L') {
        t = "mso20ll", logic = true, dds = false;
    } else if (starts("MSO1") && at(4) == '0') {
        dds = at(5) == 'B';
        logic = at(6) == 'L';
        t = hw_ver == 1 ? "mso10a" : "mso10a_v2";
    } else if (starts("MSO2") && at(4) == '0') {
        dds = at(5) == 'B';
        t = hw_ver == 1 ? "mso20a" : hw_ver == 2 ? "mso20a_v2" : hw_ver == 3 ? "mso20a_v3" : "mso20a_v4";
    } else if (starts("MSO21AL PLUS") || starts("MSO21BL PLUS")) {
        dds = starts("MSO21B");
        t = "mso21Plus_v1";
    } else if (starts("MSO2") && at(4) == '1') {
        fix14 = starts("MSO21AL-14") || starts("MSO21BL-14");
        dds = starts("MSO21B");
        t = (uint8_t)(hw_ver - 1) < 2 ? "mso21a" : "mso21a_v3";
    } else if (starts("MSO26")) {
        dds = starts("MSO26B");
        t = "mso26";
    } else if (starts("MSO41")) {
        if (hw_ver == 1 && hw_sub == 1) {
            dds = starts("MSO41B");
            t = "mso41";
        } else {
            dds = (uint8_t)(at(5) - 'B') < 2;
            meter = at(5) == 'C';
            t = hw_ver == 1 ? "mso41_12" : "mso41_20";
        }
    } else if (starts("MDDS10")) {
        t = "mdds10", logic = false, dds = true;
    } else if (starts("FG4100") || starts("FG410B")) {
        t = "fg4100", dds = true;
    } else if (starts("VB1000")) {
        t = hw_ver == 1 ? "vb1000" : "vb1000v2", dds = meter = pwr = true;
    } else if (starts("Iolab2000")) {
        t = hw_ver == 1 ? "Iolab2000" : "Iolab2000v2", dds = meter = true;
    } else if (starts("Iolab100")) {
        t = "Iolab100", dds = meter = pwr = true;
    } else if (starts("CARD1.0")) {
        t = "card10", logic = false;
    } else if (starts("HDWR20")) {
        t = hw_ver == 1 ? "mso10a" : "mso10a_v2", logic = false, dds = true;
    } else if (starts("HDWR30")) {
        t = "hdwr30", logic = false, dds = true;
    } else if (starts("KN2100")) {
        t = "Kn2100", logic = false;
    } else if (starts("KN8INCH")) {
        t = "kn8inch", logic = false;
    } else if (starts("KD100")) {
        t = "kd100", logic = false;
    } else if (starts("KD250")) {
        t = "kd250", logic = false;
    }
    if (!t) {
        LOG("unknown device model '%s'\n", nm);
        return true;
    }
    const DevTable* tb = FindDevTable(t);
    table = t;
    if (tb)
        ParseCapabilities(tb->data, tb->size, logic, dds, meter, pwr);
    if (fix14)
        osc.adc_bits = 14;
    return true;
}

void DevInfo::ParseCapabilities(const uint8_t* b, size_t n, bool logic_en, bool dds_en, bool meter_en, bool power_en)
{
    f5c = b[0];
    modules = b[1];
    int count = 0;
    for (int i = 0; i < 7; i++)
        count += (modules >> i) & 1;
    size_t pos = 2;
    for (int m = 0; m < count && pos < n; m++) {
        uint8_t type = b[pos];
        int used = 0;
        switch (type) {
        case MOD_DDS:
            if (modules & MOD_DDS) {
                used = ParseDds(b + pos, n - pos);
                if (!dds_en) {
                    modules &= ~MOD_DDS;
                    dds.channel_ids.clear();
                }
            }
            break;
        case MOD_OSC:
            if (modules & MOD_OSC)
                used = ParseOsc(b + pos, n - pos);
            break;
        case MOD_LOGIC:
            if (modules & MOD_LOGIC) {
                used = ParseLogic(b + pos, n - pos);
                if (!logic_en) {
                    modules &= ~MOD_LOGIC;
                    logic.channel_num = 0;
                }
            }
            break;
        case MOD_IO:
            if (modules & MOD_IO)
                used = ParseIo(b + pos, n - pos);
            break;
        case MOD_METER:
            if (modules & MOD_METER) {
                meter_type = b[pos];
                meter_num = b[pos + 1];
                used = 2;
                if (!meter_en) {
                    modules &= ~MOD_METER;
                    meter_num = 0;
                }
            }
            break;
        case MOD_POWER:
            if (modules & MOD_POWER) {
                used = ParsePower(b + pos, n - pos);
                if (!power_en) {
                    modules &= ~MOD_POWER;
                    power.channel_num = 0;
                }
            }
            break;
        default:
            break;
        }
        if (used < 0)
            return;
        pos += used;
    }
}

int DevInfo::ParseOsc(const uint8_t* p, size_t n)
{
    if (n < 0x45)
        return -1;
    osc.type = p[0];
    unsigned nch = p[1];
    osc.channel_ids.assign(p + 2, p + 2 + nch);
    unsigned u = nch;
    unsigned nsr = p[u + 2];
    osc.sample_rates.resize(nsr);
    for (unsigned i = 0; i < nsr; i++)
        osc.sample_rates[i] = be32(p + u + 3 + i * 4);
    u = u + 3 + nsr * 4;
    osc.adc_bits = p[u];
    osc.f117 = p[u + 1];
    osc.adc_min_v = (double)(int32_t)be32(p + u + 2) / 1000.0;
    osc.adc_max_v = (double)(int32_t)be32(p + u + 6) / 1000.0;
    osc.f128 = be16(p + u + 10);
    osc.zoom_is_dac = p[u + 12];
    osc.zoom_div = be16(p + u + 13);
    unsigned i = u + 15;
    osc.ch.assign(nch, OscChannelDesc());
    for (unsigned c = 0; c < nch; c++) {
        OscChannelDesc& cd = osc.ch[c];
        unsigned nin = p[i++];
        cd.inzoom.resize(nin);
        for (unsigned k = 0; k < nin; k++, i += 2)
            cd.inzoom[k] = (int16_t)be16(p + i);
        unsigned nz = p[i++];
        cd.zoom.resize(nz);
        cd.jiaozhun.assign(nz * nin, 0.0);
        for (unsigned z = 0; z < nz; z++) {
            cd.zoom[z].code = (int16_t)be16(p + i);
            cd.zoom[z].dac = (int16_t)be16(p + i + 2);
            i += 4;
            for (unsigned k = 0; k < nin; k++, i += 2)
                cd.jiaozhun[z * nin + k] = (double)be16(p + i);
        }
    }
    unsigned k = p[i++];
    osc.adc_adjust_num = (uint8_t)k;
    if (k) {
        for (unsigned c = 0; c < nch; c++) {
            osc.ch[c].adc_adjust.resize(k);
            for (unsigned j = 0; j < k; j++)
                osc.ch[c].adc_adjust[j] = p[i++];
        }
        for (unsigned c = 0; c < nch; c++) {
            osc.ch[c].adc_adjust_zoom.resize(k);
            for (unsigned j = 0; j < k; j++, i += 4)
                osc.ch[c].adc_adjust_zoom[j] = (double)be32(p + i) / 1000.0;
        }
    }
    osc.memory_kb = (uint32_t)(kRamSize[ram_index & 7] >> 13);
    osc.f14a = (uint16_t)(p[i + 4] * 0x100 + p[i + 5]);
    osc.range_min_mv = (int32_t)be32(p + i + 6);
    osc.range_max_mv = (int32_t)be32(p + i + 10);
    osc.f156 = p[i + 14];
    osc.trigger_styles = be32(p + i + 15);
    osc.trigger_sources = be64(p + i + 19);
    osc.support_front_percent = p[i + 27];
    osc.support_trigger_level = p[i + 28];
    osc.f165 = p[i + 29];
    osc.support_force = p[i + 30];
    osc.support_holdoff = p[i + 31];
    osc.f16c = p[i + 32];
    osc.f16d = p[i + 33];
    osc.support_acdc = p[i + 34];
    return (int)(i + 35);
}

int DevInfo::ParseDds(const uint8_t* p, size_t n)
{
    if (n < 0x27)
        return -1;
    dds.type = p[0];
    unsigned nch = p[1];
    dds.channel_ids.assign(p + 2, p + 2 + nch);
    unsigned i = 2 + nch;
    dds.clock = be32(p + i);
    dds.depth_kb = p[i + 4];
    dds.freq_bits = p[i + 5];
    dds.dac_bits = p[i + 6];
    i += 7;
    dds.vol_min_mv.resize(nch);
    dds.vol_max_mv.resize(nch);
    for (unsigned c = 0; c < nch; c++, i += 8) {
        dds.vol_min_mv[c] = (int32_t)be32(p + i);
        dds.vol_max_mv[c] = (int32_t)be32(p + i + 4);
    }
    dds.zoom_num = p[i++];
    dds.zoom.resize(nch * dds.zoom_num);
    for (unsigned c = 0; c < nch; c++)
        for (unsigned z = 0; z < dds.zoom_num; z++, i += 6) {
            dds.zoom[c * dds.zoom_num + z].freq = be32(p + i);
            dds.zoom[c * dds.zoom_num + z].zoom = (double)be16(p + i + 4) / 1000.0;
        }
    unsigned nbox = p[i++];
    dds.boxing.assign(nbox, DdsBoxing());
    unsigned filled = 0;
    while (filled < nbox) {
        uint16_t mask = be16(p + i);
        uint16_t f10 = be16(p + i + 2);
        uint32_t vmin = 0, vmax = 0;
        unsigned j = i + 4;
        if (f10) {
            vmin = be32(p + i + 4);
            vmax = be32(p + i + 8);
            j = i + 12;
        }
        double step = (double)be16(p + j) * 0.001;
        double fmin = 0, fmax = 0;
        unsigned q = j + 2;
        if (step != 0.0) {
            uint32_t a = be32(p + q);
            fmin = a ? (double)a : step;
            fmax = (double)be32(p + q + 4);
            q = j + 10;
        }
        uint16_t f44 = be16(p + q);
        uint32_t bmin = 0, bmax = 0, f60;
        unsigned r = q + 2;
        if (f44) {
            bmin = be32(p + r);
            bmax = be32(p + r + 4);
            r = q + 10;
        }
        f60 = be32(p + r);
        i = r + 4;
        for (unsigned bit = 1; bit <= 0x100 && filled < nbox; bit <<= 1) {
            if (!(mask & bit))
                continue;
            DdsBoxing& bx = dds.boxing[filled++];
            bx.style = (uint16_t)bit;
            bx.vol_min_mv = (int32_t)vmin;
            bx.vol_max_mv = (int32_t)vmax;
            bx.f10 = f10;
            bx.bias_min_mv = (int32_t)bmin;
            bx.bias_max_mv = (int32_t)bmax;
            bx.f44 = f44;
            bx.f60 = f60;
            bx.min_freq = fmin;
            bx.max_freq = fmax;
            bx.freq_step = step;
        }
        if (!mask)
            break;
    }
    uint16_t smask = be16(p + i);
    unsigned k = i + 2;
    if (smask) {
        uint32_t a = be32(p + i + 2), b = be32(p + i + 6), c = be32(p + i + 10);
        k = i + 14;
        for (DdsBoxing& bx : dds.boxing)
            if ((bx.style & smask) == bx.style) {
                bx.sweep = 1;
                bx.sweep_c = a;
                bx.sweep_a = b;
                bx.sweep_b = c;
            }
    }
    uint16_t bmask = be16(p + k);
    if (bmask)
        for (DdsBoxing& bx : dds.boxing)
            if ((bmask & bx.style) == bx.style)
                bx.burst = 1;
    dds.trigger_sources = p[k + 2];
    dds.gate_enges = p[k + 3];
    return (int)(k + 4);
}

int DevInfo::ParseLogic(const uint8_t* p, size_t n)
{
    if (n < 4)
        return -1;
    logic.type = p[0];
    logic.channel_num = p[1];
    unsigned ns = p[2];
    unsigned off, used;
    if (ns == 0) {
        logic.sample_rates = osc.sample_rates;
        off = 3;
        used = 7;
    } else {
        logic.sample_rates.resize(ns);
        for (unsigned i = 0; i < ns; i++)
            logic.sample_rates[i] = be32(p + 3 + i * 4);
        off = ns * 4 + 3;
        used = ns * 4 + 7;
    }
    logic.memory = be32(p + off);
    if (!logic.memory)
        logic.memory = osc.memory_kb;
    return (int)used;
}

int DevInfo::ParseIo(const uint8_t* p, size_t n)
{
    if (n < 2)
        return -1;
    io.type = p[0];
    io.io_num = p[1];
    unsigned i = 2;
    if (io.io_num)
        io.out_states = p[i++];
    unsigned nd = p[i++];
    io.dac.resize(nd);
    for (unsigned d = 0; d < nd; d++, i += 9) {
        io.dac[d].bits = p[i];
        io.dac[d].vol_min_mv = (int32_t)be32(p + i + 1);
        io.dac[d].vol_max_mv = (int32_t)be32(p + i + 5);
    }
    return (int)i;
}

int DevInfo::ParsePower(const uint8_t* p, size_t n)
{
    if (n < 2)
        return -1;
    power.type = p[0];
    power.channel_num = p[1];
    power.cal.resize(power.channel_num * 3);
    for (unsigned c = 0; c < power.channel_num * 3; c++)
        power.cal[c] = be32(p + 2 + c * 4);
    return 2 + 12 * (int)power.channel_num;
}

bool DevInfo::ApplyAdcAdjust(const uint8_t* p, size_t n)
{
    if (n < 4 || p[0] || p[1] || p[2] != 0xfc)
        return false;
    if (p[3] == 0x10)
        return true;
    if (p[3] != 1)
        return false;
    unsigned k = osc.adc_adjust_num, nch = osc.ChannelNum();
    if (!k)
        return true;
    size_t left = n - 4, i = 4;
    if ((size_t)k * nch <= left) {
        for (unsigned c = 0; c < nch; c++) {
            osc.ch[c].adc_adjust.resize(k);
            for (unsigned j = 0; j < k; j++)
                osc.ch[c].adc_adjust[j] = p[i++];
        }
        left -= (size_t)k * nch;
    }
    if ((size_t)k * nch * 4 <= left)
        for (unsigned c = 0; c < nch; c++) {
            osc.ch[c].adc_adjust_zoom.resize(k);
            for (unsigned j = 0; j < k; j++, i += 4)
                osc.ch[c].adc_adjust_zoom[j] = (double)be32(p + i) / 1000.0;
        }
    return true;
}

bool DevInfo::ApplyInputAdjust(const uint8_t* p, size_t n)
{
    if (n < 4 || p[0] || p[1] || p[2] != 0xfb || p[3] != 1)
        return false;
    size_t i = 4;
    for (auto& cd : osc.ch)
        for (auto& code : cd.inzoom) {
            if (i + 2 > n)
                return true;
            code = (int16_t)be16(p + i);
            i += 2;
        }
    return true;
}

void DevInfo::ApplyZoomCalibration(const uint8_t* buf, size_t n)
{
    size_t i = 0;
    for (auto& cd : osc.ch) {
        unsigned nin = (unsigned)cd.inzoom.size();
        for (unsigned z = 0; z < cd.zoom.size(); z++) {
            if (i + 2 > n)
                return;
            int16_t v = (int16_t)be16(buf + i);
            i += 2;
            if (osc.zoom_is_dac)
                cd.zoom[z].dac = v;
            else
                cd.zoom[z].code = v;
            for (unsigned k = 0; k < nin; k++, i += 2) {
                if (i + 2 > n)
                    return;
                cd.jiaozhun[z * nin + k] = (double)be16(buf + i);
            }
        }
    }
}

bool DevInfo::ApplyDdsDacVol(const uint8_t* p, size_t n)
{
    if (n < 4 || p[0] || p[1] || p[2] != 0xf0 || p[3] != 1)
        return false;
    for (unsigned c = 0; c < dds.ChannelNum() && 4 + c * 8 + 8 <= n; c++) {
        dds.vol_min_mv[c] = (int32_t)be32(p + 4 + c * 8);
        dds.vol_max_mv[c] = (int32_t)be32(p + 8 + c * 8);
    }
    return true;
}

bool DevInfo::ApplyDdsDacZoom(const uint8_t* p, size_t n)
{
    if (n < 4 || p[0] || p[1] || p[2] != 0xf1 || p[3] != 1)
        return false;
    size_t i = 4;
    for (unsigned c = 0; c < dds.ChannelNum(); c++)
        for (unsigned z = 0; z < dds.zoom_num; z++, i += 6) {
            if (i + 6 > n)
                return true;
            dds.zoom[c * dds.zoom_num + z].freq = be32(p + i);
            dds.zoom[c * dds.zoom_num + z].zoom = (double)be16(p + i + 4) / 1000.0;
        }
    return true;
}

bool DevInfo::ApplyIoDac(const uint8_t* p, size_t n)
{
    if (n < 4 || p[0] || p[1] || p[2] != 0xf2 || p[3] != 1)
        return false;
    for (unsigned d = 0; d < io.dac.size() && 4 + d * 8 + 8 <= n; d++) {
        io.dac[d].vol_min_mv = (int32_t)be32(p + 4 + d * 8);
        io.dac[d].vol_max_mv = (int32_t)be32(p + 8 + d * 8);
    }
    return true;
}

bool DevInfo::ApplyPower(const uint8_t* p, size_t n)
{
    if (n < 4 || p[0] || p[1] || p[2] != 0xf4)
        return false;
    if (p[3] == 0x10)
        return true;
    if (p[3] != 1)
        return false;
    for (unsigned c = 0; c < power.cal.size() && 4 + c * 4 + 4 <= n; c++)
        power.cal[c] = be32(p + 4 + c * 4);
    return true;
}

std::string DevInfo::Describe() const
{
    char b[4096];
    int k = std::snprintf(b, sizeof b,
                          "name=%s hw=%u.%u proto=%u table=%s modules=%02x id=%08x\n"
                          "osc: ch=%u adcbits=%u adc=[%.3f,%.3f] zoomdiv=%u mem=%uKB range=[%d,%d]mV trig=%x src=%llx acdc=%u\n",
                          name.c_str(), hw_ver, hw_sub, proto_ver, table ? table : "?", modules, only_id0,
                          osc.ChannelNum(), osc.adc_bits, osc.adc_min_v, osc.adc_max_v, osc.zoom_div, osc.memory_kb,
                          osc.range_min_mv, osc.range_max_mv, osc.trigger_styles,
                          (unsigned long long)osc.trigger_sources, osc.support_acdc);
    std::string s(b, k);
    for (unsigned c = 0; c < osc.ChannelNum(); c++) {
        const auto& cd = osc.ch[c];
        s += "  ch" + std::to_string(c) + " inzoom:";
        for (int v : cd.inzoom)
            s += " " + std::to_string(v);
        s += " zoom:";
        for (auto& z : cd.zoom)
            s += " " + std::to_string(z.code) + "/" + std::to_string(z.dac);
        s += " jz:";
        for (double v : cd.jiaozhun)
            s += " " + std::to_string((int)v);
        s += " adj:";
        for (auto v : cd.adc_adjust)
            s += " " + std::to_string(v);
        s += "\n";
    }
    if (modules & MOD_DDS) {
        k = std::snprintf(b, sizeof b, "dds: ch=%u clk=%u depth=%uK fbits=%u dacbits=%u zoomnum=%u trig=%x gate=%x\n",
                          dds.ChannelNum(), dds.clock, dds.depth_kb, dds.freq_bits, dds.dac_bits, dds.zoom_num,
                          dds.trigger_sources, dds.gate_enges);
        s.append(b, k);
        for (unsigned c = 0; c < dds.ChannelNum(); c++) {
            k = std::snprintf(b, sizeof b, "  ch%u vol=[%d,%d]\n", c, dds.vol_min_mv[c], dds.vol_max_mv[c]);
            s.append(b, k);
        }
        for (auto& bx : dds.boxing) {
            k = std::snprintf(b, sizeof b, "  box %x vol=[%d,%d] bias=[%d,%d] f=[%g,%g] step=%g sweep=%u burst=%u\n",
                              bx.style, bx.vol_min_mv, bx.vol_max_mv, bx.bias_min_mv, bx.bias_max_mv, bx.min_freq,
                              bx.max_freq, bx.freq_step, bx.sweep, bx.burst);
            s.append(b, k);
        }
    }
    if (modules & MOD_LOGIC) {
        k = std::snprintf(b, sizeof b, "logic: ch=%u rates=%zu mem=%u\n", logic.channel_num, logic.sample_rates.size(),
                          logic.memory);
        s.append(b, k);
    }
    if (modules & MOD_IO) {
        k = std::snprintf(b, sizeof b, "io: num=%u outstates=%x dacs=%zu\n", io.io_num, io.out_states, io.dac.size());
        s.append(b, k);
    }
    return s;
}

} // namespace vmopen
