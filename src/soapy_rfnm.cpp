#include <spdlog/spdlog.h>

#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>

#include "soapy_rfnm.h"
#include <librfnm/rx_stream.h>

static enum rfnm::transport transport_from_string(const std::string& s) {
    if (s == "usb") {
        return rfnm::TRANSPORT_USB;
    }
    if (s == "eth" || s == "tcp") {
        return rfnm::TRANSPORT_TCP;
    }
    if (s == "local") {
        return rfnm::TRANSPORT_LOCAL;
    }
    return rfnm::TRANSPORT_FIND;
}

static std::string transport_to_string(enum rfnm::transport transport) {
    switch (transport) {
    case rfnm::TRANSPORT_USB:
        return "usb";
    case rfnm::TRANSPORT_TCP:
        return "eth";
    case rfnm::TRANSPORT_LOCAL:
        return "local";
    default:
        return "";
    }
}

SoapyRFNM::SoapyRFNM(const SoapySDR::Kwargs& args) {
    // devices discovered by find() carry an exact transport and address; manual open
    // falls back to searching all transports, optionally filtered by serial
    enum rfnm::transport transport = rfnm::TRANSPORT_FIND;
    std::string address;
    if (args.count("transport")) {
        transport = transport_from_string(args.at("transport"));
    }
    if (transport == rfnm::TRANSPORT_TCP && args.count("address")) {
        address = args.at("address");
    }
    else if (args.count("serial")) {
        address = args.at("serial");
    }

    lrfnm = new rfnm::device(transport, address);

    rx_chan_count = lrfnm->get_rx_channel_count();

    // sane defaults, staged only - a single apply happens when the client configures
    for (size_t i = 0; i < rx_chan_count; i++) {
        lrfnm->set_rx_channel_status(i, RFNM_CH_RF_OFF, RFNM_CH_STREAM_AUTO, false);
        lrfnm->set_rx_channel_path(i, lrfnm->get_rx_channel(i)->path_preferred, false);
        lrfnm->set_rx_channel_gain(i, 0, false);
        lrfnm->set_rx_channel_rfic_lpf_bw(i,
                rfnm::device::suggested_lpf_bw(lrfnm->get_hwinfo()->clock.samp_rate), false);
    }
}

SoapyRFNM::~SoapyRFNM() {
    delete lrfnm;
}

std::string SoapyRFNM::getDriverKey() const {
    return { "RFNM" };
}

std::string SoapyRFNM::getHardwareKey() const {
    return reinterpret_cast<const char*>(lrfnm->get_hwinfo()->motherboard.user_readable_name);
}

SoapySDR::Kwargs SoapyRFNM::getHardwareInfo() const {
    const struct rfnm_dev_hwinfo* hwinfo = lrfnm->get_hwinfo();
    SoapySDR::Kwargs info;
    info["serial"] = reinterpret_cast<const char*>(hwinfo->motherboard.serial_number);
    info["protocol_version"] = std::to_string(hwinfo->protocol_version);
    for (int i = 0; i < 2; i++) {
        if (!hwinfo->daughterboard[i].board_id) {
            continue;
        }
        std::string key = (i == 0) ? "daughterboard" : "daughterboard2";
        info[key] = reinterpret_cast<const char*>(hwinfo->daughterboard[i].user_readable_name);
        info[key + "_serial"] = reinterpret_cast<const char*>(hwinfo->daughterboard[i].serial_number);
    }
    return info;
}

SoapySDR::Kwargs SoapyRFNM::getChannelInfo(const int direction, const size_t channel) const {
    SoapySDR::Kwargs args;
    args["name"] = "RB";
    if (direction == SOAPY_SDR_TX) {
        const struct rfnm_api_tx_ch* ch = lrfnm->get_tx_channel(channel);
        args["name"] += (char)('A' + ch->dgb_id);
        args["name"] += "_TX" + std::to_string(ch->dgb_ch_id + 1);
        args["dac_id"] = std::to_string(ch->dac_id);
    }
    else {
        const struct rfnm_api_rx_ch* ch = lrfnm->get_rx_channel(channel);
        args["name"] += (char)('A' + ch->dgb_id);
        args["name"] += "_RX" + std::to_string(ch->dgb_ch_id + 1);
        args["adc_id"] = std::to_string(ch->adc_id);
    }
    return args;
}

SoapySDR::ArgInfoList SoapyRFNM::getSettingInfo(const int direction, const size_t channel) const {
    SoapySDR::ArgInfoList channel_settings_args;
    if (direction == SOAPY_SDR_RX) {
        SoapySDR::ArgInfo rm_notch_arg;
        rm_notch_arg.key = "fm_notch";
        rm_notch_arg.description = "FM notch filter control";
        rm_notch_arg.type = SoapySDR::ArgInfo::STRING;
        rm_notch_arg.options = { "auto", "on", "off" };
        rm_notch_arg.value = "auto";

        SoapySDR::ArgInfo bias_tee_arg;
        bias_tee_arg.key = "bias_tee_en";
        bias_tee_arg.description = "Antenna bias tee control";
        bias_tee_arg.type = SoapySDR::ArgInfo::BOOL;
        bias_tee_arg.value = "false";

        channel_settings_args.push_back(rm_notch_arg);
        channel_settings_args.push_back(bias_tee_arg);
    }
    return channel_settings_args;
}

std::string SoapyRFNM::readSetting(const int direction, const size_t channel, const std::string& key) const {
    if (key == "bias_tee_en") {
        if (direction == SOAPY_SDR_TX) {
            return lrfnm->get_tx_channel(channel)->bias_tee == RFNM_BIAS_TEE_ON ? "true" : "false";
        }
        else {
            return lrfnm->get_rx_channel(channel)->bias_tee == RFNM_BIAS_TEE_ON ? "true" : "false";
        }
    }
    if (key == "fm_notch" && direction == SOAPY_SDR_RX) {
        const auto fm_notch_status = lrfnm->get_rx_channel(channel)->fm_notch;
        return
            (fm_notch_status == RFNM_FM_NOTCH_ON) ? "on" :
            (fm_notch_status == RFNM_FM_NOTCH_OFF) ? "off" :
            "auto";
    }
    return "";
}

void SoapyRFNM::writeSetting(const int direction, const size_t channel, const std::string& key, const std::string& value) {
    if (direction == SOAPY_SDR_RX) {
        if (key == "bias_tee_en") {
            lrfnm->set_rx_channel_bias_tee(channel,
                (value == "true") ? RFNM_BIAS_TEE_ON : RFNM_BIAS_TEE_OFF,
                false);
        }
        if (key == "fm_notch") {
            lrfnm->set_rx_channel_fm_notch(channel,
                (value == "on") ? RFNM_FM_NOTCH_ON :
                (value == "off") ? RFNM_FM_NOTCH_OFF :
                RFNM_FM_NOTCH_AUTO,
                false);
        }
        setRFNM(channel);
    }
}

std::vector<std::string> SoapyRFNM::listSensors() const {
    return { "rx_packets", "rx_dropped" };
}

SoapySDR::ArgInfo SoapyRFNM::getSensorInfo(const std::string& key) const {
    SoapySDR::ArgInfo info;
    info.key = key;
    info.type = SoapySDR::ArgInfo::INT;
    if (key == "rx_packets") {
        info.description = "RX packets delivered since the last flush";
    }
    else if (key == "rx_dropped") {
        info.description = "RX packets lost since the last flush; a rising count means the transport cannot sustain the sample rate";
    }
    return info;
}

std::string SoapyRFNM::readSensor(const std::string& key) const {
    uint64_t ok = 0, dropped = 0;
    struct rfnm::health h = {};
    if (lrfnm->get_health(&h) == RFNM_API_OK) {
        ok = h.rx_pkts_ok;
        dropped = h.rx_pkts_dropped;
    }
    if (key == "rx_packets") {
        return std::to_string(ok);
    }
    if (key == "rx_dropped") {
        return std::to_string(dropped);
    }
    return "";
}

size_t SoapyRFNM::getStreamMTU(SoapySDR::Stream* stream) const {
    return RFNM_USB_RX_PACKET_ELEM_CNT * 16;
}

size_t SoapyRFNM::getNumChannels(const int direction) const {
    switch (direction) {
    case SOAPY_SDR_TX:
        return 0; // not yet implemented
    case SOAPY_SDR_RX:
        return rx_chan_count;
    default:
        return 0;
    }
}

std::vector<double> SoapyRFNM::listSampleRates(const int direction, const size_t channel) const {
    std::vector<double> rates;

    if (direction == SOAPY_SDR_RX) {
        rates.push_back(lrfnm->get_hwinfo()->clock.samp_rate_max);
        rates.push_back(100e6);
        rates.push_back(lrfnm->get_hwinfo()->clock.samp_rate_max / 2);
        rates.push_back(lrfnm->get_hwinfo()->clock.samp_rate_max / 4);
    }

    return rates;
}

SoapySDR::RangeList SoapyRFNM::getSampleRateRange(const int direction, const size_t channel) const {
    SoapySDR::RangeList ranges;

    if (direction == SOAPY_SDR_RX) {
        ranges.push_back(SoapySDR::Range(
            static_cast<double>(lrfnm->get_hwinfo()->clock.samp_rate_min),
            static_cast<double>(lrfnm->get_hwinfo()->clock.samp_rate_max),
            static_cast<double>(lrfnm->get_hwinfo()->clock.samp_rate_step)));
    }

    return ranges;
}

double SoapyRFNM::getSampleRate(const int direction, const size_t channel) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return static_cast<double>(lrfnm->get_hwinfo()->clock.samp_rate);
    }
    else {
        return 0;
    }
}

void SoapyRFNM::setSampleRate(const int direction, const size_t channel, const double rate) {
    if (direction != SOAPY_SDR_RX) {
        return;
    }
    if (channel >= rx_chan_count) {
        throw std::runtime_error("nonexistent channel");
    }

    // A rate change only takes effect at the next channel apply (the device defers the
    // DCS reclock while ADCs stream), so quiesce and restart a live stream around it -
    // otherwise the old rate keeps flowing under the new label.
    bool was_active = stream_active;
    if (rx_stream && was_active) {
        rx_stream->stop();
        stream_active = false;
    }

    rfnm_api_failcode ret = lrfnm->set_samp_rate(static_cast<uint64_t>(rate), 2000000);

    // the anti-alias filter tracks the rate unless the client chose one explicitly
    if (ret == RFNM_API_OK && !bw_manual) {
        for (size_t i = 0; i < rx_chan_count; i++) {
            lrfnm->set_rx_channel_rfic_lpf_bw(i,
                    rfnm::device::suggested_lpf_bw(static_cast<uint64_t>(rate), &lrfnm->get_hwinfo()->clock), false);
        }
    }

    if (rx_stream && was_active) {
        if (rx_stream->start() == RFNM_API_OK) {
            stream_active = true;
        }
    }

    if (ret != RFNM_API_OK) {
        throw std::runtime_error("set_samp_rate failed");
    }
}

std::string SoapyRFNM::getNativeStreamFormat(const int direction, const size_t /*channel*/, double& fullScale) const {
    fullScale = 32768;
    return SOAPY_SDR_CS16;
}

std::vector<std::string> SoapyRFNM::getStreamFormats(const int direction, const size_t channel) const {
    std::vector<std::string> formats;
    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);
    formats.push_back(SOAPY_SDR_CS8);
    return formats;
}

// only one stream is supported, so the stream handle is this device instance
int SoapyRFNM::activateStream(SoapySDR::Stream* stream, const int flags, const long long timeNs,
        const size_t numElems) {
    // a killed client can leave channels enabled, which blocks stream creation - the
    // driver's canonical sweep disables exactly those (the configuration is fully
    // re-applied when the stream starts anyway)
    lrfnm->rx_disable_stale_channels(20000000);

    if (rx_stream->start()) {
        throw std::runtime_error("error starting stream");
    }
    stream_active = true;

    return 0;
}

int SoapyRFNM::deactivateStream(SoapySDR::Stream* stream, const int flags0, const long long int timeNs) {
    rx_stream->stop();
    stream_active = false;

    return 0;
}

std::vector<std::string> SoapyRFNM::listFrequencies(const int direction, const size_t channel) const {
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

SoapySDR::RangeList SoapyRFNM::getFrequencyRange(const int direction, const size_t channel, const std::string& name) const {
    SoapySDR::RangeList results;

    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        auto ch = lrfnm->get_rx_channel(channel);
        results.push_back(SoapySDR::Range(ch->freq_min, ch->freq_max));
    }

    return results;
}

double SoapyRFNM::getFrequency(const int direction, const size_t channel, const std::string& name) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return lrfnm->get_rx_channel(channel)->freq;
    }
    else {
        return 0;
    }
}

void SoapyRFNM::setFrequency(const int direction, const size_t channel, const std::string& name,
        const double frequency, const SoapySDR::Kwargs& args) {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        lrfnm->set_rx_channel_freq(channel, frequency, false);
        setRFNM(channel);
    }
}

std::vector<std::string> SoapyRFNM::listGains(const int direction, const size_t channel) const {
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

double SoapyRFNM::getGain(const int direction, const size_t channel, const std::string& name) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return lrfnm->get_rx_channel(channel)->gain;
    }
    else {
        return 0;
    }
}

void SoapyRFNM::setGain(const int direction, const size_t channel, const std::string& name, const double value) {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        lrfnm->set_rx_channel_gain(channel, value, false);
        setRFNM(channel);
    }
}

SoapySDR::Range SoapyRFNM::getGainRange(const int direction, const size_t channel, const std::string& name) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return SoapySDR::Range(
                lrfnm->get_rx_channel(channel)->gain_range.min,
                lrfnm->get_rx_channel(channel)->gain_range.max);
    }
    else {
        return SoapySDR::Range(0, 0);
    }
}

double SoapyRFNM::getBandwidth(const int direction, const size_t channel) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return lrfnm->get_rx_channel(channel)->rfic_lpf_bw * 1e6;
    }
    else {
        return 0;
    }
}

void SoapyRFNM::setBandwidth(const int direction, const size_t channel, const double bw) {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        // zero means automatic: the filter tracks the sample rate
        int16_t mhz;
        if (bw == 0.0) {
            bw_manual = false;
            mhz = rfnm::device::suggested_lpf_bw(lrfnm->get_hwinfo()->clock.samp_rate,
                    &lrfnm->get_hwinfo()->clock);
        }
        else {
            bw_manual = true;
            mhz = static_cast<int16_t>(bw / 1e6);
        }

        lrfnm->set_rx_channel_rfic_lpf_bw(channel, mhz, false);
        setRFNM(channel);
    }
}

SoapySDR::RangeList SoapyRFNM::getBandwidthRange(const int direction, const size_t channel) const {
    SoapySDR::RangeList bws;
    bws.push_back(SoapySDR::Range(1e6, 160e6));
    return bws;
}

std::vector<std::string> SoapyRFNM::listAntennas(const int direction, const size_t channel) const {
    std::vector<std::string> ants;
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        for (int a = 0; a < 10; a++) {
            if (lrfnm->get_rx_channel(channel)->path_possible[a] == RFNM_PATH_NULL) {
                continue;
            }
            ants.push_back(rfnm::device::rf_path_to_string(lrfnm->get_rx_channel(channel)->path_possible[a]));
        }
    }
    return ants;
}

std::string SoapyRFNM::getAntenna(const int direction, const size_t channel) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return rfnm::device::rf_path_to_string(lrfnm->get_rx_channel(channel)->path);
    }
    else {
        return "";
    }
}

void SoapyRFNM::setAntenna(const int direction, const size_t channel, const std::string& name) {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        auto path = rfnm::device::string_to_rf_path(name);
        lrfnm->set_rx_channel_path(channel, path, false);
        setRFNM(channel);
    }
}

SoapySDR::Stream* SoapyRFNM::setupStream(const int direction, const std::string& format,
        const std::vector<size_t>& channels, const SoapySDR::Kwargs& args) {
    if (direction != SOAPY_SDR_RX) {
        return nullptr;
    }

    if (rx_stream) {
        throw std::runtime_error("multiple streams unsupported");
    }

    // SoapySDR convention: an empty channel list means "automatic" -> the single default
    // RX channel (0). Without this, a client that calls setupStream(dir, format) with no
    // channels (SoapySDRUtil, some GUIs) got a zero-channel stream: rx_stream::start()
    // no-ops (channels.size()==0), the RF never enables, and every read returns all-zero IQ.
    const std::vector<size_t> chans = channels.empty() ? std::vector<size_t>{0} : channels;

    // bounds check channels before we start the stream
    uint8_t chan_mask = 0;
    for (size_t channel : chans) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        chan_mask |= rfnm::channel_flags[channel];
    }

    enum rfnm::stream_format stream_format;

    if (!format.compare(SOAPY_SDR_CF32)) {
        stream_format = rfnm::STREAM_FORMAT_CF32;
    }
    else if (!format.compare(SOAPY_SDR_CS16)) {
        stream_format = rfnm::STREAM_FORMAT_CS16;
    }
    else if (!format.compare(SOAPY_SDR_CS8)) {
        stream_format = rfnm::STREAM_FORMAT_CS8;
    }
    else {
        throw std::runtime_error("setupStream invalid format " + format);
    }

    if (lrfnm->set_stream_format(stream_format, nullptr)) {
        throw std::runtime_error("changing stream format is unsupported");
    }

    rx_stream = lrfnm->rx_stream_create(chan_mask);

    // copy over DC correction settings
    for (size_t channel : chans) {
        rx_stream->set_auto_dc_offset(dc_correction[channel], rfnm::channel_flags[channel]);
    }

    return (SoapySDR::Stream*)this;
}

void SoapyRFNM::closeStream(SoapySDR::Stream* stream) {
    if (rx_stream) {
        delete rx_stream;
        rx_stream = nullptr;
        stream_active = false;
    }
}

int SoapyRFNM::readStream(SoapySDR::Stream* stream, void* const* buffs, const size_t numElems, int& flags,
        long long int& timeNs, const long timeoutUs) {
    size_t elems_read;
    uint64_t timestamp_ns;
    rfnm_api_failcode ret = rx_stream->read(buffs, numElems, elems_read, timestamp_ns, timeoutUs);

    // Soapy contract: report recoverable conditions via return codes, never throw -
    // consumers treat exceptions as fatal and halt the stream thread, but transient
    // timeouts are NORMAL at stream rampup (DCS reclock settling).
    switch (ret) {
    case RFNM_API_OK:
        timeNs = static_cast<long long int>(timestamp_ns);
        return elems_read;
    case RFNM_API_TIMEOUT:
    case RFNM_API_DQBUF_NO_DATA:
        return SOAPY_SDR_TIMEOUT;
    case RFNM_API_DQBUF_OVERFLOW:
        return SOAPY_SDR_OVERFLOW;
    default:
        spdlog::error("Error {} reading from stream", static_cast<int>(ret));
        return SOAPY_SDR_STREAM_ERROR;
    }
}

bool SoapyRFNM::hasDCOffsetMode(const int direction, const size_t channel) const {
    return true;
}

void SoapyRFNM::setDCOffsetMode(const int direction, const size_t channel, const bool automatic) {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        dc_correction[channel] = automatic;

        if (rx_stream) {
            rx_stream->set_auto_dc_offset(automatic, rfnm::channel_flags[channel]);
        }
    }
}

bool SoapyRFNM::getDCOffsetMode(const int direction, const size_t channel) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return dc_correction[channel];
    }
    else {
        return false;
    }
}

void SoapyRFNM::setRFNM(size_t channel) {
    rfnm_api_failcode ret = lrfnm->apply(rfnm::rx_channel_apply_flags[channel], true, 20000000);

    // GCC cannot pass references to values in packed structs, so we need stack copies
    auto ch = lrfnm->get_rx_channel(channel);
    uint64_t freq = ch->freq;
    int8_t gain = ch->gain;

    switch (ret) {
    case RFNM_API_OK:
        return;
    case RFNM_API_TUNE_FAIL:
        spdlog::error("Failure tuning channel {} to {} Hz", channel, freq);
        throw std::runtime_error("Tuning failure");
    case RFNM_API_GAIN_FAIL:
        spdlog::error("Failure setting channel {} gain to {} dB", channel, gain);
        throw std::runtime_error("Gain setting failure");
    case RFNM_API_TIMEOUT:
        spdlog::error("Timeout configuring RFNM");
        throw std::runtime_error("Timeout configuring RFNM");
    case RFNM_API_USB_FAIL:
        spdlog::error("USB failure configuring RFNM");
        throw std::runtime_error("USB failure configuring RFNM");
    default:
        spdlog::error("Error {} configuring RFNM", static_cast<int>(ret));
        throw std::runtime_error("Error configuring RFNM");
    }
}

SoapySDR::Device* rfnm_device_create(const SoapySDR::Kwargs& args) {
    return new SoapyRFNM(args);
}

SoapySDR::KwargsList rfnm_device_find(const SoapySDR::Kwargs& args) {
    std::vector<struct rfnm::dev_info> devices;

    // a directed probe reaches devices that broadcast discovery cannot (e.g. across
    // subnets); otherwise search every transport
    if (args.count("address") && !args.at("address").empty()) {
        devices = rfnm::device::find(rfnm::TRANSPORT_TCP, args.at("address"));
    }
    else {
        devices = rfnm::device::find(rfnm::TRANSPORT_FIND);
    }

    SoapySDR::KwargsList ret;
    for (auto& dev : devices) {
        std::string serial = reinterpret_cast<char*>(dev.hwinfo.motherboard.serial_number);
        std::string transport = transport_to_string(dev.transport);

        // honor filter args
        if (args.count("serial") && args.at("serial") != serial) {
            continue;
        }
        if (args.count("transport") && args.at("transport") != transport) {
            continue;
        }

        SoapySDR::Kwargs info;
        info["device_id"] = "RFNM";
        info["serial"] = serial;
        info["transport"] = transport;
        if (dev.transport == rfnm::TRANSPORT_TCP) {
            info["address"] = dev.address;
        }

        info["label"] = "RFNM " + serial + " (" + transport + ")";
        for (int i = 0; i < 2; i++) {
            if (dev.hwinfo.daughterboard[i].board_id) {
                info["label"] += " ";
                info["label"] += reinterpret_cast<char*>(dev.hwinfo.daughterboard[i].user_readable_name);
            }
        }

        ret.push_back(info);
    }

    return ret;
}

[[maybe_unused]] static SoapySDR::Registry rfnm_module_registration("rfnm", &rfnm_device_find, &rfnm_device_create, SOAPY_SDR_ABI_VERSION);
