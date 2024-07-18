#include <spdlog/spdlog.h>

#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Formats.hpp>

#include "soapy_rfnm.h"
#include <librfnm/rx_stream.h>

SoapyRFNM::SoapyRFNM(const SoapySDR::Kwargs& args) {
    spdlog::info("RFNMDevice::RFNMDevice()");

    if (args.count("serial") != 0) {
        lrfnm = new rfnm::device(rfnm::LIBRFNM_TRANSPORT_USB, (std::string)args.at("serial"));
    }
    else {
        lrfnm = new rfnm::device(rfnm::LIBRFNM_TRANSPORT_USB);
    }

    rx_chan_count = lrfnm->get_rx_channel_count();

    // sane defaults
    uint16_t apply_mask = 0;
    for (size_t i = 0; i < rx_chan_count; i++) {
        lrfnm->set_rx_channel_active(i, RFNM_CH_OFF, RFNM_CH_STREAM_AUTO, false);
        lrfnm->set_rx_channel_freq(i, RFNM_MHZ_TO_HZ(2450), false);
        lrfnm->set_rx_channel_path(i, lrfnm->get_rx_channel(i)->path_preferred, false);
        lrfnm->set_rx_channel_samp_freq_div(i, 1, 2, false);
        lrfnm->set_rx_channel_gain(i, 0, false);
        lrfnm->set_rx_channel_rfic_lpf_bw(i, 80, false);
        apply_mask |= rfnm::rx_channel_apply_flags[i];
    }
    setRFNM(apply_mask);

    //s->tx.ch[0].freq = RFNM_MHZ_TO_HZ(2450);
    //s->tx.ch[0].path = s->tx.ch[0].path_preferred;
    //s->tx.ch[0].samp_freq_div_n = 2;

}

SoapyRFNM::~SoapyRFNM() {
    spdlog::info("RFNMDevice::~RFNMDevice()");
    delete lrfnm;
}

std::string SoapyRFNM::getDriverKey() const {
    spdlog::info("RFNMDevice::getDriverKey()");
    return { "RFNM" };
}

std::string SoapyRFNM::getHardwareKey() const {
    spdlog::info("RFNMDevice::getHardwareKey()");
    return { "RFNM" };
}

SoapySDR::Kwargs SoapyRFNM::getHardwareInfo() const {
    spdlog::info("RFNMDevice::getHardwareInfo()");
    return {};
}

SoapySDR::Kwargs SoapyRFNM::getChannelInfo(const int direction, const size_t channel) const {
    spdlog::info("RFNMDevice::getChannelInfo()");
    SoapySDR::Kwargs args;
    args["name"] = "RB";
    if (direction == SOAPY_SDR_TX) {
        const struct rfnm_api_tx_ch * ch = lrfnm->get_tx_channel(channel);
        args["name"] += (char)('A' + ch->dgb_id);
        args["name"] += "_TX" + std::to_string(ch->dgb_ch_id + 1);
        args["dac_id"] = std::to_string(ch->dac_id);
    } else {
        const struct rfnm_api_rx_ch * ch = lrfnm->get_rx_channel(channel);
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
        rm_notch_arg.options = {"auto", "on", "off"};
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

std::string SoapyRFNM::readSetting(const int direction, const size_t channel, const std::string &key) const {
    if (key == "bias_tee_en") {
        if (direction == SOAPY_SDR_TX) {
            return lrfnm->get_tx_channel(channel)->bias_tee == RFNM_BIAS_TEE_ON ? "true" : "false";
        } else {
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

void SoapyRFNM::writeSetting(const int direction, const size_t channel, const std::string &key, const std::string &value) {
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
        setRFNM(rfnm::rx_channel_apply_flags[channel]);
    }
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
        rates.push_back(lrfnm->get_hwinfo()->clock.dcs_clk);
        rates.push_back(lrfnm->get_hwinfo()->clock.dcs_clk / 2);
    }

    return rates;
}

double SoapyRFNM::getSampleRate(const int direction, const size_t channel) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return lrfnm->get_hwinfo()->clock.dcs_clk / lrfnm->get_rx_channel(channel)->samp_freq_div_n;
    } else {
        return 0;
    }
}

void SoapyRFNM::setSampleRate(const int direction, const size_t channel, const double rate) {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        if (rate == lrfnm->get_hwinfo()->clock.dcs_clk) {
            lrfnm->set_rx_channel_samp_freq_div(channel, 1, 1, false);
        } else if (rate == lrfnm->get_hwinfo()->clock.dcs_clk / 2) {
            lrfnm->set_rx_channel_samp_freq_div(channel, 1, 2, false);
        } else {
            throw std::runtime_error("unsupported sample rate");
        }
        setRFNM(rfnm::rx_channel_apply_flags[channel]);
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

// hack: I'm only supporting one stream for now, so I use a member variable to store it and ignore the stream argument
int SoapyRFNM::activateStream(SoapySDR::Stream* stream, const int flags, const long long timeNs,
        const size_t numElems) {
    spdlog::info("RFNMDevice::activateStream()");

    if (rx_stream->start()) {
        throw std::runtime_error("error starting stream");
    }

    return 0;
}

int SoapyRFNM::deactivateStream(SoapySDR::Stream* stream, const int flags0, const long long int timeNs) {
    spdlog::info("RFNMDevice::deactivateStream()");

    rx_stream->stop();

    return 0;
}

std::vector<std::string> SoapyRFNM::listFrequencies(const int direction, const size_t channel) const {
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

SoapySDR::RangeList SoapyRFNM::getFrequencyRange(const int direction, const size_t channel, const std::string &name) const {
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

double SoapyRFNM::getFrequency(const int direction, const size_t channel, const std::string &name) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return lrfnm->get_rx_channel(channel)->freq;
    } else {
        return 0;
    }
}

void SoapyRFNM::setFrequency(const int direction, const size_t channel, const std::string &name,
        const double frequency, const SoapySDR::Kwargs& args) {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        lrfnm->set_rx_channel_freq(channel, frequency, false);
        setRFNM(rfnm::rx_channel_apply_flags[channel]);
    }
}

std::vector<std::string> SoapyRFNM::listGains(const int direction, const size_t channel) const {
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

double SoapyRFNM::getGain(const int direction, const size_t channel, const std::string &name) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return lrfnm->get_rx_channel(channel)->gain;
    } else {
        return 0;
    }
}

void SoapyRFNM::setGain(const int direction, const size_t channel, const std::string &name, const double value) {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        lrfnm->set_rx_channel_gain(channel, value, false);
        setRFNM(rfnm::rx_channel_apply_flags[channel]);
    }
}

SoapySDR::Range SoapyRFNM::getGainRange(const int direction, const size_t channel, const std::string &name) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return SoapySDR::Range(
                lrfnm->get_rx_channel(channel)->gain_range.min,
                lrfnm->get_rx_channel(channel)->gain_range.max);
    } else {
        return SoapySDR::Range(0, 0);
    }
}

double SoapyRFNM::getBandwidth(const int direction, const size_t channel) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return lrfnm->get_rx_channel(channel)->rfic_lpf_bw * 1e6;
    } else {
        return 0;
    }
}

void SoapyRFNM::setBandwidth(const int direction, const size_t channel, const double bw) {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        if (bw == 0.0) return; // special ignore value

        lrfnm->set_rx_channel_rfic_lpf_bw(channel, bw / 1e6, false);
        setRFNM(rfnm::rx_channel_apply_flags[channel]);
    }
}

SoapySDR::RangeList SoapyRFNM::getBandwidthRange(const int direction, const size_t channel) const {
    SoapySDR::RangeList bws;
    bws.push_back(SoapySDR::Range(1e6, 100e6));
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
                break;
            }
            ants.push_back(rfnm::device::rf_path_to_string(lrfnm->get_rx_channel(channel)->path_possible[a]));
        }
    }
    else if (direction == SOAPY_SDR_TX) {
        //   ants.push_back("TXH");
        //    ants.push_back("TXW");
    }
    return ants;
}

std::string SoapyRFNM::getAntenna(const int direction, const size_t channel) const {
    if (direction == SOAPY_SDR_RX) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        return rfnm::device::rf_path_to_string(lrfnm->get_rx_channel(channel)->path);
    } else {
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
        setRFNM(rfnm::rx_channel_apply_flags[channel]);
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

    // bounds check channels before we start the stream
    uint8_t chan_mask = 0;
    for (size_t channel : channels) {
        if (channel >= rx_chan_count) {
            throw std::runtime_error("nonexistent channel");
        }

        chan_mask |= rfnm::channel_flags[channel];
    }

    enum rfnm::stream_format stream_format;

    if (!format.compare(SOAPY_SDR_CF32)) {
        stream_format = rfnm::LIBRFNM_STREAM_FORMAT_CF32;
    } else if (!format.compare(SOAPY_SDR_CS16)) {
        stream_format = rfnm::LIBRFNM_STREAM_FORMAT_CS16;
    } else if (!format.compare(SOAPY_SDR_CS8)) {
        stream_format = rfnm::LIBRFNM_STREAM_FORMAT_CS8;
    } else {
        throw std::runtime_error("setupStream invalid format " + format);
    }

    if (lrfnm->set_stream_format(stream_format, nullptr)) {
        throw std::runtime_error("changing stream format is unsupported");
    }

    rx_stream = lrfnm->rx_stream_create(chan_mask);

    // copy over DC correction settings
    for (size_t channel : channels) {
        rx_stream->set_auto_dc_offset(dc_correction[channel], rfnm::channel_flags[channel]);
    }

    return (SoapySDR::Stream*)this;
}

void SoapyRFNM::closeStream(SoapySDR::Stream* stream) {
    spdlog::info("RFNMDevice::closeStream() -> Closing stream");

    if (rx_stream) {
        delete rx_stream;
        rx_stream = nullptr;
    }
}

int SoapyRFNM::readStream(SoapySDR::Stream* stream, void* const* buffs, const size_t numElems, int& flags,
        long long int& timeNs, const long timeoutUs) {
    size_t elems_read;
    uint64_t timestamp_ns;
    rfnm_api_failcode ret = rx_stream->read(buffs, numElems, elems_read, timestamp_ns, timeoutUs);

    if (ret != RFNM_API_OK) {
        spdlog::error("Error {} reading from stream", static_cast<int>(ret));
        throw std::runtime_error("Error reading from stream");
    }

    timeNs = static_cast<long long int>(timestamp_ns);
    return elems_read;
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
    } else {
        return false;
    }
}

void SoapyRFNM::setRFNM(uint16_t applies) {
    rfnm_api_failcode ret = lrfnm->set(applies);

    size_t chan_idx;
    switch (applies) {
    case rfnm::LIBRFNM_APPLY_CH0_RX:
        chan_idx = 0;
        break;
    case rfnm::LIBRFNM_APPLY_CH1_RX:
        chan_idx = 1;
        break;
    case rfnm::LIBRFNM_APPLY_CH2_RX:
        chan_idx = 2;
        break;
    case rfnm::LIBRFNM_APPLY_CH3_RX:
        chan_idx = 3;
        break;
    default:
        chan_idx = 0;
    }

    // GCC cannot pass references to values in packed structs, so we need stack copies
    auto ch = lrfnm->get_rx_channel(chan_idx);
    uint64_t freq = ch->freq;
    int8_t gain = ch->gain;

    switch (ret) {
    case RFNM_API_OK:
        return;
    case RFNM_API_TUNE_FAIL:
        spdlog::error("Failure tuning channel {} to {} Hz", chan_idx, freq);
        throw std::runtime_error("Tuning failure");
    case RFNM_API_GAIN_FAIL:
        spdlog::error("Failure setting channel {} gain to {} dB", chan_idx, gain);
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
    spdlog::info("rfnm_device_create()");

    return new SoapyRFNM(args);
}

SoapySDR::KwargsList rfnm_device_find(const SoapySDR::Kwargs& args) {
    std::vector<struct rfnm_dev_hwinfo> hwlist = rfnm::device::find(rfnm::LIBRFNM_TRANSPORT_USB);
    std::vector< SoapySDR::Kwargs> ret;

    for (auto& hw : hwlist)
    {
        SoapySDR::Kwargs deviceInfo;

        deviceInfo["device_id"] = "RFNM";
        deviceInfo["label"] = "RFNM";
        if (hw.daughterboard[0].board_id) {
            std::string dgbn = reinterpret_cast<char*>(hw.daughterboard[0].user_readable_name);
            deviceInfo["label"] += " with " + dgbn;
        }

        if (hw.daughterboard[1].board_id) {
            std::string dgbn = reinterpret_cast<char*>(hw.daughterboard[1].user_readable_name);
            deviceInfo["label"] += " and " + dgbn + " daughterboards";
        }
        else {
            deviceInfo["label"] += " daughterboard";
        }

        std::string serial = reinterpret_cast<char*>(hw.motherboard.serial_number);
        deviceInfo["serial"] = serial;

        ret.push_back(deviceInfo);
    }

    return ret;
}

[[maybe_unused]] static SoapySDR::Registry rfnm_module_registration("RFNM", &rfnm_device_find, &rfnm_device_create, SOAPY_SDR_ABI_VERSION);
