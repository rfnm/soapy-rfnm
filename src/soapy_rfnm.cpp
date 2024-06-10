#include <spdlog/spdlog.h>

#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Formats.hpp>

#include "soapy_rfnm.h"
#include <librfnm/librfnm.h>


SoapyRFNM::SoapyRFNM(const SoapySDR::Kwargs& args) {
    spdlog::info("RFNMDevice::RFNMDevice()");

    if (args.count("serial") != 0) {
        lrfnm = new librfnm(LIBRFNM_TRANSPORT_USB, (std::string)args.at("serial"));
    }
    else {
        lrfnm = new librfnm(LIBRFNM_TRANSPORT_USB);
    }

    if (!lrfnm->s->transport_status.theoretical_mbps) {
        throw std::runtime_error("Couldn't open the RFNM USB device handle");
    }

    memset(rxbuf, 0, sizeof(rxbuf));
    //memset(txbuf, 0, sizeof(rxbuf));
    memset(&partial_rx_buf, 0, sizeof(struct rfnm_soapy_partial_buf));

    // sane defaults
    lrfnm->s->rx.ch[0].freq = RFNM_MHZ_TO_HZ(2450);
    lrfnm->s->rx.ch[0].path = lrfnm->s->rx.ch[0].path_preferred;
    lrfnm->s->rx.ch[0].samp_freq_div_n = 1;

    //s->rx.ch[1].freq = RFNM_MHZ_TO_HZ(2450);
    //s->rx.ch[1].path = s->rx.ch[1].path_preferred;
    //s->tx.ch[1].samp_freq_div_n = 2;

    //s->tx.ch[0].freq = RFNM_MHZ_TO_HZ(2450);
    //s->tx.ch[0].path = s->tx.ch[0].path_preferred;
    //s->tx.ch[0].samp_freq_div_n = 2;

}

SoapyRFNM::~SoapyRFNM() {
    spdlog::info("RFNMDevice::~RFNMDevice()");
    delete lrfnm;

    if (partial_rx_buf.buf) {
        free(partial_rx_buf.buf);
    }

    for (int i = 0; i < SOAPY_RFNM_BUFCNT; i++) {
        if (rxbuf[i].buf) {
            free(rxbuf[i].buf);
        }
    }
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

size_t SoapyRFNM::getStreamMTU(SoapySDR::Stream* stream) const {
    return RFNM_USB_RX_PACKET_ELEM_CNT * 16;
}

size_t SoapyRFNM::getNumChannels(const int direction) const {
    return direction;
}

std::vector<double> SoapyRFNM::listSampleRates(const int direction, const size_t channel) const {
    std::vector<double> rates;
    rates.push_back(lrfnm->s->hwinfo.clock.dcs_clk);
    rates.push_back(lrfnm->s->hwinfo.clock.dcs_clk / 2);
    return rates;
}

double SoapyRFNM::getSampleRate(const int direction, const size_t channel) const {
    return lrfnm->s->hwinfo.clock.dcs_clk / lrfnm->s->rx.ch[0].samp_freq_div_n;
}

void SoapyRFNM::setSampleRate(const int direction, const size_t channel, const double rate) {
    if (rate == lrfnm->s->hwinfo.clock.dcs_clk) {
        lrfnm->s->rx.ch[0].samp_freq_div_n = 1;
    } else if (rate == lrfnm->s->hwinfo.clock.dcs_clk / 2) {
        lrfnm->s->rx.ch[0].samp_freq_div_n = 2;
    } else {
        throw std::runtime_error("unsupported sample rate");
    }
    setRFNM(LIBRFNM_APPLY_CH0_RX /*| LIBRFNM_APPLY_CH0_TX  | LIBRFNM_APPLY_CH1_RX*/);
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

int SoapyRFNM::activateStream(SoapySDR::Stream* stream, const int flags, const long long timeNs,
        const size_t numElems) {
    spdlog::info("RFNMDevice::activateStream()");

    return 0;
}

int SoapyRFNM::deactivateStream(SoapySDR::Stream* stream, const int flags0, const long long int timeNs) {
    spdlog::info("RFNMDevice::deactivateStream()");

    return 0;
}

std::vector<std::string> SoapyRFNM::listFrequencies(const int direction, const size_t channel) const {
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

SoapySDR::RangeList SoapyRFNM::getFrequencyRange(const int direction, const size_t channel, const std::string &name) const {
    SoapySDR::RangeList results;
    results.push_back(SoapySDR::Range(lrfnm->s->rx.ch[0].freq_min, lrfnm->s->rx.ch[0].freq_max));
    return results;
}

double SoapyRFNM::getFrequency(const int direction, const size_t channel, const std::string &name) const {
    return lrfnm->s->rx.ch[0].freq;
}

void SoapyRFNM::setFrequency(const int direction, const size_t channel, const std::string &name,
        const double frequency, const SoapySDR::Kwargs& args) {
    lrfnm->s->rx.ch[0].freq = frequency;
    setRFNM(LIBRFNM_APPLY_CH0_RX /*| LIBRFNM_APPLY_CH0_TX  | LIBRFNM_APPLY_CH1_RX*/);
}

std::vector<std::string> SoapyRFNM::listGains(const int direction, const size_t channel) const {
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

double SoapyRFNM::getGain(const int direction, const size_t channel, const std::string &name) const {
    return lrfnm->s->rx.ch[0].gain;
}

void SoapyRFNM::setGain(const int direction, const size_t channel, const std::string &name, const double value) {
    lrfnm->s->rx.ch[0].gain = value;
    setRFNM(LIBRFNM_APPLY_CH0_RX /*| LIBRFNM_APPLY_CH0_TX  | LIBRFNM_APPLY_CH1_RX*/);
}

SoapySDR::Range SoapyRFNM::getGainRange(const int direction, const size_t channel, const std::string &name) const {
    return SoapySDR::Range(lrfnm->s->rx.ch[0].gain_range.min, lrfnm->s->rx.ch[0].gain_range.max);
}

double SoapyRFNM::getBandwidth(const int direction, const size_t channel) const {
    return lrfnm->s->rx.ch[0].rfic_lpf_bw * 1e6;
}

void SoapyRFNM::setBandwidth(const int direction, const size_t channel, const double bw) {
    if (bw == 0.0) return; //special ignore value

    lrfnm->s->rx.ch[0].rfic_lpf_bw = bw / 1e6;
    setRFNM(LIBRFNM_APPLY_CH0_RX /*| LIBRFNM_APPLY_CH0_TX  | LIBRFNM_APPLY_CH1_RX*/);
}

SoapySDR::RangeList SoapyRFNM::getBandwidthRange(const int direction, const size_t channel) const {
    SoapySDR::RangeList bws;
    bws.push_back(SoapySDR::Range(1e6, 100e6));
    return bws;
}

std::vector<std::string> SoapyRFNM::listAntennas(const int direction, const size_t channel) const {
    std::vector<std::string> ants;
    if (direction == SOAPY_SDR_RX) {
        for (int a = 0; a < 10; a++) {
            if (lrfnm->s->rx.ch[0].path_possible[a] == RFNM_PATH_NULL) {
                break;
            }
            ants.push_back(librfnm::rf_path_to_string(lrfnm->s->rx.ch[0].path_possible[a]));
        }
    }
    else if (direction == SOAPY_SDR_TX) {
        //   ants.push_back("TXH");
        //    ants.push_back("TXW");
    }
    return ants;
}

std::string SoapyRFNM::getAntenna(const int direction, const size_t channel) const {
    return librfnm::rf_path_to_string(lrfnm->s->rx.ch[0].path);
}

void SoapyRFNM::setAntenna(const int direction, const size_t channel, const std::string& name) {
    lrfnm->s->rx.ch[0].path = librfnm::string_to_rf_path(name);
    setRFNM(LIBRFNM_APPLY_CH0_RX /*| LIBRFNM_APPLY_CH0_TX  | LIBRFNM_APPLY_CH1_RX*/);
}

SoapySDR::Stream* SoapyRFNM::setupStream(const int direction, const std::string& format,
        const std::vector<size_t>& channels, const SoapySDR::Kwargs& args) {
    lrfnm->s->rx.ch[0].enable = RFNM_CH_ON;
    //s->rx.ch[1].enable = RFNM_CH_ON;
    //s->tx.ch[0].enable = RFNM_CH_ON;

    setRFNM(LIBRFNM_APPLY_CH0_RX /*| LIBRFNM_APPLY_CH0_TX  | LIBRFNM_APPLY_CH1_RX*/);

    if (!format.compare(SOAPY_SDR_CF32)) {
        //m_outbuf.format = format;
        //m_outbuf.bytes_per_sample = SoapySDR_formatToSize(SOAPY_SDR_CF32);
        lrfnm->rx_stream(LIBRFNM_STREAM_FORMAT_CF32, &outbufsize);
    }
    else if (!format.compare(SOAPY_SDR_CS16)) {
        //m_outbuf.format = format;
        //m_outbuf.bytes_per_sample = SoapySDR_formatToSize(SOAPY_SDR_CS16);
        lrfnm->rx_stream(LIBRFNM_STREAM_FORMAT_CS16, &outbufsize);
    }
    else if (!format.compare(SOAPY_SDR_CS8)) {
        //m_outbuf.format = format;
        //m_outbuf.bytes_per_sample = SoapySDR_formatToSize(SOAPY_SDR_CS8);
        lrfnm->rx_stream(LIBRFNM_STREAM_FORMAT_CS8, &outbufsize);
    }
    else {
        throw std::runtime_error("setupStream invalid format " + format);
    }

    //lrfnm->tx_stream(LIBRFNM_STREAM_FORMAT_CS16, &inbufsize);

    std::queue<struct librfnm_tx_buf*> ltxqueue;
    //std::queue<struct librfnm_rx_buf*> lrxqueue;

    partial_rx_buf.buf = (uint8_t*)malloc(outbufsize);
    for (int i = 0; i < SOAPY_RFNM_BUFCNT; i++) {
        rxbuf[i].buf = (uint8_t*)malloc(outbufsize);
        lrfnm->rx_qbuf(&rxbuf[i]);
        //txbuf[i].buf = rxbuf[i].buf;
        //txbuf[i].buf = (uint8_t*)malloc(inbufsize);

        //ltxqueue.push(&txbuf[i]);
    }

    spdlog::info("outbufsize {} inbufsize {} ", outbufsize, inbufsize);

    return (SoapySDR::Stream*)this;
}

void SoapyRFNM::closeStream(SoapySDR::Stream* stream) {
    spdlog::info("RFNMDevice::closeStream() -> Closing stream");
}

int SoapyRFNM::readStream(SoapySDR::Stream* stream, void* const* buffs, const size_t numElems, int& flags,
        long long int& timeNs, const long timeoutUs) {
    double m_readstream_time_diff;

    size_t bytes_per_ele = lrfnm->s->transport_status.rx_stream_format;

    std::chrono::time_point<std::chrono::system_clock> m_readstream_start_time = std::chrono::system_clock::now();

    struct librfnm_rx_buf* lrxbuf;
    size_t read_elems = 0;

keep_waiting:
    if (partial_rx_buf.left) {
        size_t can_write_bytes = numElems * bytes_per_ele;
        if (can_write_bytes > partial_rx_buf.left) {
            can_write_bytes = partial_rx_buf.left;
        }

        std::memcpy(((uint8_t*)buffs[0]), partial_rx_buf.buf + partial_rx_buf.offset, can_write_bytes);
        read_elems += (can_write_bytes / bytes_per_ele);

        partial_rx_buf.left -= can_write_bytes;
        partial_rx_buf.offset += can_write_bytes;
    }

    while (read_elems < numElems && !lrfnm->rx_dqbuf(&lrxbuf, LIBRFNM_CH0 /* | LIBRFNM_CH1*/, 0)) {
        size_t overflowing_by_elems = 0;
        size_t can_copy_bytes = outbufsize;

        if ((read_elems + (outbufsize / bytes_per_ele)) > numElems) {

            overflowing_by_elems = (read_elems + (outbufsize / bytes_per_ele)) - numElems;
            can_copy_bytes = outbufsize - (overflowing_by_elems * bytes_per_ele);
        }

        std::memcpy(((uint8_t*)buffs[0]) + (bytes_per_ele * read_elems), lrxbuf->buf, can_copy_bytes);

        if (overflowing_by_elems) {
            std::memcpy(partial_rx_buf.buf, (lrxbuf->buf + can_copy_bytes), outbufsize - can_copy_bytes);
            partial_rx_buf.left = outbufsize - can_copy_bytes;
            partial_rx_buf.offset = 0;
        }

        lrfnm->rx_qbuf(lrxbuf);
        read_elems += (outbufsize / bytes_per_ele) - overflowing_by_elems;
    }

    if (read_elems < numElems) {
        m_readstream_time_diff = std::chrono::duration<double>(std::chrono::system_clock::now() - m_readstream_start_time).count();
        if (m_readstream_time_diff < ((double)timeoutUs) / 1000000.0) {
            goto keep_waiting;
        }
    }

    return read_elems;
}

void SoapyRFNM::setRFNM(uint16_t applies) {
    rfnm_api_failcode ret = lrfnm->set(applies);

    // GCC cannot pass references to values in packed structs, so we need stack copies
    uint64_t freq = lrfnm->s->rx.ch[0].freq;
    int8_t gain = lrfnm->s->rx.ch[0].gain;

    switch (ret) {
    case RFNM_API_OK:
        return;
    case RFNM_API_TUNE_FAIL:
        spdlog::error("Failure tuning to {} Hz", freq);
        throw std::runtime_error("Tuning failure");
    case RFNM_API_GAIN_FAIL:
        spdlog::error("Failure setting gain to {} dB", gain);
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
    std::vector<struct rfnm_dev_hwinfo> hwlist = librfnm::find(LIBRFNM_TRANSPORT_USB);
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
