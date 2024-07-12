#pragma once

#include <array>
#include <chrono>
#include <thread>
#include <string>

//#include <libusb-1.0/libusb.h>


#include "SoapySDR/Device.hpp"


#include "librfnm/librfnm.h"


#define SOAPY_RFNM_BUFCNT LIBRFNM_MIN_RX_BUFCNT
#define MAX_RX_CHAN_COUNT 4

struct rfnm_soapy_partial_buf {
    uint8_t* buf;
    uint32_t left;
    uint32_t offset;
};

union rfnm_quad_dc_offset {
    int8_t i8[8];
    int16_t i16[8];
    float f32[8];
};

class SoapyRFNM : public SoapySDR::Device {
public:
    explicit SoapyRFNM(const SoapySDR::Kwargs& args);

    ~SoapyRFNM();

    [[nodiscard]] std::string getDriverKey() const override;

    [[nodiscard]] std::string getHardwareKey() const override;

    [[nodiscard]] SoapySDR::Kwargs getHardwareInfo() const override;

    SoapySDR::Kwargs getChannelInfo(const int direction, const size_t channel) const override;

    // Stream API
    SoapySDR::Stream* setupStream(const int direction,
        const std::string& format,
        const std::vector<size_t>& channels = std::vector<size_t>(),
        const SoapySDR::Kwargs& args = SoapySDR::Kwargs()) override;

    void closeStream(SoapySDR::Stream* stream) override;

    int activateStream(SoapySDR::Stream* stream, const int flags, const long long timeNs,
        const size_t numElems) override;

    int deactivateStream(SoapySDR::Stream* stream, const int flags0, const long long timeNs) override;

    int readStream(SoapySDR::Stream* stream, void* const* buffs, const size_t numElems, int& flags,
        long long& timeNs, const long timeoutUs) override;

    size_t getStreamMTU(SoapySDR::Stream* stream) const override;

    size_t getNumChannels(const int direction) const override;

    std::string getNativeStreamFormat(const int direction, const size_t channel, double& fullScale) const override;

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const override;

    // Sample Rate API
    std::vector<double> listSampleRates(const int direction, const size_t channel) const override;
    double getSampleRate(const int direction, const size_t channel) const override;
    void setSampleRate(const int direction, const size_t channel, const double rate) override;

    // Frequency API
    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const override;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const override;
    double getFrequency(const int direction, const size_t channel, const std::string &name) const override;
    void setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency,
            const SoapySDR::Kwargs& args) override;

    // Gain API
    std::vector<std::string> listGains(const int direction, const size_t channel) const override;
    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const override;
    double getGain(const int direction, const size_t channel, const std::string &name) const override;
    void setGain(const int direction, const size_t channel, const std::string &name, const double value) override;

    // Bandwidth API
    SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const override;
    double getBandwidth(const int direction, const size_t channel) const override;
    void setBandwidth(const int direction, const size_t channel, const double bw) override;

    // Antenna API
    std::vector<std::string> listAntennas(const int direction, const size_t channel) const override;
    std::string getAntenna(const int direction, const size_t channel) const override;
    void setAntenna(const int direction, const size_t channel, const std::string& name) override;

    // DC Offset API
    bool hasDCOffsetMode(const int direction, const size_t channel) const override;
    void setDCOffsetMode(const int direction, const size_t channel, const bool automatic) override;
    bool getDCOffsetMode(const int direction, const size_t channel) const override;

    // Channel Settings API
    SoapySDR::ArgInfoList getSettingInfo(const int direction, const size_t channel) const override;
    std::string readSetting(const int direction, const size_t channel, const std::string &key) const override;
    void writeSetting(const int direction, const size_t channel, const std::string &key, const std::string &value) override;

private:
    void setRFNM(uint16_t applies);

    rfnm_api_failcode rx_dqbuf_multi(uint32_t wait_for_ms);
    void rx_qbuf_multi();

    size_t rx_chan_count = 0;
    bool dc_correction[MAX_RX_CHAN_COUNT] = {false};
    union rfnm_quad_dc_offset dc_offsets[MAX_RX_CHAN_COUNT] = {};

    librfnm* lrfnm;

    bool stream_setup = false;
    int outbufsize = 0;
    //int inbufsize = 0;

    uint64_t sample_counter[MAX_RX_CHAN_COUNT] = {};
    double ns_per_sample[MAX_RX_CHAN_COUNT] = {};
    uint32_t last_phytimer[MAX_RX_CHAN_COUNT] = {};
    uint32_t phytimer_ticks_per_sample[MAX_RX_CHAN_COUNT] = {};

    struct librfnm_rx_buf rxbuf[SOAPY_RFNM_BUFCNT] = {};
    //struct librfnm_tx_buf txbuf[SOAPY_RFNM_BUFCNT];

    struct rfnm_soapy_partial_buf partial_rx_buf[MAX_RX_CHAN_COUNT] = {};

    struct librfnm_rx_buf * pending_rx_buf[MAX_RX_CHAN_COUNT] = {};
};
