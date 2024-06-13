#pragma once

#include <array>
#include <chrono>
#include <thread>
#include <string>

//#include <libusb-1.0/libusb.h>


#include "SoapySDR/Device.hpp"


#include "librfnm/librfnm.h"


#define SOAPY_RFNM_BUFCNT LIBRFNM_MIN_RX_BUFCNT

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
    void setDCOffsetMode(const int dir, const size_t channel, const bool automatic) override;
    bool getDCOffsetMode(const int dir, const size_t channel) const override;

private:
    void setRFNM(uint16_t applies);

    bool dc_correction = false;
    union rfnm_quad_dc_offset dc_offsets = {};

    librfnm* lrfnm;

    int outbufsize;
    int inbufsize;

    struct librfnm_rx_buf rxbuf[SOAPY_RFNM_BUFCNT];
    //struct librfnm_tx_buf txbuf[SOAPY_RFNM_BUFCNT];

    struct rfnm_soapy_partial_buf partial_rx_buf;
};
