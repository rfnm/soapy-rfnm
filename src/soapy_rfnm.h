#pragma once

#include <array>
#include <chrono>
#include <thread>
#include <string>

//#include <libusb-1.0/libusb.h>


#include "SoapySDR/Device.hpp"


#include "librfnm/librfnm.h"


#define SOAPY_RFNM_BUFCNF LIBRFNM_MIN_RX_BUFCNT

struct rfnm_soapy_partial_buf {
    uint8_t* buf;
    uint32_t left;
    uint32_t offset;
};


class SoapyRFNM : public SoapySDR::Device {
    // ctor (rfnm_construction.cpp)
public:
    explicit SoapyRFNM(const SoapySDR::Kwargs& args);

    ~SoapyRFNM() override;

public:
    [[nodiscard]] std::string getDriverKey() const override;

    [[nodiscard]] std::string getHardwareKey() const override;

    [[nodiscard]] SoapySDR::Kwargs getHardwareInfo() const override;
public:

    SoapySDR::Stream* setupStream(const int direction,
        const std::string& format,
        const std::vector<size_t>& channels = std::vector<size_t>(),
        const SoapySDR::Kwargs& args = SoapySDR::Kwargs());

    void closeStream(SoapySDR::Stream* stream);

    int activateStream(SoapySDR::Stream* stream, const int flags, const long long timeNs, const size_t numElems) override;

    int deactivateStream(SoapySDR::Stream* stream, const int flags0, const long long timeNs) override;

    int readStream(SoapySDR::Stream* stream, void* const* buffs, const size_t numElems, int& flags, long long& timeNs, const long timeoutUs);

    size_t getStreamMTU(SoapySDR::Stream* stream) const;

    size_t getNumChannels(const int direction) const;

    std::vector<double> listSampleRates(const int direction, const size_t channel) const;

    std::string getNativeStreamFormat(const int direction, const size_t channel, double& fullScale) const;

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

    void setFrequency(int direction, size_t channel, double frequency, const SoapySDR::Kwargs& args);
    void setGain(const int direction, const size_t channel, const double value);
    SoapySDR::Range getGainRange(const int direction, const size_t channel) const;

    void setBandwidth(const int direction, const size_t channel, const double bw);

    SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const;


    std::vector<std::string> listAntennas(const int direction, const size_t channel) const;
    void setAntenna(const int direction, const size_t channel, const std::string& name);


private:
    librfnm* lrfnm;

    int outbufsize;
    int inbufsize;

    struct librfnm_rx_buf rxbuf[SOAPY_RFNM_BUFCNF];
    struct librfnm_tx_buf txbuf[SOAPY_RFNM_BUFCNF];

    struct rfnm_soapy_partial_buf partial_rx_buf;
};
