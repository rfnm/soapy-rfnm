# soapy-rfnm

SoapySDR driver module for [RFNM](https://rfnm.com) software-defined radios, built on
**librfnm v2**. This v2 rewrite is `main`; it pairs with librfnm's `main` (the v2 line) —
build the two together. The pre-v2 driver is preserved on the `legacy` branch.

The module registers the `rfnm` driver with SoapySDR, so anything that speaks SoapySDR
(SoapySDRUtil, CubicSDR, SDR++ via the Soapy source, GQRX, python bindings, ...) can use
an RFNM device.

## What it provides

- Device discovery + open over every librfnm transport: `driver=rfnm` with optional
  `transport=usb|eth|local` and `serial=`/address args (e.g.
  `SoapySDRUtil --find="driver=rfnm"`, `--make="driver=rfnm,transport=eth,addr=192.168.1.50"`).
- RX streaming in CF32 and CS16 via librfnm's `rx_stream`.
- Runtime control: frequency, gain, antenna (RF path), analog filter bandwidth, and
  RX conveniences like the FM notch, mapped to the standard SoapySDR setters.
- Sample-rate listing/setting from the device's advertised rate set (a rate change is a
  multi-second reclock of the baseband processor — expect the stream to restart).

## Requirements

- SoapySDR development files (found via CMake; fetched and built automatically if absent)
- librfnm >= 0.2.0 installed with its pkg-config file (`librfnm.pc`) — use `main`
  to match this driver
- spdlog (fetched automatically via CPM if not installed)

## Build

```sh
cmake -B build
cmake --build build -j
sudo cmake --install build     # installs into SoapySDR's modules directory
SoapySDRUtil --find="driver=rfnm"
```

With the RFNM SDK ([github.com/rfnm/sdk](https://github.com/rfnm/sdk)),
`./rfnm-sdk build soapy-rfnm` checks out the matching branches and builds the module
against its own librfnm.

## License

MPL-2.0, see `LICENSE`.
