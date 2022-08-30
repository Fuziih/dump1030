# dump1030

Dump1030 is C++ program capable of detecting different types of SSR
interrogations. Seven types of messages can be detected: Mode S, Mode A, Mode A all-call, Mode A all-call (Compatibility mode), Mode C, Mode C all-call and Mode C all-call (Compatibility mode).

Dump1030 is based on the popular dump1090, especially antirez's implementation (https://github.com/antirez/dump1090).

## Installation

Clone and compile with `make`.

## Requirements

Linux computer with RTL-SDR radio capable of receiving 2.5 MSPS at 1030 mhz stable.

## Usage

The binary can be ran with default settings.

Command line arguments:
```
--device           Input rtl-sdr device index
--file             Input location and full name of test file that is used instead of rtl-sdr device
--gain             Input desired gain level for rtl-sdr device
--agc              Enable automatic gain control by RTL-SDR device
--diff             Add minimum amplitude difference between pulses and non pulses that are not right next to pulse values
--diffclose        Add minimum amplitude difference between pulses and non pulses right next to pulses
--diffratio        Add minimum ratio of non pulse amplitude divided by pulse amplitude for every non pulse value not right next to pulse value (Doesn't apply to checking p4 pulse existence)
--diffratioclose   Add minimum ratio of non pulse amplitude divided by pulse amplitude for every non pulse value right next to pulse
--diffratiop4      Same as diffratio but only for p4 check (Mode a/c only all-call and Mode a/c/s all-call (Compatibility mode) messages)
Compares only p4 pulse values with non pulse values that are after p3 but not next to pulse values.
--diffratioclosep4 Same as diffratio but only for p4 check (Mode a/c only all-call and Mode a/c/s all-call (Compatibility mode) messages)
Compares only p4 pulse values with non pulse values that are after p3 and next to pulse values.
--msgs             Show every recognized messages location and amplitude values in the message
--order            Print order of different SSR interrogation signals
--mpa              Minimum accepted pulse amplitude when there should be a pulse
--mnf              Maximum allowed noicefloor amplitude when there shouldn't be a pulse
--mnfc             Maximum allowed noicefloor amplitude for non pulse values next to pulse values.
--size             Defines size of read message in bytes when using rtl-sdr. Must be at least 512 otherwise uses default size of 262144.
--blmode           Outputs baseline values for mpa, mnf and mnfc based on accepted averages. Can be used to get baseline values based on earlier detected messages averages that can be set for detecting next messages.
--print            Print all captured amplitude data.
--continuous       Keeps detecting and reporting messages continuously. Size parameter sets update interval.
--help             Show help
```

All arguments might not be compatible with each other.


