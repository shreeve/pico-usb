# NOTE: Repo has moved to:

* https://github.com/shreeve/picousb

# PicoUSB

A very small USB library (without TinyUSB) for the rp2040.

## Summary

The rp2040 chip in the Raspberry Pi Pico/W boards supports USB Host and
USB Device modes. Most examples and implementations use the excellent
[TinyUSB](https://github.com/hathach/tinyusb) library to provide this support. However, TinyUSB has some
limitations. These include a somewhat hard coded initial configuration
that requires the code to be recompiled when changes are made.

## Goals

The goal of this repo is to offer a simple USB Device example and a
simple USB Host example, without using any code from TinyUSB. There
is currently an example in the [pico-examples](https://github.com/raspberrypi/pico-examples/tree/master/usb/device/dev_lowlevel) repository that shows
how to implement a basic low level USB Device without TinyUSB. I would
like to streamline this USB Device example and also add a USB Host
example to show how to implement basic versions of each of these.

Ideally, this could lead to a barebones and simplified USB library for
rp2040 based projects. Since the initial target is for the Pico/W
boards and since the goal is to be tiny, even "tinier" than TinyUSB,
it seems the name pico-usb is fitting.

## Contents

There is a directory called `device` which contains the simplified
USB Device example and a directory called `host` which contains
the simplified USB Host example. Hopefully, the contents of these
two directories will be very similar and have as similar code as
possible.

## Status

Currently, the code in `device` is a streamlined version
of the low level USB device from [pico-examples](https://github.com/raspberrypi/pico-examples/tree/master/usb/device/dev_lowlevel).
It's a simple example that will echo anything received on EP1 to EP2.
The CMakeLists.txt files in this repo are written to be very minimal.
They work in VScode when opening these two directories. There is probably
a way to create a top-level CMakeLists.txt file so that each directory
doesn't need one, if someone can submit a way to open the top-level
directory and build either directory that would be great. The code
in `host` now can enumerate a device and supports double buffering.
Work is being done now to add support for hubs and then a CDC class
driver.

## TODO

- [x] Initial version of [`device.c`](https://github.com/shreeve/pico-usb/blob/f6c648e3a4bfbfedd53296ae70b41596cf719e3e/device/device.c) (a simpler [`dev_lowlevel.c`](https://github.com/raspberrypi/pico-examples/tree/master/usb/device/dev_lowlevel) from `pico-examples`).
- [x] Initial version of `host.c` that just gets something/anything from a device.
- [x] Second version of `host.c` that at least can fully enumerate a device.
- [x] Add double-buffering support to `host.c`
- [ ] Improved `device.c` with larger transfer size (more than 1 packet of 64 bytes).
- [ ] Add double-buffering support to `device.c`

## Disable TinyUSB

Since this repo doesn't use TinyUSB at all, we need to disable it from the
pico-sdk code. The only way I found to do this is by commenting out the
following lines in `${PICO_SDK_PATH}/src/rp2_common/CMakeLists.txt`.

```
# pico_add_subdirectory(tinyusb)
# pico_add_subdirectory(pico_stdio_usb)
```

## Compiling

Since we are taking over the physical USB connector on the board, the
easiest way to program the board and test things out is by using a
picodebug unit. These can be purchased from numerous sources. The .vscode
directory contains some configuration for this setup.

## Using Wireshark on macOS

```
# Reboot your mac in Recovery Mode (instructions online) and launch a
# terminal. Run the following command to disable SPI. Afterwards, reboot.
$ csrutil disable

# Reboot as normal and find your USB controller name (mine is "XHC2")
$ tcpdump -D

1.ap1 [Up, Running, Wireless, Association status unknown]
2.en1 [Up, Running, Wireless, Associated]
3.awdl0 [Up, Running, Wireless, Associated]
4.llw0 [Up, Running, Wireless, Not associated]
5.utun0 [Up, Running]
6.utun1 [Up, Running]
7.utun2 [Up, Running]
8.utun3 [Up, Running]
9.lo0 [Up, Running, Loopback]
10.anpi0 [Up, Running, Disconnected]
11.anpi1 [Up, Running, Disconnected]
12.en0 [Up, Running, Disconnected]
13.en4 [Up, Running, Disconnected]
14.en5 [Up, Running, Disconnected]
15.en2 [Up, Running, Disconnected]
16.en3 [Up, Running, Disconnected]
17.bridge0 [Up, Running, Disconnected]
18.gif0 [none]
19.stf0 [none]
20.XHC0 [none]
21.XHC1 [none]
22.XHC2 [none]

# Enable capturing USB on the XHC2 interface
sudo ifconfig XHC2 up

# Launch Wireshark and inspect USB traffic on the XHC2 interface
```

## Host Example

The current status, as of the end of February 2024, showing the output from [enumeration of a Pico Debug probe](https://github.com/shreeve/pico-usb/blob/main/enumeration.md).

## License

BSD-3-Clause license, the same as code in [pico-examples](https://github.com/raspberrypi/pico-examples/tree/master/usb/device/dev_lowlevel).
