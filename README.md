# pico-usb

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
in `host` is being put together now, based on trying to read the spec
from the datasheet. If anyone has some minimal code for USB Host, it
would be great to add it to build out the USB Host example.

## TODO

- [x] Initial version of [`device.c`](https://github.com/shreeve/pico-usb/blob/f6c648e3a4bfbfedd53296ae70b41596cf719e3e/device/device.c) (a simpler [`dev_lowlevel.c`](https://github.com/raspberrypi/pico-examples/tree/master/usb/device/dev_lowlevel) from `pico-examples`).
- [ ] Improved `device.c` with larger packet size (more than 64 bytes).
- [ ] Initial version of `host.c` (I can't find a good reference example).
- [ ] Add double-buffering support to `device.c`
- [ ] Add double-buffering support to `host.c`

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

## License

BSD-3-Clause license, the same as code in [pico-examples](https://github.com/raspberrypi/pico-examples/tree/master/usb/device/dev_lowlevel).
