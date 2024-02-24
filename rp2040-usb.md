## `RP2040:` USB Controller

### Memory Layout

The USB controller uses two blocks of memory.

The first block of memory contains endpoint data buffers and consists of 4,096
bytes of DPSRAM (dual-port static random access memory), beginning at address
0x5010000. The second block of memory contains additional registers and consists
of 156 bytes of standard (non-DPSRAM) memory, beginning at address 0x50110000.

DPSRAM memory is different from most memory on the RP2040 because it:

* Can be accessed by the processor and USB controller at the same time
* Supports 8, 16, and 32 bit access (most registers only support 32-bit access)
* Does not support the usb_hw_set and usb_hw_clear aliases

In the two memory block tables below, **bold** indicates the minimal set of the
registers and buffers to be able to perform *USB device enumeration* in either
host or device mode.

### `HOST:` Memory Block 1: 0x50100000 - 0x50100FFF

| Offset | Bytes | Host             | Info  |
| ---    | ---   | ---              | ---   |
| 0x00   | 8     | **SETUP packet** | 8 bytes for setup packet     |
| 0x08   | 4     | ECR1             | ECR for 1st polled endpoint  |
| 0x10   | 4     | ECR2             | ECR for 2nd polled endpoint  |
| 0x18   | 4     | ECR3             | ECR for 3rd polled endpoint  |
| ...    | ...   | ...              | ... |
| 0x78   | 4     | ECR15            | ECR for 15th polled endpoint |
| 0x80   | 4     | **BCR**          | Buffer control register (BCR) for software endpoint |
| 0x88   | 4     | BCR1             | BCR for 1st polled endpoint  |
| 0x90   | 4     | BCR2             | BCR for 2nd polled endpoint  |
| 0x98   | 4     | BCR3             | BCR for 3rd polled endpoint  |
| ...    | ...   | ...              | ... |
| 0x100  | 4     | **ECR**          | Endpoint control register (ECR) for software endpoint |
| ...    | ...   | ...              | ... |
| 0x180  | 64    | **Buffer 1**     | Data for 1st data buffer     |
| 0x1c0  | 64    | Buffer 2         | Data for 2nd data buffer     |
| 0x200  | 64    | Buffer 3         | Data for 3rd data buffer     |
| ...    | ...   | ...              | ... |
| 0xFC0  | 64    | Buffer 58        | Data for 58th data buffer    |

### `DEVICE:` Memory Block 1: 0x50100000 - 0x50100FFF

| Offset | Bytes | Device           | Info  |
| ---    | ---   | ---              | ---   |
| 0x00   | 8     | **SETUP packet** | 8 bytes for setup packet |
| 0x08   | 4     | ECR1/IN          | ECR for EP1/IN   |
| 0x10   | 4     | ECR1/OUT         | ECR for EP1/OUT  |
| 0x18   | 4     | ECR2/IN          | ECR for EP2/IN   |
| 0x20   | 4     | ECR2/OUT         | ECR for EP2/OUT  |
| ...    | ...   | ...              | ... |
| 0x78   | 4     | ECR15/IN         | ECR for EP15/IN  |
| 0x7C   | 4     | ECR15/OUT        | ECR for EP15/OUT |
| 0x80   | 4     | **BCR0/IN**      | BCR for EP0/IN   |
| 0x84   | 4     | **BCR0/OUT**     | BCR for EP0/OUT  |
| 0x88   | 4     | BCR1/IN          | BCR for EP1/IN   |
| 0x8C   | 4     | BCR1/OUT         | BCR for EP1/OUT  |
| ...    | ...   | ...              | ... |
| 0x100  | 64    | **EP0B**         | EP0 buffer 0 (shared between IN and OUT) |
| 0x140  | 64    | EP0B1            | EP0 buffer 1 (if double buffering on EP0) |
| 0x180  | 64    | Buffer 3         |     |
| ...    | ...   | ...              | ... |
| 0xFC0  | 64    | Buffer 60        | Data for 60th data buffer |

### Memory Block 2: 0x50110000 - 0x5011009B

| Offset | Host    | Info  |
| ---    | ---     | ---   |
| 0x00 | ADDR_ENDP (**DAR**) | Device and endpoint address (DAR) register for software endpoint |
| 0x04 | ADDR_ENDP1 | DAR for 1st polled endpoint |
| 0x08 | ADDR_ENDP2 | DAR for 2nd polled endpoint |
| 0x0c | ADDR_ENDP3 | DAR for 3rd polled endpoint |
| ...  | ...  | ... |
| 0x3C | ADDR_ENDP15 | DAR for 15th polled endpoint |
| 0x40 | **MAIN_CTRL** | Main control register |
| 0x44 | SOF_WR | `HOST:` Set the frame number for the next SOF (Start of Frame) sent by the host controller. SOF packets are sent every 1ms and the host will increment the frame number by 1 each time. |
| 0x48 | SOF_RD | `DEVICE:` Read the frame number from the last SOF (Start of Frame) received by the device controller. |
| 0x4C | SIE_CTRL (**SCR**) | SIE control register |
| 0x50 | SIE_STATUS (**SSR**) | SIE status register |
| 0x54 | INT_EP_CTRL | Polled endpoint control register |
| 0x58 | **BUFF_STATUS** | Buffer status register. When an endpoint enables an interrupt for its buffer status, this register will be set each time that buffer completes. |
| 0x5C | BUFF_CPU_SHOULD_HANDLE | When an endpoint is set for double buffering and sends an interrupt per buffer completion, this register toggles between 0 for buf_0 and 1 for buf_1 to indicate the buffer to handle. |
| 0x60 | EP_ABORT | `DEVICE:` For each bit in this register, a NAK will be sent for every access to the corresponding endpoint, overriding the BCR for that endpoint. |
| 0x64 | EP_ABORT_DONE | `DEVICE:` Used in conjunction with EP_ABORT. Set once an endpoint is idle, so the programmer knows it is safe to modify the BCR. |
| 0x68 | EP_STALL_ARM | `DEVICE:` This bit must be set in conjunction with the STALL bit in the BCR to send a STALL on EP0. The controller clears these bits when a SETUP packet is received. |
| 0x6C | NAK_POLL | `HOST:` If a device replies with a NAK, this value indicates the number of microseconds (μs) before trying again. |
| 0x70 | EP_STATUS_STALL_NAK | `DEVICE:` Each bit indicates that the corresponding endpoint has a NAK or STALL condition. EP0 is enabled via SIE_CTRL and all other endpoints are enabled in their ECR. |
| 0x74 | **USB_MUXING** | Controls where the USB controller is connected (usually `to_phy`). |
| 0x78 | **USB_PWR** | Controls VBUS and overcurrent configuration. |
| 0x7C | USBPHY_DIRECT | In conjunction with USBPHY_DIRECT_OVERRIDE, this register allows direct control of the USB Phy. |
| 0x80 | USBPHY_DIRECT_OVERRIDE | Enables overrides for each control in USBPHY_DIRECT. |
| 0x84 | USBPHY_TRIM | Used to adjust trim values for USB Phy pull down resistors. |
| 0x88 | Unknown | Unknown |
| 0x8C | INTR | Interrupt status (show all) |
| 0x90 | **INTE** | Enable interrupts |
| 0x94 | INTF | Forced interrupts |
| 0x98 | **INTS** | Interrupt status (masked or forced) |

### Endpoint control register (ECR)

The endpoint control register (ECR) controls how the endpoint behaves.

| Bits  | Info  |
| ---   | ---   |
| 31    | Enable this endpoint |
| 30    | 0: single buffer (64 bytes), 1: double buffering (64 bytes x 2)|
| 29    | Raise an interrupt for every buffer transferred |
| 28    | Raise an interrupt for every 2 buffers transferred (only for double buffering) |
| 27:26 | Type of endpoint, 0: control, 1: isochronous, 2: bulk, 3: interrupt |
| 25:18 | Polling interval minus 1ms (14 means poll every 15ms) |
| 15:0  | Byte offset for DPSRAM data buffer (lowest 6 bits must be zero) |

### Buffer control register (BCR)

The buffer control register (BCR) controls how to deal with single or double
buffering for each endpoint. If the endpoint is configured for single buffering,
then only the low bits (15:0) are used and they refer to the first buffer
(buffer 0). If the endpoint is configured for double buffering, then the high
bits (31:16) refer to the second/double buffer (buffer 1). Notice that each 16
bit half of this register is not exactly the same.

This register is also special because the upper 16 bits and lower 16 bits can be
read or written to independently. If this endpoint is single buffered, this has
no impact. But, if the endpoint is double buffered, then care should be taken to
write each 16 bit half of the register independently. In practice, this means it
is better to consider this 32 bit register as two separate 16 bit registers when
writing to it.

| Bits   | Info  |
| ---    | ---   |
| *High* | *Only used if double buffering and refers only to the second/double buffer (buffer 1)* |
| 31     | FULL buffer. `HOST:` 0: buffer is empty, ready to receive an IN packet from the device; 1: buffer is full, ready to send an OUT packet to the device. `DEVICE:` 0: buffer is empty, ready to receive an OUT packet from the host; 1: buffer is full, ready to send an IN packet to the host.
| 30     | LAST buffer in the transfer. If INTE has TRANS_COMPLETE set (bit 3), then the TRANS_COMPLETE interrupt will be raised when this buffer completes. |
| 29     | DATA PID for this packet. 0: DATA0, 1: DATA1. |
| 28:27  | `Isochronous endpoints:` Byte offset of this second buffer relative to the first buffer, 0: 128, 1: 256, 3: 1024. |
| 26     | Buffer is AVAILABLE for, and now owned by, the controller. 0: controller clears this bit to indicate that the processer now owns this buffer, 1: processor sets this bit to indicate that the controller now owns this buffer. |
| 25:16  | Buffer length for this buffer. |

| Bits   | Info  |
| ---    | ---   |
| *Low*  | *Used for the first buffer (buffer 0) of this endpoint* |
| 15     | FULL buffer. `HOST:` 0: buffer is empty, ready to receive an IN packet from the device; 1: buffer is full, ready to send an OUT packet to the device. `DEVICE:` 0: buffer is empty, ready to receive an OUT packet from the host; 1: buffer is full, ready to send an IN packet to the host.
| 14     | LAST buffer in the transfer. If INTE has TRANS_COMPLETE set (bit 3), then the TRANS_COMPLETE interrupt will be raised when this buffer completes. |
| 13     | DATA PID for this packet. 0: DATA0, 1: DATA1. |
| 12     | `DEVICE:` Reset buffer select to buffer 0. |
| 11     | `HOST:` Received a STALL. `DEVICE:` Send a STALL. |
| 10     | Buffer is AVAILABLE for, and now owned by, the controller. 0: controller clears this bit to indicate that the processer now owns this buffer, 1: processor sets this bit to indicate that the controller now owns this buffer. |
| 9:0    | Buffer length for this buffer. |
