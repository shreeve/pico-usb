## `RP2040:` USB Controller

### Memory Layout

The USB controller uses two blocks of memory.

The first block of memory
contains endpoint data buffers and consists of 4,096 bytes of DPSRAM (dual-port
static random access memory), beginning at address 0x5010000. The second block
of memory contains additional registers and consists of 156 bytes of standard
(non-DPSRAM) memory, beginning at address 0x50110000.

DPSRAM memory is different from most memory on the RP2040 because it:

* Can be accessed by the processor and USB controller at the same time
* Supports 8, 16, and 32 bit access (most registers only support 32-bit access)
* Does not support the usb_hw_set and usb_hw_clear aliases

In the two memory block tables below, **bold** indicates the minimal set of the
registers and buffers to be able to perform *USB device enumeration* in either
host or device mode.

### `HOST:` Memory Block 1: 0x50100000 - 0x50100FFF

| Offset | Bytes | Host             | Notes |
| ---    | ---   | ---              | ---   |
| 0x00   | 8     | **SETUP packet** | 8 bytes for setup packet     |
| 0x08   | 4     | ECR1             | ECR for 1st polled endpoint  |
| 0x10   | 4     | ECR2             | ECR for 2nd polled endpoint  |
| 0x18   | 4     | ECR3             | ECR for 3rd polled endpoint  |
| ...    | ...   | ...              | ... |
| 0x78   | 4     | ECR15            | ECR for 15th polled endpoint |
| 0x80   | 4     | **BCR**          | Buffer control register (BCR) for software endpoint    |
| 0x88   | 4     | BCR1             | BCR for 1st polled endpoint  |
| 0x90   | 4     | BCR2             | BCR for 2nd polled endpoint  |
| 0x98   | 4     | BCR3             | BCR for 3rd polled endpoint  |
| ...    | ...   | ...              | ... |
| 0x100  | 4     | **ECR**          | Endpoint control register (ECR) for software endpoint    |
| ...    | ...   | ...              | ... |
| 0x180  | 64    | **Buffer 1**     | Data for 1st data buffer     |
| 0x1c0  | 64    | Buffer 2         | Data for 2nd data buffer     |
| 0x200  | 64    | Buffer 3         | Data for 3rd data buffer     |
| ...    | ...   | ...              | ... |
| 0xFC0  | 64    | Buffer 58        | Data for 58th data buffer    |

### `DEVICE:` Memory Block 1: 0x50100000 - 0x50100FFF

| Offset | Bytes | Device           | Notes |
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
| 0xFC0  | 64    | Buffer 60        | Data for 60th data buffer    |

### Memory Block 2: 0x50110000 - 0x5011009B

| Offset | Host    | Notes |
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
