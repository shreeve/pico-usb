// ============================================================================
// PicoUSB - A USB host and device library for the Raspberry Pi Pico/W
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: January 29, 2024
//   Note: This is a work in progress. It is not yet functional.
//  Legal: Same license as the Raspberry Pi Pico SDK
//
// Thanks to Ha Thach TinyUSB for https://github.com/hathach/tinyusb
// Thanks to Miroslav Nemecek for https://github.com/Panda381/PicoLibSDK
// ============================================================================

#include <stdio.h>                // For printf
#include <string.h>               // For memcpy

#include "pico/stdlib.h"          // Pico stdlib
#include "pico/util/queue.h"      // Multicore and IRQ safe queue
#include "hardware/regs/usb.h"    // USB hardware registers from pico-sdk
#include "hardware/structs/usb.h" // USB hardware structs from pico-sdk
#include "hardware/irq.h"         // Interrupts and definitions
#include "hardware/resets.h"      // Resetting the native USB controller

#include "usb_common.h"           // USB 2.0 definitions
#include "helpers.h"              // Helper functions

// ==[ PicoUSB ]===============================================================

#define MAX_HUBS      0
#define MAX_DEVICES   2
#define MAX_ENDPOINTS 4
#define TEMP_BUF_SIZE 256

#define MAKE_U16(x, y) (((x) << 8) | ((y)     ))
#define SWAP_U16(x)    (((x) >> 8) | ((x) << 8))

#define SDK_ALIGNED(bytes) __attribute__ ((aligned(bytes)))
#define SDK_INLINE         __attribute__ ((always_inline)) static inline
#define SDK_NOINLINE       __attribute__ ((noinline))
#define SDK_PACKED         __attribute__ ((packed))
#define SDK_WEAK           __attribute__ ((weak))

#define memclr(ptr, len) memset((ptr), 0, (len))
#define nop() __asm volatile("nop" ::: "memory")

#define usb_hw_clear ((usb_hw_t *) hw_clear_alias_untyped(usb_hw))
#define usb_hw_set   ((usb_hw_t *) hw_set_alias_untyped  (usb_hw))

SDK_INLINE uint8_t get_speed() {
    return (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) \
                              >> USB_SIE_STATUS_SPEED_LSB;
}

SDK_INLINE bool is_host_mode() {
    return (usb_hw->main_ctrl & USB_MAIN_CTRL_HOST_NDEVICE_BITS);
}

SDK_INLINE uint8_t line_state() {
    return (usb_hw->sie_status & USB_SIE_STATUS_LINE_STATE_BITS) \
                              >> USB_SIE_STATUS_LINE_STATE_LSB;
}

// ==[ Endpoints ]=============================================================

typedef void (*endpoint_c)(uint8_t *buf, uint16_t len);

typedef struct endpoint {
    uint8_t    dev_addr  ; // Device address // HOST ONLY
    uint8_t    ep_addr   ; // Endpoint address
    uint8_t    type      ; // Transfer type: control/bulk/interrupt/isochronous
    uint16_t   maxsize   ; // Maximum packet size
    uint16_t   interval  ; // Polling interval in ms
    bool       configured; // Endpoint is configured
    bool       active    ; // Transfer is active
    uint8_t    data_pid  ; // Toggle between DATA0/DATA1 packets
    volatile               // Data buffer is volative
    uint8_t   *data_buf  ; // Data buffer in DPSRAM
    uint8_t   *user_buf  ; // User buffer in RAM or flash
    uint16_t   bytes_left; // Bytes remaining
    uint16_t   bytes_done; // Bytes transferred
    endpoint_c cb        ; // Callback function
} endpoint_t;

static endpoint_t eps[MAX_ENDPOINTS], *epx = eps;

static uint8_t temp_buf[TEMP_BUF_SIZE];

SDK_INLINE const char *ep_dir(endpoint_t *ep) {
    return ep->ep_addr & USB_DIR_IN ? "IN" : "OUT";
}

SDK_INLINE bool ep_in(endpoint_t *ep) {
    return ep->ep_addr & USB_DIR_IN;
}

SDK_INLINE uint8_t ep_num(endpoint_t *ep) {
    return ep->ep_addr & ~USB_DIR_IN;
}

SDK_INLINE void clear_endpoint(endpoint_t *ep) {
    ep->active     = false;
    ep->user_buf   = temp_buf; // TODO: Add something like a ring buffer here?
    ep->bytes_left = 0;
    ep->bytes_done = 0;
}

void show_endpoint(endpoint_t *ep, const char *str) {
    printf(" EP%d_%-3s│ 0x%02x │ Device %u (%s)\n",
             ep_num(ep), ep_dir(ep), ep->ep_addr, ep->dev_addr, str);
}

void setup_endpoint(endpoint_t *ep, usb_endpoint_descriptor_t *usb) {

    // Populate the endpoint
    *ep = (endpoint_t) {
        .dev_addr   = ep->dev_addr,
        .ep_addr    = usb->bEndpointAddress,
        .type       = usb->bmAttributes,
        .maxsize    = usb->wMaxPacketSize,
        .interval   = usb->bInterval,
        .configured = true,
        .active     = false,
        .data_pid   = 0,
        .data_buf   = usbh_dpram->epx_data,
        .user_buf   = temp_buf,
        .bytes_left = 0,
        .bytes_done = 0,
        .cb         = NULL,
    };

    // We're done unless this is EPX or an interrupt endpoint
    if (ep != epx && ep->type != USB_TRANSFER_TYPE_INTERRUPT) {
        show_endpoint(ep, "Reset");
        return;
    }

    // Helper variables
    uint32_t type   = ep->type;
    uint32_t ms     = ep->interval;
    uint32_t lsb    = EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB;
    uint32_t offset = offsetof(usb_host_dpram_t, epx_data); // TODO: Make this generic, not EPX specific

    // Get the ECR
    uint32_t ecr = EP_CTRL_ENABLE_BITS             // Enable endpoint
                 | EP_CTRL_INTERRUPT_PER_BUFFER    // An interrupt per buffer
                 | type << EP_CTRL_BUFFER_TYPE_LSB // Set transfer type
                 | (ms ? ms - 1 : 0) << lsb        // Interrupt polling in ms
                 | offset;                         // Data buffer offset

    // Debug output
    show_endpoint(ep, "Reset");
    bindump(" ECR", ecr);

    // Set the ECR
    usbh_dpram->epx_ctrl = ecr;
}

SDK_INLINE void setup_epx() {
    setup_endpoint(epx, &((usb_endpoint_descriptor_t) {
        .bLength          = sizeof(usb_endpoint_descriptor_t),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = 0,
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 8, // Default per USB 2.0 spec
     // .wMaxPacketSize   = 64,
        .bInterval        = 0,
    }));
}

void reset_endpoints() {

    // Clear out all endpoints
    memclr(eps, sizeof(eps));

    // Allocate the endpoints
    setup_epx();
}

endpoint_t *find_endpoint(uint8_t dev_addr, uint8_t ep_addr) {
    bool want_ep0 = !(ep_addr & ~USB_DIR_IN);

    for (uint8_t i = 0; i < MAX_ENDPOINTS; i++) {
        endpoint_t *ep = &eps[i];
        if (ep->configured && ep->dev_addr == dev_addr) {
            if (want_ep0 && !(ep->ep_addr & ~USB_DIR_IN)) return ep;
            if (ep->ep_addr == ep_addr) return ep;
        }
    }
    panic("Invalid endpoint 0x%02x for device %u", ep_addr, dev_addr);
    return NULL;
}

// Allocate the next endpoint
endpoint_t *next_ep(uint8_t dev_addr, usb_endpoint_descriptor_t *usb) {
    endpoint_t *ep = NULL;

    for (uint8_t i = 1; i < MAX_ENDPOINTS; i++) {
        ep = &eps[i];
        if (!ep->configured) {
            ep->dev_addr = dev_addr;
            setup_endpoint(ep, usb);
            ep->configured = true;
            return ep;
        }
    }
    panic("No free endpoints remaining"); // TODO: Handle this properly
    return NULL;
}

// ==[ Buffers ]===============================================================

void handle_buffer(endpoint_t *ep) {
    if (!ep->active) show_endpoint(ep, "Inactive"), panic("Halted");

    // -- Sync current buffer ---------------------------------------------------

    // Workaround for RP2040-E4
    uint32_t bcr = usbh_dpram->epx_buf_ctrl;      // Buffer control register
    uint32_t bch = usb_hw->buf_cpu_should_handle; // Check for CPU handle bits
    if (bch & 1u) bcr >>= 16;                     // Perform bitshift correction // TODO: Process all affected buffers

    uint16_t len = bcr & USB_BUF_CTRL_LEN_MASK;   // Buffer length
    bool    full = bcr & USB_BUF_CTRL_FULL;       // Is buffer marked as full?
    bool      in = ep_in(ep);                     // IN or OUT endpoint?

    // Inbound buffers must be full and outbound buffers must be empty
    assert(in == full);

    // Copy the inbound data buffer to the user buffer
    if (in) {
        memcpy(ep->user_buf, (void *) ep->data_buf, len);
        ep->user_buf += len;
    }

    // Update byte counts
    ep->bytes_done += len;
    ep->bytes_left -= len;

    // Short packet (below maxsize) means the transfer is done
    if (len < ep->maxsize) {
        ep->bytes_left = 0;
    }

    // -- Debug output --------------------------------------------------------

    if (ep->bytes_done) {
        hexdump("│Data", usbh_dpram->epx_data, ep->bytes_done, 1);
    } else {
        char *str = ep_in(ep) ? "│ZLP/I" : "│ZLP/O";
        bindump(str, 0);
    }

    // Toggle DATA0/DATA1 each packet
    ep->data_pid ^= 1;

    // -- Prepare next buffer ---------------------------------------------------

    if (ep->bytes_left) {

        // Update byte counts
        len = MIN(ep->maxsize, ep->bytes_left);

        // Calculate new BCR
        uint8_t pid = ep->data_pid;
        bool mas = ep->bytes_left > ep->maxsize; // Are there more packets?
        bcr = (in  ? 0 : USB_BUF_CTRL_FULL)      // IN/Recv=0, OUT/Send=1
            | (mas ? 0 : USB_BUF_CTRL_LAST)      // Trigger TRANS_COMPLETE
            | (pid ?     USB_BUF_CTRL_DATA1_PID  // Use DATA1 if needed
                       : USB_BUF_CTRL_DATA0_PID) // Use DATA0 if needed
            |            USB_BUF_CTRL_AVAIL      // Buffer available now
            | len;                               // Length of next buffer

        // Copy the user buffer to the outbound data buffer
        if (!in) {
            memcpy((void *) ep->data_buf, ep->user_buf, len);
            ep->user_buf += len;
        }

        // Update BCR
        bindump("│BCR •", bcr);
        usbh_dpram->epx_buf_ctrl = bcr & ~USB_BUF_CTRL_AVAIL;
        nop(); // TODO: I think we can remove this one
        nop();
        nop();
        usbh_dpram->epx_buf_ctrl = bcr;
    }
}

// ==[ Devices ]===============================================================

enum {
    DISCONNECTED,
    LOW_SPEED,
    FULL_SPEED,
};

enum {
    DEVICE_DISCONNECTED,
    DEVICE_ALLOCATED,
    DEVICE_CONNECTED,
    DEVICE_ADDRESSED,
    DEVICE_CONFIGURED,
    DEVICE_ACTIVE,
    DEVICE_SUSPENDED,
};

typedef struct device {
    uint8_t  speed       ; // Device speed (0:disconnected, 1:full, 2:high)
    uint8_t  state       ; // Current device state
 // uint8_t  class       ; // Device class
 // uint8_t  subclass    ; // Device subclass
 // uint8_t  protocol    ; // Device protocol
 // uint16_t vid         ; // Vendor Id  (0x0403: FTDI)
 // uint16_t pid         ; // Product Id (0xcd18: Abaxis Piccolo Xpress)
 // uint16_t revision    ; // Revision number
 // uint8_t  manufacturer; // String index of manufacturer
 // uint8_t  product     ; // String index of product
 // uint8_t  serial      ; // String index of serial number
} device_t;

static device_t devices[MAX_DEVICES], *dev0 = devices;

// Get a device by its address
SDK_INLINE device_t *get_device(uint8_t dev_addr) {
    if (dev_addr < MAX_DEVICES) return &devices[dev_addr];
    panic("Device %u does not exist", dev_addr); // TODO: Handle this properly
    return NULL;
}

// Find the next device address
uint8_t next_dev_addr() {
    for (uint8_t i = 1; i < MAX_DEVICES; i++) {
        if (devices[i].state == DEVICE_DISCONNECTED) {
            devices[i].state = DEVICE_ALLOCATED;
            return i;
        }
    }
    panic("No free devices remaining"); // TODO: Handle this properly
    return 0;
}

// Reset a device
void reset_device(uint8_t dev_addr) {
    device_t *dev = get_device(dev_addr);
    memclr(dev, sizeof(device_t));
    // TODO: Surely, there must be more work to do here?
    // memset(dev->itf2drv, TUSB_INDEX_INVALID_8, sizeof(dev->itf2drv));
    // memset(dev->ep2drv , TUSB_INDEX_INVALID_8, sizeof(dev->ep2drv ));
}

void reset_devices() {

    // Clear out all devices
    memclr(devices, sizeof(devices));
}

// ==[ Transfers ]=============================================================

enum {
    TRANSFER_SUCCESS, // used
    TRANSFER_FAILED,  //
    TRANSFER_STALLED, // used
    TRANSFER_TIMEOUT, //
    TRANSFER_INVALID, //
};

enum {
    USB_SIE_CTRL_BASE = USB_SIE_CTRL_VBUS_EN_BITS       // Supply VBUS
                      | USB_SIE_CTRL_SOF_EN_BITS        // Enable full speed
                      | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS // Enable low speed
                      | USB_SIE_CTRL_PULLDOWN_EN_BITS   // Ready for devices
                      | USB_SIE_CTRL_EP0_INT_1BUF_BITS  // One bit per EP0 buf
};

// TODO: Clear a stall and toggle data PID back to DATA0
// TODO: Abort a transfer if not yet started and return true on success

void start_control_transfer(endpoint_t *ep, usb_setup_packet_t *packet) {
    uint8_t size = sizeof(usb_setup_packet_t); // Size of the setup packet
    uint8_t left = packet->wLength;            // Length of the data phase

    // Sanity checks
    if ( ep_num(ep))     panic("Control transfers must use EP0");
    if (!ep->configured) panic("Endpoint not configured");
    if ( ep->active)     panic("Only one control transfer at a time");
    if ( ep->type)       panic("Control transfers require a control EP");

    // Validate device address and state
    uint8_t dev_addr = ep->dev_addr;
    device_t *dev = get_device(dev_addr);
    if (!dev->state || (dev_addr ? dev->state <  DEVICE_ADDRESSED
                                 : dev->state >= DEVICE_ADDRESSED)) {
        panic("Invalid device %u", dev_addr); // TODO: Handle this properly
    }

    // Transfer is now active
    ep->active     = true;
    ep->data_pid   = 1; // With SEND_SETUP, this is always DATA1
    ep->ep_addr    = packet->bmRequestType & USB_DIR_IN;
    ep->bytes_left = left;
    ep->bytes_done = 0;
    ep->user_buf   = temp_buf; // TODO: Maybe use a ring buffer here?

    // Debug output
    show_endpoint(ep, "Start");

    // Copy the setup packet
    memcpy((void *) usbh_dpram->setup_packet, packet, size);

    // If there's no data phase, flip the direction for the USB controller
    bool in = ep_in(ep);
    if (!left) {
        in = !in;
        ep->ep_addr ^= USB_DIR_IN;
    }

    // Calculate register values
    uint32_t scr, dar, bcr;
    uint8_t pid = ep->data_pid;
    bool mas = left > ep->maxsize; // Are there more packets?
    scr =            USB_SIE_CTRL_BASE              // SIE_CTRL defaults
     // | (ls  ? 0 : USB_SIE_CTRL_PREAMBLE_EN_BITS) // Preamble (LS on FS hub)
        |            USB_SIE_CTRL_SEND_SETUP_BITS   // Send SETUP transaction
        | (in  ?     USB_SIE_CTRL_RECEIVE_DATA_BITS // Receive bit means IN
                   : USB_SIE_CTRL_SEND_DATA_BITS)   // Send bit means OUT
        |            USB_SIE_CTRL_START_TRANS_BITS; // Start the transfer now
    dar = dev_addr;                                 // Device address for EP0
    bcr = (in  ? 0 : USB_BUF_CTRL_FULL)             // IN/Recv=0, OUT/Send=1
        | (mas ? 0 : USB_BUF_CTRL_LAST)             // Trigger TRANS_COMPLETE
        | (pid ?     USB_BUF_CTRL_DATA1_PID         // Use DATA1 if needed
                   : USB_BUF_CTRL_DATA0_PID)        // Use DATA0 if needed
        |            USB_BUF_CTRL_AVAIL             // Buffer available now
        | MIN(ep->maxsize, left);                   // Length of next buffer

    // Debug output
    bindump(" INTR", usb_hw->intr);
    bindump(" INTS", usb_hw->intr);
    bindump(" SSR" , usb_hw->sie_status);   // SIE status register
    bindump(" SCR" , scr);                  // SIE control register
    bindump(" DAR" , dar);                  // Device address register
    bindump(" ECR" , usbh_dpram->epx_ctrl); // EPX control register
    bindump(" BCR" , bcr);                  // EPX buffer control register

    hexdump("<Setup", packet, size, 1);

    // NOTE: When clk_sys (usually 133Mhz) and clk_usb (usually 48MHz) are not
    // the same, the processor and the USB controller run at different speeds.
    // To properly coordinate them, we must sometimes waste clk_sys cycles to
    // allow time for clk_usb to catch up. Each clk_sys cycle is 133/48 times
    // faster than a clk_usb cycle, which is 2.77 (roughly 3) times as fast.
    // So, for each 1 clk_usb cycle, we should waste 3 clk_sys cycles.
    //
    // For the USB controller, the START_TRANS bit in SCR and the AVAILABLE bit
    // in BCR need special care. These bits trigger processor actions when they
    // are set, but they will execute too soon since the USB controller needs
    // more time to perform the actions specified in the other bits. Thus, we
    // need to set the USB specific bits first, delay a few cycles, and then
    // set the bits for the processor. The datasheet shows how long to wait:
    //
    // For SCR, Datasheet § 4.1.2.7 (p. 390) says START_TRANS needs two clk_usb
    // For BCR, Datasheet § 4.1.2.5.1 (p. 383) says AVAILABLE needs one clk_usb
    //
    // We have several values to set, so we order them as shown below. Notice
    // that this sets SCR without START_TRANS and then, in 6 cycles, it sets it
    // again but this time including START_TRANS. BCR is similar, but will be
    // set again after 3 cycles. The setting of DAR and two NOP's are inserted
    // to make everything line up correctly.

    // Set registers optimally => scr, dar, bcr, nop, nop, bcr, scr
    usb_hw->sie_ctrl         = scr & ~USB_SIE_CTRL_START_TRANS_BITS;
    usbh_dpram->epx_buf_ctrl = bcr & ~USB_BUF_CTRL_AVAIL;
    usb_hw->dev_addr_ctrl    = dar;
    nop();
    nop();
    usbh_dpram->epx_buf_ctrl = bcr;
    nop(); // TODO: Might not need this one
    usb_hw->sie_ctrl         = scr;
}

// Transfer a ZLP, but it makes several critical assumptions so be careful!
// TODO: Merge with start_control_transfer (ep has info, packet would be NULL)
void transfer_zlp(void *arg) {
    endpoint_t *ep = (endpoint_t *) arg;

    // Update the endpoint
    ep->active    = true;       // Transfer is now active
    ep->data_pid  = 1;          // Control status stage is always DATA1
    ep->ep_addr  ^= USB_DIR_IN; // Flip the direction

    // Calculate register values
    uint32_t scr, dar, bcr;
    uint8_t pid = ep->data_pid;
    bool in = ep_in(ep);
    scr =            USB_SIE_CTRL_BASE               // SIE_CTRL defaults
     // | (ls  ? 0 : USB_SIE_CTRL_PREAMBLE_EN_BITS)  // Preamble (LS on FS hub)
        | (in  ?     USB_SIE_CTRL_RECEIVE_DATA_BITS  // Receive bit means IN
                   : USB_SIE_CTRL_SEND_DATA_BITS)    // Send bit means OUT
        |            USB_SIE_CTRL_START_TRANS_BITS;  // Start the transfer now
    dar = ep->dev_addr | ep_num(ep)                  // Device address
                  << USB_ADDR_ENDP_ENDPOINT_LSB;     // EP number
    bcr = (in  ? 0 : USB_BUF_CTRL_FULL)              // IN/Recv=0, OUT/Send=1
        |            USB_BUF_CTRL_LAST               // Trigger TRANS_COMPLETE
        | (pid ?     USB_BUF_CTRL_DATA1_PID          // Use DATA1 if needed
                   : USB_BUF_CTRL_DATA0_PID)         // Use DATA0 if needed
        |            USB_BUF_CTRL_AVAIL;             // Buffer available now

    // Debug output
    bindump(" SSR", usb_hw->sie_status);   // SIE status register
    bindump(" SCR", scr);                  // SIE control register
    bindump(" DAR", dar);                  // Device address register
    bindump(" ECR", usbh_dpram->epx_ctrl); // EPX control register
    bindump(" BCR", bcr);                  // EPX buffer control register

    printf("%cZLP\n", in ? '>' : '<');

    // Set registers optimally => scr, nop, bcr, nop, nop, bcr, scr
    usb_hw->sie_ctrl         = scr & ~USB_SIE_CTRL_START_TRANS_BITS;
    usbh_dpram->epx_buf_ctrl = bcr & ~USB_BUF_CTRL_AVAIL;
    usb_hw->dev_addr_ctrl    = dar;
    nop();
    nop();
    usbh_dpram->epx_buf_ctrl = bcr;
    nop(); // TODO: Might not need this one
    usb_hw->sie_ctrl         = scr;
}

// ==[ Enumeration ]===========================================================

enum {
    ENUMERATION_START,
    ENUMERATION_GET_MAXSIZE,
    ENUMERATION_SET_ADDRESS,
    ENUMERATION_GET_DEVICE,
    ENUMERATION_GET_CONFIG,
    ENUMERATION_SET_CONFIG,
    ENUMERATION_END,
};

void get_device_descriptor(endpoint_t *ep) {
    printf("Get device descriptor\n");

    // If we're using device 0, only ask for 8 bytes
    start_control_transfer(ep, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(USB_DT_DEVICE, 0),
        .wIndex        = 0,
        .wLength       = ep->dev_addr ? sizeof(usb_device_descriptor_t) : 8,
    }));
}

void set_device_address(endpoint_t *ep) {
    printf("Set device address to %u\n", ep->dev_addr);

    // EPX is used to set a new device address
    start_control_transfer(epx, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_OUT
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_SET_ADDRESS,
        .wValue        = ep->dev_addr,
        .wIndex        = 0,
        .wLength       = 0,
    }));
}

void get_configuration_descriptor(endpoint_t *ep) {
    printf("Get configuration descriptor\n");

    start_control_transfer(ep, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(USB_DT_CONFIG, 0),
        .wIndex        = 0,
        .wLength       = sizeof(usb_configuration_descriptor_t),
     // .wLength       = 98,
    }));
}

void set_configuration(endpoint_t *ep, uint16_t cfg) {
    printf("Set configuration to %u\n", cfg);

    start_control_transfer(ep, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_OUT
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_SET_CONFIGURATION,
        .wValue        = cfg,
        .wIndex        = 0,
        .wLength       = 0,
    }));
}

void enumerate(void *arg) {
    endpoint_t *ep = (endpoint_t *) arg;

    // Prepare to advance the enumeration
    static uint8_t step;
    static uint8_t new_addr;

    if (!ep) step = ENUMERATION_START;

    switch (step++) {

        case ENUMERATION_START:
            printf("Enumeration started\n");

// TODO: Remove this testing shortcut
// get_configuration_descriptor(epx);
// break;

            printf("Starting GET_MAXSIZE\n");
            get_device_descriptor(epx);
            break;

        case ENUMERATION_GET_MAXSIZE: {
            uint8_t maxsize0 = ((usb_device_descriptor_t *) epx->data_buf)
                ->bMaxPacketSize0;

            printf("Starting SET_ADDRESS\n");

            // Allocate a new device
            new_addr      = next_dev_addr();
            device_t *dev = get_device(new_addr);
            dev->speed    = dev0->speed;
            dev->state    = DEVICE_ALLOCATED;

            // Allocate EP0 for the new device
            endpoint_t *ep = next_ep(new_addr, &((usb_endpoint_descriptor_t) {
                .bLength          = sizeof(usb_endpoint_descriptor_t),
                .bDescriptorType  = USB_DT_ENDPOINT,
                .bEndpointAddress = 0,
                .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
                .wMaxPacketSize   = maxsize0,
                .bInterval        = 0,
            }));
            ep->dev_addr = new_addr;

            set_device_address(ep);
        }   break;

        case ENUMERATION_SET_ADDRESS: {
            endpoint_t *ep = find_endpoint(new_addr, 0);
            device_t *dev = get_device(ep->dev_addr);
            dev->state = DEVICE_ADDRESSED;

            printf("Starting GET_DEVICE\n");
            get_device_descriptor(ep);
        }   break;

        case ENUMERATION_GET_DEVICE: {
            usb_device_descriptor_t *desc;
            desc = (usb_device_descriptor_t *) ep->data_buf; // TODO: We should have a different buffer here...

            printf("\nConnected device:\n");
            printb("  USB version:\t"        , desc->bcdUSB);
            printf("  Device class:\t%u\n"   , desc->bDeviceClass);
            printf("    Subclass:\t%u\n"     , desc->bDeviceSubClass);
            printf("    Protocol:\t%u\n"     , desc->bDeviceProtocol);
            printf("  Packet size:\t%u\n"    , desc->bMaxPacketSize0);
            printf("  Vendor ID:\t0x%04x\n"  , desc->idVendor);
            printf("  Product ID:\t0x%04x\n" , desc->idProduct);
            printb("  Revision:\t"           , desc->bcdDevice);
            printf("  Manufacturer:\t[#%u]\n", desc->iManufacturer);
            printf("  Product:\t[#%u]\n"     , desc->iProduct);
            printf("  Serial:\t[#%u]\n"      , desc->iSerialNumber);
            printf("\n");

            printf("Starting GET_CONFIG\n");
            get_configuration_descriptor(ep);
        }   break;

        case ENUMERATION_GET_CONFIG: {
            usb_configuration_descriptor_t *desc;
            desc = (usb_configuration_descriptor_t *) ep->data_buf; // TODO: We should have a different buffer here...

            printf("\nConfiguration descriptor:\n");
            printf("  Total length:\t%u\n"  , desc->wTotalLength);
            printf("  Interfaces:\t%u\n"    , desc->bNumInterfaces);
            printf("  Config Value:\t%u\n"  , desc->bConfigurationValue);
            printf("  Config Name:\t[#%u]\n", desc->iConfiguration);
            printf("  Attributes:\t");
            {
                char *sp = desc->bmAttributes & 0x40 ? "Self-powered"  : NULL;
                char *rw = desc->bmAttributes & 0x20 ? "Remote wakeup" : NULL;

                if (sp && rw) printf("%s, %s\n", sp, rw);
                else if  (sp) printf("%s\n", sp);
                else if  (rw) printf("%s\n", rw);
                else          printf("None\n");
            }
            printf("  Max power:\t%umA\n"   , desc->bMaxPower * 2);
            printf("\n");

            printf("Starting SET_CONFIG\n");
            set_configuration(ep, 1);
        }   break;

        case ENUMERATION_SET_CONFIG:
            device_t *dev = get_device(ep->dev_addr);
            dev->state = DEVICE_CONFIGURED;

            printf("Enumeration completed\n");
            break;
    }
}

// ==[ Resets ]================================================================

void reset_usb_host() {
    printf("USB host reset\n\n");

    // Reset controller
    reset_block       (RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    // Clear state
    memset(usb_hw    , 0, sizeof(*usb_hw    ));
    memset(usbh_dpram, 0, sizeof(*usbh_dpram));

    // Configure USB host controller
    usb_hw->muxing    = USB_USB_MUXING_TO_PHY_BITS       // Connect to USB Phy
                      | USB_USB_MUXING_SOFTCON_BITS;     // Soft connect
    usb_hw->pwr       = USB_USB_PWR_VBUS_DETECT_BITS     // Enable VBUS detect
                      | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS // Enable controller
                      | USB_MAIN_CTRL_HOST_NDEVICE_BITS; // Enable USB Host
    usb_hw->sie_ctrl  = USB_SIE_CTRL_BASE;               // SIE_CTRL defaults
    usb_hw->inte      = USB_INTE_HOST_CONN_DIS_BITS      // Connect/disconnect
                      | USB_INTE_STALL_BITS              // Stall detected
                      | USB_INTE_BUFF_STATUS_BITS        // Buffer ready
                      | USB_INTE_TRANS_COMPLETE_BITS     // Transfer complete
                      | USB_INTE_HOST_RESUME_BITS        // Device wakes host
                      | USB_INTE_ERROR_DATA_SEQ_BITS     // DATA0/DATA1 wrong
                      | USB_INTE_ERROR_RX_TIMEOUT_BITS   // Receive timeout
                      | (0xffffffff ^ 0x00000004);       // NOTE: Debug all on

    irq_set_enabled(USBCTRL_IRQ, true);

    reset_devices();
    reset_endpoints();

    bindump(" INT", usb_hw->inte);
}

// ==[ Tasks ]=================================================================

enum {
    TASK_CONNECT,
    TASK_TRANSFER,
    TASK_FUNCTION,
};

typedef struct {
    uint8_t type;
    uint32_t guid;

    union {
        struct {
            uint8_t speed;
        } connect;

        struct {
            uint8_t  dev_addr;
            uint8_t  ep_addr;
            uint16_t len;
            uint8_t  status;
        } transfer;

        struct {
            void (*fn) (void *);
            void *arg;
        } function;
    };
} task_t;

static uint32_t guid = 1; // TODO: Remove this little debug helper variable

static queue_t *queue = &((queue_t) { 0 });

const char *task_name(uint8_t type) {
    switch (type) {
        case TASK_CONNECT:  return "TASK_CONNECT";
        case TASK_TRANSFER: return "TASK_TRANSFER";
        case TASK_FUNCTION: return "TASK_FUNCTION";
        default:            return "UNKNOWN";
    }
    panic("Unknown task queued");
}

const char *function_name(void (*fn) (void *)) {
    if (fn == enumerate   ) return "enumerate";
    if (fn == transfer_zlp) return "transfer_zlp";
    panic("Unknown function queued");
}

void usb_task() {
    task_t task;

    while (queue_try_remove(queue, &task)) {
        uint8_t type = task.type;
        printf("\n=> Start task #%u: %s\n", task.guid, task_name(type));
        switch (type) {
            case TASK_CONNECT:

                // TODO: See if we can get this to work
                // // Prevent nested connections
                // if (dev0->state == DEVICE_ENUMERATING) {
                //     printf("Only one device can be enumerated at a time\n");
                //     break;
                // }

                // Initialize dev0
                reset_device(0); // TODO: Is this really necessary?
                dev0->speed = task.connect.speed;
                dev0->state = DEVICE_CONNECTED;

                // Show the device connection and speed
                char *str = dev0->speed == LOW_SPEED ? "low" : "full";
                printf("Device connected (%s speed)\n", str);

                // // Queue the enumeration process
                // queue_add_blocking(queue, &((task_t) {
                //     .type          = TASK_FUNCTION,
                //     .guid          = guid++,
                //     .function.fn   = enumerate,
                //     .function.arg  = NULL,
                // })); // enumerate(NULL);

                // Let's just call enumerate directly?
                enumerate(NULL);

                break;

//             case TASK_TRANSFER: {
//                 uint8_t dev_addr = task.transfer.dev_addr;
//                 uint8_t ep_addr  = task.transfer.ep_addr;
//                 uint16_t len     = task.transfer.len;
//
//                 // Lookup endpoint
//                 endpoint_t *ep = find_endpoint(dev_addr, ep_addr);
//
//                 // Debug output, unless this is a ZLP on dev0
//                 // if (dev_addr || len) {
//                     show_endpoint(ep, "Transfer");
//                     hexdump(" Data", usbh_dpram->epx_data, len, 1);
//                 // }
//
//                 // Advance the enumeration
//
//             }   break;

            case TASK_FUNCTION: {
                printf("Calling %s\n", function_name(task.function.fn));
                task.function.fn(task.function.arg);
            }   break;

            default:
                printf("Unknown task queued\n");
                break;
        }
        printf("=> Finish task #%u: %s\n", task.guid, task_name(type));
    }
}

// ==[ Interrupts ]============================================================

void printf_interrupts(uint32_t ints) {
    if (ints & USB_INTS_HOST_CONN_DIS_BITS   ) printf(", device"  );
    if (ints & USB_INTS_STALL_BITS           ) printf(", stall"   );
    if (ints & USB_INTS_BUFF_STATUS_BITS     ) printf(", buffer"  );
    if (ints & USB_INTS_TRANS_COMPLETE_BITS  ) printf(", last"    );
    if (ints & USB_INTS_ERROR_RX_TIMEOUT_BITS) printf(", timeout" );
    if (ints & USB_INTS_ERROR_DATA_SEQ_BITS  ) printf(", dataseq" );
    if (ints & USB_INTS_HOST_RESUME_BITS     ) printf(", power"   );
}

// Interrupt handler
void isr_usbctrl() {
    uint32_t ints = usb_hw->ints;
    task_t task;

    // Workaround for RP2040-E4
    uint32_t bcr = usbh_dpram->epx_buf_ctrl;      // Buffer control register
    uint32_t bch = usb_hw->buf_cpu_should_handle; // Check for CPU handle bits
    if (bch & 1u) bcr >>= 16;                     // Perform bitshift correction // TODO: Process all affected buffers

    // Show system state
    printf( "\n=> New ISR #%u", guid++);
    printf_interrupts(ints);
    printf( "\n");
    printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
    printf( "│Frame\t│ %4u │ %-35s │%12s│\n", usb_hw->sof_rd, "Interrupt Handler", "");
    bindump("│INTR", usb_hw->intr);
    bindump("│INTS", ints);
    bindump("│SSR", usb_hw->sie_status);
    bindump("│SCR", usb_hw->sie_ctrl);
    bindump("│DAR", usb_hw->dev_addr_ctrl);
    bindump("│ECR", usbh_dpram->epx_ctrl);
    bindump("│BCR", bcr);
    bindump("│BCH", bch);

    // Connection (attach or detach)
    if (ints &  USB_INTS_HOST_CONN_DIS_BITS) {
        ints ^= USB_INTS_HOST_CONN_DIS_BITS;

        // Get the device speed
        uint8_t speed = get_speed();

        // Clear the interrupt
        usb_hw_clear->sie_status = USB_SIE_STATUS_SPEED_BITS;

        // Handle connect and disconnect
        if (speed) {
            queue_add_blocking(queue, &((task_t) {
                .type          = TASK_CONNECT,
                .guid          = guid++,
                .connect.speed = speed,
            }));
        } else {
            setup_epx(); // TODO: There's a lot more to do here
        }
    }

    // Stall detected (higher priority than BUFF_STATUS and TRANS_COMPLETE)
    if (ints &  USB_INTS_STALL_BITS) {
        ints ^= USB_INTS_STALL_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;

        // TODO: Add this back in once we have a proper way to handle STALL
        // // Queue the stalled transfer
        // queue_add_blocking(queue, &((task_t) {
        //     .type              = TASK_TRANSFER,
        //     .guid              = guid++,
        //     .transfer.dev_addr = 42, // TODO: Need to flesh this out
        //     .transfer.ep_addr  = 37, // TODO: Need to flesh this out
        //     .transfer.len      = 0,  // TODO: Need to flesh this out
        //     .transfer.status   = TRANSFER_STALLED,
        // }));
    }

    // Buffer processing is needed
    if (ints &  USB_INTS_BUFF_STATUS_BITS) {
        ints ^= USB_INTS_BUFF_STATUS_BITS;

        // Find the buffer(s) that are ready
        uint32_t bits = usb_hw->buf_status;
        uint32_t mask = 1u;

        // See if EPX is single or double buffered
        uint32_t ecr = usbh_dpram->epx_ctrl;
        bool dubs = (bits & mask) && (ecr & EP_CTRL_DOUBLE_BUFFERED_BITS);
        printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
        bindump(dubs ? "│BUF/2" : "│BUF/1", bits);

        // NOTE: Miroslav says we should handle these in pairs of IN/OUT
        // endpoints, since they "come in pairs". So, we would deal with
        // EP3IN/EP3OUT at the same time and mask with 0b11, etc.

        // Use the DAR to determine dev_addr and ep_addr
        volatile uint32_t dar = usb_hw->dev_addr_ctrl;
        uint8_t dev_addr =  dar & USB_ADDR_ENDP_ADDRESS_BITS;
        uint8_t ep_addr  = (dar & USB_ADDR_ENDP_ENDPOINT_BITS) >>
                                  USB_ADDR_ENDP_ENDPOINT_LSB;

        // Lookup the endpoint
        endpoint_t *ep = find_endpoint(dev_addr, ep_addr);
        handle_buffer(ep); usb_hw_clear->buf_status = ~0; bits ^= 0x01; // TODO: TOTAL HACK!

//         // Check the interrupt/asynchronous endpoints (IN and OUT)
//         for (uint8_t i = 0; i <= MAX_ENDPOINTS && bits; i++) {
//             for (uint8_t j = 0; j < 2; j++) {
//                 mask = 1 << (i * 2 + j);
//                 if (bits &  mask) {
//                     bits ^= mask;
//                     handle_buffer(&eps[i]);
//                 }
//             }
//         }

        // Panic if we missed any buffers
        if (bits) panic("Unhandled buffer mask: %032b", bits);
    }

    // Transfer complete (last packet)
    if (ints &  USB_INTS_TRANS_COMPLETE_BITS) {
        ints ^= USB_INTS_TRANS_COMPLETE_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;

        // NOTE: TRANS_COMPLETE triggers when (see datasheet, p. 401):
        //
        // 1. SETUP packet is sent without {RECEIVE,SEND}_DATA_BITS in SCR
        // 2. IN or OUT packet is transferred with BUF_CTRL_LAST set in BCR
        // 3. An IN packet is received with a zero length status packet (ZLP)

        // Use the DAR to determine dev_addr and ep_addr
        volatile uint32_t dar = usb_hw->dev_addr_ctrl;
        uint8_t dev_addr =  dar & USB_ADDR_ENDP_ADDRESS_BITS;
        uint8_t ep_addr  = (dar & USB_ADDR_ENDP_ENDPOINT_BITS) >>
                                  USB_ADDR_ENDP_ENDPOINT_LSB;

        // Lookup the endpoint
        endpoint_t *ep = find_endpoint(dev_addr, ep_addr);
        uint16_t len = ep->bytes_done;

        // Panic if the endpoint is not active
        if (!ep->active) panic("EP should still be active in TRANS_COMPLETE");

        // Debug output
        printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
        printf( "│Trans\t│ %4u │ Device %-28u │ Task #%-4u │\n", ep->bytes_done, ep->dev_addr, guid);
        if (ep->bytes_done) hexdump("│Data", usbh_dpram->epx_data, ep->bytes_done, 1);

        // Clear the endpoint (since its complete)
        clear_endpoint(ep);

        // Queue a ZLP or advance the enumeration
        queue_add_blocking(queue, &((task_t) {
            .type         = TASK_FUNCTION,
            .guid         = guid++,
            .function.fn  = len ? transfer_zlp : enumerate,
            .function.arg = ep,
        }));
    }

    // Receive timeout (waited too long without seeing an ACK)
    if (ints &  USB_INTS_ERROR_RX_TIMEOUT_BITS) {
        ints ^= USB_INTS_ERROR_RX_TIMEOUT_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;

        printf("Receive timeout\n");

        panic("Panic here for now");
    }

    // Data error (IN packet from device has wrong data PID)
    if (ints &  USB_INTS_ERROR_DATA_SEQ_BITS) {
        ints ^= USB_INTS_ERROR_DATA_SEQ_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;

        panic("Data sequence error");
    }

    // Device resumed (device initiated)
    if (ints &  USB_INTS_HOST_RESUME_BITS) {
        ints ^= USB_INTS_HOST_RESUME_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;

        printf("Device initiated resume\n");
    }

    // Were any interrupts missed?
    if (ints) panic("Unhandled IRQ 0x%04x", ints);

    // TODO: I see a lot of NAK's being set in SSR... this will clear it, but can we prevent it or deal with it better?
    // usb_hw_clear->sie_status = 1 << 28u; // Clear the NAK??? // ALERT: Get rid of this!!!

    printf("└───────┴──────┴─────────────────────────────────────┴────────────┘\n");
}

// ==[ Main ]==================================================================

int main() {
    stdio_init_all();
    printf("\033[2J\033[H\n==[ USB host example]==\n\n");
    reset_usb_host();

    queue_init(queue, sizeof(task_t), 64);

    while (1) {
        usb_task();
    }
}
