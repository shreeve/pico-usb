// ============================================================================
// PicoUSB - A USB host and device library for the Raspberry Pi Pico/W
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: January 29, 2024
//   Note: This is a work in progress. It is not yet functional.
//  Legal: Same license as the Raspberry Pi Pico SDK
//
// Thanks to the TinyUSB project for inspiration and code snippets! Also, thank
// you Miroslav Nemecek for the https://github.com/Panda381/PicoLibSDK project.
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

// ==[ PicoUSB ]===============================================================

#include "helpers.h"              // Helper functions

#define memclr(ptr, len) memset((ptr), 0, (len))

#define MAKE_U16(x, y) (((x) << 8) | ((y)     ))
#define SWAP_U16(x)    (((x) >> 8) | ((x) << 8))

#define SDK_ALIGNED(bytes) __attribute__ ((aligned(bytes)))
#define SDK_ALWAYS_INLINE  __attribute__ ((always_inline))
#define SDK_PACKED         __attribute__ ((packed))
#define SDK_WEAK           __attribute__ ((weak))

#define MAX_DEVICES   2
#define MAX_ENDPOINTS 1

// ==[ Hardware: rp2040 ]======================================================

#define usb_hw_set   ((usb_hw_t *) hw_set_alias_untyped  (usb_hw))
#define usb_hw_clear ((usb_hw_t *) hw_clear_alias_untyped(usb_hw))

// NOTE: Which is better? Same? Should they use an inlined function?
// #define INLINE   __forceinline             // request to inline this function
// #define NOINLINE __attribute__((noinline)) // request to not inline this function

#define busy_wait_2_cycles() __asm volatile("b 1f\n1:\n"     :::"memory") // remove?
#define busy_wait_3_cycles() __asm volatile("nop\nnop\nnop\n":::"memory") // static inline?

#define hw_set_staged3(reg, value, or_mask) \
    reg = (value); \
    busy_wait_3_cycles(); \
    reg = (value) | (or_mask);

#define hw_set_staged6(reg, value, or_mask) \
    reg = (value); \
    busy_wait_3_cycles(); \
    busy_wait_3_cycles(); \
    reg = (value) | (or_mask);

SDK_ALWAYS_INLINE static inline bool is_host_mode() {
    return (usb_hw->main_ctrl & USB_MAIN_CTRL_HOST_NDEVICE_BITS);
}

SDK_ALWAYS_INLINE static inline uint8_t get_speed() {
    return (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) \
                              >> USB_SIE_STATUS_SPEED_LSB;
}

SDK_ALWAYS_INLINE static inline uint8_t line_state() {
    return (usb_hw->sie_status & USB_SIE_STATUS_LINE_STATE_BITS) \
                              >> USB_SIE_STATUS_LINE_STATE_LSB;
}

enum {
    USB_SIE_CTRL_BASE = USB_SIE_CTRL_VBUS_EN_BITS       // Supply VBUS
                      | USB_SIE_CTRL_SOF_EN_BITS        // Enable full speed
                      | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS // Enable low speed
                      | USB_SIE_CTRL_PULLDOWN_EN_BITS   // Ready for devices
                      | USB_SIE_CTRL_EP0_INT_1BUF_BITS  // One bit per EP0 buf
};

// ==[ Events ]================================================================

enum {
    EVENT_CONNECT,
    EVENT_TRANSFER,
    EVENT_FUNCTION,
};

enum {
    TRANSFER_SUCCESS, // used
    TRANSFER_FAILED,  //
    TRANSFER_STALLED, // used
    TRANSFER_TIMEOUT, //
    TRANSFER_INVALID, //
};

typedef struct {
    uint8_t type;
    uint8_t dev_addr;

    union {
        struct {
            uint8_t speed;
        } conn;

        struct {
            uint8_t ep_addr;
            uint8_t result;
            uint16_t len;
        } xfer;

        struct {
            void (*call) (void *);
            void *arg;
        } fn;
    };
} event_t;

static queue_t queue_struct, *queue = &queue_struct;

// ==[ Endpoints ]=============================================================

typedef void (*endpoint_c)(uint8_t *buf, uint16_t len);

typedef struct endpoint {
    uint8_t    dev_addr  ; // Device address // HOST ONLY
    uint8_t    ep_addr   ; // Endpoint address
    uint16_t   maxsize   ; // Maximum packet size
    uint8_t    type      ; // Transfer type
    uint8_t    data_pid  ; // Toggle DATA0/DATA1 each packet
    uint16_t   interval  ; // Polling interval in ms

    uint8_t    ep_num    ; // Endpoint number         // TODO: Needed? Use a define? Inline?
    bool       sender    ; // Endpoint is for sending // TODO: Needed? Derived...
    bool       active    ; // Transfer is active      // TODO: Needed? Other ways to check?

    volatile
    uint8_t   *data_buf  ; // Data buffer
    uint8_t   *user_buf  ; // User buffer
    uint16_t   bytes_left; // Bytes remaining
    uint16_t   bytes_done; // Bytes transferred
    endpoint_c cb        ; // Callback function
} endpoint_t;

// TODO: For right now, only define EPX. Later, we'll make this dynamic.
static endpoint_t eps[MAX_ENDPOINTS], *epx = eps;

SDK_WEAK void epx_cb(uint8_t *buf, uint16_t len) {
    printf("Inside the EPX callback...\n");
}

// Setup an endpoint
void setup_endpoint(endpoint_t *ep, usb_endpoint_descriptor_t *usb) {

    // Populate the endpoint
    *ep = (endpoint_t) {
        .dev_addr   = 0,                            // Device address // HOST ONLY
        .ep_addr    = usb->bEndpointAddress,
        .maxsize    = 0,                            // Maximum packet size
        .type       = usb->bmAttributes,
        .interval   = usb->bInterval,
        .data_pid   = 1,                            // Toggle DATA0/DATA1 each packet
        .ep_num     = usb->bEndpointAddress & 0x0f, // Endpoint number
        .sender     = false,                        // Endpoint is for sending
        .active     = false,                        // Transfer is active
        .data_buf   = (uint8_t *) 0x0180,           // Data buffer
        .user_buf   = 0,                            // User buffer
        .bytes_left = 0,                            // Bytes remaining
        .bytes_done = 0,                            // Bytes transferred
        .cb         = epx_cb,                       // Callback function
    };

    // Helper variables
    bool     in           = ep->ep_addr & USB_DIR_IN;
    uint32_t type         = ep->type;
    uint32_t ms           = ep->interval;
    uint32_t interval_lsb = EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB;
    uint32_t offset       = offsetof(usb_host_dpram_t, epx_data); // TODO: Make this generic, not epx-specific

    // Get the endpoint control register (ECR) value
    uint32_t ecr = EP_CTRL_ENABLE_BITS               // Enable endpoint
                 | EP_CTRL_INTERRUPT_PER_BUFFER      // An interrupt per buffer
                 | type << EP_CTRL_BUFFER_TYPE_LSB   // Set transfer type
                 | (ms ? ms - 1 : 0) << interval_lsb // Interrupt polling in ms
                 | offset;                           // Data buffer offset

    // Debug output
    printf(" EP%d_%s│ 0x%02x │ Buffer 0x%04x\n",
           ep->ep_num, in ? "IN " : "OUT", ep->ep_addr, offset);
    bindump(" ECR", ecr);

    // Set the endpoint control register (ECR)
    usbh_dpram->epx_ctrl = ecr;
}

// Setup endpoints
void setup_endpoints() {

    // Clear out all endpoints
    memclr(eps, sizeof(eps));

    // Configure the first endpoint as EPX
    static usb_endpoint_descriptor_t usb_epx = {
        .bLength          = sizeof(usb_endpoint_descriptor_t),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = 0,
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 64,
        .bInterval        = 0
    };
    setup_endpoint(epx, &usb_epx);

    // Dynamically allocate the others
}

// endpoint_t *find_endpoint(uint8_t dev_addr, uint8_t ep_addr) {
//     if (!(ep_addr & 0x7f)) return epx; // EP0 is special
//     for (uint8_t i = 0; i < USB_MAX_ENDPOINTS; i++) {
//         if (eps[i].maxsize && eps[i].dev_addr == dev_addr && eps[i].ep_addr == ep_addr) {
//             return &eps[i];
//         }
//     }
//     return NULL;
// }

SDK_ALWAYS_INLINE static inline void reset_endpoint(endpoint_t *ep) {
    ep->active     = false;
    ep->user_buf   = 0; // TODO: Add something like a ring buffer here?
    ep->bytes_left = 0;
    ep->bytes_done = 0;
}

// ==[ Buffers ]===============================================================

// Prepare an endpoint buffer and return its buffer control register value
uint32_t prepare_buffer(endpoint_t *ep, uint8_t buf_id) {
    uint16_t len = MIN(ep->bytes_left, ep->maxsize);
    uint32_t bcr = ep->data_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID
                 | USB_BUF_CTRL_AVAIL
                 | len;
    ep->bytes_left -= len;
    ep->data_pid ^= 1u;

    // Copy data to buffer if we're sending
    if (ep->sender) {
        memcpy((void *) (ep->data_buf + buf_id * 64), ep->user_buf, len);
        ep->user_buf += len;
        bcr |= USB_BUF_CTRL_FULL;
    }

    // Is this the last buffer? This only really matters for host mode. It will
    // trigger the TRANS_COMPLETE interrupt but will also stop it polling. We
    // only really care about this for setup packets being sent, though.
    if (!ep->bytes_left) {
        bcr |= USB_BUF_CTRL_LAST;
    }

    if (buf_id) bcr = bcr << 16;

    return bcr;
}

void prepare_buffers(endpoint_t *ep) {
    // const bool host = is_host_mode();
    // const bool in = ep->usb->bEndpointAddress & USB_DIR_IN;
    // const bool allow_double = host ? !in : in; // TODO: host/out and device/in? Doesn't seem right

    bool allow_double = false; // TODO: This is a hack for now

    uint32_t ecr = usbh_dpram->epx_ctrl;
    uint32_t bcr = prepare_buffer(ep, 0) | USB_BUF_CTRL_SEL;

    // Double buffering is only supported in specific cases // TODO: Properly determine this
    if (ep->bytes_left && allow_double) {
        bcr |=  prepare_buffer(ep, 1); // TODO: Fix isochronous for buf_1!
        ecr |=  EP_CTRL_DOUBLE_BUFFERED_BITS;
    } else {
        ecr &= ~EP_CTRL_DOUBLE_BUFFERED_BITS;
    }

    // Update ECR and BCR
    usbh_dpram->epx_ctrl     = ecr;
    usbh_dpram->epx_buf_ctrl = bcr;
}

// Sync an endpoint buffer, while updating and returning byte counts
uint16_t sync_buffer(endpoint_t *ep, uint8_t buf_id) {
    uint32_t bcr = usbh_dpram->epx_buf_ctrl; if (buf_id) bcr = bcr >> 16;
    uint16_t len = bcr & USB_BUF_CTRL_LEN_MASK;
    bool    full = bcr & USB_BUF_CTRL_FULL;

    // Buffer must be full for reads, empty for writes
    assert(ep->sender ^ full);

    // Copy data if we're receiving
    if (!ep->sender) {
        memcpy(ep->user_buf, (void *) (ep->data_buf + buf_id * 64), len);
        ep->user_buf += len;
    }
    ep->bytes_done += len;

    // A short packet (below maxsize) means the transfer is done
    if (len < ep->maxsize) {
        ep->bytes_left = 0;
    }

    return len;
}

// TODO: Later, find all of the endpoint_lock_update calls and put something there...

bool still_transferring(endpoint_t *ep) {
    if (!ep->active) panic("EP 0x%02x not active\n", ep->ep_addr);

    // Update endpoint with buffer status, honor double buffering if present
    if (sync_buffer(ep, 0) == ep->maxsize) { // Full buf_0
        if (usbh_dpram->epx_ctrl & EP_CTRL_DOUBLE_BUFFERED_BITS) {
            sync_buffer(ep, 1); // TODO: What if buf_1 is 0 bytes or short?
        }
    }

    if (!ep->bytes_left) return false; // Transfer is complete

    // // Handle Errata 15: Buffer status may not be set during last 200µs of a frame
    // if (e15_is_critical_frame_period(ep)) {
    //     ep->pending = 1;
    // } else {
        prepare_buffers(ep); // Calculate and set ECR/BCR to transfer next buffer
    // }

    return true; // Transfer should continue
}

void handle_buffer(uint32_t bit, endpoint_t *ep) {
    if (still_transferring(ep)) return;

    assert(ep->active);

    queue_add_blocking(queue, &((event_t) {
        .type     = EVENT_TRANSFER,
        .dev_addr = ep->dev_addr,
        .xfer     = {
            .ep_addr = ep->ep_addr,
            .result  = TRANSFER_SUCCESS,
            .len     = ep->bytes_done,
        },
    }));
    reset_endpoint(ep);
}

// ==[ Transfers ]=============================================================

// TODO: Is there a "generic" transfer here that we can call from control...
// Start a transfer

// NOTE: This is a single/global/static control transfer object.
// Control transfers: since most controllers do not support multiple control
// transfers on multiple devices concurrently and control transfers are mainly
// used for enumeration, we will only execute control transfers one at a time.

// Control transfers: since most controllers do not support multiple control transfers
// on multiple devices concurrently and control transfers are not used much except for
// enumeration, we will only execute control transfers one at a time.
//
// CFG_TUH_MEM_SECTION struct {
//   CFG_TUH_MEM_ALIGN tusb_control_request_t request;
//   uint8_t* buffer;
//   tuh_xfer_cb_t complete_cb;
//   uintptr_t user_data;
//
//   uint8_t daddr;
//   volatile uint8_t stage;
//   volatile uint16_t actual_len;
// } _ctrl_xfer;

// Submit a transfer and, when complete, queue an EVENT_TRANSFER event
// Abort a transfer, only if not yet started. Return true if queue xfer aborted
// Send a SETUP transfer. When complete, queue an EVENT_TRANSFER event
// Clear a stall and toggle data PID back to DATA0

// struct tuh_xfer_t {
//   uint8_t daddr;
//   uint8_t ep_addr;
//   xfer_result_t result;
//   uint32_t actual_len;      // excluding setup packet
//   union {
//     tusb_control_request_t const* setup; // setup packet pointer if control transfer
//     uint32_t buflen;                     // expected length if not control transfer (not available in callback)
//   };
//   uint8_t*      buffer; // not available in callback if not control transfer
//   tuh_xfer_cb_t complete_cb;
//   uintptr_t     user_data;
// };
//
// tuh_xfer_t xfer = {
//   .daddr       = daddr,
//   .ep_addr     = 0,
//   .setup       = &request,
//   .buffer      = buffer,
//   .complete_cb = complete_cb,
//   .user_data   = user_data
// };

// Start a control transfer
void start_control_transfer(endpoint_t *ep, usb_setup_packet_t *packet) {
    uint8_t size = sizeof(usb_setup_packet_t);
    uint32_t ecr, bcr, scr;

    // TODO: Add assert conditions...

    // Determine direction
    bool in = packet->bmRequestType & USB_DIR_IN; // TODO: Do we need a "packet_dir" and "data_dir" type of thing?
    ep->sender = !in;

    // Control transfers start with a setup packet // TODO: is "for (i=0; i<8; i++)"" better???
    memcpy((void *) usbh_dpram->setup_packet, packet, size);

    // Calculate register values
    ecr = usbh_dpram->epx_ctrl;
    bcr = USB_BUF_CTRL_LAST
        | USB_BUF_CTRL_DATA1_PID
        | USB_BUF_CTRL_SEL // TODO: Says device only?
        | size;
    scr = USB_SIE_CTRL_BASE                    // SIE_CTRL defaults
        | USB_SIE_CTRL_SEND_SETUP_BITS         // Send a SETUP packet
        | (in ? USB_SIE_CTRL_RECEIVE_DATA_BITS // Receive if IN to host
              : USB_SIE_CTRL_SEND_DATA_BITS);  // Send if OUT from host
        // TODO: preamble (LS on FS)

    // Debug output
    bindump(" ECR", ecr);
    bindump(" BCR", bcr | USB_BUF_CTRL_AVAIL);
    bindump(" SCR", scr | USB_SIE_CTRL_START_TRANS_BITS);

    if (size == 0) {
        printf("<ZLP\n");
    } else {
        printf("<Setup");
        hexdump(packet, size, 1);
    }

    // Transfer is now active
    ep->active = true;

    // NOTE: We might be able to collapse theses delays...

    // Set BCR (epx_buf_ctrl)
    // Datasheet § 4.1.2.5.1 (p. 383) says that when clk_sys (usually 133Mhz)
    // and clk_usb (usually 48MHz) are different, we must wait one USB clock
    // cycle before setting the AVAILABLE bit. Based on this, we should wait
    // 133MHz/48MHz * 1 clk_usb cycle = 2.8 clk_sys cycles (rounds up to 3).
    hw_set_staged3(usbh_dpram->epx_buf_ctrl, bcr, USB_BUF_CTRL_AVAIL);

    // Set SCR (sie_ctrl)
    // Datasheet § 4.1.2.7 (p. 390) says that when clk_sys (usually 133Mhz)
    // and clk_usb (usually 48MHz) are different, we must wait two USB clock
    // cycles before setting the START_TRANS bit. Based on this, we need
    // 133MHz/48MHz * 2 clk_usb cycles = 5.6 clk_sys cycles (rounds up to 6).
    //
    // NOTE: TinyUSB doesn't wait here, just sayin'... can we combine w/above?
    hw_set_staged6(usb_hw->sie_ctrl, scr, USB_SIE_CTRL_START_TRANS_BITS);
}

//     // Set target device address and endpoint number
//     // NOTE: 19:16=ep_num, 6:0=dev_addr
//     // *ep->dac = (uint32_t) (dev_addr | (ep_num << USB_ADDR_ENDP_ENDPOINT_LSB));
//     *ep->dac = 0;
//
//         // Set BCR
//         bcr = USB_BUF_CTRL_FULL // Indicates we've populated the buffer
//             | USB_BUF_CTRL_LAST
//             | USB_BUF_CTRL_DATA1_PID
//             | USB_BUF_CTRL_SEL;
//         //  | size; // just happens to be zero
//
//         // Set SCR
//         scr = USB_SIE_CTRL_BASE            // SIE_CTRL defaults
//             | USB_SIE_CTRL_SEND_DATA_BITS; // OUT from host is send

// // Send a zero length status packet (ZLP)
// SDK_ALWAYS_INLINE static inline void send_zlp(endpoint_t *ep) {
//     start_control_transfer(ep, NULL, 0); // TODO: This isn't correct... it should be the end of a transfer
// }

// ==[ Device Enumeration ]=================================================

enum {
    DISCONNECTED,
    LOW_SPEED,
    FULL_SPEED,
};

enum {
    DEVICE_DISCONNECTED,
    DEVICE_CONNECTED,
    DEVICE_ADDRESSED,
    DEVICE_CONFIGURED,
    DEVICE_ACTIVE,
    DEVICE_SUSPENDED,
};

// TODO: We might be able to use the above DEVICE_* states to drive the enumeration
enum {
    ENUMERATION_START,
    ENUMERATION_GET_MAXSIZE,
    ENUMERATION_SET_ADDRESS,
    ENUMERATION_GET_DEVICE,
    ENUMERATION_GET_CONFIG,
    ENUMERATION_SET_CONFIG,
    ENUMERATION_END,
};

typedef struct device {
    uint8_t  speed       ; // Device speed (0:disconnected, 1:full, 2:high)
    uint8_t  state       ; // Current device state
    uint8_t  maxsize     ; // Maximum packet size // TODO: Is this just EP0?
    uint16_t vid         ; // Vendor Id  (0x0403: FTDI)
    uint16_t pid         ; // Product Id (0xcd18: Abaxis Piccolo Xpress)
    uint8_t  manufacturer; // String index of manufacturer
    uint8_t  product     ; // String index of product
    uint8_t  serial      ; // String index of serial number
} device_t;

// TODO: For right now, only define dev0 and 1 device. Later, we'll make this dynamic.
static device_t devices[MAX_DEVICES], *dev0 = devices;

// Get device descriptor
void get_device_descriptor() {
    printf("Get device descriptor\n");

    // Determine the device and how many bytes to ask for
    device_t *dev = &devices[epx->dev_addr]; // TODO: Make this a function call, with bounds checking, etc.
    uint16_t len = dev->maxsize;

    // Setup packet
    usb_setup_packet_t packet = {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(USB_DT_DEVICE, 0),
        .wIndex        = 0,
        .wLength       = len ? len : 8, // Only 8 bytes if we don't know
    };

    start_control_transfer(epx, &packet);
}

// // Set device address
// void set_device_address() {
//     uint8_t dev_addr = 1;
//
//     printf("Set device address to %u\n", dev_addr);
//
//     // Setup packet
//     usb_setup_packet_t packet = {
//         .bmRequestType = USB_DIR_OUT
//                        | USB_REQ_TYPE_STANDARD
//                        | USB_REQ_TYPE_RECIPIENT_DEVICE,
//         .bRequest      = USB_REQUEST_SET_ADDRESS,
//         .wValue        = dev_addr,
//         .wIndex        = 0,
//         .wLength       = 0,
//     };
//
//     start_control_transfer(epx, &packet, sizeof(packet));
// }

// static uint8_t get_new_address(bool is_hub) {
//   uint8_t start;
//   uint8_t end;
//
//   if ( is_hub ) {
//     start = CFG_TUH_DEVICE_MAX;
//     end   = start + CFG_TUH_HUB;
//   } else {
//     start = 0;
//     end   = start + CFG_TUH_DEVICE_MAX;
//   }
//
//   for (uint8_t idx = start; idx < end; idx++) {
//     if (!_usbh_devices[idx].connected) return (idx+1);
//   }
//
//   return 0; // invalid address
// }

void enumerate() {
    static uint8_t step;
    if (!dev0->maxsize) step = ENUMERATION_START;

    // Handle prior step
    switch (step) {
        case ENUMERATION_START:
            printf("Start enumeration\n");
            break;

        case ENUMERATION_GET_MAXSIZE:
            // Set dev0->maxsize
            break;

        case ENUMERATION_SET_ADDRESS:
            // Set device address
            break;

        case ENUMERATION_GET_DEVICE:
            // Load the device info
            break;

        case ENUMERATION_GET_CONFIG:
            // Load the vid, pid, manufacturer, product, and serial
            break;

        case ENUMERATION_SET_CONFIG:
            // Set device configuration
            break;
    }

    // Handle next step
    switch (++step) {
        case ENUMERATION_GET_MAXSIZE:
            printf("Get maximum packet size\n");
            get_device_descriptor();
            break;

        case ENUMERATION_SET_ADDRESS:
            printf("Set device address\n");
            break;

        case ENUMERATION_GET_DEVICE:
            printf("Get device descriptor\n");
            break;

        case ENUMERATION_GET_CONFIG:
            printf("Get configuration descriptor\n");
            break;

        case ENUMERATION_SET_CONFIG:
            printf("Set device configuration\n");
            break;

        case ENUMERATION_END:
            printf("End enumeration\n");
            dev0->state = DEVICE_CONFIGURED;
            break;
    }
}

// ==[ Interrupts ]============================================================

// Interrupt handler
void isr_usbctrl() {
    volatile uint32_t intr = usb_hw->intr;
    volatile uint32_t ints = usb_hw->ints;
    static event_t event; // TODO: We can probably push event queueing to one function

    printf("┌───────┬──────┬──────────────────────────────────────────────────┐\n");
    printf("│Frame\t│ %4u │%50s│\n", usb_hw->sof_rd, "");
    bindump("│INTR", intr);
    bindump("│INTS", ints);
    bindump("│SIE", usb_hw->sie_status);
    bindump("│DAC", usb_hw->dev_addr_ctrl);
    bindump("│ECR", usbh_dpram->epx_ctrl);
    bindump("│BCR", usbh_dpram->epx_buf_ctrl);

    // Connection event (attach or detach)
    if (ints &  USB_INTS_HOST_CONN_DIS_BITS) {
        ints ^= USB_INTS_HOST_CONN_DIS_BITS;

        // Get the device speed and clear the interrupt
        uint8_t speed = get_speed();
        usb_hw_clear->sie_status = USB_SIE_STATUS_SPEED_BITS; // TODO: If I clear too soon, will it block reading the speed? If I don't clear it, will it continue to fire?

        if (speed) {
            printf("│ISR\t│ Device connected\n");
            queue_add_blocking(queue, &((event_t) {
                .type       = EVENT_CONNECT,
                .dev_addr   = 0,
                .conn.speed = speed,
            }));
        } else {
            printf("│ISR\t│ Device disconnected\n");
        }
    }

    // Stall detected (higher priority than BUFF_STATUS and TRANS_COMPLETE)
    if (ints &  USB_INTS_STALL_BITS) {
        ints ^= USB_INTS_STALL_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;

        printf("│ISR\t│ Stall detected\n");

        // Queue the stalled transfer
        event.type         = EVENT_TRANSFER;
        event.xfer.ep_addr = 37; // TODO: Will need this and maybe some more info?
        event.xfer.result  = TRANSFER_STALLED;
        event.xfer.len     = 0; // TODO: Do we need this?
        queue_add_blocking(queue, &event);
    }

    // Buffer(s) ready
    if (ints &  USB_INTS_BUFF_STATUS_BITS) {
        ints ^= USB_INTS_BUFF_STATUS_BITS;

        // Find the buffer(s) that are ready
        uint32_t bits = usb_hw->buf_status;
        uint32_t mask = 1u;

        // See if EPX is single or double buffered
        uint32_t ecr = usbh_dpram->epx_ctrl;
        bool dubs = (bits & mask) && (ecr & EP_CTRL_DOUBLE_BUFFERED_BITS);
        char *str = dubs ? "│BUF(2)" : "│BUF(1)";
        bindump(str, bits);

        // Clear all buffer bits, panic later if we missed any
        usb_hw_clear->buf_status = (uint32_t) ~0;

        // NOTE: Miroslav says we should handle these in pairs of IN/OUT
        // endpoints, since they "come in pairs". So, we would deal with
        // EP3IN/EP3OUT at the same time and mask with 0b11, etc.

        // Check the interrupt/asynchronous endpoints (IN and OUT)
        // for (uint8_t i = 0; i <= USB_HOST_INTERRUPT_ENDPOINTS && bits; i++) {
        for (uint8_t i = 0; i <= 1 && bits; i++) { // TODO: This is hacked
            for (uint8_t j = 0; j < 2; j++) {
                mask = 1 << (i * 2 + j);
                if (bits &  mask) {
                    bits ^= mask;
                    handle_buffer(mask, &eps[i]);
                }
            }
        }

        // Panic if we missed any buffers
        if (bits) panic("Unhandled buffer mask: %032b\n", bits);

        uint8_t len = 8;
        if (len) {
            printf("│> Data");
            hexdump(usbh_dpram->epx_data, len, 1);
        } else {
            printf("│<ZLP\n"); // which direction?!
        }
    }

    // Transfer complete (last packet)
    if (ints &  USB_INTS_TRANS_COMPLETE_BITS) {
        ints ^= USB_INTS_TRANS_COMPLETE_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;

        // NOTE: TRANS_COMPLETE will fire in these cases (see datasheet p. 401)
        //
        // 1. A setup packet was sent without a following IN or OUT. This can
        //    occur if USB_SIE_CTRL_{RECEIVE,SEND}_DATA_BITS are NOT set when
        //    sending the setup packet (like TinyUSB does). In our case, we DO
        //    set those bits, so we should NOT see a TRANS_COMPLETE. (Verified!)
        // 2. An IN packet is received and the LAST_BUFF bit was set in the
        //    buffer control register. (Who would set this? Us? Why?)
// *==> // 3. An IN packet is received with zero length (ZLP). QUESTION: Does this also set LAST_BUFF? Is this why (2) is here?
        // 4. An OUT packet is sent and the LAST_BUFF bit was set.
        // 5. QUESTION: What if we sent an OUT with buflen=0? Would it trigger?

//         // TODO: Nearly same as BUFF_STATUS, how can we share code better?
//         if (usb_hw->sie_ctrl & USB_SIE_CTRL_SEND_SETUP_BITS) {
//             printf("│ISR\t│      │ Setup packet sent\n");
//
//             endpoint_t *ep = epx;
//             assert(ep->active);
//             event.type         = EVENT_TRANSFER;
//             event.dev_addr     = ep->dev_addr;
//             event.xfer.ep_addr = ep->ep_addr;
//             event.xfer.result  = TRANSFER_SUCCESS;
//             event.xfer.len     = ep->bytes_done = 8; // Size of a setup packet
//             // reset_endpoint(ep);
//             queue_add_blocking(queue, &event);
//         }
    }

    // Receive timeout (too long without an ACK)
    if (ints &  USB_INTS_ERROR_RX_TIMEOUT_BITS) {
        ints ^= USB_INTS_ERROR_RX_TIMEOUT_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;

        printf("│ISR\t│ Receive timeout\n");
    }

    // Data error (IN packet from device has wrong data PID)
    if (ints &  USB_INTS_ERROR_DATA_SEQ_BITS) {
        ints ^= USB_INTS_ERROR_DATA_SEQ_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;

        printf("│ISR\t│ Data error\n");

        panic("ERROR: USB Host data sequence error\n");
    }

    // Device resumed (device initiated)
    if (ints &  USB_INTS_HOST_RESUME_BITS) {
        ints ^= USB_INTS_HOST_RESUME_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;

        printf("│ISR\t│ Device resume\n");
    }

    // Any missed?
    if (ints) {
        panic("│ISR\t│ Unhandled IRQ 0x%04x\n", ints);
    }

    printf("└───────┴──────┴──────────────────────────────────────────────────┘\n");
}

// ==[ Resets ]================================================================

// Reset USB host
void usb_host_reset() {
    printf("USB host reset\n\n");

// static void clear_device(usbh_device_t* dev) {
//   tu_memclr(dev, sizeof(usbh_device_t));
//   memset(dev->itf2drv, TUSB_INDEX_INVALID_8, sizeof(dev->itf2drv)); // invalid mapping
//   memset(dev->ep2drv , TUSB_INDEX_INVALID_8, sizeof(dev->ep2drv )); // invalid mapping
// }
//   tu_memclr(&_dev0, sizeof(_dev0));
//   tu_memclr(_usbh_devices, sizeof(_usbh_devices));
//   tu_memclr(&_ctrl_xfer, sizeof(_ctrl_xfer));

    // Reset controller
    reset_block       (RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    // Clear state
    memset(usb_hw    , 0, sizeof(*usb_hw    ));
    memset(usbh_dpram, 0, sizeof(*usbh_dpram));

    // Configure USB host controller
    usb_hw->muxing    = USB_USB_MUXING_TO_PHY_BITS                // Connect USB Phy
                      | USB_USB_MUXING_SOFTCON_BITS;              // TODO: What is this?
    usb_hw->pwr       = USB_USB_PWR_VBUS_DETECT_BITS              // Enable VBUS detection
                      | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS; // Enable VBUS detection
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS          // Enable controller
                      | USB_MAIN_CTRL_HOST_NDEVICE_BITS;          // Enable USB Host mode
    usb_hw->sie_ctrl  = USB_SIE_CTRL_BASE;                        // Default SIE_CTRL bits
    usb_hw->inte      = USB_INTE_HOST_CONN_DIS_BITS               // Device connect/disconnect
                      | USB_INTE_STALL_BITS                       // Stall detected
                      | USB_INTE_BUFF_STATUS_BITS                 // Buffer ready
                      | USB_INTE_TRANS_COMPLETE_BITS              // Transfer complete
                      | USB_INTE_HOST_RESUME_BITS                 // Device resumed
                      | USB_INTE_ERROR_DATA_SEQ_BITS              // Data error
                      | USB_INTE_ERROR_RX_TIMEOUT_BITS;           // Receive timeout

    // Setup hardware endpoints
    setup_endpoints();

    bindump(" INT", usb_hw->inte);

    irq_set_enabled(USBCTRL_IRQ, true);
}

// ==[ Main ]==================================================================

void usb_task() {
    static event_t event;

    while (queue_try_remove(queue, &event)) { // TODO: Can this starve out other work? Should it be "if (...) {" instead?
        switch (event.type) {
            case EVENT_CONNECT:

                // TODO: See if we can get this to work
                // // Prevent nested connections
                // if (dev0->state == DEVICE_ENUMERATING) {
                //     printf("Only one device can be enumerated at a time\n");
                //     break;
                // }

                // Initialize device 0
                memclr(dev0, sizeof(device_t));
                dev0->speed = event.conn.speed;
                dev0->state = DEVICE_CONNECTED;

                // Show the device connection and speed
                char *str = dev0->speed == LOW_SPEED ? "low" : "full";
                printf("Device connected (%s speed)\n", str);

                // Start the enumeration process
                enumerate();
                break;

            case EVENT_TRANSFER:
                printf("Transfer complete (from queue)\n");
                // if (event.xfer.len == 0) {
                //     send_zlp(epx);
                // }
                break;

            case EVENT_FUNCTION:
                printf("Function call\n");
                event.fn.call(event.fn.arg);
                break;

            default:
                printf("Unknown event type\n");
                break;
        }
    }
}

int main() {
    stdio_init_all();
    printf("\033[2J\033[H\n==[ USB host example]==\n\n");
    usb_host_reset();

    queue_init(queue, sizeof(event_t), 64);

    while (1) {
        usb_task();
    }
}
