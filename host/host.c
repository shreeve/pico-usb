// =============================================================================
// PicoUSB - A USB host and device library for the Raspberry Pi Pico/W
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: January 29, 2024
//   Note: This is a work in progress. It is not yet functional.
//  Legal: Same license as the Raspberry Pi Pico SDK
//
// Thanks to Ha Thach for TinyUSB and https://github.com/hathach/tinyusb
// Thanks to Miroslav Nemecek for his https://github.com/Panda381/PicoLibSDK
// =============================================================================

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

// ==[ PicoUSB ]================================================================

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

// ==[ Endpoints ]==============================================================

typedef void (*endpoint_c)(uint8_t *buf, uint16_t len);

typedef struct {
    uint8_t    dev_addr  ; // Device address // HOST ONLY
    uint8_t    ep_addr   ; // Endpoint address
    uint8_t    type      ; // Transfer type: control/bulk/interrupt/isochronous
    uint16_t   maxsize   ; // Maximum packet size
    uint16_t   interval  ; // Polling interval in ms
    bool       configured; // Endpoint is configured
    bool       active    ; // Transfer is active
    bool       setup     ; // SETUP packet flag // TODO: How useful is this?
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

    // Calculate the ECR
    uint32_t type   = ep->type;
    uint32_t ms     = ep->interval;
    uint32_t lsb    = EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB;
    uint32_t offset = offsetof(usb_host_dpram_t, epx_data); // TODO: Make this generic, not EPX specific
    uint32_t ecr    = EP_CTRL_ENABLE_BITS                 // Enable endpoint
                    | EP_CTRL_DOUBLE_BUFFERED_BITS        // Double buffering
                    | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER // INT per double
                    | type << EP_CTRL_BUFFER_TYPE_LSB     // Set transfer type
                    | (ms ? ms - 1 : 0) << lsb            // Polling time in ms
                    | offset;                             // Data buffer offset

    // Debug output
    show_endpoint(ep, "Reset");
    bindump(" ECR", ecr);

    // Set the ECR
    usbh_dpram->epx_ctrl = ecr;
}

SDK_INLINE void clear_endpoint(endpoint_t *ep) {
    ep->active     = false;
    ep->setup      = false;
    ep->user_buf   = temp_buf; // TODO: Add something like a ring buffer here?
    ep->bytes_left = 0;
    ep->bytes_done = 0;
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

endpoint_t *next_endpoint(uint8_t dev_addr, usb_endpoint_descriptor_t *usb) {
    for (uint8_t i = 1; i < MAX_ENDPOINTS; i++) {
        endpoint_t *ep = &eps[i];
        if (!ep->configured) {
            ep->dev_addr = dev_addr;
            setup_endpoint(ep, usb);
            return ep;
        }
    }
    panic("No free endpoints remaining"); // TODO: Handle this properly
    return NULL;
}

void reset_epx() {
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
    reset_epx();
}

// ==[ Buffers ]================================================================

// Sync current buffer by checking its BCR half and returning the buffer length
uint16_t sync_buffer(endpoint_t *ep, uint8_t buf_id, uint32_t bcr) {
    bool     in   = ep_in(ep);                   // Buffer is inbound
    bool     full = bcr & USB_BUF_CTRL_FULL;     // Buffer is full (populated)
    uint16_t len  = bcr & USB_BUF_CTRL_LEN_MASK; // Buffer length

    // Inbound buffers must be full and outbound buffers must be empty
    assert(in == full);

    // Copy inbound data from the data buffer to the user buffer
    if (in && len) {
        memcpy(ep->user_buf, (void *) (ep->data_buf + buf_id * 64), len);
        hexdump(buf_id ? "│IN/2" : "│IN/1", ep->user_buf, len, 1);
        ep->user_buf += len;
    }

    // Update byte counts
    ep->bytes_done += len;

    // Short packet (below maxsize) means the transfer is done
    if (len < ep->maxsize) {
        ep->bytes_left = 0;
    }

    return len;
}

// Prepare next buffer and return its BCR half
uint32_t next_buffer(endpoint_t *ep, uint8_t buf_id) {
    bool     in  = ep_in(ep);                         // Buffer is inbound
    bool     mas = ep->bytes_left > ep->maxsize;      // Any more packets?
    uint8_t  pid = ep->data_pid;                      // Set DATA0/DATA1
    uint16_t len = MIN(ep->maxsize, ep->bytes_left);  // Buffer length
    uint32_t bcr = (in  ? 0 : USB_BUF_CTRL_FULL)      // IN/Recv=0, OUT/Send=1
                 | (mas ? 0 : USB_BUF_CTRL_LAST)      // Trigger TRANS_COMPLETE
                 | (pid ?     USB_BUF_CTRL_DATA1_PID  // Use DATA1 if needed
                            : USB_BUF_CTRL_DATA0_PID) // Use DATA0 if needed
                 |            USB_BUF_CTRL_AVAIL      // Buffer available now
                 | len;                               // Length of next buffer

    // Toggle DATA0/DATA1 pid
    ep->data_pid = pid ^ 1u;

    // Copy outbound data from the user buffer to the data buffer
    if (!in && len) {
        memcpy((void *) (ep->data_buf + buf_id * 64), ep->user_buf, len);
        hexdump(buf_id ? "│OUT/2" : "│OUT/1", ep->user_buf, len, 1);
        ep->user_buf += len;
    }

    // Update byte counts
    ep->bytes_left -= len;

    return bcr;
}

void handle_buffers(endpoint_t *ep) {
    if (!ep->active) show_endpoint(ep, "Inactive"), panic("Halted");

    // -- Sync current buffer(s) -----------------------------------------------

    uint32_t ecr = usbh_dpram->epx_ctrl;              // ECR is single or double
    uint32_t bcr = usbh_dpram->epx_buf_ctrl;          // Buffer control register
    if (ecr & EP_CTRL_DOUBLE_BUFFERED_BITS) {         // When double buffered...
     // sync_buffer(ep, 0, bcr); // TODO: Remove, this is for the 1 INT per buff
        if (sync_buffer(ep, 0, bcr) == ep->maxsize)   // If first buffer is full
            if (ep->bytes_left)                       // And, there's more data
                sync_buffer(ep, 1, bcr >> 16);        // Then, sync second also
    } else {                                          // When single buffered...
        uint32_t bch = usb_hw->buf_cpu_should_handle; // Check CPU handling bits
        if (bch & 1u) bcr >>= 16;                     // Do RP2040-E4 workaround
        sync_buffer(ep, 0, bcr);                      // And sync the one buffer
    }

    // -- Debug output ---------------------------------------------------------

    if (!ep->bytes_done) {
        char *str = ep_in(ep) ? "│ZLP/I" : "│ZLP/O";
        bindump(str, 0);
    }

    // Return if the transfer is done
    if (!ep->bytes_left) return;

    // -- Prepare next buffer(s) -----------------------------------------------

    ecr = usbh_dpram->epx_ctrl; // TODO: Add ep->ecr so it'll work with any endpoint
    bcr = next_buffer(ep, 0);

    if (~bcr & USB_BUF_CTRL_LAST) {
        ecr |= EP_CTRL_DOUBLE_BUFFERED_BITS;
        bcr |= next_buffer(ep, 1) << 16;
    } else {
        ecr &= ~EP_CTRL_DOUBLE_BUFFERED_BITS;
    }

    // Debug output
    if (ep->bytes_left + ep->bytes_done) {
        printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
        printf( "│Buff\t│ %4s │ %-35s │%12s│\n", "", "Buffer Handler", "");
        bindump("│DAR", usb_hw->dev_addr_ctrl); // Device address register
        bindump("│SSR", usb_hw->sie_status);    // SIE status register
        bindump("│SCR", usb_hw->sie_ctrl);      // SIE control register
        bindump("│ECR", ecr);                   // EPX control register
        bindump("│BCR", bcr);                   // EPX buffer control register
        printf( "└───────┴──────┴─────────────────────────────────────┴────────────┘\n");
    }

    // Available bits for the buffer control register
    uint32_t available = USB_BUF_CTRL_AVAIL << 16 | USB_BUF_CTRL_AVAIL;

    // Update ECR and BCR (do BCR first so controller has time to respond)
    usbh_dpram->epx_buf_ctrl = bcr & ~available;
    usbh_dpram->epx_ctrl     = ecr;
    nop();
    nop();
    usbh_dpram->epx_buf_ctrl = bcr;
}

// ==[ Devices ]================================================================

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

typedef struct {
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

// ==[ Transfers ]==============================================================

enum {
    TRANSFER_SUCCESS, // used
    TRANSFER_FAILED,  //
    TRANSFER_STALLED, // used
    TRANSFER_TIMEOUT, //
    TRANSFER_INVALID, //
};

enum {
    USB_SIE_CTRL_BASE = USB_SIE_CTRL_PULLDOWN_EN_BITS   // Ready for devices
                      | USB_SIE_CTRL_VBUS_EN_BITS       // Supply VBUS
                      | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS // Enable low speed
                      | USB_SIE_CTRL_SOF_EN_BITS        // Enable full speed
};

// TODO: Clear a stall and toggle data PID back to DATA0
// TODO: Abort a transfer if not yet started and return true on success

void transfer(endpoint_t *ep) {
    bool in = ep_in(ep);
    bool su = ep->setup && !ep->bytes_done; // Start of a SETUP packet

    // TODO: Come up with a way to show a SETUP, ZLP, or DATA transfers here
    if (su) {
        uint32_t *packet = (uint32_t *) usbh_dpram->setup_packet;
        hexdump(" SETUP", packet, sizeof(usb_setup_packet_t), 1);
    }

    // If there's no data phase, flip the endpoint direction
    if (!ep->bytes_left) {
        in = !in;
        ep->ep_addr ^= USB_DIR_IN;
    }

    // Calculate registers
    uint8_t  lsb = USB_ADDR_ENDP_ENDPOINT_LSB;       // LSB for the ep_num
    uint32_t dar = ep->dev_addr | ep_num(ep) << lsb; // Has dev_addr and ep_num
    uint32_t scr = USB_SIE_CTRL_BASE                 // SIE_CTRL defaults
 //   | (ls  ? 0 : USB_SIE_CTRL_PREAMBLE_EN_BITS)    // Preamble (LS on FS hub)
      | (!su ? 0 : USB_SIE_CTRL_SEND_SETUP_BITS)     // Toggle SETUP packet
      | (in  ?     USB_SIE_CTRL_RECEIVE_DATA_BITS    // Receive bit means IN
                 : USB_SIE_CTRL_SEND_DATA_BITS)      // Send bit means OUT
      |            USB_SIE_CTRL_START_TRANS_BITS;    // Start the transfer now

    // Perform the transfer
    usb_hw->dev_addr_ctrl = dar;
    usb_hw->sie_ctrl      = scr & ~USB_SIE_CTRL_START_TRANS_BITS;
    next_buffers(ep); // Queue next buffers and give SCR time to settle
    usb_hw->sie_ctrl      = scr;
}

void start_control_transfer(endpoint_t *ep, usb_setup_packet_t *setup) {
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

    // Copy the setup packet
    memcpy((void*) usbh_dpram->setup_packet, setup, sizeof(usb_setup_packet_t));

    // Transfer is now active
    ep->active     = true;
    ep->setup      = true;
    ep->data_pid   = 1;
    ep->ep_addr    = setup->bmRequestType & USB_DIR_IN;
    ep->bytes_left = setup->wLength;
    ep->bytes_done = 0;
    ep->user_buf   = temp_buf;

    // Debug output
    show_endpoint(ep, "Start");

    transfer(ep);
}

void transfer_zlp(void *arg) {
    endpoint_t *ep = (endpoint_t *) arg;

    // Transfer is now active
    ep->active     = true;
    ep->setup      = false;
    ep->data_pid   = 1;
    ep->ep_addr    = ep->ep_addr;
    ep->bytes_left = 0;
    ep->bytes_done = 0;
    ep->user_buf   = temp_buf;

    // // Debug output
    // show_endpoint(ep, "ZLP");

    transfer(ep);
}

// ==[ Enumeration ]============================================================

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

            printf("Starting GET_MAXSIZE\n");
            get_device_descriptor(epx);
            break;

        case ENUMERATION_GET_MAXSIZE: {
            uint8_t maxsize0 =
                ((usb_device_descriptor_t *) epx->data_buf)->bMaxPacketSize0;

            printf("Starting SET_ADDRESS\n");

            // Allocate a new device
            new_addr      = next_dev_addr();
            device_t *dev = get_device(new_addr);
            dev->speed    = dev0->speed;
            dev->state    = DEVICE_ALLOCATED;

            // Allocate EP0 for the new device
            endpoint_t *ep = next_endpoint(new_addr, &((usb_endpoint_descriptor_t) {
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
            desc = (usb_device_descriptor_t *) temp_buf; // TODO: We should have a different buffer here...

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

// ==[ Setup USB Host ]=========================================================

void setup_usb_host() {
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

// ==[ Tasks ]==================================================================

enum {
    TASK_CONNECT,
    TASK_TRANSFER,
    TASK_CALLBACK,
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
        } callback;
    };
} task_t;

static uint32_t guid = 1;

static queue_t *queue = &((queue_t) { 0 });

const char *task_name(uint8_t type) {
    switch (type) {
        case TASK_CONNECT:  return "TASK_CONNECT";
        case TASK_TRANSFER: return "TASK_TRANSFER";
        case TASK_CALLBACK: return "TASK_CALLBACK";
        default:            return "UNKNOWN";
    }
    panic("Unknown task queued");
}

const char *callback_name(void (*fn) (void *)) {
    if (fn == enumerate   ) return "enumerate";
    if (fn == transfer_zlp) return "transfer_zlp";
    panic("Unknown callback queued");
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

                // Start enumeration
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

            case TASK_CALLBACK: {
                printf("Calling %s\n", callback_name(task.callback.fn));
                task.callback.fn(task.callback.arg);
            }   break;

            default:
                printf("Unknown task queued\n");
                break;
        }
        printf("=> Finish task #%u: %s\n", task.guid, task_name(type));
    }
}

// ==[ Interrupts ]=============================================================

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
    task_t task;

    // Load some registers into local variables
    uint32_t ints = usb_hw->ints;
    uint32_t dar  = usb_hw->dev_addr_ctrl;              // dev_addr/ep_num
    uint32_t ecr  = usbh_dpram->epx_ctrl;               // Endpoint control
    uint32_t bcr  = usbh_dpram->epx_buf_ctrl;           // Buffer control
    bool     dub  = ecr & EP_CTRL_DOUBLE_BUFFERED_BITS; // EPX double buffered

    // Fix RP2040-E4 by shifting buffer control registers for affected buffers
    if (!dub && (usb_hw->buf_cpu_should_handle & 1u)) bcr >>= 16; // Fix EPX
    // TODO: Add a similar fix for all polled endpoints

    // Get device address and endpoint information
    uint8_t dev_addr =  dar & USB_ADDR_ENDP_ADDRESS_BITS;
    uint8_t ep_addr  = (dar & USB_ADDR_ENDP_ENDPOINT_BITS) >>
                              USB_ADDR_ENDP_ENDPOINT_LSB;
    endpoint_t *ep = find_endpoint(dev_addr, ep_addr);

    // Show system state
    printf( "\n=> New ISR #%u", guid++);
    printf_interrupts(ints);
    printf( "\n");
    printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
    printf( "│Frame\t│ %4u │ %-35s │%12s│\n", usb_hw->sof_rd, "Interrupt Handler", "");
    bindump("│INTR", usb_hw->intr);
    bindump("│INTS", ints);
    bindump("│DAR" , dar);
    bindump("│SSR" , usb_hw->sie_status);
    bindump("│SCR" , usb_hw->sie_ctrl);
    bindump("│ECR" , ecr);
    bindump("│BCR" , bcr);

    // Connection (attach or detach)
    if (ints &  USB_INTS_HOST_CONN_DIS_BITS) {
        ints ^= USB_INTS_HOST_CONN_DIS_BITS;

        // Get the device speed
        uint8_t speed = (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS)
                                           >> USB_SIE_STATUS_SPEED_LSB;

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
            reset_epx(); // TODO: There's a lot more to do here
        }
    }

    // Stall detected (higher priority than BUFF_STATUS and TRANS_COMPLETE)
    if (ints &  USB_INTS_STALL_BITS) {
        ints ^= USB_INTS_STALL_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;

        printf("Stall detected\n");

//         // Queue the stalled transfer
//         queue_add_blocking(queue, &((task_t) {
//             .type              = TASK_TRANSFER,
//             .guid              = guid++,
//             .transfer.dev_addr = 999, // TODO: Need to flesh this out
//             .transfer.ep_addr  = 999, // TODO: Need to flesh this out
//             .transfer.len      = 999, // TODO: Need to flesh this out
//             .transfer.status   = TRANSFER_STALLED,
//         }));
    }

    // Buffer processing is needed
    if (ints &  USB_INTS_BUFF_STATUS_BITS) {
        ints ^= USB_INTS_BUFF_STATUS_BITS;

        // Find the buffer(s) that are ready
        uint32_t bits = usb_hw->buf_status;
        uint32_t mask = 1u;

        // Show single/double buffer status of EPX and which buffers are ready
        printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
        bindump(dub ? "│BUF/2" : "│BUF/1", bits);

        // Lookup the endpoint
        handle_buffers(ep); usb_hw_clear->buf_status = ~0; bits ^= 0x01; // TODO: TOTAL HACK!

//         // Check the polled endpoints (IN and OUT)
//         for (uint8_t i = 0; i <= MAX_ENDPOINTS && bits; i++) {
//             for (uint8_t j = 0; j < 2; j++) {
//                 mask = 1 << (i * 2 + j);
//                 if (bits &  mask) {
//                     bits ^= mask;
//                     handle_buffers(&eps[i]);
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
        // 1. SETUP packet sent without {RECEIVE,SEND}_DATA_BITS in SCR
        // 2. IN or OUT packet transferred with LAST set in BCR
        // 3. IN short packet (less than maxsize) transferred

        // Panic if the endpoint is not active
        if (!ep->active) panic("EP should still be active in TRANS_COMPLETE");

        // Get the length of the transfer
        uint16_t len = ep->bytes_done;

        // Debug output
        if (len) {
            printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
            printf( "│XFER\t│ %4u │ Device %-28u │ Task #%-4u │\n", len, ep->dev_addr, guid);
            hexdump("│Data", temp_buf, len, 1);
        }

        // Clear the endpoint (since its complete)
        clear_endpoint(ep);

        // Queue a ZLP or advance the enumeration
        queue_add_blocking(queue, &((task_t) {
            .type         = TASK_CALLBACK,
            .guid         = guid++,
            .callback.fn  = len ? transfer_zlp : enumerate,
            .callback.arg = ep,
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

// ==[ Main ]===================================================================

int main() {
    stdio_init_all();
    printf("\033[2J\033[H\n==[ USB host example]==\n\n");
    setup_usb_host();

    queue_init(queue, sizeof(task_t), 64);

    while (1) {
        usb_task();
    }
}
