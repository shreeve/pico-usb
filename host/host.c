// =============================================================================
// PicoUSB - A USB host and device library for the rp2040 (Raspberry Pi Pico/W)
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: January 29, 2024
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

// User defined limits
#define USER_HUBS      0
#define USER_DEVICES   2 // Not including dev0
#define USER_ENDPOINTS 4 // Not including any EP0s

enum {
    MAX_DEVICES   =   1 + USER_DEVICES,
    MAX_ENDPOINTS =   1 + USER_DEVICES + USER_ENDPOINTS,
    MAX_POLLED    =  15, // Maximum polled endpoints
    MAX_TEMP      = 255, // Must be 255 or less
};

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

// TODO: Add debug levels: 0=none, 1=errors, 2=info, 3=debug
// const char *box = "┌─┬┐"  // ╔═╦╗ // ┏━┳┓ // ╭─┬╮ // 0 1 2 3
//                   "│ ││"  // ║ ║║ // ┃ ┃┃ // │ ││ // 4 5 6 7
//                   "├─┼┤"  // ╠═╬╣ // ┣━╋┫ // ├─┼┤ // 8 9 a b
//                   "└─┴┘"; // ╚═╩╝ // ┗━┻┛ // ╰─┴╯ // c d e f

static uint8_t ctrl_buf[MAX_TEMP]; // Buffer for control transfers (shared)
static uint8_t temp_buf[MAX_TEMP]; // TODO: Where is this needed???

void usb_task(); // Forward declaration

// ==[ Endpoints ]==============================================================

typedef void (*endpoint_c)(uint8_t *buf, uint16_t len);

typedef struct {
    uint8_t    dev_addr  ; // Device address // HOST ONLY
    uint8_t    ep_addr   ; // Endpoint address
    uint8_t    type      ; // Transfer type: control/bulk/interrupt/isochronous
    uint16_t   maxsize   ; // Maximum packet size
    uint16_t   interval  ; // Polling interval in ms
    uint8_t    data_pid  ; // Toggle between DATA0/DATA1 packets
    bool       configured; // Endpoint is configured
    bool       active    ; // Transfer is active
    bool       setup     ; // Setup packet flag

    // Hardware registers and data buffer
    io_rw_32  *ecr       ; // Endpoint control register
    io_rw_32  *bcr       ; // Buffer control register
    volatile               // Data buffer is volative
    uint8_t   *buf       ; // Data buffer in DPSRAM

    // Shared with application code
    uint8_t   *user_buf  ; // User buffer in DPSRAM, RAM, or flash
    uint16_t   bytes_left; // Bytes left to transfer
    uint16_t   bytes_done; // Bytes done transferring
    endpoint_c cb        ; // Callback function
} endpoint_t;

static endpoint_t eps[MAX_ENDPOINTS], *epx = eps;

SDK_INLINE const char *ep_dir(endpoint_t *ep) {
    return ep->ep_addr & USB_DIR_IN ? "IN" : "OUT";
}

SDK_INLINE bool ep_in(endpoint_t *ep) {
    return ep->ep_addr & USB_DIR_IN;
}

SDK_INLINE uint8_t ep_num(endpoint_t *ep) {
    return ep->ep_addr & ~USB_DIR_IN;
}

SDK_INLINE void show_endpoint(endpoint_t *ep) {
    printf(" │ %-3uEP%-2d%3s │\n", ep->dev_addr, ep_num(ep), ep_dir(ep));
}

SDK_INLINE void clear_endpoint(endpoint_t *ep) {
    ep->active     = false;
    ep->data_pid   = 0;

    // Transfer state
    ep->setup      = false;
    ep->user_buf   = NULL;
    ep->bytes_left = 0;
    ep->bytes_done = 0;
}

void setup_endpoint(endpoint_t *ep, usb_endpoint_descriptor_t *usb,
                    uint8_t *user_buf) {

    // Populate the endpoint (clears all fields not present)
    *ep = (endpoint_t) {
        .dev_addr = ep->dev_addr,
        .ep_addr  = usb->bEndpointAddress,
        .type     = usb->bmAttributes,
        .maxsize  = usb->wMaxPacketSize,
        .interval = usb->bInterval,
        .user_buf = user_buf != NULL ? user_buf : temp_buf,
    };

    // Setup the necessary registers and data buffer pointer
    if (ep->interval) {
        if (!ep_num(ep)) panic("EP0 cannot be polled");
        uint8_t most = MIN(USER_ENDPOINTS, MAX_POLLED);
        for (uint8_t i = 0; i < most; i++) {
            if (usbh_dpram->int_ep_ctrl[i].ctrl) continue; // Skip if being used
            ep->ecr = &usbh_dpram->int_ep_ctrl       [i].ctrl;
            ep->bcr = &usbh_dpram->int_ep_buffer_ctrl[i].ctrl;
            ep->buf = &usbh_dpram->epx_data[(i + 2) * 64]; // Can't do ISO?
            break;
        }
        if (!ep->ecr) panic("No free polled endpoints remaining");
    } else {
        ep->ecr = &usbh_dpram->epx_ctrl;
        ep->bcr = &usbh_dpram->epx_buf_ctrl;
        ep->buf = &usbh_dpram->epx_data[0];
    }

    // Calculate the ECR
    uint32_t type   = ep->type;
    uint32_t ms     = ep->interval;
    uint32_t lsb    = EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB;
    uint32_t offset = (uint32_t) ep->buf & 0x0fff;        // Offset from DSPRAM
    uint32_t style  = ep->interval                        // Polled endpoint?
                    ? EP_CTRL_INTERRUPT_PER_BUFFER        // Y: Single buffering
                    : EP_CTRL_DOUBLE_BUFFERED_BITS        // N: Double buffering
                    | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER;// and an INT per pair
    uint32_t ecr    = EP_CTRL_ENABLE_BITS                 // Enable endpoint
                    | style                               // Set buffering style
                    | type << EP_CTRL_BUFFER_TYPE_LSB     // Set transfer type
                    | (ms ? ms - 1 : 0) << lsb            // Polling time in ms
                    | offset;                             // Data buffer offset

    // Set the ECR and mark this endpoint as configured
   *ep->ecr = ecr;
    ep->configured = true;
}

endpoint_t *find_endpoint(uint8_t dev_addr, uint8_t ep_addr) {
    bool want_ep0 = !(ep_addr & ~USB_DIR_IN);
    if (!dev_addr && want_ep0) return epx;

    for (uint8_t i = 1; i < MAX_ENDPOINTS; i++) {
        endpoint_t *ep = &eps[i];
        if (ep->configured && ep->dev_addr == dev_addr) {
            if (want_ep0 && !(ep->ep_addr & ~USB_DIR_IN)) return ep;
            if (ep->ep_addr == ep_addr) return ep;
        }
    }
    panic("Invalid endpoint 0x%02x for device %u", ep_addr, dev_addr);
    return NULL;
}

endpoint_t *next_endpoint(uint8_t dev_addr, usb_endpoint_descriptor_t *usb,
                          uint8_t *user_buf) {

    for (uint8_t i = 1; i < MAX_ENDPOINTS; i++) {
        endpoint_t *ep = &eps[i];
        if (!ep->configured) {
            ep->dev_addr = dev_addr;
            setup_endpoint(ep, usb, user_buf);
            return ep;
        }
    }
    panic("No free endpoints remaining");
    return NULL;
}

void reset_epx() {
    setup_endpoint(epx, &((usb_endpoint_descriptor_t) {
        .bLength          = sizeof(usb_endpoint_descriptor_t),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = 0,
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 8, // Default per USB 2.0 spec
        .bInterval        = 0,
    }), NULL);
}

// Clear out all endpoints
void reset_endpoints() {
    memclr(eps, sizeof(eps));
    reset_epx();
}

// ==[ Buffers ]================================================================

enum { // Used to mask availability in the BCR (enum resolves at compile time)
    UNAVAILABLE = ~(USB_BUF_CTRL_AVAIL << 16 | USB_BUF_CTRL_AVAIL)
};

// Read a buffer and return its length
uint16_t read_buffer(endpoint_t *ep, uint8_t buf_id, uint32_t bcr) {
    bool     in   = ep_in(ep);                   // Buffer is inbound
    bool     full = bcr & USB_BUF_CTRL_FULL;     // Buffer is full (populated)
    uint16_t len  = bcr & USB_BUF_CTRL_LEN_MASK; // Buffer length

    // Inbound buffers must be full and outbound buffers must be empty
    assert(in == full);

    // If we are reading data, copy it from the data buffer to the user buffer
    if (in && len) {
        uint8_t *ptr = &ep->user_buf[ep->bytes_done];
        memcpy(ptr, (void *) (ep->buf + buf_id * 64), len);
        hexdump(buf_id ? "│IN/2" : "│IN/1", ptr, len, 1); // ~7.5 ms
        ep->bytes_done += len;
    }

    // // Update byte counts
    // ep->bytes_left -= len;

    // Short packet (below maxsize) means the transfer is done
    if (len < ep->maxsize) {
        ep->bytes_left = 0;
    }

    return len;
}

// Prepare a buffer and return its half of the BCR
uint16_t prep_buffer(endpoint_t *ep, uint8_t buf_id) {
    bool     in  = ep_in(ep);                         // Buffer is inbound
    bool     mas = ep->bytes_left > ep->maxsize;      // Any more packets?
    uint8_t  pid = ep->data_pid;                      // Set DATA0/DATA1
    uint16_t len = MIN(ep->maxsize, ep->bytes_left);  // Buffer length
    uint16_t bcr = (in  ? 0 : USB_BUF_CTRL_FULL)      // IN/Recv=0, OUT/Send=1
                 | (mas ? 0 : USB_BUF_CTRL_LAST)      // Trigger TRANS_COMPLETE
                 | (pid ?     USB_BUF_CTRL_DATA1_PID  // Use DATA1 if needed
                            : USB_BUF_CTRL_DATA0_PID) // Use DATA0 if needed
                 |            USB_BUF_CTRL_AVAIL      // Buffer available now
                 | len;                               // Length of next buffer

    // Toggle DATA0/DATA1 pid
    ep->data_pid = pid ^ 1u;

    // If we are sending data, copy it from the user buffer to the data buffer
    if (!in && len) {
        uint8_t *ptr = &ep->user_buf[ep->bytes_done];
        memcpy((void *) (ep->buf + buf_id * 64), ptr, len);
        hexdump(buf_id ? "│OUT/2" : "│OUT/1", ptr, len, 1);
        ep->bytes_done += len;
    }

    // Update byte counts
    ep->bytes_left -= len;

    return bcr;
}

// Send buffer(s) immediately for active transfers, new ones still need SIE help
void send_buffers(endpoint_t *ep) {
    uint32_t ecr = *ep->ecr;
    uint32_t bcr = prep_buffer(ep, 0);

    // Set ECR and BCR based on whether the transfer should be double buffered
    if (~bcr & USB_BUF_CTRL_LAST) {
        ecr |= EP_CTRL_DOUBLE_BUFFERED_BITS;
        bcr |= prep_buffer(ep, 1) << 16;
    } else {
        ecr &= ~EP_CTRL_DOUBLE_BUFFERED_BITS;
    }

    // Update ECR and BCR (set BCR first so controller has time to settle)
    *ep->bcr = bcr & UNAVAILABLE;
    *ep->ecr = ecr;
    nop();
    nop();
    *ep->bcr = bcr;
}

// Processes buffers in ISR context
void handle_buffers(endpoint_t *ep) {
    if (!ep->active) show_endpoint(ep), panic("Halted");

    // Read current buffer(s)
    uint32_t ecr = *ep->ecr;                          // ECR is single or double
    uint32_t bcr = *ep->bcr;                          // Buffer control register
    if (ecr & EP_CTRL_DOUBLE_BUFFERED_BITS) {         // When double buffered...
        if (read_buffer(ep, 0, bcr) == ep->maxsize)   // If first buffer is full
            read_buffer(ep, 1, bcr >> 16);            // Then, read second also
    } else {                                          // When single buffered...
        uint32_t bch = usb_hw->buf_cpu_should_handle; // Check CPU handling bits
        if (bch & 1u) bcr >>= 16;                     // Do RP2040-E4 workaround
        read_buffer(ep, 0, bcr);                      // And read the one buffer
    }

    // Send next buffer(s)
    if (ep->bytes_left) send_buffers(ep);
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
    DEVICE_ENUMERATING,
    DEVICE_ADDRESSED,
    DEVICE_ACTIVE,
    DEVICE_SUSPENDED,
};

typedef struct {
    uint8_t  state       ; // Current device state
    uint8_t  speed       ; // Device speed (0:disconnected, 1:full, 2:high)
    uint8_t  class       ; // Device class
    uint8_t  subclass    ; // Device subclass
    uint8_t  protocol    ; // Device protocol
    uint16_t vid         ; // Vendor Id  (0x0403: FTDI)
    uint16_t pid         ; // Product Id (0xcd18: Abaxis Piccolo Xpress)
    uint16_t version     ; // Version number
    uint8_t  manufacturer; // String index of manufacturer
    uint8_t  product     ; // String index of product
    uint8_t  serial      ; // String index of serial number
} device_t;

static device_t devices[MAX_DEVICES], *dev0 = devices;

// Get a device by its address
SDK_INLINE device_t *get_device(uint8_t dev_addr) {
    if (dev_addr < MAX_DEVICES) return &devices[dev_addr];
    panic("Device %u does not exist", dev_addr);
    return NULL;
}

// Find the next device address
uint8_t next_dev_addr() {
    for (uint8_t i = 1; i < MAX_DEVICES; i++) {
        if (devices[i].state == DEVICE_DISCONNECTED) {
            devices[i].state =  DEVICE_ALLOCATED;
            return i;
        }
    }
    panic("No free devices remaining");
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

// Clear out all devices
void reset_devices() {
    memclr(devices, sizeof(devices));
}

// ==[ Transfers ]==============================================================

enum {
    TRANSFER_SUCCESS, // not used yet
    TRANSFER_FAILED,  // not used yet
    TRANSFER_STALLED, // not used yet
    TRANSFER_TIMEOUT, // not used yet
    TRANSFER_INVALID, // not used yet
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

    // Debug output
    if (ep->setup || (*ep->bcr & 0x3f)) {
        printf("\n");
        printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
        printf( "│Frame  │ %4u │ %-35s", usb_hw->sof_rd, "Transfer started");
        show_endpoint(ep);
        printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
        bindump("│DAR", usb_hw->dev_addr_ctrl);
        bindump("│SSR", usb_hw->sie_status);
        bindump("│SCR", usb_hw->sie_ctrl);
        bindump("│ECR", *ep->ecr);
        bindump("│BCR", *ep->bcr);
        if (ep->setup) {
            uint32_t *packet = (uint32_t *) usbh_dpram->setup_packet;
            printf( "├───────┼──────┼─────────────────────────────────────┴────────────┤\n");
            hexdump("│SETUP", packet, sizeof(usb_setup_packet_t), 1);
            printf( "└───────┴──────┴──────────────────────────────────────────────────┘\n");
        } else {
            printf( "└───────┴──────┴─────────────────────────────────────┴────────────┘\n");
        }
    }

    // Mark the endpoint as active
    ep->active = true;

    // Perform the transfer (also gives SCR some time to settle)
    usb_hw->dev_addr_ctrl = dar;
    usb_hw->sie_ctrl      = scr & ~USB_SIE_CTRL_START_TRANS_BITS;
    send_buffers(ep); // ~20 μs
    usb_hw->sie_ctrl      = scr;
}

void transfer_zlp(void *arg) {
    endpoint_t *ep = (endpoint_t *) arg;

    // Send the ZLP transfer
    ep->data_pid = 1;
    transfer(ep);
}

void control_transfer(endpoint_t *ep, usb_setup_packet_t *setup) {
    if ( ep_num(ep))     panic("Control transfers must use EP0");
    if (!ep->configured) panic("Endpoint not configured");
    if ( ep->active)     panic("Control transfers per device must be serial");
    if ( ep->type)       panic("Control transfers require a control endpoint");

    // Copy the setup packet
    memcpy((void*) usbh_dpram->setup_packet, setup, sizeof(usb_setup_packet_t));

    // Send the control transfer
    ep->setup      = true;
    ep->data_pid   = 1;
    ep->ep_addr    = setup->bmRequestType & USB_DIR_IN;
    ep->bytes_left = setup->wLength;
    ep->bytes_done = 0;
    transfer(ep);
}

// ==[ Descriptors ]============================================================

SDK_INLINE void get_descriptor(endpoint_t *ep, uint8_t type, uint8_t len) {
    control_transfer(ep, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(type, 0),
        .wIndex        = 0,
        .wLength       = len,
    }));
}

void get_string_descriptor_blocking(endpoint_t *ep, uint8_t index) {
    control_transfer(ep, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(USB_DT_STRING, index),
        .wIndex        = 0,
        .wLength       = MAX_TEMP,
    }));

    do { usb_task(); } while (ep->active); // This transfer...
    do { usb_task(); } while (ep->active); // The ZLP...
}

void show_device_descriptor(void *ptr) {
    usb_device_descriptor_t *d = (usb_device_descriptor_t *) ptr;

    printf("Connected Device:\n");
    printf("  Total Length: %u\n"    , d->bLength);
    printb("  USB Version:  "        , d->bcdUSB);
    printf("  Device Class: %u\n"    , d->bDeviceClass);
    printf("    Subclass:   %u\n"    , d->bDeviceSubClass);
    printf("    Protocol:   %u\n"    , d->bDeviceProtocol);
    printf("  Packet Size:  %u\n"    , d->bMaxPacketSize0);
    printf("  Vendor Id:    0x%04x\n", d->idVendor);
    printf("  Product Id:   0x%04x\n", d->idProduct);
    printb("  Version:      "        , d->bcdDevice);
    printf("  Manufacturer: [#%u]\n" , d->iManufacturer);
    printf("  Product:      [#%u]\n" , d->iProduct);
    printf("  Serial:       [#%u]\n" , d->iSerialNumber);
    printf("\n");
}

void show_configuration_descriptor(void *ptr) {
    usb_configuration_descriptor_t *d = (usb_configuration_descriptor_t *) ptr;

    printf("Configuration Descriptor:\n");
    printf("  Total Length: %u\n"   , d->wTotalLength);
    printf("  Interfaces:   %u\n"   , d->bNumInterfaces);
    printf("  Config Value: %u\n"   , d->bConfigurationValue);
    printf("  Config Name:  [#%u]\n", d->iConfiguration);
    printf("  Attributes:   ");
    {
        char *sp = d->bmAttributes & 0x40 ? "Self-powered"  : NULL;
        char *rw = d->bmAttributes & 0x20 ? "Remote wakeup" : NULL;

        if (sp && rw) printf("%s, %s\n", sp, rw);
        else if  (sp) printf("%s\n", sp);
        else if  (rw) printf("%s\n", rw);
        else          printf("None\n");
    }
    printf("  Max power:    %umA\n" , d->bMaxPower * 2);
    printf("\n");
}

void show_string_blocking(endpoint_t *ep, uint8_t index) {
    uint8_t *ptr = ep->user_buf;

    // Request a string and wait for it
    get_string_descriptor_blocking(ep, index);

    // Prepare to parse Unicode string
    uint8_t   len =              *ptr / 2 ;
    uint16_t *uni = (uint16_t *) (ptr + 2);

    // Convert Unicode string to UTF-8
    char *str = (char[MAX_TEMP]) { 0 }, *utf = str;
    while (len--) {
        uint16_t u = *uni++;
        if (u < 0x80) {
            *utf++ = (char)          u;
        } else if (u < 0x800) {
            *utf++ = (char) (0xc0 | (u >>  6));
            *utf++ = (char) (0x80 | (u        & 0x3f));
        } else {
            *utf++ = (char) (0xe0 | (u >> 12));
            *utf++ = (char) (0x80 |((u >>  6) & 0x3f));
            *utf++ = (char) (0x80 | (u        & 0x3f));
        }
    }
    *utf++ = 0;

    printf("[String #%u]: \"%s\"\n", index, str);
}

// ==[ Classes ]================================================================

void cdch_init() {
    printf("CDC Host Driver Initialized\n");
}

bool cdch_open(uint8_t dev_addr, const usb_interface_descriptor_t *ifd,
               uint16_t len) {
    printf("CDC Host Driver Opened\n");
    return true;
}

bool cdch_config(uint8_t dev_addr, uint8_t itf_num) {
    printf("CDC Host Driver Configured\n");
    return true;
}

bool cdch_cb(uint8_t dev_addr, uint8_t ep_addr, // Ugh... xfer_result_t result,
             uint32_t xferred_bytes) {
    printf("CDC Host Driver Callback\n");
    return true;
}

void cdch_close(uint8_t dev_addr) {
    printf("CDC Host Driver Closed\n");
}

// ==[ Drivers ]================================================================

typedef struct {
  const char *name;
  void (* const init  )(void);
  bool (* const open  )(uint8_t dev_addr, const usb_interface_descriptor_t *ifd,
                        uint16_t len);
  bool (* const config)(uint8_t dev_addr, uint8_t itf_num);
  bool (* const cb    )(uint8_t dev_addr, uint8_t ep_addr, // Ugh... xfer_result_t result,
                        uint32_t xferred_bytes);
  void (* const close )(uint8_t dev_addr);
} driver_t;

static const driver_t drivers[] = {
    {
        .name   = "CDC",
        .init   = cdch_init,
        .open   = cdch_open,
        .config = cdch_config,
        .cb     = cdch_cb,
        .close  = cdch_close,
    }
};

enum {
    DRIVER_COUNT = sizeof(drivers) / sizeof(driver_t)
};

// Determine the length of an interface descriptor by adding up all its elements
static uint16_t interface_len(usb_interface_descriptor_t *ifd,
                              uint8_t ias, uint16_t max) {
    uint8_t  *cur = (uint8_t *) ifd;
    uint16_t  len = 0;

    while (ias--) {
        len += *cur;
        cur += *cur;
        while (len < max) {
            if  (cur[1] == USB_DT_INTERFACE_ASSOCIATION) return len;
            if ((cur[1] == USB_DT_INTERFACE) &&
               ((usb_interface_descriptor_t *) cur)->bAlternateSetting == 0) {
                break;
            }
            len += *cur;
            cur += *cur;
        }
    }

    return len;
}

// Parse a configuration descriptor and enable drivers for each interface
bool enable_drivers(endpoint_t *ep) {
    usb_configuration_descriptor_t *cfd; // Configuration descriptor
    usb_interface_descriptor_t     *ifd; // Interface descriptor

    // The configuration descriptor is a long list of other descriptors
    cfd = (usb_configuration_descriptor_t *) ep->user_buf;
    uint8_t  *cur = (uint8_t *) cfd;         // Start of descriptors
    uint8_t  *end = cur + cfd->wTotalLength; // End of descriptors

    // Some helper variables for looping through the list of descriptors
    device_t *dev = get_device(ep->dev_addr); // Current device
    uint16_t  len = 0;                        // How far to advance each time

    // Iterate through each descriptor in the main configuration descriptor
    for (cur += *cur; cur < end; cur += len) {
        uint8_t ias = 1; // Number of interface associations

        // Debug output
        hexdump("|DRV", cur, *cur, 1);

        // Optional: Interface Assocation Descriptor (IAD)
        if (cur[1] == USB_DT_INTERFACE_ASSOCIATION) {
            ias = ((usb_interface_assoc_descriptor_t *) cur)->bInterfaceCount;
            cur += *cur; // Advance to the next descriptor
        }

        // Required: Interface Descriptor
        if (cur[1] == USB_DT_INTERFACE) {
            ifd = (usb_interface_descriptor_t *) cur;
        } else {
            panic("Missing interface descriptor");
        }

        // Special case: CDC needs two interfaces (CDC Control + CDC Data)
        if (ias                     == 1             &&
            ifd->bInterfaceClass    == USB_CLASS_CDC &&
            ifd->bInterfaceSubClass == USB_SUBCLASS_CDC_ABSTRACT_CONTROL_MODEL)
            ias = 2;

        // Special case: Add MIDI here... any others?

        // Ensure we have something at least as big as an interface descriptor
        len = interface_len(ifd, ias, (uint16_t) (end - cur));
        if (len < sizeof(usb_interface_descriptor_t))
            panic("Interface descriptor is not big enough");

        // Try to find a driver for this interface
        for (uint8_t i = 0; i < DRIVER_COUNT; i++) {
//             driver_t *driver = &drivers[i];
//
//             if (driver->open(dev_addr, cur, len)) {
//                 printf("  %s driver opened\n", driver->name);
//
//                 // Bind each interface association to the driver
//                 for (uint8_t j = 0; j < ias; j++) {
//                     uint8_t k = cur->bInterfaceNumber + j;
//                     dev->itf2drv[k] = i; // TODO: This needs to start with an invalid value
//                 }
//
//                 // Bind all endpoints to the driver
//                 endpoint_bind_driver(dev->ep2drv, cur, len, i);
//
//                 break;
//             }

            // Complain if we didn't find a matching driver
            if (i == DRIVER_COUNT - 1) {
                printf("Interface %u skipped: class=%u subclass=%u protocol=%u\n",
                    ifd->bInterfaceNumber,
                    ifd->bInterfaceClass,
                    ifd->bInterfaceSubClass,
                    ifd->bInterfaceProtocol
                );
            }
        }
    }

    printf("Whoa... that was cool\n");
}

// ==[ Enumeration ]============================================================

enum {
    ENUMERATION_START,
    ENUMERATION_GET_MAXSIZE,
    ENUMERATION_SET_ADDRESS,
    ENUMERATION_GET_DEVICE,
    ENUMERATION_GET_CONFIG_SHORT,
    ENUMERATION_GET_CONFIG_FULL,
    ENUMERATION_SET_CONFIG,
    ENUMERATION_END,
};

void get_device_descriptor(endpoint_t *ep) {
    printf("Get device descriptor\n");

    uint8_t len = ep->dev_addr ? sizeof(usb_device_descriptor_t) : 8;
    get_descriptor(ep, USB_DT_DEVICE, len);
}

void set_device_address(endpoint_t *ep) {
    printf("Set device address to %u\n", ep->dev_addr);

    // TODO: Allow devices to change their address (not just from zero)
    control_transfer(epx, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_OUT
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_SET_ADDRESS,
        .wValue        = ep->dev_addr,
        .wIndex        = 0,
        .wLength       = 0,
    }));
}

void get_configuration_descriptor(endpoint_t *ep, uint8_t len) {
    printf("Get configuration descriptor\n");

    get_descriptor(ep, USB_DT_CONFIG, len);
}

void set_configuration(endpoint_t *ep, uint16_t cfg) {
    printf("Set configuration to %u\n", cfg);

    control_transfer(ep, &((usb_setup_packet_t) {
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

    // TODO: We need a way to ensure only one device enumerating at a time!

    switch (step++) {

        case ENUMERATION_START:
            printf("Enumeration started\n");

            printf("Starting GET_MAXSIZE\n");
            get_device_descriptor(epx); // TODO: We need to make sure we snag the value right when it comes back
            break;

        case ENUMERATION_GET_MAXSIZE: {
            uint8_t maxsize0 =
                ((usb_device_descriptor_t *) epx->user_buf)->bMaxPacketSize0;

            // Allocate a new device
            new_addr      = next_dev_addr();
            device_t *dev = get_device(new_addr);
            dev->state    = DEVICE_ENUMERATING;
            dev->speed    = dev0->speed;

            // Allocate EP0 on the new device (uses the shared ctrl_buf buffer)
            endpoint_t *ep = next_endpoint(new_addr, &((usb_endpoint_descriptor_t) {
                .bLength          = sizeof(usb_endpoint_descriptor_t),
                .bDescriptorType  = USB_DT_ENDPOINT,
                .bEndpointAddress = 0,
                .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
                .wMaxPacketSize   = maxsize0,
                .bInterval        = 0,
            }), ctrl_buf);
            ep->dev_addr = new_addr;

            printf("Starting SET_ADDRESS\n");
            set_device_address(ep);
        }   break;

        case ENUMERATION_SET_ADDRESS: {
            endpoint_t *ep  = find_endpoint(new_addr, 0);
            device_t   *dev = get_device(ep->dev_addr);

            dev0->state = DEVICE_ALLOCATED;
            dev->state  = DEVICE_ADDRESSED;

            printf("Starting GET_DEVICE\n");
            get_device_descriptor(ep);
        }   break;

        case ENUMERATION_GET_DEVICE: {
            show_device_descriptor(ep->user_buf);
            uint8_t len = sizeof(usb_configuration_descriptor_t);

            printf("Starting GET_CONFIG_SHORT (%u bytes)\n", len);
            get_configuration_descriptor(ep, len);
        }   break;

        case ENUMERATION_GET_CONFIG_SHORT: {
            uint8_t len =
                ((usb_configuration_descriptor_t *) ep->user_buf)->wTotalLength;
            if (len > MAX_TEMP) {
                show_configuration_descriptor(ep->user_buf);
                panic("Configuration descriptor too large");
            }

            printf("Starting GET_CONFIG_FULL (%u bytes)\n", len);
            get_configuration_descriptor(ep, len);
        }   break;

        case ENUMERATION_GET_CONFIG_FULL: {
            show_configuration_descriptor(ep->user_buf);
            enable_drivers(ep);

            printf("Starting SET_CONFIG\n");
            set_configuration(ep, 1);
        }   break;

        case ENUMERATION_SET_CONFIG:
            device_t *dev = get_device(ep->dev_addr);
            dev->state = DEVICE_ACTIVE;

            printf("Enumeration completed\n");

            show_string_blocking(ep, 1);
            show_string_blocking(ep, 2);
            show_string_blocking(ep, 3);

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

    printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
    reset_devices();
    reset_endpoints();
    bindump("│INT", usb_hw->inte);
    printf( "└───────┴──────┴─────────────────────────────────────┴────────────┘\n");
}

// ==[ Tasks ]==================================================================

enum {
    TASK_CALLBACK,
    TASK_CONNECT,
    TASK_TRANSFER,
};

typedef struct {
    uint8_t type;
    uint32_t guid;

    union {
        struct {
            void (*fn) (void *);
            void *arg;
        } callback;

        struct {
            uint8_t speed;
        } connect;

        struct {
            endpoint_t *ep;     // TODO: Risky to just sent this pointer?
            uint16_t    len;    // TODO: Should we point to a safe buffer of this length?
            uint8_t     status; // TODO: Are we even using this?
        } transfer;
    };
} task_t;

static uint32_t guid = 1;

static queue_t *queue = &((queue_t) { 0 });

SDK_INLINE const char *task_name(uint8_t type) {
    switch (type) {
        case TASK_CALLBACK: return "TASK_CALLBACK";
        case TASK_CONNECT:  return "TASK_CONNECT";
        case TASK_TRANSFER: return "TASK_TRANSFER";
        default:            return "UNKNOWN";
    }
    panic("Unknown task queued");
}

SDK_INLINE const char *callback_name(void (*fn) (void *)) {
    if (fn == enumerate   ) return "enumerate";
    if (fn == transfer_zlp) return "transfer_zlp";
    printf("Calling unknown callback function\n");
}

void usb_task() {
    task_t task;

    while (queue_try_remove(queue, &task)) {
        uint8_t type = task.type;
        printf("\n=> %u) New task, %s\n\n", task.guid, task_name(type)); // ~3 ms (sprintf was ~31 μs, ring_printf was 37 μs)
        switch (type) {

            case TASK_CALLBACK: {
                printf("Calling %s\n", callback_name(task.callback.fn));
                task.callback.fn(task.callback.arg);
            }   break;

            case TASK_CONNECT: {
                static uint64_t last_attempt;

                // For now, ignore rapid device connects
                if (last_attempt && (time_us_64() - last_attempt < 1000000)) {
                    printf("Connections allowed only once every second\n");
                    break;
                }
                last_attempt = time_us_64();

                // Initialize dev0
                reset_device(0); // TODO: Is this really necessary?
                dev0->state = DEVICE_ENUMERATING;
                dev0->speed = task.connect.speed;

                // Show the device connection and speed
                char *str = dev0->speed == LOW_SPEED ? "low" : "full";
                printf("Device connected (%s speed)\n", str);

                // Start enumeration
                enumerate(NULL);

            }   break;

            case TASK_TRANSFER: {
                endpoint_t *ep  = task.transfer.ep;
                uint16_t    len = task.transfer.len;

                // Handle the transfer
                device_t *dev = get_device(ep->dev_addr);
                if (len) {
                    printf("Calling transfer_zlp\n");
                    transfer_zlp(ep);
                } else if (dev->state < DEVICE_ACTIVE) {
                    printf("Calling enumerate\n");
                    enumerate(ep);
                } else {
                    printf("Transfer completed\n");
                }
           }   break;

            default:
                printf("Unknown task queued\n");
                break;
        }
        // printf("=> %u) Finish task: %s\n", task.guid, task_name(type));
    }
}

// ==[ Interrupts ]=============================================================

SDK_INLINE void printf_interrupts(uint32_t ints) {
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
    printf( "\n=> %u) New ISR", guid++);
    printf_interrupts(ints);
    printf( "\n\n");
    printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
    printf( "│Frame  │ %4u │ %-35s", usb_hw->sof_rd, "Interrupt Handler");
    show_endpoint(ep);
    printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
    bindump("│INTR", usb_hw->intr);
    bindump("│INTS", ints);
    bindump("│DAR" , dar);
    bindump("│SSR" , usb_hw->sie_status);
    bindump("│SCR" , usb_hw->sie_ctrl);
    bindump("│ECR" , ecr);
    bindump("│BCR" , bcr);
    bool flat = false; // For the last line of debug output

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

            // Show connection info
            printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
            printf( "│CONNECT│ %-4s │ %-35s │ Task #%-4u │\n", "", "New device connected", guid);

            queue_add_blocking(queue, &((task_t) { // ~20 μs
                .type          = TASK_CONNECT,
                .guid          = guid++,
                .connect.speed = speed,
            }));
        } else {
            reset_epx(); // TODO: There's more to do here
        }
    }

    // Stall detected (higher priority than BUFF_STATUS and TRANS_COMPLETE)
    if (ints &  USB_INTS_STALL_BITS) {
        ints ^= USB_INTS_STALL_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;

        printf("Stall detected\n");

//         // Queue the stalled transfer
//         queue_add_blocking(queue, &((task_t) {
//             .type            = TASK_TRANSFER,
//             .guid            = guid++,
//             .transfer.ep     = ep,  // TODO: Need to flesh this out
//             .transfer.len    = 999, // TODO: Need to flesh this out
//             .transfer.status = TRANSFER_STALLED,
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
        handle_buffers(ep); usb_hw_clear->buf_status = 0x1; bits ^= 0x1; // TODO: TOTAL HACK!

//         // Check the polled endpoints (IN and OUT)
//         for (uint8_t i = 0; i < MAX_ENDPOINTS && bits; i++) {
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

        // Panic if the endpoint is not active
        if (!ep->active) panic("Endpoints must be active to be completed");

        // Get the transfer length (actual bytes transferred)
        uint16_t len = ep->bytes_done;

        // Debug output
        if (len) {
            printf( "├───────┼──────┼─────────────────────────────────────┴────────────┤\n");
            printf( "│XFER\t│ %4u │ Device %-28u   Task #%-4u │\n", len, ep->dev_addr, guid);
            hexdump("│Data", temp_buf, len, 1); // TODO: We should use the base buffer address for this endpoint
            flat = true;
        } else {
            char *str = ep_in(ep) ? "IN" : "OUT";
            printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
            printf( "│ZLP\t│ %-4s │ Device %-28u │ Task #%-4u │\n", str, ep->dev_addr, guid);
        }

        // Clear the endpoint (since its complete)
        clear_endpoint(ep);

        // Queue the transfer task
        queue_add_blocking(queue, &((task_t) {
            .type            = TASK_TRANSFER,
            .guid            = guid++,
            .transfer.ep     = ep,
            .transfer.len    = len,
            .transfer.status = TRANSFER_SUCCESS, // TODO: Is this needed?
        }));
    }

    // Receive timeout (waited too long without seeing an ACK)
    if (ints &  USB_INTS_ERROR_RX_TIMEOUT_BITS) {
        ints ^= USB_INTS_ERROR_RX_TIMEOUT_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;

        printf("Receive timeout\n");

        panic("Timed out waiting for data");
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
    if (ints) panic("Unhandled IRQ bitmask 0x%04x", ints);

    // TODO: How should we deal with NAKs seen in the SSR?
    // usb_hw_clear->sie_status = 1 << 28u; // Clear the NAK???

    printf("└───────┴──────┴─────────────────────────────────────%s────────────┘\n", flat ? "─" : "┴");
}

// ==[ Main ]===================================================================

int main() {
    stdout_uart_init();
    printf("\033[2J\033[H\n==[ USB host example]==\n\n");
    setup_usb_host();

    queue_init(queue, sizeof(task_t), 64);

    while (1) {
        usb_task();
    }
}
