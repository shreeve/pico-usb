// ============================================================================
// PicoUSB - A USB host and device library for the Raspberry Pi Pico/W
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: January 29, 2024
//  Legal: Same license as the Raspberry Pi Pico SDK
// Thanks: Thanks to the TinyUSB project for inspiration and code snippets!
//  Notes: This is a work in progress. It is not yet functional.
// ============================================================================

#include <stdio.h>                // For printf
#include <string.h>               // For memcpy

#include "pico/stdlib.h"          // Pico stdlib
#include "pico/util/queue.h"      // A beautifully simple queue
#include "hardware/regs/usb.h"    // USB hardware registers from pico-sdk
#include "hardware/structs/usb.h" // USB hardware structs from pico-sdk
#include "hardware/irq.h"         // For interrupt enable and numbers
#include "hardware/resets.h"      // For resetting the native USB controller

// ==[ USB 2.0 ]===============================================================

#include "usb_common.h"

typedef struct usb_descriptor               usb_descriptor_t;
typedef struct usb_device_descriptor        usb_device_descriptor_t;
typedef struct usb_configuration_descriptor usb_configuration_descriptor_t;
typedef struct usb_interface_descriptor     usb_interface_descriptor_t;
typedef struct usb_endpoint_descriptor      usb_endpoint_descriptor_t;
typedef struct usb_endpoint_descriptor_long usb_endpoint_descriptor_long_t;
typedef struct usb_setup_packet             usb_setup_packet_t;

// ==[ PicoUSB ]===============================================================

#define usb_hw_set   ((usb_hw_t *) hw_set_alias_untyped  (usb_hw))
#define usb_hw_clear ((usb_hw_t *) hw_clear_alias_untyped(usb_hw))

#define PU_ALIGNED(bytes) __attribute__ ((aligned(bytes)))
#define PU_ALWAYS_INLINE  __attribute__ ((always_inline))
#define PU_PACKED         __attribute__ ((packed))
#define PU_WEAK           __attribute__ ((weak))

#define MAKE_U16(x,y)    (((x) << 8) | ((y)     ))
#define SWAP_U16(x)      (((x) >> 8) | ((x) << 8))

static bool configured = false;

// ==[ Event queue ]===========================================================

enum {
    EVENT_CONNECTION,
    EVENT_TRANSFER,
    EVENT_FUNCTION,
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

// static event_t event_struct, *event = &event_struct;
static queue_t queue_struct, *queue = &queue_struct;

// ==[ Helpers ]===============================================================

#include "hexdump.h"

PU_ALWAYS_INLINE static inline uint8_t dev_speed() {
    return (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) \
                              >> USB_SIE_STATUS_SPEED_LSB;
}

// ==[ Endpoints ]=============================================================

// // Set up an endpoint's control register
// void usb_setup_endpoint(const struct usb_endpoint *ep) {
//
//     // // Grok the desired endpoint
//     // uint8_t ep_addr = ep->descriptor->bEndpointAddress;
//     // uint8_t ep_num = ep_addr & 0x0f;
//     // bool in = ep_addr & USB_DIR_IN;
//     // printf("Initialized EP%d_%s (0x%02x) with buffer address 0x%p\n",
//     //        ep_num, in ? "IN " : "OUT", ep_addr, ep->data_buffer);
//
//     // Set ep_ctrl register for this endpoint (skip EP0 since it uses SIE_CTRL)
//     if (ep->endpoint_control) {
//         uint32_t type = ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB;
//         uint32_t offset = ((uint32_t) ep->data_buffer) ^ ((uint32_t) usbh_dpram);
//         uint32_t interval = ep->descriptor->bInterval;
//         if (interval) {
//             interval = (interval - 1) << EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB;
//         }
//         *ep->endpoint_control = EP_CTRL_ENABLE_BITS          | // Enable EP
//                                 EP_CTRL_INTERRUPT_PER_BUFFER | // One IRQ per
//                                 type     | // Control, iso, bulk, or interrupt
//                                 interval | // Interrupt interval in ms
//                                 offset   ; // Address base offset in DSPRAM
//     }
// }

// ==[ Enumeration ]===========================================================

enum {
    USB_SIE_CTRL_BASE = USB_SIE_CTRL_VBUS_EN_BITS       // Supply VBUS to device
                      | USB_SIE_CTRL_SOF_EN_BITS        // Enable full speed bus
                      | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS // Enable low speed bus
                      | USB_SIE_CTRL_PULLDOWN_EN_BITS   // Enable devices to connect
                      | USB_SIE_CTRL_EP0_INT_1BUF_BITS  // Interrupt on every buffer
};

// Get device descriptor
void get_device_descriptor() {
    usb_setup_packet_t request = {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(USB_DT_DEVICE, 0),
        .wIndex        = 0,
        .wLength       = sizeof(usb_setup_packet_t),
    };

    // Copy this request to the transfer buffer
    memcpy((void *) usbh_dpram->setup_packet, &request, sizeof(request));
    printf("< Setup");
    hexdump(&request, sizeof(request), 1);

    // Pluck from the request
    bool in = request.bmRequestType & USB_DIR_IN;
    uint16_t len = request.wLength;

    // Execute the transfer (Do we need to delay after setting AVAIL? Or, does the wait on START_TRANS take care of this?)
    usbh_dpram->epx_buf_ctrl = 0x00007000 | len;  // LAST, DATA1, SEL, !AVAIL | len
    busy_wait_at_least_cycles(12); // TODO: Anything better? Why not 3 or 6 cycles? TinyUSB doesn't use this in hcd_edpt_xfer().
    usbh_dpram->epx_buf_ctrl = 0x00007400 | len;  // LAST, DATA1, SEL,  AVAIL | len
    printf("BCR: 0x%04x\n", usbh_dpram->epx_buf_ctrl);
    usb_hw->dev_addr_ctrl = (uint32_t) 0; // NOTE: 19:16=ep_num, 6:0=dev_addr
    uint32_t bits = USB_SIE_CTRL_BASE               // Default SIE_CTRL bits
                  | USB_SIE_CTRL_SEND_SETUP_BITS   // Send a SETUP packet
            | (in ? USB_SIE_CTRL_RECEIVE_DATA_BITS // IN to host is receive
                  : USB_SIE_CTRL_SEND_DATA_BITS);  // OUT from host is send
    usb_hw->sie_ctrl = bits; // TODO: Might need USB_SIE_CTRL_PREAMBLE_EN_BITS (LS on FS hub)
    busy_wait_at_least_cycles(12); // TODO: Anything better? Why not 3 or 6 cycles? TinyUSB doesn't use this in hcd_edpt_xfer().
    usb_hw->sie_ctrl = bits | USB_SIE_CTRL_START_TRANS_BITS;
}

// // Set device address
// void set_device_address() {
//     const uint8_t data[] = "\x00\x05\x01\x00\x00\x00\x00\x00";
//     const uint8_t size = 8;
//     uint32_t bits;
//
//     printf("< Setup");
//     hexdump(data, size, 2);
//     printf("Set device address to 1\n");
//
//     memcpy((void *) usbh_dpram->setup_packet, data, size);
//     usbh_dpram->epx_ctrl     = EP_CTRL_ENABLE_BITS          | // Enable EPX
//                                EP_CTRL_INTERRUPT_PER_BUFFER | // One IRQ per buffer
//                                0x180                        ; // Data buffer offset
//     usbh_dpram->epx_buf_ctrl = 0x00007400  | // LAST, DATA1, SEL, AVAIL
//                                size        ; // Size
//     usb_hw->dev_addr_ctrl = (uint32_t) 0   ; // NOTE: 19:16=ep_num, 6:0=dev_addr
//     bits = USB_SIE_CTRL_BASE               | // Default SIE_CTRL bits
//         //    USB_SIE_CTRL_SEND_DATA_BITS  | // Request a response
//            USB_SIE_CTRL_SEND_SETUP_BITS    ; // Send a SETUP packet
//     usb_hw->sie_ctrl = bits                ; // TODO: Might need USB_SIE_CTRL_PREAMBLE_EN_BITS (LS on FS hub)
//     busy_wait_at_least_cycles(12)          ; // TODO: Anything better? Why not 3 or 6 cycles? TinyUSB doesn't use this in hcd_edpt_xfer().
//     usb_hw->sie_ctrl = bits | USB_SIE_CTRL_START_TRANS_BITS;
// }

void start_enumeration() {
    printf("Start enumeration\n");

    // EPX setup (do earlier)
    usbh_dpram->epx_ctrl =
          EP_CTRL_ENABLE_BITS          // Enable EP
        | EP_CTRL_INTERRUPT_PER_BUFFER // One IRQ per
        | USB_TRANSFER_TYPE_CONTROL << EP_CTRL_BUFFER_TYPE_LSB // Control transfer
        | 0x180;                       // Data buffer offset

    get_device_descriptor();

    // ==[ Send a ZLP, can't do here... we need to do after the XFER was completed... ]==


}

// ==[ Interrupt ]=============================================================

// Interrupt handler
void isr_usbctrl() {
    uint32_t ints = usb_hw->ints, x;
    uint16_t size = 0;
    static event_t event;

    printf("────────┬───────────────────────────────────────────────────────────────────────\n");
    x = ints                    ; printf("IRQ\t│ %032b 0x%08x\n", x, x);
    x = usb_hw->sie_status      ; printf("SIE\t│ %032b 0x%08x\n", x, x);
    x = usb_hw->dev_addr_ctrl   ; printf("DEV\t│ %032b 0x%08x\n", x, x);
    x = usbh_dpram->epx_ctrl    ; printf("ECR\t│ %032b 0x%08x\n", x, x);
    x = usbh_dpram->epx_buf_ctrl; printf("BCR\t│ %032b 0x%08x\n", x, x);

    // Connection event (attach or detach)
    if (ints &  USB_INTS_HOST_CONN_DIS_BITS) {
        ints ^= USB_INTS_HOST_CONN_DIS_BITS;

        uint8_t speed = dev_speed();

        if (speed) {
            printf("ISR: Device connected\n");
            event.type = EVENT_CONNECTION;
            event.dev_addr = 0;
            event.conn.speed = speed;
            queue_add_blocking(queue, &event);
        } else {
            printf("ISR: Device disconnected\n");
        }

        // Clear speed change interrupt
        usb_hw_clear->sie_status = USB_SIE_STATUS_SPEED_BITS;
    }

    // Stall detected (before buffer status)
    if (ints &  USB_INTS_STALL_BITS) {
        ints ^= USB_INTS_STALL_BITS;

        printf("ISR: Stall detected\n");

        // Clear the stall
        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;
        // hw_xfer_complete(&epx, XFER_RESULT_STALLED);
    }

    // Buffer(s) ready
    if (ints &  USB_INTS_BUFF_STATUS_BITS) {
        ints ^= USB_INTS_BUFF_STATUS_BITS;

        // Show buffer status
        x = usb_hw->buf_status; printf("BUF\t| %032b 0x%08x\n", x, x);

        // For now, we know it's EP0_IN... (we're cheating)
        x = usbh_dpram->epx_buf_ctrl & USB_BUF_CTRL_LEN_MASK; // Buffer length

        printf("Buffer status (%u bytes)\n", x);
        if (x) {
            printf("> Data");
            hexdump(usbh_dpram->epx_data, x, 1);
        } else {
            printf("> ZLP\n");
        }

        // hw_handle_buff_status();

        // Clear all buffers
        usb_hw_clear->buf_status = (uint32_t) ~0;
    }

    // Transfer complete (last packet)
    if (ints &  USB_INTS_TRANS_COMPLETE_BITS) {
        ints ^= USB_INTS_TRANS_COMPLETE_BITS;

        printf("ISR: Transfer complete\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;
        // hw_trans_complete();
    }

    // Receive timeout (too long without an ACK)
    if (ints &  USB_INTS_ERROR_RX_TIMEOUT_BITS) {
        ints ^= USB_INTS_ERROR_RX_TIMEOUT_BITS;

        printf("ISR: Receive timeout\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;
    }

    // Data error (IN packet from device has wrong data PID)
    if (ints &  USB_INTS_ERROR_DATA_SEQ_BITS) {
        ints ^= USB_INTS_ERROR_DATA_SEQ_BITS;

        printf("ISR: Data error\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;
        panic("ERROR: USB Host data sequence error\n");
    }

    // Device resumed (device initiated)
    if (ints &  USB_INTS_HOST_RESUME_BITS) {
        ints ^= USB_INTS_HOST_RESUME_BITS;

        printf("ISR: Device resume\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;
    }

    // Any missed?
    if (ints) {
        panic("ISR: Unhandled IRQ 0x%04x\n", ints);
    }
}

// ==[ Resets ]================================================================

// Reset USB host
void usb_host_reset() {

    // Reset controller
    reset_block       (RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    // Clear state
    memset(usb_hw    , 0, sizeof(*usb_hw    ));
    memset(usbh_dpram, 0, sizeof(*usbh_dpram));

    // Setup endpoints
    // usb_setup_endpoint(&epx);
    // usb_setup_endpoints();

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

    printf("USB host reset\n");
    irq_set_enabled(USBCTRL_IRQ, true);
}

// ==[ Main ]==================================================================

void puh_task() {
    static event_t event;

    if (queue_try_remove(queue, &event)) {
        switch (event.type) {
            case EVENT_CONNECTION:
                printf("Device connected\n");
                printf("Speed: %u\n", event.conn.speed);
                start_enumeration();
                break;

            case EVENT_TRANSFER:
                printf("Transfer complete\n");
                printf("EP: %u\n", event.xfer.ep_addr);
                printf("Result: %u\n", event.xfer.result);
                printf("Length: %u\n", event.xfer.len);
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
        puh_task();
    }
}
