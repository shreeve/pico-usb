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
#include "pico/util/queue.h"      // Multicore and IRQ safe queue
#include "hardware/regs/usb.h"    // USB hardware registers from pico-sdk
#include "hardware/structs/usb.h" // USB hardware structs from pico-sdk
#include "hardware/irq.h"         // Interrupts and definitions
#include "hardware/resets.h"      // Resetting the native USB controller

#include "usb_common.h"           // USB 2.0 definitions

// ==[ Helpers ]===============================================================

#include "helpers.h"

#define MAKE_U16(x, y) (((x) << 8) | ((y)     ))
#define SWAP_U16(x)    (((x) >> 8) | ((x) << 8))

#define SDK_ALIGNED(bytes) __attribute__ ((aligned(bytes)))
#define SDK_ALWAYS_INLINE  __attribute__ ((always_inline))
#define SDK_PACKED         __attribute__ ((packed))
#define SDK_WEAK           __attribute__ ((weak))

SDK_ALWAYS_INLINE static inline bool is_host_mode() {
    return (usb_hw->main_ctrl & USB_MAIN_CTRL_HOST_NDEVICE_BITS);
}

SDK_ALWAYS_INLINE static inline uint8_t dev_speed() {
    return (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) \
                              >> USB_SIE_STATUS_SPEED_LSB;
}

SDK_ALWAYS_INLINE static inline uint8_t line_state() {
    return (usb_hw->sie_status & USB_SIE_STATUS_LINE_STATE_BITS) \
                              >> USB_SIE_STATUS_LINE_STATE_LSB;
}

#define usb_hw_set   ((usb_hw_t *) hw_set_alias_untyped  (usb_hw))
#define usb_hw_clear ((usb_hw_t *) hw_clear_alias_untyped(usb_hw))

#define hw_set_wait_set(reg, value, cycles, or_mask) \
    reg = (value); \
    busy_wait_at_least_cycles(cycles); \
    reg = (value) | (or_mask);

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

// ==[ Endpoints ]=============================================================

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

//   tuh_xfer_t xfer =
//   {
//     .daddr       = daddr,
//     .ep_addr     = 0,
//     .setup       = &request,
//     .buffer      = buffer,
//     .complete_cb = complete_cb,
//     .user_data   = user_data
//   };
//
//   bool const ret = tuh_control_xfer(&xfer);
//
//   // if blocking, user_data could be pointed to xfer_result
//   if ( !complete_cb && user_data )
//   {
//     *((xfer_result_t*) user_data) = xfer.result;
//   }

// typedef enum {
//   XFER_RESULT_SUCCESS = 0,
//   XFER_RESULT_FAILED,
//   XFER_RESULT_STALLED,
//   XFER_RESULT_TIMEOUT,
//   XFER_RESULT_INVALID
// } xfer_result_t;

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

// NOTE: This is a single/global/static control transfer object
// // Control transfers: since most controllers do not support multiple control transfers
// // on multiple devices concurrently and control transfers are not used much except for
// // enumeration, we will only execute control transfers one at a time.
// CFG_TUSB_MEM_SECTION struct {
//   tusb_control_request_t request TU_ATTR_ALIGNED(4);
//   uint8_t* buffer;
//   tuh_xfer_cb_t complete_cb;
//   uintptr_t user_data;
//
//   uint8_t daddr;
//   volatile uint8_t stage;
//   volatile uint16_t actual_len;
// } _ctrl_xfer;

enum {
    USB_SIE_CTRL_BASE = USB_SIE_CTRL_VBUS_EN_BITS       // Supply VBUS to device
                      | USB_SIE_CTRL_SOF_EN_BITS        // Enable full speed bus
                      | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS // Enable low speed bus
                      | USB_SIE_CTRL_PULLDOWN_EN_BITS   // Enable devices to connect
                      | USB_SIE_CTRL_EP0_INT_1BUF_BITS  // Interrupt on every buffer
};

// NOTE: SIE_CTRL
//
// 4 | STOP_TRANS   | Host: Stop transaction                 | SC
// 3 | RECEIVE_DATA | Host: Receive transaction (IN to host) | RW
// 2 | SEND_DATA    | Host: Send transaction (OUT from host) | RW
// 1 | SEND_SETUP   | Host: Send Setup packet                | RW
// 0 | START_TRANS  | Host: Start transaction                | SC

// #define EP0_OUT_ADDR (USB_DIR_OUT | 0)
// #define EP0_IN_ADDR  (USB_DIR_IN  | 0)
//
// typedef void (*usb_ep_handler)(uint8_t *buf, uint16_t len);
//
// void ep0_out_handler(uint8_t *buf, uint16_t len) {
//     ; // Nothing to do
//     // TODO: Find out when this is called...
// }
//
// void ep0_in_handler(uint8_t *buf, uint16_t len) {
//     // if (should_set_address) {
//     //     usb_hw->dev_addr_ctrl = device_address; // Set hardware device address
//     //     should_set_address = false;
//     // } else {
//     //     // Prepare for a ZLP from host on EP0_OUT
//     //     usb_start_transfer(usb_get_endpoint(EP0_OUT_ADDR), NULL, 0);
//     // }
// }
//
// static const struct usb_endpoint_descriptor ep0_out = { // EP0, out to device
//     .bLength          = sizeof(struct usb_endpoint_descriptor),
//     .bDescriptorType  = USB_DT_ENDPOINT,
//     .bEndpointAddress = EP0_OUT_ADDR,
//     .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
//     .wMaxPacketSize   = 64,
//     .bInterval        = 0
// };
//
// static const struct usb_endpoint_descriptor ep0_in = { // EP0, in to host
//     .bLength          = sizeof(struct usb_endpoint_descriptor),
//     .bDescriptorType  = USB_DT_ENDPOINT,
//     .bEndpointAddress = EP0_IN_ADDR,
//     .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
//     .wMaxPacketSize   = 64,
//     .bInterval        = 0
// };
//
// struct usb_endpoint {
//     usb_endpoint_descriptor_t *descriptor;
//     usb_ep_handler handler;
//
//     volatile uint32_t *endpoint_control;
//     volatile uint32_t *buffer_control;
//     volatile uint8_t  *data_buffer;
//
//     uint8_t next_datapid; // Toggle DATA0/DATA1 each packet
// };
//
// static struct usb_endpoint epx = {
//     .descriptor       = &ep0_out,
//     .handler          = NULL,
//     .endpoint_control = &usbh_dpram->epx_ctrl,
//     .buffer_control   = &usbh_dpram->epx_buf_ctrl,
//     .data_buffer      = &usbh_dpram->epx_data[0],
//     .next_datapid     = 1, // Starts with DATA1
// };

// struct usb_device {
//     const struct usb_device_descriptor        *device_descriptor;
//     const struct usb_configuration_descriptor *config_descriptor;
//     const struct usb_interface_descriptor     *interface_descriptor;
//     const unsigned char                       *lang_descriptor;
//     const unsigned char                       **descriptor_strings;
//     struct usb_endpoint                       endpoints[USB_NUM_ENDPOINTS];
//
//     // TODO: Integrate the stuff below...
//     // uint8_t  ep0_size; ???
//     // uint8_t  speed; // 0: unknown, 1: full, 2: high, 3: super
//     // struct SDK_PACKED {
//     //     uint8_t  speed; // 0: unknown, 1: full, 2: high, 3: super
//     //     volatile uint8_t addressed  : 1; // After SET_ADDR
//     //     volatile uint8_t connected  : 1; // After first transfer
//     //     volatile uint8_t enumerating : 1; // enumeration is in progress, false if not connected or all interfaces are configured
//     //     volatile uint8_t configured : 1; // After SET_CONFIG and all drivers are configured
//     //     volatile uint8_t suspended  : 1; // Bus suspended
//     // };
//     // uint8_t itf2drv[CFG_TUH_INTERFACE_MAX];  // map interface number to driver (0xff is invalid)
//     // uint8_t ep2drv[CFG_TUH_ENDPOINT_MAX][2]; // map endpoint to driver ( 0xff is invalid ), can use only 4-bit each
//     // tu_edpt_state_t ep_status[CFG_TUH_ENDPOINT_MAX][2];
// };

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

// Get device descriptor
void get_device_descriptor() {
    usb_setup_packet_t request = {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(USB_DT_DEVICE, 0),
        .wIndex        = 0,
        .wLength       = sizeof(request),
    };

    // Copy this request to the transfer buffer
    memcpy((void *) usbh_dpram->setup_packet, &request, sizeof(request));
    printf("< Setup");
    hexdump(&request, sizeof(request), 1);

// epx.descriptor = in ? &ep0_in : &ep0_out;

    // Pluck from the request
    bool in = request.bmRequestType & USB_DIR_IN;
    uint16_t len = request.wLength;

    // Execute the transfer
    usb_hw->dev_addr_ctrl = (uint32_t) 0; // NOTE: 19:16=ep_num, 6:0=dev_addr

    // Endpoint control bits (No EP0, except for IRQ control via SIE_CTRL)
    // #define EP_CTRL_ENABLE_BITS (1u << 31u)
    // #define EP_CTRL_DOUBLE_BUFFERED_BITS (1u << 30)
    // #define EP_CTRL_INTERRUPT_PER_BUFFER (1u << 29)
    // #define EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER (1u << 28)
    // #define EP_CTRL_INTERRUPT_ON_NAK (1u << 16)
    // #define EP_CTRL_INTERRUPT_ON_STALL (1u << 17)
    // #define EP_CTRL_BUFFER_TYPE_LSB 26u
    // #define EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB 16u

    // Buffer control bits (all EPs are treated the same)
    // #define USB_BUF_CTRL_FULL      0x00008000u
    // #define USB_BUF_CTRL_LAST      0x00004000u
    // #define USB_BUF_CTRL_DATA0_PID 0x00000000u
    // #define USB_BUF_CTRL_DATA1_PID 0x00002000u
    // #define USB_BUF_CTRL_SEL       0x00001000u
    // #define USB_BUF_CTRL_STALL     0x00000800u
    // #define USB_BUF_CTRL_AVAIL     0x00000400u
    // #define USB_BUF_CTRL_LEN_MASK  0x000003FFu
    // #define USB_BUF_CTRL_LEN_LSB   0

    // // Values here are used on the IN transaction of the control transfer
    // uint32_t ecr = ...

    // Values here are used on the IN transaction of the control transfer
    uint32_t bcr = USB_BUF_CTRL_LAST
                 | USB_BUF_CTRL_DATA1_PID
                 | USB_BUF_CTRL_SEL;
    bindump(" BCR", bcr);
    hw_set_wait_set(usbh_dpram->epx_buf_ctrl, bcr, 12, USB_BUF_CTRL_AVAIL);

//   if ( ep == &epx ) {
//     hw_endpoint_xfer_start(ep, buffer, buflen);
//
//     // Assumes you have set up buffer control, endpoint control etc
//     // for host we have to initiate the transfer
//     usb_hw->dev_addr_ctrl = (uint32_t) (dev_addr | (ep_num << USB_ADDR_ENDP_ENDPOINT_LSB));
//
//     uint32_t flags = USB_SIE_CTRL_START_TRANS_BITS | SIE_CTRL_BASE |
//                      (ep_dir ? USB_SIE_CTRL_RECEIVE_DATA_BITS : USB_SIE_CTRL_SEND_DATA_BITS) |
//                      (need_pre(dev_addr) ? USB_SIE_CTRL_PREAMBLE_EN_BITS : 0);
//     // START_TRANS bit on SIE_CTRL seems to exhibit the same behavior as the AVAILABLE bit
//     // described in RP2040 Datasheet, release 2.1, section "4.1.2.5.1. Concurrent access".
//     // We write everything except the START_TRANS bit first, then wait some cycles.
//     usb_hw->sie_ctrl = flags & ~USB_SIE_CTRL_START_TRANS_BITS;
//     busy_wait_at_least_cycles(12);
//     usb_hw->sie_ctrl = flags;
//   } else {
//     hw_endpoint_xfer_start(ep, buffer, buflen);
//   }

    // Send the setup request // TODO: preamble (LS on FS)
    uint32_t scr = USB_SIE_CTRL_BASE              // Default SIE_CTRL bits
                 | USB_SIE_CTRL_SEND_SETUP_BITS   // Send a SETUP packet
           | (in ? USB_SIE_CTRL_RECEIVE_DATA_BITS // IN to host is receive
                 : USB_SIE_CTRL_SEND_DATA_BITS);  // OUT from host is send
    bindump(" SCR", scr);
    hw_set_wait_set(usb_hw->sie_ctrl, scr, 12, USB_SIE_CTRL_START_TRANS_BITS);
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
}

// ==[ Interrupt ]=============================================================

// Interrupt handler
void isr_usbctrl() {
    volatile uint32_t ints = usb_hw->ints;
    uint16_t size = 0;
    static event_t event;

    printf("┌───────┬──────────────────────────────────────────────────────────────────────┐\n");
    printf("│Frame\t│ %u\n", usb_hw->sof_rd);
    bindump("│IRQ", ints);
    bindump("│SIE", usb_hw->sie_status);
    bindump("│DEV", usb_hw->dev_addr_ctrl);
    bindump("│ECR", usbh_dpram->epx_ctrl);
    bindump("│BCR", usbh_dpram->epx_buf_ctrl);

    // Connection event (attach or detach)
    if (ints &  USB_INTS_HOST_CONN_DIS_BITS) {
        ints ^= USB_INTS_HOST_CONN_DIS_BITS;

        uint8_t speed = dev_speed();

        if (speed) {
            printf("│ISR\t│ Device connected\n");
            event.type = EVENT_CONNECTION;
            event.dev_addr = 0;
            event.conn.speed = speed;
            queue_add_blocking(queue, &event);
        } else {
            printf("│ISR\t│ Device disconnected\n");
        }

        // Clear speed change interrupt
        usb_hw_clear->sie_status = USB_SIE_STATUS_SPEED_BITS;
    }

    // Stall detected (before buffer status)
    if (ints &  USB_INTS_STALL_BITS) {
        ints ^= USB_INTS_STALL_BITS;

        printf("│ISR\t│ Stall detected\n");

        // Clear the stall
        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;
        // hw_xfer_complete(&epx, XFER_RESULT_STALLED);
    }

    // Buffer(s) ready
    if (ints &  USB_INTS_BUFF_STATUS_BITS) {
        ints ^= USB_INTS_BUFF_STATUS_BITS;

        // Show buffer status
        bindump("│BUF", usb_hw->buf_status);

        // For now, we know it's EP0_IN... (we're cheating)
        uint8_t len = usbh_dpram->epx_buf_ctrl & USB_BUF_CTRL_LEN_MASK;

        if (len) {
            printf("│> Data");
            hexdump(usbh_dpram->epx_data, len, 1);
        } else {
            printf("│> ZLP\n");
        }

// static void __tusb_irq_path_func(hw_handle_buff_status)()
// {
//   uint32_t remaining_buffers = usb_hw->buf_status;
//   pico_trace("buf_status 0x%08x\n", remaining_buffers);
//
//   // Check EPX first
//   uint bit = 0b1;
//   if ( remaining_buffers & bit )
//   {
//     remaining_buffers &= ~bit;
//     struct hw_endpoint * ep = &epx;
//
//     uint32_t ep_ctrl = *ep->endpoint_control;
//     if ( ep_ctrl & EP_CTRL_DOUBLE_BUFFERED_BITS )
//     {
//       TU_LOG(3, "Double Buffered: ");
//     }
//     else
//     {
//       TU_LOG(3, "Single Buffered: ");
//     }
//     TU_LOG_HEX(3, ep_ctrl);
//
//     _handle_buff_status_bit(bit, ep);
//   }
//
//   // Check "interrupt" (asynchronous) endpoints for both IN and OUT
//   for ( uint i = 1; i <= USB_HOST_INTERRUPT_ENDPOINTS && remaining_buffers; i++ )
//   {
//     // EPX is bit 0 & 1
//     // IEP1 IN  is bit 2
//     // IEP1 OUT is bit 3
//     // IEP2 IN  is bit 4
//     // IEP2 OUT is bit 5
//     // IEP3 IN  is bit 6
//     // IEP3 OUT is bit 7
//     // etc
//     for ( uint j = 0; j < 2; j++ )
//     {
//       bit = 1 << (i * 2 + j);
//       if ( remaining_buffers & bit )
//       {
//         remaining_buffers &= ~bit;
//         _handle_buff_status_bit(bit, &ep_pool[i]);
//       }
//     }
//   }
//
//   if ( remaining_buffers )
//   {
//     panic("Unhandled buffer %d\n", remaining_buffers);
//   }

        // Clear all buffers
        usb_hw_clear->buf_status = (uint32_t) ~0;
    }

    // Transfer complete (last packet)
    if (ints &  USB_INTS_TRANS_COMPLETE_BITS) {
        ints ^= USB_INTS_TRANS_COMPLETE_BITS;

        printf("│ISR\t│ Transfer complete\n");

        if (usb_hw->sie_ctrl & USB_SIE_CTRL_SEND_SETUP_BITS) {

// TODO: Mark this as complete...
//   uint8_t dev_addr = ep->dev_addr;
//   uint8_t ep_addr = ep->ep_addr;
//   uint xferred_len = ep->xferred_len;
//   hw_endpoint_reset_transfer(ep);
//   hcd_event_xfer_complete(dev_addr, ep_addr, xferred_len, xfer_result, true);

            printf("│XSD\t│ Device connected\n");
            event.type = EVENT_TRANSFER;
            // TODO: Do we set the EP number, transferred length, and result?
            event.xfer.ep_addr = 37;
            event.xfer.result  = 42;
            event.xfer.len     = 22;
            queue_add_blocking(queue, &event); // TODO: How "quick" is this queue? Race condition?
        }

        usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;
        // hw_trans_complete();
    }

    // Receive timeout (too long without an ACK)
    if (ints &  USB_INTS_ERROR_RX_TIMEOUT_BITS) {
        ints ^= USB_INTS_ERROR_RX_TIMEOUT_BITS;

        printf("│ISR\t│ Receive timeout\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;
    }

    // Data error (IN packet from device has wrong data PID)
    if (ints &  USB_INTS_ERROR_DATA_SEQ_BITS) {
        ints ^= USB_INTS_ERROR_DATA_SEQ_BITS;

        printf("│ISR\t│ Data error\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;
        panic("ERROR: USB Host data sequence error\n");
    }

    // Device resumed (device initiated)
    if (ints &  USB_INTS_HOST_RESUME_BITS) {
        ints ^= USB_INTS_HOST_RESUME_BITS;

        printf("│ISR\t│ Device resume\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;
    }

    // Any missed?
    if (ints) {
        panic("│ISR\t│ Unhandled IRQ 0x%04x\n", ints);
    }

    printf("└───────┴──────────────────────────────────────────────────────────────────────┘\n");
}

// ==[ Resets ]================================================================

// Reset USB host
void usb_host_reset() {

// TODO: Host reset...
//   // Device
//   tu_memclr(&_dev0, sizeof(_dev0));
//   tu_memclr(_usbh_devices, sizeof(_usbh_devices));
//   tu_memclr(&_ctrl_xfer, sizeof(_ctrl_xfer));
//
//   for(uint8_t i=0; i<TOTAL_DEVICES; i++) { clear_device(&_usbh_devices[i]); }
//
//   // Class drivers
//   for (uint8_t drv_id = 0; drv_id < TOTAL_DRIVER_COUNT; drv_id++) {
//     usbh_class_driver_t const* driver = get_driver(drv_id);
//     if (driver) driver->init();
//   }
//   _usbh_controller = controller_id;;
//   hcd_int_enable(controller_id);

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

void usb_task() {
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
        usb_task();
    }
}

// ==[ Done ]==================================================================

// #include "hardware/sync.h" // For __dmb()

// TODO: Class drivers...
// static usbh_class_driver_t const usbh_class_drivers[] = {
//     #if CFG_TUH_CDC
//     {
//         DRIVER_NAME("CDC")
//         .init       = cdch_init,
//         .open       = cdch_open,
//         .set_config = cdch_set_config,
//         .xfer_cb    = cdch_xfer_cb,
//         .close      = cdch_close

// static void clear_device(usbh_device_t* dev) {
//   tu_memclr(dev, sizeof(usbh_device_t));
//   memset(dev->itf2drv, TUSB_INDEX_INVALID_8, sizeof(dev->itf2drv)); // invalid mapping
//   memset(dev->ep2drv , TUSB_INDEX_INVALID_8, sizeof(dev->ep2drv )); // invalid mapping
// }

// NOTE: Endpoint API
// // Submit a transfer, when complete hcd_event_xfer_complete() must be invoked
// bool hcd_edpt_xfer(uint8_t rhport, uint8_t daddr, uint8_t ep_addr, uint8_t * buffer, uint16_t buflen);
//
// // Abort a queued transfer. Note: it can only abort transfer that has not been started
// // Return true if a queued transfer is aborted, false if there is no transfer to abort
// bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr);
//
// // Submit a special transfer to send 8-byte Setup Packet, when complete hcd_event_xfer_complete() must be invoked
// bool hcd_setup_send(uint8_t rhport, uint8_t daddr, uint8_t const setup_packet[8]);
//
// // clear stall, data toggle is also reset to DATA0
// bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr);

// Buffer for preparing output to send to host
// static uint8_t ep0_buf[64];

// ==[ Interrupt ]=============================================================

// AVAILABLE bit   => datasheet page 383 says to wait 1 usb_clk cycle
// START_TRANS bit => datasheet page 390 says to wait 2 usb_clk cycles

// Cycles to wait: usb_cycles * cpu_clk / usb_clk
// 2 cycles * 133MHz / 48MHz = 5.5833 cycles =>  6 cycles # ((133-1)/48+1)*2 => 6
// 2 cycles * 125MHz / 48MHz = 5.3333 cycles =>  6 cycles # ((125-1)/48+1)*2 => 6
// 1 cycle  * 133MHz / 48MHz = 2.7708 cycles =>  3 cycles # ((133-1)/48+1)*1 => 3
// 1 cycle  * 576MHz / 48MHz = 12     cycles => 12 cycles # ((576-1)/48+1)*1 => 12
// 2 cycles * 576MHz / 48MHz = 24     cycles => 24 cycles # ((576-1)/48+1)*2 => 24

// Each one takes 1 cycle and wastes 1 cycle, so 2 cycles * 6 = 12 cycles total
// __asm volatile (
//        "b 1f\n"
//     "1: b 1f\n"
//     "1: b 1f\n"
//     "1: b 1f\n"
//     "1: b 1f\n"
//     "1: b 1f\n"
//     "1:\n"
//     : : : "memory"
// );

// TinyUSB comment: "We don't need to delay in host mode, since host is in charge".
// Explanation... if this many cycles are going to pass anyway as we monkey around
// with the USB hardware, then we might as well wait for them to pass before and
// we might not even need to wait!

// static uint8_t _usbh_ctrl_buf[CFG_TUH_ENUMERATION_BUFSIZE];

// // Could be a nice way to delay a bit...
// uint32_t start;;
// printf("Frame 1: %u\n", start = usb_hw->sof_rd);
// while (usb_hw->sof_rd - start < 500) { tight_loop_contents(); }
// printf("Frame 2: %u\n", usb_hw->sof_rd);
