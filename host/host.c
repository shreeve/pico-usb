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
#include "helpers.h"              // Helper functions

#define MAKE_U16(x, y) (((x) << 8) | ((y)     ))
#define SWAP_U16(x)    (((x) >> 8) | ((x) << 8))

#define SDK_ALIGNED(bytes) __attribute__ ((aligned(bytes)))
#define SDK_ALWAYS_INLINE  __attribute__ ((always_inline))
#define SDK_PACKED         __attribute__ ((packed))
#define SDK_WEAK           __attribute__ ((weak))

// ==[ PicoUSB ]===============================================================

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

static queue_t queue_struct, *queue = &queue_struct;

// ==[ Hardware: rp2040 ]======================================================

#define usb_hw_set   ((usb_hw_t *) hw_set_alias_untyped  (usb_hw))
#define usb_hw_clear ((usb_hw_t *) hw_clear_alias_untyped(usb_hw))

#define busy_wait_3_cycles() __asm volatile("nop\nnop\nnop\n":::"memory")

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

SDK_ALWAYS_INLINE static inline uint8_t dev_speed() {
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

typedef void (*hw_endpoint_cb)(uint8_t *buf, uint16_t len);

typedef struct hw_endpoint {
    usb_endpoint_descriptor_t *usb; // USB descriptor
    volatile uint32_t *ecr;         // Endpoint control register
    volatile uint32_t *bcr;         // Buffer control register
    volatile uint8_t *buf;          // Data buffer
    uint8_t pid;                    // Toggle DATA0/DATA1 each packet
    hw_endpoint_cb cb;              // Callback function
    bool on;                        // Endpoint is on
} hw_endpoint_t;

// ==[ Endpoints ]=============================================================

#define EP0_OUT_ADDR (USB_DIR_OUT | 0)
#define EP0_IN_ADDR  (USB_DIR_IN  | 0)

static usb_endpoint_descriptor_t usb_epx = {
    .bLength          = sizeof(usb_endpoint_descriptor_t),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = EP0_OUT_ADDR, // Will switch between IN or OUT
    .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
    .wMaxPacketSize   = 64,
    .bInterval        = 0
};

void epx_cb(uint8_t *buf, uint16_t len) {
    printf("Inside the EPX callback...\n");
}

static hw_endpoint_t epx = {
    .usb = &usb_epx,
    .ecr = &usbh_dpram->epx_ctrl,
    .bcr = &usbh_dpram->epx_buf_ctrl,
    .buf = &usbh_dpram->epx_data[0],
    .pid = 1, // Starts with DATA1
    .cb  = epx_cb,
    .on  = false,
};

// Set up endpoint control register (ECR)
void setup_hw_endpoint(hw_endpoint_t *ep) {
    if (!ep || !ep->ecr) return;

    // Determine configuration
    uint32_t type = ep->usb->bmAttributes;
    uint32_t ms = ep->usb->bInterval;
    uint32_t interval_lsb = EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB;
    uint32_t offset = ((uint32_t) ep->buf) ^ ((uint32_t) usbh_dpram); // TODO: Why not just use subtraction?

    // Summarize endpoint configuration
    uint8_t ep_addr = ep->usb->bEndpointAddress;
    uint8_t ep_num = ep_addr & 0x0f;
    bool in = ep_addr & USB_DIR_IN;
    printf(" EP%d_%s│ 0x%02x │ Buffer offset 0x%04x\n",
           ep_num, in ? "IN " : "OUT", ep_addr, offset);

    // Set the endpoint control register
    *ep->ecr = EP_CTRL_ENABLE_BITS               // Endpoint enabled
             | EP_CTRL_INTERRUPT_PER_BUFFER      // One interrupt per buffer
             | type << EP_CTRL_BUFFER_TYPE_LSB   // Transfer type
             | (ms ? ms - 1 : 0) << interval_lsb // Interrupt interval in ms
             | offset;                           // Data buffer offset

    bindump(" ECR", *ep->ecr);

    ep->on = true;
}

// ==[ Transfers ]=============================================================

// Start a transfer
void start_transfer(hw_endpoint_t *ep, usb_setup_packet_t *packet, size_t size) {

    // Ensure endpoint is on
    if (!ep || !ep->on) {
        setup_hw_endpoint(ep);
    }

    // Set target device address and endpoint number
    // NOTE: 19:16=ep_num, 6:0=dev_addr
    // usb_hw->dev_addr_ctrl = (uint32_t) (dev_addr | (ep_num << USB_ADDR_ENDP_ENDPOINT_LSB));
    usb_hw->dev_addr_ctrl = 0;

    // Copy the packet to the transfer buffer
    memcpy((void *) usbh_dpram->setup_packet, packet, size); // TODO: for (i=0; i<8; i++) needed? better?
    bool in = packet->bmRequestType & USB_DIR_IN;

    // Set BCR
    uint32_t bcr = USB_BUF_CTRL_LAST
                 | USB_BUF_CTRL_DATA1_PID
                 | USB_BUF_CTRL_SEL
                 | size;

    // Datasheet § 4.1.2.5.1 (p. 383) says that when clk_sys (usually 133Mhz)
    // and clk_usb (usually 48MHz) are different, we must wait one USB clock
    // cycle before setting the AVAILABLE bit. Based on this, we should wait
    // 133MHz/48MHz * 1 clk_usb cycle = 2.8 clk_sys cycles (rounds up to 3).
    hw_set_staged3(usbh_dpram->epx_buf_ctrl, bcr, USB_BUF_CTRL_AVAIL);

    // Send the setup packet, using SIE_CTRL // TODO: preamble (LS on FS)
    uint32_t scr = USB_SIE_CTRL_BASE              // SIE_CTRL defaults
                 | USB_SIE_CTRL_SEND_SETUP_BITS   // Send a SETUP packet
           | (in ? USB_SIE_CTRL_RECEIVE_DATA_BITS // IN to host is receive
                 : USB_SIE_CTRL_SEND_DATA_BITS);  // OUT from host is send

    // Debug output
    bindump(" ECR", *ep->ecr);
    bindump(" BCR", bcr | USB_BUF_CTRL_AVAIL);
    bindump(" SCR", scr | USB_SIE_CTRL_START_TRANS_BITS);
    printf("< Setup");
    hexdump(packet, size, 1);

    // Datasheet § 4.1.2.7 (p. 390) says that when clk_sys (usually 133Mhz)
    // and clk_usb (usually 48MHz) are different, we must wait two USB clock
    // cycles before setting the START_TRANS bit. Based on this, we need
    // 133MHz/48MHz * 2 clk_usb cycles = 5.6 clk_sys cycles (rounds up to 6).
    // NOTE: TinyUSB doesn't wait here, just sayin'... can we combine w/above?
    hw_set_staged6(usb_hw->sie_ctrl, scr, USB_SIE_CTRL_START_TRANS_BITS);
}

// Send a zero length status packet (ZLP)
void send_zlp(hw_endpoint_t *ep) {

    // Ensure endpoint is on
    if (!ep || !ep->on) {
        setup_hw_endpoint(ep);
    }

    // Set target device address and endpoint number
    // usb_hw->dev_addr_ctrl = (uint32_t) (dev_addr | (ep_num << USB_ADDR_ENDP_ENDPOINT_LSB));
    usb_hw->dev_addr_ctrl = 0;

    // Set BCR
    uint32_t bcr = USB_BUF_CTRL_FULL // Indicates we've populated the buffer
                 | USB_BUF_CTRL_LAST
                 | USB_BUF_CTRL_DATA1_PID
                 | USB_BUF_CTRL_SEL;
    hw_set_staged3(usbh_dpram->epx_buf_ctrl, bcr, USB_BUF_CTRL_AVAIL);

    // Set SCR
    uint32_t scr = USB_SIE_CTRL_BASE              // SIE_CTRL defaults
                 | USB_SIE_CTRL_SEND_DATA_BITS;   // OUT from host is send

    // Debug output
    bindump(" ECR", *ep->ecr);
    bindump(" BCR", bcr | USB_BUF_CTRL_AVAIL);
    bindump(" SCR", scr | USB_SIE_CTRL_START_TRANS_BITS);
    printf("<ZLP\n");

    // NOTE: TinyUSB doesn't wait here, but the datasheet says we should...
    hw_set_staged6(usb_hw->sie_ctrl, scr, USB_SIE_CTRL_START_TRANS_BITS);
}

// NOTE: This is a single/global/static control transfer object.
// Control transfers: since most controllers do not support multiple control
// transfers on multiple devices concurrently and control transfers are mainly
// used for enumeration, we will only execute control transfers one at a time.

// Submit a transfer and, when complete, push an EVENT_TRANSFER completed event
// Abort a transfer, only if not yet started. Return true if queue xfer aborted
// Send a SETUP transfer. When complete, push an EVENT_TRANSFER completed event
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

//   tuh_xfer_t xfer = {
//     .daddr       = daddr,
//     .ep_addr     = 0,
//     .setup       = &request,
//     .buffer      = buffer,
//     .complete_cb = complete_cb,
//     .user_data   = user_data
//   };

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

//   bool const ret = tuh_control_xfer(&xfer);
//
//   // if blocking, user_data could be pointed to xfer_result
//   if ( !complete_cb && user_data ) {
//     *((xfer_result_t*) user_data) = xfer.result;
//   }

// typedef enum {
//   XFER_RESULT_SUCCESS = 0,
//   XFER_RESULT_FAILED,
//   XFER_RESULT_STALLED,
//   XFER_RESULT_TIMEOUT,
//   XFER_RESULT_INVALID
// } xfer_result_t;

// ==[ Enumeration ]===========================================================

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

// Get device descriptor
void get_device_descriptor() {
    printf("Get device descriptor\n");

    static usb_setup_packet_t packet = {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(USB_DT_DEVICE, 0),
        .wIndex        = 0,
        .wLength       = 8, // If dev_addr > 0, then use: sizeof(usb_device_descriptor_t)
    };

    start_transfer(&epx, &packet, sizeof(packet));
}

// Set device address
void set_device_address() {
    uint8_t dev_addr = 1;
    printf("Set device address to %u\n", dev_addr);

    // Setup packet
    usb_setup_packet_t packet = {
        .bmRequestType = USB_DIR_OUT
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_SET_ADDRESS,
        .wValue        = dev_addr,
        .wIndex        = 0,
        .wLength       = 0,
    };

    start_transfer(&epx, &packet, sizeof(packet));
}

void start_enumeration() {
    printf("Start enumeration\n");

// static uint8_t _usbh_ctrl_buf[CFG_TUH_ENUMERATION_BUFSIZE];

    get_device_descriptor();
}

// ==[ Interrupt ]=============================================================

// Interrupt handler
void isr_usbctrl() {
    volatile uint32_t intr = usb_hw->intr;
    volatile uint32_t ints = usb_hw->ints;
    uint16_t size = 0;
    static event_t event;

    printf("┌───────┬──────┬──────────────────────────────────────────────────┐\n");
    printf("│Frame\t│ %4u │%50s│\n", usb_hw->sof_rd, "");
    bindump("│INTR", intr);
    bindump("│INTS", ints);
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
            printf("│<ZLP\n"); // which direction?!
        }

// static void __tusb_irq_path_func(hw_handle_buff_status)()
// {
//   uint32_t remaining_buffers = usb_hw->buf_status;
//   pico_trace("buf_status 0x%08x\n", remaining_buffers);
//
//   // Check EPX first
//   uint bit = 0b1;
//   if ( remaining_buffers & bit ) {
//     remaining_buffers &= ~bit;
//     hw_endpoint_t * ep = &epx;
//
//     uint32_t ep_ctrl = *ep->ecr;
    // if ( ep_ctrl & EP_CTRL_DOUBLE_BUFFERED_BITS ) {
    //   TU_LOG(3, "Double Buffered: ");
    // } else {
    //   TU_LOG(3, "Single Buffered: ");
    // }
//     TU_LOG_HEX(3, ep_ctrl);
//
//     _handle_buff_status_bit(bit, ep);
//   }

// static void __tusb_irq_path_func(_handle_buff_status_bit)(uint bit, hw_endpoint_t *ep)
// {
//   usb_hw_clear->buf_status = bit;
//   // EP may have been stalled?
//   assert(ep->active);
//   bool done = hw_endpoint_xfer_continue(ep);
//   if ( done ) {
//     hw_xfer_complete(ep, XFER_RESULT_SUCCESS);
//   }
// }

// // Returns true if transfer is complete
// bool __tusb_irq_path_func(hw_endpoint_xfer_continue)(hw_endpoint_t *ep) {
//   hw_endpoint_lock_update(ep, 1);
//
//   // Part way through a transfer
//   if (!ep->active) {
//     panic("Can't continue xfer on inactive ep %d %s", tu_edpt_number(ep->ep_addr), ep_dir_string[tu_edpt_dir(ep->ep_addr)]);
//   }
//
//   // Update EP struct from hardware state
//   _hw_endpoint_xfer_sync(ep);
//
//   // Now we have synced our state with the hardware. Is there more data to transfer?
//   // If we are done then notify tinyusb
//   if (ep->remaining_len == 0) {
//     pico_trace("Completed transfer of %d bytes on ep %d %s\n",
//                ep->xferred_len, tu_edpt_number(ep->ep_addr), ep_dir_string[tu_edpt_dir(ep->ep_addr)]);
//     // Notify caller we are done so it can notify the tinyusb stack
//     hw_endpoint_lock_update(ep, -1);
//     return true;
//   } else {
//       hw_endpoint_start_next_buffer(ep);
//     }
//   }
//
//   hw_endpoint_lock_update(ep, -1);
//
//   // More work to do
//   return false;
// }

//   // Check "interrupt" (asynchronous) endpoints for both IN and OUT
//   for ( uint i = 1; i <= USB_HOST_INTERRUPT_ENDPOINTS && remaining_buffers; i++ ) {
//     // EPX is bit 0 & 1
//     // IEP1 IN  is bit 2
//     // IEP1 OUT is bit 3
//     // IEP2 IN  is bit 4
//     // IEP2 OUT is bit 5, etc...
//    for ( uint j = 0; j < 2; j++ ) {
//       bit = 1 << (i * 2 + j);
//       if ( remaining_buffers & bit ) {
//         remaining_buffers &= ~bit;
//         _handle_buff_status_bit(bit, &ep_pool[i]);
//       }
//     }
//   }
//
//   if ( remaining_buffers ) {
//     panic("Unhandled buffer %d\n", remaining_buffers);
//   }

        // Clear all buffers
        usb_hw_clear->buf_status = (uint32_t) ~0;

        // set_device_address();
        // get_device_descriptor();
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

    printf("└───────┴─────────────────────────────────────────────────────────┘\n");
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
    // setup_hw_endpoints();
    setup_hw_endpoint(&epx);

    bindump(" INT", usb_hw->inte);

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
                if (event.xfer.len == 0) {
                    send_zlp(&epx);
                }
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
