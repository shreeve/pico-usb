// ==[ USB 2.0 ]===============================================================

#include "usb_common.h"

#define EP0_OUT_ADDR (USB_DIR_OUT | 0)
#define EP0_IN_ADDR  (USB_DIR_IN  | 0)

static const struct usb_endpoint_descriptor ep0_out = { // EP0, out to device
    .bLength          = sizeof(struct usb_endpoint_descriptor),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = EP0_OUT_ADDR,
    .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
    .wMaxPacketSize   = 64,
    .bInterval        = 0
};

static const struct usb_endpoint_descriptor ep0_in = { // EP0, in to host
    .bLength          = sizeof(struct usb_endpoint_descriptor),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = EP0_IN_ADDR,
    .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
    .wMaxPacketSize   = 64,
    .bInterval        = 0
};


// ==[ PicoUSB ]===============================================================

#include <stdio.h> // For printf
#include <string.h> // For memcpy

#include "pico/stdlib.h" // Pico stdlib
#include "hardware/regs/usb.h" // USB hardware registers from pico-sdk
#include "hardware/structs/usb.h" // USB hardware structs from pico-sdk
#include "hardware/irq.h" // For interrupt enable and numbers
#include "hardware/resets.h" // For resetting the native USB controller
#include "hardware/sync.h" // For __dmb()

#define usb_hw_set   ((usb_hw_t *) hw_set_alias_untyped  (usb_hw))
#define usb_hw_clear ((usb_hw_t *) hw_clear_alias_untyped(usb_hw))

typedef void (*usb_ep_handler)(uint8_t *buf, uint16_t len);

void ep0_out_handler(uint8_t *buf, uint16_t len);
void ep0_in_handler (uint8_t *buf, uint16_t len);

struct usb_endpoint {
    const struct usb_endpoint_descriptor *descriptor;
    usb_ep_handler handler;

    volatile uint32_t *endpoint_control;
    volatile uint32_t *buffer_control;
    volatile uint8_t  *data_buffer;

    uint8_t next_datapid; // Toggle DATA0/DATA1 each packet
};

static struct usb_endpoint epx = {
    .descriptor       = &ep0_out,
    .handler          = NULL,
    .endpoint_control = &usbh_dpram->epx_ctrl,
    .buffer_control   = &usbh_dpram->epx_buf_ctrl,
    .data_buffer      = &usbh_dpram->epx_data[0],
    .next_datapid     = 1, // Starts with DATA1
};

// ==[ Endpoints ]=============================================================

// Set up an endpoint's control register
void usb_setup_endpoint(const struct usb_endpoint *ep) {

    // Grok the desired endpoint
    uint8_t ep_addr = ep->descriptor->bEndpointAddress;
    uint8_t ep_num = ep_addr & 0x0f;
    bool in = ep_addr & USB_DIR_IN;
    printf("Initialized EP%d_%s (0x%02x) with buffer address 0x%p\n",
           ep_num, in ? "IN " : "OUT", ep_addr, ep->data_buffer);

    // Set ep_ctrl register for this endpoint (skip EP0 since it uses SIE_CTRL)
    if (ep->endpoint_control) {
        uint32_t type = ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB;
        uint32_t offset = ((uint32_t) ep->data_buffer) ^ ((uint32_t) usb_dpram);
        uint32_t interval = ep->descriptor->bInterval;
        if (interval) {
            interval = (interval - 1) << EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB;
        }
        *ep->endpoint_control = EP_CTRL_ENABLE_BITS          | // Enable EP
                                EP_CTRL_INTERRUPT_PER_BUFFER | // One IRQ per
                                type     | // Control, iso, bulk, or interrupt
                                interval | // Interrupt interval in ms
                                offset   ; // Address base offset in DSPRAM
    }
}

// ==[ Transfers ]=============================================================

// Hex dump (mode: 0 = hex; 1 = hex + ascii; 2 = hex + ascii + no newline)
void hexdump(const void* data, size_t size, uint mode) {
    const unsigned char* byte = (const unsigned char *) data;
    size_t i, j;

    for (i = 0; i < size; i += 16) {
        printf("\t| %08zx | ", i); // Print the offset

        // Print hex values
        for (j = 0; j < 16; j++) {
            if (i + j < size) {
                printf("%02x ", byte[i + j]);
            } else {
                printf("   "); // Pad if less than 16 bytes in the line
            }
        }

        printf(" | ");

        if (mode > 1) return;

        // Print ASCII values
        if (mode == 1) {
            for (j = 0; j < 16; j++) {
                if (i + j < size) {
                    unsigned char ch = byte[i + j];
                    printf("%c", (ch >= 32 && ch <= 126) ? ch : '.');
                }
            }
        }

        printf("\n");
    }
}

// ==[ Handlers ]==============================================================

void ep0_out_handler(uint8_t *buf, uint16_t len) {
    ; // Nothing to do
    // TODO: Find out when this is called...
}

void ep0_in_handler(uint8_t *buf, uint16_t len) {
    if (should_set_address) {
        usb_hw->dev_addr_ctrl = device_address; // Set hardware device address
        should_set_address = false;
    } else {
        // Prepare for a ZLP from host on EP0_OUT
        usb_start_transfer(usb_get_endpoint(EP0_OUT_ADDR), NULL, 0);
    }
}

// ==[ Resets ]================================================================

enum {
    USB_SIE_CTRL_BASE = USB_SIE_CTRL_VBUS_EN_BITS       | // Supply VBUS to device
                        USB_SIE_CTRL_SOF_EN_BITS        | // Enable full speed bus
                        USB_SIE_CTRL_KEEP_ALIVE_EN_BITS | // Enable low speed bus
                        USB_SIE_CTRL_PULLDOWN_EN_BITS   | // Enable devices to connect
                        USB_SIE_CTRL_EP0_INT_1BUF_BITS    // Interrupt on every buffer
};

// Reset USB host
void usb_host_reset() {

    // Reset controller
    reset_block       (RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    // Clear state
    memset(usb_hw    , 0, sizeof(*usb_hw    ));
    memset(usbh_dpram, 0, sizeof(*usbh_dpram));

    // Setup endpoints
    usb_setup_endpoint(&epx);
    // usb_setup_endpoints();

    // Configure USB host controller
    usb_hw->muxing    = USB_USB_MUXING_TO_PHY_BITS               | // Connect USB Phy
                        USB_USB_MUXING_SOFTCON_BITS              ; // TODO: What is this?
    usb_hw->pwr       = USB_USB_PWR_VBUS_DETECT_BITS             | // Enable VBUS detection
                        USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS ; // Enable VBUS detection
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS         | // Enable controller
                        USB_MAIN_CTRL_HOST_NDEVICE_BITS          ; // Enable USB Host mode
    usb_hw->sie_ctrl  = USB_SIE_CTRL_BASE                        ; // Default SIE_CTRL bits
    usb_hw->inte      = USB_INTE_HOST_CONN_DIS_BITS              | // Device connect/disconnect
                        USB_INTE_STALL_BITS                      | // Stall detected
                        USB_INTE_BUFF_STATUS_BITS                | // Buffer ready
                        USB_INTE_TRANS_COMPLETE_BITS             | // Transfer complete
                        USB_INTE_HOST_RESUME_BITS                | // Device resumed
                        USB_INTE_ERROR_DATA_SEQ_BITS             | // Data error
                        USB_INTE_ERROR_RX_TIMEOUT_BITS           ; // Receive timeout

    printf("\nUSB host reset\n\n");
    irq_set_enabled(USBCTRL_IRQ, true);
}

// ==[ Helpers ]===============================================================

PU_ALWAYS_INLINE static inline bool is_host_mode(void) {
    return (usb_hw->main_ctrl & USB_MAIN_CTRL_HOST_NDEVICE_BITS);
}

PU_ALWAYS_INLINE static inline uint8_t dev_speed(void) {
    return (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) \
                              >> USB_SIE_STATUS_SPEED_LSB;
}

PU_ALWAYS_INLINE static inline uint8_t line_state(void) {
    return (usb_hw->sie_status & USB_SIE_STATUS_LINE_STATE_BITS) \
                              >> USB_SIE_STATUS_LINE_STATE_LSB;
}

// ==[ Interrupt ]=============================================================

// Interrupt handler
void isr_usbctrl() {
    uint32_t ints = usb_hw->ints;

    // Connection event (attach or detach)
    if (ints &  USB_INTS_HOST_CONN_DIS_BITS) {
        ints ^= USB_INTS_HOST_CONN_DIS_BITS;

        printf("device speed: 0b%02b\n", dev_speed());

        // Clear speed change interrupt
        usb_hw_clear->sie_status = USB_SIE_STATUS_SPEED_BITS;
    }

    // Stall detected (before buffer status)
    if (ints &  USB_INTS_STALL_BITS) {
        ints ^= USB_INTS_STALL_BITS;

        printf("stall detected\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;
        // hw_xfer_complete(&epx, XFER_RESULT_STALLED);
    }

    // Buffer ready
    if (ints &  USB_INTS_BUFF_STATUS_BITS) {
        ints ^= USB_INTS_BUFF_STATUS_BITS;

        printf("buffer ready: 0b%032b\n", usb_hw->buf_status);

        // hw_handle_buff_status();
        usb_hw_clear->buf_status = (uint32_t) -1; // clear all buffers (DONT DO THIS)
    }

    // Transfer complete (last packet)
    if (ints &  USB_INTS_TRANS_COMPLETE_BITS) {
        ints ^= USB_INTS_TRANS_COMPLETE_BITS;

        printf("transfer complete\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;
        // hw_trans_complete();
    }

    // Receive timeout (too long without an ACK)
    if (ints &  USB_INTS_ERROR_RX_TIMEOUT_BITS) {
        ints ^= USB_INTS_ERROR_RX_TIMEOUT_BITS;

        printf("receive timeout\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;
    }

    // Data error (IN packet from device has wrong data PID)
    if (ints &  USB_INTS_ERROR_DATA_SEQ_BITS) {
        ints ^= USB_INTS_ERROR_DATA_SEQ_BITS;

        printf("data error\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;
        panic("ERROR: USB Host data sequence error\n");
    }

    // Device resumed (device initiated)
    if (ints &  USB_INTS_HOST_RESUME_BITS) {
        ints ^= USB_INTS_HOST_RESUME_BITS;

        printf("device resume\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;
    }

    // Any missed?
    if (ints) {
        panic("Unhandled IRQ 0x%04x\n", ints);
    }
}

// ==[ Main ]==================================================================

int main() {
    stdio_init_all();
    printf("\033[2J\033[H\n==[ USB host example]==\n\n");
    usb_host_reset();

    // Wait until configured
    while (!configured) { tight_loop_contents(); }
    sleep_ms(500); // brief pause

    printf("\nUSB host configured\n\n");

    // Everything is interrupt driven so just loop here
    while (1) { tight_loop_contents(); }
}
