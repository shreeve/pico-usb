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

#include <stdio.h>                // For printf
#include <string.h>               // For memcpy

#include "pico/stdlib.h"          // Pico stdlib
#include "pico/util/queue.h"      // A beautifully simple queue
#include "hardware/regs/usb.h"    // USB hardware registers from pico-sdk
#include "hardware/structs/usb.h" // USB hardware structs from pico-sdk
#include "hardware/irq.h"         // For interrupt enable and numbers
#include "hardware/resets.h"      // For resetting the native USB controller

#define usb_hw_set   ((usb_hw_t *) hw_set_alias_untyped  (usb_hw))
#define usb_hw_clear ((usb_hw_t *) hw_clear_alias_untyped(usb_hw))

#define PU_ALIGNED(bytes) __attribute__ ((aligned(bytes)))
#define PU_ALWAYS_INLINE  __attribute__ ((always_inline))
#define PU_PACKED         __attribute__ ((packed))
#define PU_WEAK           __attribute__ ((weak))

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

PU_ALWAYS_INLINE static inline uint8_t dev_speed() {
    return (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) \
                              >> USB_SIE_STATUS_SPEED_LSB;
}

// Hex dump (mode: 0 = hex; 1 = hex + ascii; 2 = hex + ascii + no newline)
void hexdump(const void* data, size_t size, uint mode) {
    const unsigned char* byte = (const unsigned char *) data;
    size_t i, j;

    for (i = 0; i < size; i += 16) {
        printf("\t│ %08zx │ ", i); // Print the offset

        // Print hex values
        for (j = 0; j < 16; j++) {
            if (i + j < size) {
                printf("%02x ", byte[i + j]);
            } else {
                printf("   "); // Pad if less than 16 bytes in the line
            }
        }

        printf(" │ ");

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

// ==[ Enumeration ]===========================================================

enum {
    USB_SIE_CTRL_BASE = USB_SIE_CTRL_VBUS_EN_BITS       | // Supply VBUS to device
                        USB_SIE_CTRL_SOF_EN_BITS        | // Enable full speed bus
                        USB_SIE_CTRL_KEEP_ALIVE_EN_BITS | // Enable low speed bus
                        USB_SIE_CTRL_PULLDOWN_EN_BITS   | // Enable devices to connect
                        USB_SIE_CTRL_EP0_INT_1BUF_BITS    // Interrupt on every buffer
};

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
            // printf("Device connected\n");
            event.type = EVENT_CONNECTION;
            event.dev_addr = 0;
            event.conn.speed = speed;
            queue_add_blocking(queue, &event);
        } else {
            printf("Device disconnected\n");
        }

        // Clear speed change interrupt
        usb_hw_clear->sie_status = USB_SIE_STATUS_SPEED_BITS;
    }

    // Stall detected (before buffer status)
    if (ints &  USB_INTS_STALL_BITS) {
        ints ^= USB_INTS_STALL_BITS;

        printf("Stall detected\n");

        // Clear the stall
        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;
        // hw_xfer_complete(&epx, XFER_RESULT_STALLED);
    }

    // Buffer(s) ready
    if (ints &  USB_INTS_BUFF_STATUS_BITS) {
        ints ^= USB_INTS_BUFF_STATUS_BITS;

        // Show buffer status
        x = usb_hw->buf_status; printf("BUF\t| %032b 0x%08x\n", x, x);

        // hw_handle_buff_status();

        // Clear all buffers
        usb_hw_clear->buf_status = (uint32_t) ~0;
    }

    // Transfer complete (last packet)
    if (ints &  USB_INTS_TRANS_COMPLETE_BITS) {
        ints ^= USB_INTS_TRANS_COMPLETE_BITS;

        printf("Transfer complete\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;
        // hw_trans_complete();
    }

    // Receive timeout (too long without an ACK)
    if (ints &  USB_INTS_ERROR_RX_TIMEOUT_BITS) {
        ints ^= USB_INTS_ERROR_RX_TIMEOUT_BITS;

        printf("Receive timeout\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;
    }

    // Data error (IN packet from device has wrong data PID)
    if (ints &  USB_INTS_ERROR_DATA_SEQ_BITS) {
        ints ^= USB_INTS_ERROR_DATA_SEQ_BITS;

        printf("Data error\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;
        panic("ERROR: USB Host data sequence error\n");
    }

    // Device resumed (device initiated)
    if (ints &  USB_INTS_HOST_RESUME_BITS) {
        ints ^= USB_INTS_HOST_RESUME_BITS;

        printf("Device resume\n");

        usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;
    }

    // Any missed?
    if (ints) {
        panic("Unhandled IRQ 0x%04x\n", ints);
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

    printf("USB host reset\n");
    irq_set_enabled(USBCTRL_IRQ, true);
}

// ==[ Main ]==================================================================

void start_enumeration() {
    printf("Will start enumeration now...\n");

    // Get device descriptor
    usb_setup_packet_t request = {
        .bmRequestType = USB_DIR_IN            |
                         USB_REQ_TYPE_STANDARD |
                         USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = USB_DT_DEVICE,
        .wIndex        = 0,
        .wLength       = sizeof(usb_setup_packet_t),
    };
}

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
