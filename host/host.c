// ==[ USB 2.0 ]===============================================================

#include "usb_common.h"

// usb_endpoint_descriptor
// usb_interface_descriptor
// usb_configuration_descriptor
// usb_device_descriptor

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

// ==[ Resets ]================================================================

// Reset USB host
void usb_host_reset() {

    // Reset controller
    reset_block       (RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    // Clear state
    memset(usb_hw   , 0, sizeof(*usb_hw   ));
    memset(usb_dpram, 0, sizeof(*usb_dpram));

    // Setup host mode
    usb_hw->muxing    = USB_USB_MUXING_TO_PHY_BITS              | // Connect USB Phy
                        USB_USB_MUXING_SOFTCON_BITS             ; // TODO: What is this?
    usb_hw->pwr       = USB_USB_PWR_VBUS_DETECT_BITS            | // Enable VBUS detection
                        USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS; // Enable VBUS detection
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS        | // Enable controller
                        USB_MAIN_CTRL_HOST_NDEVICE_BITS         ; // Host mode
    usb_hw->sie_ctrl  = USB_SIE_CTRL_VBUS_EN_BITS               | // Supply VBUS to device
                        USB_SIE_CTRL_SOF_EN_BITS                | // Enable full speed bus
                        USB_SIE_CTRL_KEEP_ALIVE_EN_BITS         | // Enable low speed bus
                        USB_SIE_CTRL_PULLDOWN_EN_BITS           ; // Enable devices to connect
    usb_hw->inte      = USB_INTE_HOST_CONN_DIS_BITS             | // Device connect/disconnect
                        USB_INTE_STALL_BITS                     | // Stall detected
                        USB_INTE_BUFF_STATUS_BITS               | // Buffer ready
                        USB_INTE_TRANS_COMPLETE_BITS            | // Transfer complete
                        USB_INTE_HOST_RESUME_BITS               | // Device resumed
                        USB_INTE_ERROR_DATA_SEQ_BITS            | // Data error
                        USB_INTE_ERROR_RX_TIMEOUT_BITS          ; // Receive timeout

    // usb_setup_endpoints();
    irq_set_enabled(USBCTRL_IRQ, true); // irq_set_exclusive_handler(USBCTRL_IRQ, isr_usbctrl);
    printf("USB host reset\n\n");
}

// ==[ Helpers ]===============================================================

PU_ALWAYS_INLINE static inline bool is_host_mode(void) {
    return (usb_hw->main_ctrl & USB_MAIN_CTRL_HOST_NDEVICE_BITS);
}

PU_ALWAYS_INLINE static inline bool is_vbus_overcurr(void) {
    return (usb_hw->sie_status & USB_SIE_STATUS_VBUS_OVER_CURR_BITS);
}

PU_ALWAYS_INLINE static inline uint8_t dev_speed(void) {
    return (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) \
                              >> USB_SIE_STATUS_SPEED_LSB;
}

PU_ALWAYS_INLINE static inline uint8_t line_state(void) {
    return (usb_hw->sie_status & USB_SIE_STATUS_LINE_STATE_BITS) \
                              >> USB_SIE_STATUS_LINE_STATE_LSB;
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
