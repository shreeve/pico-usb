// ==[ USB 2.0 ]===============================================================

#include "usb_common.h"

// ==[ PicoUSB ]===============================================================

// ==[ Resets ]================================================================

// Reset USB host
void usb_host_reset() {

    // Reset controller
    reset_block       (RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    // Clear state
    memset(usb_hw   , 0, sizeof(*usb_hw   ));
    memset(usb_dpram, 0, sizeof(*usb_dpram));
    irq_set_enabled(USBCTRL_IRQ, true);

    // Setup host mode
    usb_hw->muxing    = USB_USB_MUXING_TO_PHY_BITS              |
                        USB_USB_MUXING_SOFTCON_BITS             ;
    usb_hw->pwr       = USB_USB_PWR_VBUS_DETECT_BITS            |
                        USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS        |
                        USB_MAIN_CTRL_HOST_NDEVICE_BITS         ;
    usb_hw->sie_ctrl  = USB_SIE_CTRL_EP0_INT_1BUF_BITS          |
                        USB_SIE_CTRL_SOF_EN_BITS                |
                        USB_SIE_CTRL_KEEP_ALIVE_EN_BITS         |
                        USB_SIE_CTRL_PULLDOWN_EN_BITS           ;
    usb_hw->inte      = USB_INTE_BUFF_STATUS_BITS               |
                        USB_INTE_HOST_CONN_DIS_BITS             |
                        USB_INTE_HOST_RESUME_BITS               |
                        USB_INTE_STALL_BITS                     |
                        USB_INTE_TRANS_COMPLETE_BITS            |
                        USB_INTE_ERROR_RX_TIMEOUT_BITS          |
                        USB_INTE_ERROR_DATA_SEQ_BITS            ;

    // usb_setup_endpoints();
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;

    return true;
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
