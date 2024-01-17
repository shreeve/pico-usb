#include <stdio.h> // For printf
#include <string.h> // For memcpy

#include "pico/stdlib.h" // Pico stdlib
#include "hardware/regs/usb.h" // USB registers from pico-sdk
#include "hardware/structs/usb.h" // USB hardware structs from pico-sdk
#include "hardware/irq.h" // For interrupt enable and numbers
#include "hardware/resets.h" // For resetting the USB controller

#include "usb_common.h" // Includes descriptor structs

// ==[ Declarations ]==========================================================

#define usb_hw_set   ((usb_hw_t *)hw_set_alias_untyped  (usb_hw))
#define usb_hw_clear ((usb_hw_t *)hw_clear_alias_untyped(usb_hw))

#define EP0_OUT_ADDR (USB_DIR_OUT | 0)
#define EP0_IN_ADDR  (USB_DIR_IN  | 0)
#define EP1_OUT_ADDR (USB_DIR_OUT | 1)
#define EP2_IN_ADDR  (USB_DIR_IN  | 2)

void ep0_out_handler(uint8_t *buf, uint16_t len);
void ep0_in_handler (uint8_t *buf, uint16_t len);
void ep1_out_handler(uint8_t *buf, uint16_t len);
void ep2_in_handler (uint8_t *buf, uint16_t len);

typedef void (*usb_ep_handler)(uint8_t *buf, uint16_t len);

// Global device address
static bool should_set_address = false;
static uint8_t dev_addr = 0;
static volatile bool configured = false;

// Global data buffer for EP0
static uint8_t ep0_buf[64];

struct usb_endpoint {
    const struct usb_endpoint_descriptor *descriptor;
    usb_ep_handler handler;

    volatile uint32_t *endpoint_control;
    volatile uint32_t *buffer_control;
    volatile uint8_t  *data_buffer;

    uint8_t next_datapid; // toggle datapid (DATA0/DATA1) after each packet
};

struct usb_device {
    const struct usb_device_descriptor        *device_descriptor;
    const unsigned char                       *lang_descriptor;
    const unsigned char                       **descriptor_strings;
    const struct usb_configuration_descriptor *config_descriptor;
    const struct usb_interface_descriptor     *interface_descriptor;
    struct usb_endpoint                       endpoints[USB_NUM_ENDPOINTS];
};

// ==[ Configuration ]=========================================================

static const struct usb_endpoint_descriptor ep0_out = {
    .bLength          = sizeof(struct usb_endpoint_descriptor),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = EP0_OUT_ADDR, // EP number 0, OUT from host
    .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
    .wMaxPacketSize   = 64,
    .bInterval        = 0
};

static const struct usb_endpoint_descriptor ep0_in = {
    .bLength          = sizeof(struct usb_endpoint_descriptor),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = EP0_IN_ADDR, // EP number 0, OUT from host
    .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
    .wMaxPacketSize   = 64,
    .bInterval        = 0
};

static const struct usb_endpoint_descriptor ep1_out = {
    .bLength          = sizeof(struct usb_endpoint_descriptor),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = EP1_OUT_ADDR, // EP number 1, OUT from host
    .bmAttributes     = USB_TRANSFER_TYPE_BULK,
    .wMaxPacketSize   = 64,
    .bInterval        = 0
};

static const struct usb_endpoint_descriptor ep2_in = {
    .bLength          = sizeof(struct usb_endpoint_descriptor),
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = EP2_IN_ADDR, // EP number 2, IN to host
    .bmAttributes     = USB_TRANSFER_TYPE_BULK,
    .wMaxPacketSize   = 64,
    .bInterval        = 0
};

static const struct usb_interface_descriptor interface_descriptor = {
    .bLength            = sizeof(struct usb_interface_descriptor),
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = 0,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 2,    // Two endpoints
    .bInterfaceClass    = 0xff, // Vendor specific endpoint
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface         = 0
};

static const struct usb_configuration_descriptor config_descriptor = {
    .bLength             = sizeof(struct usb_configuration_descriptor),
    .bDescriptorType     = USB_DT_CONFIG,
    .wTotalLength        = (sizeof(config_descriptor) +
                            sizeof(interface_descriptor) +
                            sizeof(ep1_out) +
                            sizeof(ep2_in)),
    .bNumInterfaces      = 1,    // One interface
    .bConfigurationValue = 1,    // Configuration 1
    .iConfiguration      = 0,    // No string
    .bmAttributes        = 0xc0, // attributes: self powered, no remote wakeup
    .bMaxPower           = 0x32  // 100ma
};

static const struct usb_device_descriptor device_descriptor = {
    .bLength            = sizeof(struct usb_device_descriptor),
    .bDescriptorType    = USB_DT_DEVICE,
    .bcdUSB             = 0x0110, // USB 1.1 device
    .bDeviceClass       = 0,      // Specified in interface descriptor
    .bDeviceSubClass    = 0,      // No subclass
    .bDeviceProtocol    = 0,      // No protocol
    .bMaxPacketSize0    = 64,     // Max packet size for ep0
    .idVendor           = 0x0000, // Your vendor id
    .idProduct          = 0x0001, // Your product ID
    .bcdDevice          = 0,      // No device revision number
    .iManufacturer      = 1,      // Manufacturer string index
    .iProduct           = 2,      // Product string index
    .iSerialNumber      = 0,      // No serial number
    .bNumConfigurations = 1       // One configuration
};

static const unsigned char *descriptor_strings[] = {
    (unsigned char *) "Raspberry Pi",    // Vendor
    (unsigned char *) "Pico Test Device" // Product
};

static const unsigned char lang_descriptor[] = {
    4,         // bLength
    0x03,      // bDescriptorType == String Descriptor
    0x09, 0x04 // language id = us english
};

static struct usb_device device = {
    .device_descriptor    = &device_descriptor,
    .interface_descriptor = &interface_descriptor,
    .config_descriptor    = &config_descriptor,
    .lang_descriptor      = lang_descriptor,
    .descriptor_strings   = descriptor_strings,
    .endpoints = {
        {
            .descriptor       = &ep0_out,
            .handler          = &ep0_out_handler,
            .endpoint_control = NULL, // NA for EP0
            .buffer_control   = &usb_dpram->ep_buf_ctrl[0].out,
            .data_buffer      = &usb_dpram->ep0_buf_a[0], // EP0 in and out share
        },
        {
            .descriptor       = &ep0_in,
            .handler          = &ep0_in_handler,
            .endpoint_control = NULL, // NA for EP0,
            .buffer_control   = &usb_dpram->ep_buf_ctrl[0].in,
            .data_buffer      = &usb_dpram->ep0_buf_a[0], // EP0 in and out share
        },
        {
            .descriptor       = &ep1_out,
            .handler          = &ep1_out_handler,
            .endpoint_control = &usb_dpram->ep_ctrl[0].out, // EP1 starts at offset 0
            .buffer_control   = &usb_dpram->ep_buf_ctrl[1].out,
            .data_buffer      = &usb_dpram->epx_data[0 * 64], // First free EPX buffer
        },
        {
            .descriptor       = &ep2_in,
            .handler          = &ep2_in_handler,
            .endpoint_control = &usb_dpram->ep_ctrl[1].in,
            .buffer_control   = &usb_dpram->ep_buf_ctrl[2].in,
            .data_buffer      = &usb_dpram->epx_data[1 * 64], // Second free EPX buffer
        }
    }
};

// ==[ Endpoints ]=============================================================

// set up ep_ctrl register for an endpoint (not EP0)
void usb_setup_endpoint(const struct usb_endpoint *ep) {
    printf("Set up endpoint 0x%02x with buffer address 0x%p\n", ep->descriptor->bEndpointAddress, ep->data_buffer);

    // EP0 doesn't have one so return if that is the case
    if (!ep->endpoint_control) return;

    // Get the data buffer as an offset of the USB controller's DPRAM
    uint32_t dpram_offset = ((uint32_t) (ep->data_buffer)) ^ ((uint32_t) usb_dpram); // TODO: This ok?
    uint32_t reg = EP_CTRL_ENABLE_BITS          | // enable endpoint
                   EP_CTRL_INTERRUPT_PER_BUFFER | // one irq per transferred buffer
                   (ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB) | // ctrl, iso, bulk, int
                   dpram_offset;
    *ep->endpoint_control = reg;
}

// set up ep_ctrl register for all endpoints (not EP0)
void usb_setup_endpoints() {
    const struct usb_endpoint *endpoints = device.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && endpoints[i].handler) {
            usb_setup_endpoint(&endpoints[i]);
        }
    }
}

// ep addr -> &endpoints[i]
struct usb_endpoint *usb_get_endpoint_configuration(uint8_t addr) {
    struct usb_endpoint *endpoints = device.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor &&
           (endpoints[i].descriptor->bEndpointAddress == addr)) {
            return &endpoints[i];
        }
    }
    return NULL;
}

// ==[ Handlers ]==============================================================

// start a transfer on an endpoint
void usb_start_transfer(struct usb_endpoint *ep, uint8_t *buf, uint16_t len) {
    assert(len <= 64);

    printf("Start transfer EP addr 0x%02x of %d byte%s\n",
            ep->descriptor->bEndpointAddress, len, len == 1 ? "" : "s");

    // Prepare buffer control register value
    uint32_t val = len | USB_BUF_CTRL_AVAIL;

    if (ep->descriptor->bEndpointAddress & USB_DIR_IN) { // copy from user memory to usb memory
        memcpy((void *) ep->data_buffer, (void *) buf, len);
        val |= USB_BUF_CTRL_FULL; // mark buffer as full
    }

    val |= ep->next_datapid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_datapid ^= 1; // flip for next xfer

    *ep->buffer_control = val;
}

void ep0_out_handler(uint8_t *buf, uint16_t len) {
    ; // nothing to do
}

// EP0 IN transfer complete
// Finish the SET_ADDRESS or receive a zero length status packet (ZLSP) from host
void ep0_in_handler(uint8_t *buf, uint16_t len) {
    if (should_set_address) {
        usb_hw->dev_addr_ctrl = dev_addr; // Set actual device address in hardware
        should_set_address = false;
    } else { // Receive a ZLSP from the host on EP0 OUT
        struct usb_endpoint *ep = usb_get_endpoint_configuration(EP0_OUT_ADDR);
        usb_start_transfer(ep, NULL, 0);
    }
}

void ep1_out_handler(uint8_t *buf, uint16_t len) {
    printf("RX %d bytes from host\n", len);
    struct usb_endpoint *ep = usb_get_endpoint_configuration(EP2_IN_ADDR); // Send data back to host
    usb_start_transfer(ep, buf, len);
}

void ep2_in_handler(uint8_t *buf, uint16_t len) {
    printf("Sent %d bytes to host\n", len);
    usb_start_transfer(usb_get_endpoint_configuration(EP1_OUT_ADDR), NULL, 64); // Get ready to rx again from host
}

// ==[ Descriptors ]===========================================================

// send device descriptor to host
void usb_handle_device_descriptor(volatile struct usb_setup_packet *pkt) {
    const struct usb_device_descriptor *dd = device.device_descriptor;
    struct usb_endpoint *ep = usb_get_endpoint_configuration(EP0_IN_ADDR); // EP0 in
    ep->next_datapid = 1; // force datapid to 1
    usb_start_transfer(ep, (uint8_t *) dd, MIN(sizeof(struct usb_device_descriptor), pkt->wLength));
}

// send config descriptor to host
void usb_handle_config_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t *buf = &ep0_buf[0];

    // First request will want just the config descriptor
    const struct usb_configuration_descriptor *cd = device.config_descriptor;
    memcpy((void *) buf, cd, sizeof(struct usb_configuration_descriptor));
    buf += sizeof(struct usb_configuration_descriptor);

    // If we more than just the config descriptor copy it all
    if (pkt->wLength >= cd->wTotalLength) {
        memcpy((void *) buf, device.interface_descriptor, sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);
        const struct usb_endpoint *ep = device.endpoints;

        // Copy all the endpoint descriptors starting from EP1
        for (uint i = 2; i < USB_NUM_ENDPOINTS; i++) {
            if (ep[i].descriptor) {
                memcpy((void *) buf, ep[i].descriptor, sizeof(struct usb_endpoint_descriptor));
                buf += sizeof(struct usb_endpoint_descriptor);
            }
        }
    }

    // Send data: get len by working out end of buffer subtract start of buffer
    uint32_t len = (uint32_t) buf - (uint32_t) &ep0_buf[0];
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], MIN(len, pkt->wLength));
}

// Helper to convert a C string to a unicode string descriptor
uint8_t usb_prepare_string_descriptor(const unsigned char *str) {
    // 2 for bLength + bDescriptorType + strlen * 2 because string is unicode. i.e. other byte will be 0
    uint8_t bLength = 2 + (strlen((const char *)str) * 2);
    static const uint8_t bDescriptorType = 0x03;

    volatile uint8_t *buf = &ep0_buf[0];
    *buf++ = bLength;
    *buf++ = bDescriptorType;

    uint8_t c;

    do {
        c = *str++;
        *buf++ = c;
        *buf++ = 0;
    } while (c != '\0');

    return bLength;
}

// send string descriptor to host
void usb_handle_string_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t i = pkt->wValue & 0xff;
    uint8_t len = 0;

    if (i == 0) {
        len = 4;
        memcpy(&ep0_buf[0], device.lang_descriptor, len);
    } else {
        // Prepare fills in ep0_buf
        len = usb_prepare_string_descriptor(device.descriptor_strings[i - 1]);
    }

    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], MIN(len, pkt->wLength));
}

// ==[ Commands ]==============================================================

// send a ZLSP to host
void usb_acknowledge_out_request(void) {
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
}

// handle SET_ADDR request from the host
// Actually set in ep0_in_handler since we must acknowledge the request first as a device with address zero
void usb_set_device_address(volatile struct usb_setup_packet *pkt) {
    dev_addr = (pkt->wValue & 0xff); // set address is goofy because we have to send a ZLSP first with address 0
    printf("Set address to %d\n", dev_addr);
    should_set_address = true; // will set address in the callback phase
    usb_acknowledge_out_request();
}

// handle SET_CONFIGURATION request from the host
void usb_set_device_configuration(volatile struct usb_setup_packet *pkt) {
    printf("Device Enumerated!\n"); // Only one configuration so just acknowledge the request
    usb_acknowledge_out_request();
    configured = true;
}

// respond to a setup packet from the host
void usb_handle_setup_packet(void) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *) &usb_dpram->setup_packet;
    uint8_t req_direction = pkt->bmRequestType;
    uint8_t req = pkt->bRequest;
    usb_get_endpoint_configuration(EP0_IN_ADDR)->next_datapid = 1; // reset to DATA1 for EP0 IN

    if (req_direction == USB_DIR_OUT) {
        if (req == USB_REQUEST_SET_ADDRESS) {
            usb_set_device_address(pkt);
        } else if (req == USB_REQUEST_SET_CONFIGURATION) {
            usb_set_device_configuration(pkt);
        } else {
            usb_acknowledge_out_request();
            printf("Other OUT request (0x%02x)\n", pkt->bRequest);
        }
    } else if (req_direction == USB_DIR_IN) {
        if (req == USB_REQUEST_GET_DESCRIPTOR) {
            uint16_t descriptor_type = pkt->wValue >> 8;

            switch (descriptor_type) {
                case USB_DT_DEVICE:
                    usb_handle_device_descriptor(pkt);
                    printf("GET DEVICE DESCRIPTOR\n");
                    break;
                case USB_DT_CONFIG:
                    usb_handle_config_descriptor(pkt);
                    printf("GET CONFIG DESCRIPTOR\n");
                    break;
                case USB_DT_STRING:
                    usb_handle_string_descriptor(pkt);
                    printf("GET STRING DESCRIPTOR\n");
                    break;
                default:
                    printf("Unhandled GET_DESCRIPTOR type 0x%04x\n", descriptor_type);
            }
        } else {
            printf("Other IN request (0x%02x)\n", pkt->bRequest);
        }
    }
}

// ==[ Buffers ]===============================================================

// notify an endpoint that a transfer has completed
static void usb_handle_ep_buff_done(struct usb_endpoint *ep) {
    uint32_t buffer_control = *ep->buffer_control;
    uint16_t len = buffer_control & USB_BUF_CTRL_LEN_MASK; // get the ep's transfer length
    ep->handler((uint8_t *) ep->data_buffer, len); // call ep's buffer done handler
}

// find ep configuration for an ep_num and directio and notify of transfer completion
static void usb_handle_buff_done(uint ep_num, bool in) {
    uint8_t ep_addr = ep_num | (in ? USB_DIR_IN : 0);
    printf("EP %d (%s) done\n", ep_num, in ? "IN" : "OUT");
    for (uint i = 0; i < USB_NUM_ENDPOINTS; i++) {
        struct usb_endpoint *ep = &device.endpoints[i];
        if (ep->descriptor && ep->handler) {
            if (ep->descriptor->bEndpointAddress == ep_addr) {
                usb_handle_ep_buff_done(ep);
                return;
            }
        }
    }
}

// handle a "buffer status" irq, meaning one or more buffers have been sent/received
// notify each endpoint where this is the case
static void usb_handle_buff_status() {
    uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;

    uint bit = 1u;
    for (uint i = 0; remaining_buffers && i < USB_NUM_ENDPOINTS * 2; i++) {
        if (remaining_buffers & bit) {
            usb_hw_clear->buf_status = bit; // clear this in advance
            usb_handle_buff_done(i >> 1u, !(i & 1u)); // IN transfer for even i, OUT transfer for odd i
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

// ==[ Interrupt ]=============================================================

// bus reset from the host by setting the device address back to 0
void usb_bus_reset(void) {
    dev_addr = 0; // set address back to 0
    should_set_address = false;
    usb_hw->dev_addr_ctrl = 0;
    configured = false;
}

// usb interrupt handler
void isr_usbctrl(void) {
    uint32_t status = usb_hw->ints;
    uint32_t handled = 0;

    // Setup packet received
    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        usb_handle_setup_packet();
    }

    // Buffer status, one or more buffers have completed
    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        usb_handle_buff_status();
    }

    // Bus is reset
    if (status & USB_INTS_BUS_RESET_BITS) {
        printf("BUS RESET\n");
        handled |= USB_INTS_BUS_RESET_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        usb_bus_reset();
    }

    if (status ^ handled) {
        panic("Unhandled IRQ 0x%04x\n", (uint) (status ^ handled));
    }
}

// ==[ Hardware reset ]========================================================

// reset USB device
void usb_device_init() {

    // Reset controller
    reset_block       (RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    // Clear state
    memset(usb_dpram, 0, sizeof(*usb_dpram));
    irq_set_enabled(USBCTRL_IRQ, true);

    // Setup device mode
    usb_hw->muxing    = USB_USB_MUXING_TO_PHY_BITS              |
                        USB_USB_MUXING_SOFTCON_BITS             ;
    usb_hw->pwr       = USB_USB_PWR_VBUS_DETECT_BITS            |
                        USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS        ;
    usb_hw->sie_ctrl  = USB_SIE_CTRL_EP0_INT_1BUF_BITS          ;
    usb_hw->inte      = USB_INTE_BUFF_STATUS_BITS               |
                        USB_INTE_BUS_RESET_BITS                 |
                        USB_INTE_SETUP_REQ_BITS                 ;

    usb_setup_endpoints();
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

// ==[ Main ]==================================================================

int main(void) {
    stdio_init_all();
    printf("USB Device example\n");
    usb_device_init();

    // Wait until configured
    while (!configured) { tight_loop_contents(); }

    // Get ready to rx from host
    usb_start_transfer(usb_get_endpoint_configuration(EP1_OUT_ADDR), NULL, 64);

    // Everything is interrupt driven so just loop here
    while (1) { tight_loop_contents(); }
}
