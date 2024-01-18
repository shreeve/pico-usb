// ==[ USB 2.0 ]===============================================================

#include "usb_common.h"

#define EP0_OUT_ADDR (USB_DIR_OUT | 0)
#define EP0_IN_ADDR  (USB_DIR_IN  | 0)
#define EP1_OUT_ADDR (USB_DIR_OUT | 1)
#define EP2_IN_ADDR  (USB_DIR_IN  | 2)

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
    .bInterfaceNumber   = 0,    // Starts at zero
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 2,    // Two endpoints (EP0 doesn't count)
    .bInterfaceClass    = 0xff, // Interface class (0xff = Vendor specific)
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface         = 5     // String #5
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
    .iConfiguration      = 4,    // String #4
    .bmAttributes        = 0xc0, // Attributes: Self-powered, No remote wakeup
    .bMaxPower           = 50    // 100ma (Expressed in 2mA units)
};

static const struct usb_device_descriptor device_descriptor = {
    .bLength            = sizeof(struct usb_device_descriptor),
    .bDescriptorType    = USB_DT_DEVICE,
    .bcdUSB             = 0x0110, // USB 1.1 device
    .bDeviceClass       = 0,      // Defer to interface descriptor
    .bDeviceSubClass    = 0,      // No subclass
    .bDeviceProtocol    = 0,      // No protocol
    .bMaxPacketSize0    = 64,     // Max packet size for EP0
    .idVendor           = 0x0000, // Vendor id
    .idProduct          = 0x0001, // Product id
    .bcdDevice          = 0x0001, // Device release number (xx.yy)
    .iManufacturer      = 1,      // String #1
    .iProduct           = 2,      // String #2
    .iSerialNumber      = 3,      // String #3
    .bNumConfigurations = 1       // One configuration
};

// ==[ PicoUSB ]===============================================================

#include <stdio.h> // For printf
#include <string.h> // For memcpy

#include "pico/stdlib.h" // Pico stdlib
#include "hardware/regs/usb.h" // USB registers from pico-sdk
#include "hardware/structs/usb.h" // USB hardware structs from pico-sdk
#include "hardware/irq.h" // For interrupt enable and numbers
#include "hardware/resets.h" // For resetting the USB controller

#define usb_hw_set   ((usb_hw_t *)hw_set_alias_untyped  (usb_hw))
#define usb_hw_clear ((usb_hw_t *)hw_clear_alias_untyped(usb_hw))

void ep0_out_handler(uint8_t *buf, uint16_t len);
void ep0_in_handler (uint8_t *buf, uint16_t len);
void ep1_out_handler(uint8_t *buf, uint16_t len);
void ep2_in_handler (uint8_t *buf, uint16_t len);

typedef void (*usb_ep_handler)(uint8_t *buf, uint16_t len);

struct usb_endpoint {
    const struct usb_endpoint_descriptor *descriptor;
    usb_ep_handler handler;

    volatile uint32_t *endpoint_control;
    volatile uint32_t *buffer_control;
    volatile uint8_t  *data_buffer;

    uint8_t next_datapid; // Toggle DATA0/DATA1 each packet
};

struct usb_device {
    const struct usb_device_descriptor        *device_descriptor;
    const struct usb_configuration_descriptor *config_descriptor;
    const struct usb_interface_descriptor     *interface_descriptor;
    const unsigned char                       *lang_descriptor;
    const unsigned char                       **descriptor_strings;
    struct usb_endpoint                       endpoints[USB_NUM_ENDPOINTS];
};

static const unsigned char lang_descriptor[] = {
    4,         // bLength
    0x03,      // bDescriptorType == String descriptor
    0x09, 0x04 // Language id = US English
};

static const unsigned char *descriptor_strings[] = {
    (unsigned char *) "PicoUSB", // String #1: Vendor
    (unsigned char *) "Demo"   , // String #2: Product
    (unsigned char *) "12345"  , // String #3: Serial
    (unsigned char *) "Simple" , // String #4: Configuration
    (unsigned char *) "Basic"    // String #5: Interface
};

static struct usb_device device = {
    .device_descriptor    = &device_descriptor,
    .config_descriptor    = &config_descriptor,
    .interface_descriptor = &interface_descriptor,
    .lang_descriptor      = lang_descriptor,
    .descriptor_strings   = descriptor_strings,
    .endpoints = {
        {
            .descriptor       = &ep0_out,
            .handler          = &ep0_out_handler,
            .endpoint_control = NULL, // Controls for EP0 come from SIE_CTRL
            .buffer_control   = &usb_dpram->ep_buf_ctrl[0].out,
            .data_buffer      = &usb_dpram->ep0_buf_a[0], // EP0 in/out share
        },
        {
            .descriptor       = &ep0_in,
            .handler          = &ep0_in_handler,
            .endpoint_control = NULL, // Controls for EP0 come from SIE_CTRL
            .buffer_control   = &usb_dpram->ep_buf_ctrl[0].in,
            .data_buffer      = &usb_dpram->ep0_buf_a[0], // EP0 in/out share
        },
        {
            .descriptor       = &ep1_out,
            .handler          = &ep1_out_handler,
            .endpoint_control = &usb_dpram->ep_ctrl[0].out, // EP1 uses index 0
            .buffer_control   = &usb_dpram->ep_buf_ctrl[1].out,
            .data_buffer      = &usb_dpram->epx_data[0 * 64], // First buffer
        },
        {
            .descriptor       = &ep2_in,
            .handler          = &ep2_in_handler,
            .endpoint_control = &usb_dpram->ep_ctrl[1].in, // EP2 uses index 1
            .buffer_control   = &usb_dpram->ep_buf_ctrl[2].in,
            .data_buffer      = &usb_dpram->epx_data[1 * 64], // Second buffer
        }
    }
};

static uint8_t ep0_buf[64];
static uint8_t device_address = 0;
static bool should_set_address = false;
static volatile bool configured = false;

// ==[ Endpoints ]=============================================================

// Set up an endpoint's control register (except EP0)
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
        *ep->endpoint_control = EP_CTRL_ENABLE_BITS          | // Enable EP
                                EP_CTRL_INTERRUPT_PER_BUFFER | // One IRQ per
                                type   | // Control, iso, bulk, or interrupt
                                offset ; // Address base offset in DSPRAM
    }
}

// Set up all endpoints
void usb_setup_endpoints() {
    const struct usb_endpoint *endpoints = device.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && endpoints[i].handler) {
            usb_setup_endpoint(&endpoints[i]);
        }
    }
}

// Get the endpoint for an endpoint address (EP number + direction)
struct usb_endpoint *usb_get_endpoint(uint8_t addr) {
    struct usb_endpoint *endpoints = device.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor &&
           (endpoints[i].descriptor->bEndpointAddress == addr)) {
            return &endpoints[i];
        }
    }
    return NULL;
}

// ==[ Transfers ]=============================================================

void hexdump(const void* data, size_t size) {
    const unsigned char* byte = (const unsigned char*) data;
    size_t i, j;

    for (i = 0; i < size; i += 16) {
        printf("  %08zx  ", i); // Print the offset

        // Print hex values
        for (j = 0; j < 16; j++) {
            if (i + j < size) {
                printf("%02x ", byte[i + j]);
            } else {
                printf("   "); // Pad if less than 16 bytes in the line
            }
        }

        printf("  ");

        // Print ASCII values
        for (j = 0; j < 16; j++) {
            if (i + j < size) {
                unsigned char ch = byte[i + j];
                printf("%c", (ch >= 32 && ch <= 126) ? ch : '.');
            }
        }

        printf("\n");
    }
}

// Start a transfer on an endpoint
void usb_start_transfer(struct usb_endpoint *ep, uint8_t *buf, uint16_t len) {
    assert(len <= 64);

    // Grok the desired endpoint
    uint8_t ep_addr = ep->descriptor->bEndpointAddress;
    uint8_t ep_num = ep_addr & 0x0f;
    bool in = ep_addr & USB_DIR_IN;
    if (len == 0) {
        printf("  [Transfer on EP%d_%s (0x%02x) ZLP]\n",
                ep_num, in ? "IN " : "OUT", ep_addr);
    } else {
        printf("  [Transfer on EP%d_%s (0x%02x) %d byte%s]\n",
                ep_num, in ? "IN " : "OUT", ep_addr, len, len == 1 ? "" : "s");
        hexdump((const void*) buf, (size_t) len);
    }

    // Set the buffer control register and copy the buffer if needed
    uint32_t val = len | USB_BUF_CTRL_AVAIL;
    if (in) {
        memcpy((void *) ep->data_buffer, (void *) buf, len);
        val |= USB_BUF_CTRL_FULL; // Mark buffer as full
    }
    val |= ep->next_datapid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_datapid ^= 1; // Flip for the next transfer
    *ep->buffer_control = val;
}

// Send a ZLP (zero length packet) to host
void usb_send_zlp() {
    usb_start_transfer(usb_get_endpoint(EP0_IN_ADDR), NULL, 0);
}

// ==[ Handlers ]==============================================================

void ep0_out_handler(uint8_t *buf, uint16_t len) {
    ; // Nothing to do
}

void ep0_in_handler(uint8_t *buf, uint16_t len) {
    if (should_set_address) {
        usb_hw->dev_addr_ctrl = device_address; // Set hardware device address
        should_set_address = false;
    } else { // Receive a ZLP from host on EP0_OUT
        usb_start_transfer(usb_get_endpoint(EP0_OUT_ADDR), NULL, 0);
    }
}

void ep1_out_handler(uint8_t *buf, uint16_t len) {
    printf("Received %d bytes from host\n", len);
    usb_start_transfer(usb_get_endpoint(EP2_IN_ADDR), buf, len);
}

void ep2_in_handler(uint8_t *buf, uint16_t len) {
    printf("Sent %d bytes to host\n", len);
    usb_start_transfer(usb_get_endpoint(EP1_OUT_ADDR), NULL, 64);
}

// ==[ Setup ]=================================================================

// Send device descriptor to host
void usb_handle_device_descriptor(volatile struct usb_setup_packet *pkt) {
    const struct usb_device_descriptor *dd = device.device_descriptor;
    uint16_t len = MIN(sizeof(struct usb_device_descriptor), pkt->wLength);
    usb_get_endpoint(EP0_IN_ADDR)->next_datapid = 1; // Reset to DATA1
    usb_start_transfer(usb_get_endpoint(EP0_IN_ADDR), (uint8_t *) dd, len);
}

// Send config descriptor to host
void usb_handle_config_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t *buf = &ep0_buf[0];

    // Always include the config descriptor
    const struct usb_configuration_descriptor *cd = device.config_descriptor;
    memcpy((void *) buf, cd, sizeof(struct usb_configuration_descriptor));
    buf += sizeof(struct usb_configuration_descriptor);

    // If more than the config descriptor is requested, send everything
    if (pkt->wLength >= cd->wTotalLength) {
        memcpy((void *) buf, device.interface_descriptor,
               sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);

        // Copy all the endpoint descriptors starting from EP1
        const struct usb_endpoint *ep = device.endpoints;
        for (uint i = 2; i < USB_NUM_ENDPOINTS; i++) {
            if (ep[i].descriptor) {
                memcpy((void *) buf, ep[i].descriptor,
                       sizeof(struct usb_endpoint_descriptor));
                buf += sizeof(struct usb_endpoint_descriptor);
            }
        }
    }

    uint32_t len = (uint32_t) buf - (uint32_t) &ep0_buf[0];
    len = MIN(len, pkt->wLength);
    usb_start_transfer(usb_get_endpoint(EP0_IN_ADDR), &ep0_buf[0], len);
}

// Helper to convert a C string to a unicode string descriptor (2 + strlen * 2)
uint8_t usb_prepare_string_descriptor(const unsigned char *str) {
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

// Send string descriptor to host
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

    usb_start_transfer(usb_get_endpoint(EP0_IN_ADDR), &ep0_buf[0], MIN(len, pkt->wLength));
}

// Handle SET_ADDR request from host
void usb_set_device_address(volatile struct usb_setup_packet *pkt) {
    // This is a little weird because we must first acknowledge the request
    // using a device address of zero. We do that here and then set a flag
    // to perform the actual update in the ep0_in_handler.
    device_address = (pkt->wValue & 0xff);
    should_set_address = true; // Will set address in the callback phase
}

// Handle SET_CONFIGURATION request from host
void usb_set_device_configuration(volatile struct usb_setup_packet *pkt) {
    configured = true;
}

// Respond to a setup packet from host
void usb_handle_setup_packet() {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *) &usb_dpram->setup_packet;
    uint8_t req_direction = pkt->bmRequestType;
    uint8_t req = pkt->bRequest;

    // Log the setup packet
    hexdump((const void*) pkt, sizeof(struct usb_setup_packet));

    usb_get_endpoint(EP0_IN_ADDR)->next_datapid = 1; // Reset to DATA1

    if (req_direction == USB_DIR_OUT) {
        if (req == USB_REQUEST_SET_ADDRESS) {
            printf("SET ADDRESS to %d\n", (pkt->wValue & 0xff));
            usb_set_device_address(pkt);
        } else if (req == USB_REQUEST_SET_CONFIGURATION) {
            printf("SET CONFIGURATION to %d\n", (pkt->wValue & 0xff));
            usb_set_device_configuration(pkt);
        } else {
            printf("Other OUT request\n");
        }
        usb_send_zlp(); // TODO: Confirm how we should handle
    } else if (req_direction == USB_DIR_IN) {
        if (req == USB_REQUEST_GET_DESCRIPTOR) {
            uint16_t descriptor_type = pkt->wValue >> 8;

            switch (descriptor_type) {
                case USB_DT_DEVICE:
                    printf("GET DEVICE DESCRIPTOR\n");
                    usb_handle_device_descriptor(pkt);
                    break;
                case USB_DT_CONFIG:
                    printf("GET CONFIG DESCRIPTOR\n");
                    usb_handle_config_descriptor(pkt);
                    break;
                case USB_DT_STRING:
                    printf("GET STRING DESCRIPTOR\n");
                    usb_handle_string_descriptor(pkt);
                    break;
                default:
                    printf("Unhandled GET_DESCRIPTOR\n");
            }
        } else {
            printf("Unhandled IN request\n");
            usb_send_zlp(); // TODO: Confirm how we should handle
        }
    }
}

// ==[ Buffers ]===============================================================

// Notify an endpoint that a transfer has completed
static void usb_handle_ep_buff_done(struct usb_endpoint *ep) {
    uint32_t buffer_control = *ep->buffer_control;
    uint16_t len = buffer_control & USB_BUF_CTRL_LEN_MASK; // Get buffer length

    ep->handler((uint8_t *) ep->data_buffer, len); // Call buffer done handler
}

// Notify an endpoint that a transfer has completed
static void usb_handle_buff_done(uint ep_num, bool in) {
    uint8_t ep_addr = ep_num | (in ? USB_DIR_IN : USB_DIR_OUT);

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

// Notify the given endpoints that a transfer has completed
static void usb_handle_buff_status() {
    uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;
    uint bit = 1u;

    for (uint i = 0; remaining_buffers && i < USB_NUM_ENDPOINTS * 2; i++) {
        if (remaining_buffers & bit) {
            usb_hw_clear->buf_status = bit; // Clear this in advance
            usb_handle_buff_done(i >> 1u, !(i & 1u)); // even=IN, odd=OUT
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

// ==[ Resets ]================================================================

// Reset USB bus
void usb_bus_reset() {
    device_address = 0; // Set address to zero
    usb_hw->dev_addr_ctrl = 0;
    should_set_address = false;
    configured = false;
}

// Reset USB device
void usb_device_reset() {

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
    usb_hw->inte      = USB_INTE_BUS_RESET_BITS                 |
                        USB_INTE_SETUP_REQ_BITS                 |
                        USB_INTE_BUFF_STATUS_BITS               ;

    usb_setup_endpoints();
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
    printf("USB device attached\n");
}

// ==[ Interrupt ]=============================================================

// Interrupt handler
void isr_usbctrl() {
    uint32_t status = usb_hw->ints;
    uint32_t handled = 0;

    // TODO: Does the order of processing these matter?

    // Bus is reset
    if (status & USB_INTS_BUS_RESET_BITS) {
        handled |= USB_INTS_BUS_RESET_BITS;
        printf("BUS RESET\n");
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        usb_bus_reset();
    }

    // Setup packet received
    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        printf("SETUP PACKET:\n");
        usb_handle_setup_packet();
    }

    // Buffer status, one or more buffers have completed
    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        usb_handle_buff_status();
    }

    if (status ^ handled) {
        panic("Unhandled IRQ 0x%04x\n", (uint) (status ^ handled));
    }
}

// ==[ Main ]==================================================================

int main() {
    stdio_init_all();
    printf("\n==[ USB device example]==\n\n");
    usb_device_reset();

    // Wait until configured
    while (!configured) { tight_loop_contents(); }

    // Get ready to rx from host
    usb_start_transfer(usb_get_endpoint(EP1_OUT_ADDR), NULL, 64);

    // Everything is interrupt driven so just loop here
    while (1) { tight_loop_contents(); }
}
