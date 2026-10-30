/*
 * usb_descriptors.c - USB CDC device: VID 0x2E8A / PID 0x000A (the Pico SDK
 * CDC pair, so Windows binds usbser without a driver), serial = flash ID.
 */
#include <string.h>
#include "tusb.h"
#include "pico/unique_id.h"

#define USB_VID 0x2E8A
#define USB_PID 0x000A

static tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL };

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

static char serial_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

static const char *string_desc_arr[] = {
    NULL,                           /* 0: language (0x0409) */
    "Jan IngenHousz Institute",     /* 1: manufacturer */
    "pa-detector",                  /* 2: product */
    serial_str,                     /* 3: serial */
    "pa-detector CDC",              /* 4: CDC interface */
};

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t desc[32];
    uint8_t n;
    if (index == 0) {
        desc[1] = 0x0409;
        n = 1;
    } else {
        if (index >= sizeof string_desc_arr / sizeof string_desc_arr[0]) return NULL;
        if (index == 3 && !serial_str[0]) pico_get_unique_board_id_string(serial_str, sizeof serial_str);
        const char *s = string_desc_arr[index];
        size_t len = strlen(s);
        if (len > 31) len = 31;
        for (size_t i = 0; i < len; i++) desc[1 + i] = (uint16_t)s[i];
        n = (uint8_t)len;
    }
    desc[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return desc;
}
