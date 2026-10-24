#include "driver_api.h"
#include "utils.h"

/* Generic USB HID mouse -> pointer (touch point 0) + A/B/Home/+/-.
 * Claimed as a fallback for HID devices no other driver wants.
 *
 * Two report formats are handled:
 *  - boot style (default): endpoint 0x81, [buttons][dx s8][dy s8][wheel s8]
 *  - 32c2:0066 "2.4G Wireless" dongle (e.g. heyday BEMS01): the mouse is
 *    interface 1, endpoint 0x82, reports are
 *    [0x02][buttons][x lo][x hi:4 | y lo:4][y hi][wheel][pan], X/Y 12-bit signed.
 *    The same endpoint also carries report IDs 0x01/0x03/0xB5, which we ignore. */

#define MOUSE_SENS 40 /* touch units per mouse count; range is 0..0x7fff */

enum mouse_format_e {
    MOUSE_FMT_BOOT,
    MOUSE_FMT_RID2_12BIT,
};

enum mouse_buttons_e {
    MOUSE_BTN_LEFT,
    MOUSE_BTN_RIGHT,
    MOUSE_BTN_MIDDLE,
    MOUSE_BTN_BACK,
    MOUSE_BTN_FORWARD,
    MOUSE_BTN_WHEEL_UP,
    MOUSE_BTN_WHEEL_DOWN,
    MOUSE_BTN_COUNT
};

struct mouse_private_data_t {
    s32 x;
    s32 y;
    u8 endpoint;
    u8 format;
};
static_assert(sizeof(struct mouse_private_data_t) <= EGC_INPUT_DEVICE_PRIVATE_DATA_SIZE);

static const egc_gamepad_button_e s_button_map[MOUSE_BTN_COUNT] = {
    [MOUSE_BTN_LEFT] = EGC_GAMEPAD_BUTTON_SOUTH,       /* Wiimote A */
    [MOUSE_BTN_RIGHT] = EGC_GAMEPAD_BUTTON_EAST,       /* Wiimote B */
    [MOUSE_BTN_MIDDLE] = EGC_GAMEPAD_BUTTON_LEFT_STICK, /* Home */
    [MOUSE_BTN_BACK] = EGC_GAMEPAD_BUTTON_NORTH,       /* 1 */
    [MOUSE_BTN_FORWARD] = EGC_GAMEPAD_BUTTON_WEST,     /* 2 */
    [MOUSE_BTN_WHEEL_UP] = EGC_GAMEPAD_BUTTON_START,   /* + */
    [MOUSE_BTN_WHEEL_DOWN] = EGC_GAMEPAD_BUTTON_BACK,  /* - */
};

static const egc_device_description_t s_device_description = {
    .vendor_id = 0,
    .product_id = 0,
    /* clang-format off */
    .available_buttons =
        BIT(EGC_GAMEPAD_BUTTON_SOUTH) |
        BIT(EGC_GAMEPAD_BUTTON_EAST) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_STICK) |
        BIT(EGC_GAMEPAD_BUTTON_NORTH) |
        BIT(EGC_GAMEPAD_BUTTON_WEST) |
        BIT(EGC_GAMEPAD_BUTTON_START) |
        BIT(EGC_GAMEPAD_BUTTON_BACK),
    .available_axes = 0,
    /* clang-format on */
    .type = EGC_DEVICE_TYPE_GAMEPAD,
    .num_touch_points = 1, /* enables fakemote's "direct" IR pointer mode */
    .num_leds = 0,
    .num_accelerometers = 0,
    .has_rumble = false,
};

static int mouse_request_data(egc_input_device_t *device);

static inline s32 clamp_pos(s32 v)
{
    if (v < 0)
        return 0;
    if (v > EGC_GAMEPAD_TOUCH_RES)
        return EGC_GAMEPAD_TOUCH_RES;
    return v;
}

static void intr_transfer_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    struct mouse_private_data_t *priv = (void *)device->private_data;
    const u8 *report = (const u8 *)transfer->data;

    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED) {
        s32 dx = 0, dy = 0;
        s8 wheel = 0;
        u32 buttons = 0;
        bool valid = false;

        if (priv->format == MOUSE_FMT_RID2_12BIT) {
            if (transfer->length >= 6 && report[0] == 0x02) {
                buttons = report[1] & 0x1f;
                dx = report[2] | ((report[3] & 0x0f) << 8);
                dy = (report[3] >> 4) | (report[4] << 4);
                if (dx & 0x800)
                    dx -= 0x1000;
                if (dy & 0x800)
                    dy -= 0x1000;
                wheel = (s8)report[5];
                valid = true;
            }
        } else if (transfer->length >= 3) {
            buttons = report[0] & 0x1f;
            dx = (s8)report[1];
            dy = (s8)report[2];
            wheel = transfer->length >= 4 ? (s8)report[3] : 0;
            valid = true;
        }

        if (valid) {
            struct egc_input_state_t state;
            u8 *p = (u8 *)&state;
            for (unsigned i = 0; i < sizeof(state); i++)
                p[i] = 0;

            if (wheel > 0)
                buttons |= 1 << MOUSE_BTN_WHEEL_UP;
            else if (wheel < 0)
                buttons |= 1 << MOUSE_BTN_WHEEL_DOWN;

            priv->x = clamp_pos(priv->x + dx * MOUSE_SENS);
            priv->y = clamp_pos(priv->y + dy * MOUSE_SENS);

            state.gamepad.buttons = egc_device_driver_map_buttons(buttons, MOUSE_BTN_COUNT, s_button_map);
            state.gamepad.touch_points[0].x = priv->x;
            state.gamepad.touch_points[0].y = priv->y;
            state.gamepad.touch_points[1].x = -1;

            egc_device_driver_report_input(device, &state);
        }
    }

    mouse_request_data(device);
}

static int mouse_request_data(egc_input_device_t *device)
{
    struct mouse_private_data_t *priv = (void *)device->private_data;
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_intr_transfer_async(
        device, priv->endpoint, NULL, 0, intr_transfer_cb);
    return transfer != NULL ? 0 : -1;
}

/* Never matched by VID/PID: used as a fallback, see egc.c */
static bool mouse_driver_ops_probe(u16 vid, u16 pid)
{
    return false;
}

static int mouse_driver_ops_init(egc_input_device_t *device, u16 vid, u16 pid)
{
    struct mouse_private_data_t *priv = (void *)device->private_data;
    priv->x = EGC_GAMEPAD_TOUCH_RES / 2;
    priv->y = EGC_GAMEPAD_TOUCH_RES / 2;
    if (vid == 0x32c2 && pid == 0x0066) {
        priv->endpoint = EGC_USB_ENDPOINT_IN | 2;
        priv->format = MOUSE_FMT_RID2_12BIT;
    } else {
        priv->endpoint = EGC_USB_ENDPOINT_IN | 1;
        priv->format = MOUSE_FMT_BOOT;
    }
    device->desc = &s_device_description;
    egc_device_driver_set_timer(device, 1000 * 500, 0);
    return 0;
}

static bool mouse_driver_ops_timer(egc_input_device_t *device)
{
    mouse_request_data(device);
    return false;
}

const egc_device_driver_t mouse_usb_device_driver = {
    .probe = mouse_driver_ops_probe,
    .init = mouse_driver_ops_init,
    .timer = mouse_driver_ops_timer,
};
