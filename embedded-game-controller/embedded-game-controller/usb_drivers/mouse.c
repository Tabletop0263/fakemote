#include "driver_api.h"
#include "utils.h"

/* Generic USB HID mouse -> pointer (touch point 0) + A/B/Home/+/-.
 * Assumes the common 3-5 byte layout: [buttons][dx s8][dy s8][wheel s8].
 * Claimed as a fallback for HID devices no other driver wants. */

#define MOUSE_ENDPOINT (EGC_USB_ENDPOINT_IN | 1)
#define MOUSE_SENS     40 /* touch units per mouse count; range is 0..0x7fff */

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

    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED && transfer->length >= 3) {
        struct egc_input_state_t state;
        u8 *p = (u8 *)&state;
        for (unsigned i = 0; i < sizeof(state); i++)
            p[i] = 0;

        s8 wheel = transfer->length >= 4 ? (s8)report[3] : 0;
        u32 buttons = report[0] & 0x1f;
        if (wheel > 0)
            buttons |= 1 << MOUSE_BTN_WHEEL_UP;
        else if (wheel < 0)
            buttons |= 1 << MOUSE_BTN_WHEEL_DOWN;

        priv->x = clamp_pos(priv->x + (s8)report[1] * MOUSE_SENS);
        priv->y = clamp_pos(priv->y + (s8)report[2] * MOUSE_SENS);

        state.gamepad.buttons = egc_device_driver_map_buttons(buttons, MOUSE_BTN_COUNT, s_button_map);
        state.gamepad.touch_points[0].x = priv->x;
        state.gamepad.touch_points[0].y = priv->y;
        state.gamepad.touch_points[1].x = -1;

        egc_device_driver_report_input(device, &state);
    }

    mouse_request_data(device);
}

static int mouse_request_data(egc_input_device_t *device)
{
    const egc_usb_transfer_t *transfer = egc_device_driver_issue_intr_transfer_async(
        device, MOUSE_ENDPOINT, NULL, 0, intr_transfer_cb);
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
