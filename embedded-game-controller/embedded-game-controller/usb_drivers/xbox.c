#include "driver_api.h"
#include "utils.h"

/*
 * Wired Xbox controllers over USB:
 *  - Xbox 360 (XInput protocol)
 *  - Xbox One / Series X|S (GIP protocol)
 *
 * They are vendor-class USB devices (not HID), so IOS serves them through
 * /dev/usb/ven instead of /dev/usb/hid.
 */

#define XBOX_VID           0x045e
#define XBOX_PID_360       0x028e
#define XBOX_IN_BUF_SIZE   64
#define XBOX_MAX_ERRORS    8
#define XBOX_360_MIN_LEN   14
#define XBOX_ONE_MIN_LEN   18

enum xbox_type_e {
    XBOX_TYPE_360,
    XBOX_TYPE_ONE,
};

enum xb_buttons_e {
    XB_BUTTON_UP,
    XB_BUTTON_DOWN,
    XB_BUTTON_LEFT,
    XB_BUTTON_RIGHT,
    XB_BUTTON_START,
    XB_BUTTON_BACK,
    XB_BUTTON_L3,
    XB_BUTTON_R3,
    XB_BUTTON_LB,
    XB_BUTTON_RB,
    XB_BUTTON_GUIDE,
    XB_BUTTON_A,
    XB_BUTTON_B,
    XB_BUTTON_X,
    XB_BUTTON_Y,
    XB_BUTTON_LT,
    XB_BUTTON_RT,
    XB_BUTTON_COUNT
};

struct xbox_private_data_t {
    u8 type;
    u8 ep_in;
    u8 ep_out;
    u8 seq;
    u8 errors;
    u8 guide;
    u8 needs_s_init;
};
static_assert(sizeof(struct xbox_private_data_t) <= EGC_INPUT_DEVICE_PRIVATE_DATA_SIZE);

static const egc_gamepad_button_e s_button_map[XB_BUTTON_COUNT] = {
    [XB_BUTTON_UP] = EGC_GAMEPAD_BUTTON_DPAD_UP,
    [XB_BUTTON_DOWN] = EGC_GAMEPAD_BUTTON_DPAD_DOWN,
    [XB_BUTTON_LEFT] = EGC_GAMEPAD_BUTTON_DPAD_LEFT,
    [XB_BUTTON_RIGHT] = EGC_GAMEPAD_BUTTON_DPAD_RIGHT,
    [XB_BUTTON_START] = EGC_GAMEPAD_BUTTON_START,
    [XB_BUTTON_BACK] = EGC_GAMEPAD_BUTTON_BACK,
    [XB_BUTTON_L3] = EGC_GAMEPAD_BUTTON_LEFT_STICK,
    [XB_BUTTON_R3] = EGC_GAMEPAD_BUTTON_RIGHT_STICK,
    [XB_BUTTON_LB] = EGC_GAMEPAD_BUTTON_LEFT_SHOULDER,
    [XB_BUTTON_RB] = EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER,
    [XB_BUTTON_GUIDE] = EGC_GAMEPAD_BUTTON_GUIDE,
    [XB_BUTTON_A] = EGC_GAMEPAD_BUTTON_SOUTH,
    [XB_BUTTON_B] = EGC_GAMEPAD_BUTTON_EAST,
    [XB_BUTTON_X] = EGC_GAMEPAD_BUTTON_WEST,
    [XB_BUTTON_Y] = EGC_GAMEPAD_BUTTON_NORTH,
    [XB_BUTTON_LT] = EGC_GAMEPAD_BUTTON_LEFT_PADDLE1,
    [XB_BUTTON_RT] = EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1,
};

static const egc_device_description_t s_device_description = {
    .vendor_id = XBOX_VID,
    .product_id = XBOX_PID_360,
    /* clang-format off */
    .available_buttons =
        BIT(EGC_GAMEPAD_BUTTON_DPAD_UP) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_DOWN) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_LEFT) |
        BIT(EGC_GAMEPAD_BUTTON_DPAD_RIGHT) |
        BIT(EGC_GAMEPAD_BUTTON_NORTH) |
        BIT(EGC_GAMEPAD_BUTTON_EAST) |
        BIT(EGC_GAMEPAD_BUTTON_SOUTH) |
        BIT(EGC_GAMEPAD_BUTTON_WEST) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_SHOULDER) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_SHOULDER) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_PADDLE1) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_PADDLE1) |
        BIT(EGC_GAMEPAD_BUTTON_BACK) |
        BIT(EGC_GAMEPAD_BUTTON_GUIDE) |
        BIT(EGC_GAMEPAD_BUTTON_START) |
        BIT(EGC_GAMEPAD_BUTTON_LEFT_STICK) |
        BIT(EGC_GAMEPAD_BUTTON_RIGHT_STICK),
    .available_axes =
        BIT(EGC_GAMEPAD_AXIS_LEFTX) |
        BIT(EGC_GAMEPAD_AXIS_LEFTY) |
        BIT(EGC_GAMEPAD_AXIS_RIGHTX) |
        BIT(EGC_GAMEPAD_AXIS_RIGHTY) |
        BIT(EGC_GAMEPAD_AXIS_LEFT_TRIGGER) |
        BIT(EGC_GAMEPAD_AXIS_RIGHT_TRIGGER),
    /* clang-format on */
    .type = EGC_DEVICE_TYPE_GAMEPAD,
    .num_touch_points = 0,
    .num_leds = 0,
    .num_accelerometers = 0,
    .has_rumble = false,
};

/* Zero-filled buffer used as the (ignored) payload of IN transfers */
static const u8 s_in_dummy[XBOX_IN_BUF_SIZE];

#define XB_SET(cond, btn)                                                                          \
    do {                                                                                           \
        if (cond)                                                                                  \
            buttons |= 1u << (btn);                                                                \
    } while (0)

/* The USB data is little endian, while Starlet runs big endian */
static inline s16 le16s(const u8 *p)
{
    return (s16)(u16)(p[0] | (p[1] << 8));
}

static inline u16 le16u(const u8 *p)
{
    return (u16)(p[0] | (p[1] << 8));
}

static void xbox_request_data(egc_input_device_t *device);

/* Xbox 360: 20 byte report, 0x00 0x14 header */
static bool parse_360(const u8 *d, u32 len, egc_input_state_t *state)
{
    u32 buttons = 0;

    if (len < XBOX_360_MIN_LEN || d[0] != 0x00)
        return false;

    XB_SET(d[2] & 0x01, XB_BUTTON_UP);
    XB_SET(d[2] & 0x02, XB_BUTTON_DOWN);
    XB_SET(d[2] & 0x04, XB_BUTTON_LEFT);
    XB_SET(d[2] & 0x08, XB_BUTTON_RIGHT);
    XB_SET(d[2] & 0x10, XB_BUTTON_START);
    XB_SET(d[2] & 0x20, XB_BUTTON_BACK);
    XB_SET(d[2] & 0x40, XB_BUTTON_L3);
    XB_SET(d[2] & 0x80, XB_BUTTON_R3);
    XB_SET(d[3] & 0x01, XB_BUTTON_LB);
    XB_SET(d[3] & 0x02, XB_BUTTON_RB);
    XB_SET(d[3] & 0x04, XB_BUTTON_GUIDE);
    XB_SET(d[3] & 0x10, XB_BUTTON_A);
    XB_SET(d[3] & 0x20, XB_BUTTON_B);
    XB_SET(d[3] & 0x40, XB_BUTTON_X);
    XB_SET(d[3] & 0x80, XB_BUTTON_Y);
    XB_SET(d[4] > 0x40, XB_BUTTON_LT);
    XB_SET(d[5] > 0x40, XB_BUTTON_RT);

    state->gamepad.buttons = egc_device_driver_map_buttons(buttons, XB_BUTTON_COUNT, s_button_map);
    /* Xbox stick Y axes already have "up" as positive, same as EGC */
    state->gamepad.axes[EGC_GAMEPAD_AXIS_LEFTX] = le16s(d + 6);
    state->gamepad.axes[EGC_GAMEPAD_AXIS_LEFTY] = le16s(d + 8);
    state->gamepad.axes[EGC_GAMEPAD_AXIS_RIGHTX] = le16s(d + 10);
    state->gamepad.axes[EGC_GAMEPAD_AXIS_RIGHTY] = le16s(d + 12);
    state->gamepad.axes[EGC_GAMEPAD_AXIS_LEFT_TRIGGER] = (d[4] << 7) | (d[4] >> 1);
    state->gamepad.axes[EGC_GAMEPAD_AXIS_RIGHT_TRIGGER] = (d[5] << 7) | (d[5] >> 1);
    return true;
}

/* Xbox One / Series (GIP): command in d[0]. 0x20 = input, 0x07 = Xbox button */
static bool parse_one(const u8 *d, u32 len, egc_input_state_t *state,
                      struct xbox_private_data_t *priv)
{
    u32 buttons = 0;

    if (len >= 5 && d[0] == 0x07) {
        priv->guide = d[4] & 0x01;
        return false;
    }

    if (len < XBOX_ONE_MIN_LEN || d[0] != 0x20)
        return false;

    XB_SET(d[4] & 0x04, XB_BUTTON_START);
    XB_SET(d[4] & 0x08, XB_BUTTON_BACK);
    XB_SET(d[4] & 0x10, XB_BUTTON_A);
    XB_SET(d[4] & 0x20, XB_BUTTON_B);
    XB_SET(d[4] & 0x40, XB_BUTTON_X);
    XB_SET(d[4] & 0x80, XB_BUTTON_Y);
    XB_SET(d[5] & 0x01, XB_BUTTON_UP);
    XB_SET(d[5] & 0x02, XB_BUTTON_DOWN);
    XB_SET(d[5] & 0x04, XB_BUTTON_LEFT);
    XB_SET(d[5] & 0x08, XB_BUTTON_RIGHT);
    XB_SET(d[5] & 0x10, XB_BUTTON_LB);
    XB_SET(d[5] & 0x20, XB_BUTTON_RB);
    XB_SET(d[5] & 0x40, XB_BUTTON_L3);
    XB_SET(d[5] & 0x80, XB_BUTTON_R3);
    XB_SET(priv->guide, XB_BUTTON_GUIDE);

    u16 lt = le16u(d + 6) & 0x3ff; /* 10 bits */
    u16 rt = le16u(d + 8) & 0x3ff;
    XB_SET(lt > 0x100, XB_BUTTON_LT);
    XB_SET(rt > 0x100, XB_BUTTON_RT);

    state->gamepad.buttons = egc_device_driver_map_buttons(buttons, XB_BUTTON_COUNT, s_button_map);
    state->gamepad.axes[EGC_GAMEPAD_AXIS_LEFTX] = le16s(d + 10);
    state->gamepad.axes[EGC_GAMEPAD_AXIS_LEFTY] = le16s(d + 12);
    state->gamepad.axes[EGC_GAMEPAD_AXIS_RIGHTX] = le16s(d + 14);
    state->gamepad.axes[EGC_GAMEPAD_AXIS_RIGHTY] = le16s(d + 16);
    state->gamepad.axes[EGC_GAMEPAD_AXIS_LEFT_TRIGGER] = (lt << 5) | (lt >> 5);
    state->gamepad.axes[EGC_GAMEPAD_AXIS_RIGHT_TRIGGER] = (rt << 5) | (rt >> 5);
    return true;
}

static void intr_in_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    struct xbox_private_data_t *priv = (void *)device->private_data;

    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED) {
        egc_input_state_t state;
        bool valid;

        priv->errors = 0;
        memset(&state, 0, sizeof(state));
        if (priv->type == XBOX_TYPE_360)
            valid = parse_360(transfer->data, transfer->length, &state);
        else
            valid = parse_one(transfer->data, transfer->length, &state, priv);
        if (valid)
            egc_device_driver_report_input(device, &state);
    } else if (++priv->errors >= XBOX_MAX_ERRORS) {
        /* Give up instead of spinning on a broken endpoint */
        return;
    }

    xbox_request_data(device);
}

static void xbox_request_data(egc_input_device_t *device)
{
    struct xbox_private_data_t *priv = (void *)device->private_data;

    egc_device_driver_issue_intr_transfer_async(device, priv->ep_in, (void *)s_in_dummy,
                                                XBOX_IN_BUF_SIZE, intr_in_cb);
}

/* Xbox One: the controller only starts sending input after a "power on" packet */
static void send_one_init(egc_input_device_t *device, bool s_variant);

static void init_out_cb(egc_usb_transfer_t *transfer)
{
    egc_input_device_t *device = transfer->device;
    struct xbox_private_data_t *priv = (void *)device->private_data;

    if (transfer->status == EGC_USB_TRANSFER_STATUS_COMPLETED && priv->needs_s_init) {
        priv->needs_s_init = 0;
        send_one_init(device, true);
    }
}

static void send_one_init(egc_input_device_t *device, bool s_variant)
{
    struct xbox_private_data_t *priv = (void *)device->private_data;
    u8 pkt[5] = { 0x05, 0x20, 0x00, 0x01, 0x00 };

    if (s_variant) {
        pkt[3] = 0x0f;
        pkt[4] = 0x06;
    }
    pkt[2] = ++priv->seq;
    egc_device_driver_issue_intr_transfer_async(device, priv->ep_out, pkt, sizeof(pkt), init_out_cb);
}

static bool xbox_driver_ops_probe(u16 vid, u16 pid)
{
    static const egc_device_id_t compatible[] = {
        { XBOX_VID, XBOX_PID_360 }, /* Xbox 360 wired controller */
        { XBOX_VID, 0x02d1 },       /* Xbox One controller */
        { XBOX_VID, 0x02dd },       /* Xbox One controller (2015 firmware) */
        { XBOX_VID, 0x02e3 },       /* Xbox One Elite controller */
        { XBOX_VID, 0x02ea },       /* Xbox One S controller */
        { XBOX_VID, 0x0b00 },       /* Xbox One Elite Series 2 controller */
        { XBOX_VID, 0x0b12 },       /* Xbox Series X|S controller */
    };

    return egc_device_driver_is_compatible(vid, pid, compatible, ARRAY_SIZE(compatible));
}

static int xbox_driver_ops_init(egc_input_device_t *device, u16 vid, u16 pid)
{
    struct xbox_private_data_t *priv = (void *)device->private_data;
    u8 ep_in = 0, ep_out = 0;

    /* The endpoints depend on the model, so ask the backend */
    if (egc_device_driver_get_usb_endpoints(device, &ep_in, &ep_out) < 0 || ep_in == 0)
        return -1;

    memset(priv, 0, sizeof(*priv));
    priv->type = pid == XBOX_PID_360 ? XBOX_TYPE_360 : XBOX_TYPE_ONE;
    priv->ep_in = ep_in;
    priv->ep_out = ep_out;
    priv->needs_s_init = pid == 0x02ea || pid == 0x0b00;

    if (priv->type == XBOX_TYPE_ONE && ep_out == 0)
        return -1;

    device->desc = &s_device_description;

    /* Give the device half a second to settle before talking to it */
    egc_device_driver_set_timer(device, 1000 * 500, 0);
    return 0;
}

static bool xbox_driver_ops_timer(egc_input_device_t *device)
{
    struct xbox_private_data_t *priv = (void *)device->private_data;

    if (priv->type == XBOX_TYPE_ONE)
        send_one_init(device, false);
    xbox_request_data(device);
    /* Return false to destroy the timer */
    return false;
}

const egc_device_driver_t xbox_usb_device_driver = {
    .probe = xbox_driver_ops_probe,
    .init = xbox_driver_ops_init,
    .timer = xbox_driver_ops_timer,
};
