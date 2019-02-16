#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "hid-ids.h"

#define IS_L_OR_R_SET(B) (B[0] == 0x3F && (B[2] & 0x40)\
                         || B[0] == 0x30 && (B[3] & 0x40)\
                         || B[0] == 0x30 && (B[5] & 0x40))

#define IS_SL_SET(B) (B[0] == 0x3F && (B[1] & 0x10)\
                         || B[0] == 0x30 && (B[3] & 0x20)\
                         || B[0] == 0x30 && (B[5] & 0x20))

#define IS_SR_SET(B) (B[0] == 0x3F && (B[1] & 0x20)\
                         || B[0] == 0x30 && (B[3] & 0x10)\
                         || B[0] == 0x30 && (B[5] & 0x10))

static bool emulate_3button = true;
module_param(emulate_3button, bool, 0644);
MODULE_PARM_DESC(emulate_3button, "Emulate a middle button");

static int middle_button_start = -350;
static int middle_button_stop = +350;

static bool emulate_scroll_wheel = true;
module_param(emulate_scroll_wheel, bool, 0644);
MODULE_PARM_DESC(emulate_scroll_wheel, "Emulate a scroll wheel");

static unsigned int scroll_speed = 32;
static int param_set_scroll_speed(const char *val,
        const struct kernel_param *kp) {
    unsigned long speed;
    if (!val || kstrtoul(val, 0, &speed) || speed > 63)
        return -EINVAL;
    scroll_speed = speed;
    return 0;
}

static bool scroll_acceleration = false;
module_param(scroll_acceleration, bool, 0644);
MODULE_PARM_DESC(scroll_acceleration, "Accelerate sequential scroll events");

static bool report_undeciphered;
module_param(report_undeciphered, bool, 0644);
MODULE_PARM_DESC(report_undeciphered, "Report undeciphered multi-touch state field using a MSC_RAW event");

#define JOY_CON_LOW_REPORT_ID 0x3F
#define JOY_CON_MEDIUM_REPORT_ID 0x30
#define TRACKPAD_REPORT_ID 0x01
#define TRACKPAD2_USB_REPORT_ID 0x02
#define TRACKPAD2_BT_REPORT_ID 0x31
#define MOUSE_REPORT_ID    0x29
#define DOUBLE_REPORT_ID   0xf7
/* These definitions are not precise, but they're close enough.  (Bits
 * 0x03 seem to indicate the aspect ratio of the touch, bits 0x70 seem
 * to be some kind of bit mask -- 0x20 may be a near-field reading,
 * and 0x40 is actual contact, and 0x10 may be a start/stop or change
 * indication.)
 */
#define TOUCH_STATE_MASK  0xf0
#define TOUCH_STATE_NONE  0x00
#define TOUCH_STATE_START 0x30
#define TOUCH_STATE_DRAG  0x40

#define SCROLL_ACCEL_DEFAULT 7

/* Touch surface information. Dimension is in hundredths of a mm, min and max
 * are in units. */
#define MOUSE_DIMENSION_X (float)9056
#define MOUSE_MIN_X -1100
#define MOUSE_MAX_X 1258
#define MOUSE_RES_X ((MOUSE_MAX_X - MOUSE_MIN_X) / (MOUSE_DIMENSION_X / 100))
#define MOUSE_DIMENSION_Y (float)5152
#define MOUSE_MIN_Y -1589
#define MOUSE_MAX_Y 2047
#define MOUSE_RES_Y ((MOUSE_MAX_Y - MOUSE_MIN_Y) / (MOUSE_DIMENSION_Y / 100))

#define TRACKPAD_DIMENSION_X (float)13000
#define TRACKPAD_MIN_X -2909
#define TRACKPAD_MAX_X 3167
#define TRACKPAD_RES_X \
    ((TRACKPAD_MAX_X - TRACKPAD_MIN_X) / (TRACKPAD_DIMENSION_X / 100))
#define TRACKPAD_DIMENSION_Y (float)11000
#define TRACKPAD_MIN_Y -2456
#define TRACKPAD_MAX_Y 2565
#define TRACKPAD_RES_Y \
    ((TRACKPAD_MAX_Y - TRACKPAD_MIN_Y) / (TRACKPAD_DIMENSION_Y / 100))

#define TRACKPAD2_DIMENSION_X (float)16000
#define TRACKPAD2_MIN_X -3678
#define TRACKPAD2_MAX_X 3934
#define TRACKPAD2_RES_X \
    ((TRACKPAD2_MAX_X - TRACKPAD2_MIN_X) / (TRACKPAD2_DIMENSION_X / 100))
#define TRACKPAD2_DIMENSION_Y (float)11490
#define TRACKPAD2_MIN_Y -2478
#define TRACKPAD2_MAX_Y 2587
#define TRACKPAD2_RES_Y \
    ((TRACKPAD2_MAX_Y - TRACKPAD2_MIN_Y) / (TRACKPAD2_DIMENSION_Y / 100))

static LIST_HEAD(joycon_list);

enum config {
    INVALID,
    LEFT,
    RIGHT,
    COMPOSITE
};

enum state {
    INITIAL,
    FINDING_PEER,
    CONNECTED,
};

struct joycon {
    struct hid_device *hdev;
    struct hid_device *hdev2;
    enum config config;
    enum state state;
    struct input_dev *input;
    struct list_head list_head;
};

/**
 * struct magicmouse_sc - Tracks Magic Mouse-specific data.
 * @input: Input device through which we report events.
 * @quirks: Currently unused.
 * @ntouches: Number of touches in most recent touch report.
 * @scroll_accel: Number of consecutive scroll motions.
 * @scroll_jiffies: Time of last scroll motion.
 * @touches: Most recent data for a touch, indexed by tracking ID.
 * @tracking_ids: Mapping of current touch input data to @touches.
 */
struct magicmouse_sc {
    struct input_dev *input;
    unsigned long quirks;

    int ntouches;
    int scroll_accel;
    unsigned long scroll_jiffies;

    struct {
        short x;
        short y;
        short scroll_x;
        short scroll_y;
        u8 size;
    } touches[16];
    int tracking_ids[16];
};

static int magicmouse_firm_touch(struct magicmouse_sc *msc)
{
    int touch = -1;
    int ii;

    /* If there is only one "firm" touch, set touch to its
     * tracking ID.
     */
    for (ii = 0; ii < msc->ntouches; ii++) {
        int idx = msc->tracking_ids[ii];
        if (msc->touches[idx].size < 8) {
            /* Ignore this touch. */
        } else if (touch >= 0) {
            touch = -1;
            break;
        } else {
            touch = idx;
        }
    }

    return touch;
}

static void magicmouse_emit_buttons(struct magicmouse_sc *msc, int state)
{
    int last_state = test_bit(BTN_LEFT, msc->input->key) << 0 |
        test_bit(BTN_RIGHT, msc->input->key) << 1 |
        test_bit(BTN_MIDDLE, msc->input->key) << 2;

    if (emulate_3button) {
        int id;

        /* If some button was pressed before, keep it held
         * down.  Otherwise, if there's exactly one firm
         * touch, use that to override the mouse's guess.
         */
        if (state == 0) {
            /* The button was released. */
        } else if (last_state != 0) {
            state = last_state;
        } else if ((id = magicmouse_firm_touch(msc)) >= 0) {
            int x = msc->touches[id].x;
            if (x < middle_button_start)
                state = 1;
            else if (x > middle_button_stop)
                state = 2;
            else
                state = 4;
        } /* else: we keep the mouse's guess */

        input_report_key(msc->input, BTN_MIDDLE, state & 4);
    }

    input_report_key(msc->input, BTN_LEFT, state & 1);
    input_report_key(msc->input, BTN_RIGHT, state & 2);

    if (state != last_state)
        msc->scroll_accel = SCROLL_ACCEL_DEFAULT;
}

static void magicmouse_emit_touch(struct magicmouse_sc *msc, int raw_id, u8 *tdata)
{
    struct input_dev *input = msc->input;
    int id, x, y, size, orientation, touch_major, touch_minor, state, down;
    int pressure = 0;

    if (input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE) {
        id = (tdata[6] << 2 | tdata[5] >> 6) & 0xf;
        x = (tdata[1] << 28 | tdata[0] << 20) >> 20;
        y = -((tdata[2] << 24 | tdata[1] << 16) >> 20);
        size = tdata[5] & 0x3f;
        orientation = (tdata[6] >> 2) - 32;
        touch_major = tdata[3];
        touch_minor = tdata[4];
        state = tdata[7] & TOUCH_STATE_MASK;
        down = state != TOUCH_STATE_NONE;
    } else if (input->id.product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2) {
        id = tdata[8] & 0xf;
        x = (tdata[1] << 27 | tdata[0] << 19) >> 19;
        y = -((tdata[3] << 30 | tdata[2] << 22 | tdata[1] << 14) >> 19);
        size = tdata[6];
        orientation = (tdata[8] >> 5) - 4;
        touch_major = tdata[4];
        touch_minor = tdata[5];
        pressure = tdata[7];
        state = tdata[3] & 0xC0;
        down = state == 0x80;
    } else { /* USB_DEVICE_ID_APPLE_MAGICTRACKPAD */
        id = (tdata[7] << 2 | tdata[6] >> 6) & 0xf;
        x = (tdata[1] << 27 | tdata[0] << 19) >> 19;
        y = -((tdata[3] << 30 | tdata[2] << 22 | tdata[1] << 14) >> 19);
        size = tdata[6] & 0x3f;
        orientation = (tdata[7] >> 2) - 32;
        touch_major = tdata[4];
        touch_minor = tdata[5];
        state = tdata[8] & TOUCH_STATE_MASK;
        down = state != TOUCH_STATE_NONE;
    }

    /* Store tracking ID and other fields. */
    msc->tracking_ids[raw_id] = id;
    msc->touches[id].x = x;
    msc->touches[id].y = y;
    msc->touches[id].size = size;

    /* If requested, emulate a scroll wheel by detecting small
     * vertical touch motions.
     */
    if (emulate_scroll_wheel && (input->id.product !=
                USB_DEVICE_ID_APPLE_MAGICTRACKPAD2)) {
        unsigned long now = jiffies;
        int step_x = msc->touches[id].scroll_x - x;
        int step_y = msc->touches[id].scroll_y - y;

        /* Calculate and apply the scroll motion. */
        switch (state) {
            case TOUCH_STATE_START:
                msc->touches[id].scroll_x = x;
                msc->touches[id].scroll_y = y;

                /* Reset acceleration after half a second. */
                if (scroll_acceleration && time_before(now,
                            msc->scroll_jiffies + HZ / 2))
                    msc->scroll_accel = max_t(int,
                            msc->scroll_accel - 1, 1);
                else
                    msc->scroll_accel = SCROLL_ACCEL_DEFAULT;

                break;
            case TOUCH_STATE_DRAG:
                step_x /= (64 - (int)scroll_speed) * msc->scroll_accel;
                if (step_x != 0) {
                    msc->touches[id].scroll_x -= step_x *
                        (64 - scroll_speed) * msc->scroll_accel;
                    msc->scroll_jiffies = now;
                    input_report_rel(input, REL_HWHEEL, -step_x);
                }

                step_y /= (64 - (int)scroll_speed) * msc->scroll_accel;
                if (step_y != 0) {
                    msc->touches[id].scroll_y -= step_y *
                        (64 - scroll_speed) * msc->scroll_accel;
                    msc->scroll_jiffies = now;
                    input_report_rel(input, REL_WHEEL, step_y);
                }
                break;
        }
    }

    if (down)
        msc->ntouches++;

    input_mt_slot(input, id);
    input_mt_report_slot_state(input, MT_TOOL_FINGER, down);

    /* Generate the input events for this touch. */
    if (down) {
        input_report_abs(input, ABS_MT_TOUCH_MAJOR, touch_major << 2);
        input_report_abs(input, ABS_MT_TOUCH_MINOR, touch_minor << 2);
        input_report_abs(input, ABS_MT_ORIENTATION, -orientation);
        input_report_abs(input, ABS_MT_POSITION_X, x);
        input_report_abs(input, ABS_MT_POSITION_Y, y);

        if (input->id.product == USB_DEVICE_ID_APPLE_MAGICTRACKPAD2)
            input_report_abs(input, ABS_MT_PRESSURE, pressure);

        if (report_undeciphered) {
            if (input->id.product == USB_DEVICE_ID_APPLE_MAGICMOUSE)
                input_event(input, EV_MSC, MSC_RAW, tdata[7]);
            else if (input->id.product !=
                    USB_DEVICE_ID_APPLE_MAGICTRACKPAD2)
                input_event(input, EV_MSC, MSC_RAW, tdata[8]);
        }
    }
}

static void sendCommand(struct hid_device *hdev,
                        unsigned char subcommand,
                        char *data,
                        unsigned char dataLen) {
        
    static char globalPacketNumber = 0;

    u8 *outputBuffer = kzalloc(0x40, GFP_KERNEL); //TODO: define

    if (outputBuffer) {
        // Change to Standard full mode

        globalPacketNumber = (globalPacketNumber + 1) % 0xF;

        outputBuffer[0] = 0x1;
        outputBuffer[1] = globalPacketNumber;
        outputBuffer[10] = subcommand;
        memcpy(outputBuffer + 11, data, dataLen);
        hid_hw_output_report(hdev, outputBuffer, 0x40);

        kfree(outputBuffer);
    }
}

static int checkForSingleSelection(struct joycon *joycon, u8 *data, int size)
{
    if (IS_SL_SET(data) && IS_SR_SET(data))
    {
        hid_hw_stop(joycon->hdev);
        hid_hw_start(joycon->hdev, HID_CONNECT_DEFAULT);

        if (joycon->input) {
            joycon->state = CONNECTED;
            sendCommand(joycon->hdev, 0x30, (char[]){0x01}, 1);
        }
    }
}

static int checkForCompositeSelection(struct joycon *joycon, u8 *data, int size)
{
    struct list_head *pos;
    struct joycon *leftJoycon = NULL;
    struct joycon *rightJoycon = NULL;

    if (IS_L_OR_R_SET(data))
    {
        joycon->state = FINDING_PEER;
    }

    list_for_each(pos, &joycon_list)
    {
        struct joycon *jc = list_entry(pos, struct joycon, list_head);

        if (jc->config == LEFT && jc->state == FINDING_PEER)
        {
            leftJoycon = jc;
        }
        
        if (jc->config == RIGHT && jc->state == FINDING_PEER)
        {
            rightJoycon = jc;
        }

        if (leftJoycon != NULL && rightJoycon != NULL)
        {
            hid_hw_stop(leftJoycon->hdev);
            hid_hw_start(leftJoycon->hdev,
                         HID_CONNECT_HIDRAW | HID_CONNECT_HIDDEV);

            if (leftJoycon->input) {
                leftJoycon->state = CONNECTED;
                leftJoycon->config = COMPOSITE;

                hid_set_drvdata(rightJoycon->hdev, leftJoycon);
                leftJoycon->hdev2 = rightJoycon->hdev;
                list_del(&rightJoycon->list_head);
                devm_kfree(&rightJoycon->hdev->dev, rightJoycon);

                sendCommand(leftJoycon->hdev, 0x30, (char[]){0x01}, 1);
                sendCommand(leftJoycon->hdev2, 0x30, (char[]){0x01}, 1);
            }
            
        }
    }
}

static int magicmouse_raw_event(struct hid_device *hdev,
        struct hid_report *report, u8 *data, int size)
{
    struct joycon *joycon = hid_get_drvdata(hdev);
    struct input_dev *input = joycon->input;
    int x = 0, y = 0, ii, clicks = 0, npoints;

    printk("Report %d", report->id);

    if (input)
    {
        switch (data[0]) {
            case JOY_CON_LOW_REPORT_ID:
            {
                if (joycon->config == COMPOSITE)
                {
                    if (hdev == joycon->hdev)
                    {
                        input_report_key(input, BTN_DPAD_LEFT, data[1] & 0x01);
                        input_report_key(input, BTN_DPAD_DOWN, data[1] & 0x02);
                        input_report_key(input, BTN_DPAD_UP, data[1] & 0x04);
                        input_report_key(input, BTN_DPAD_RIGHT, data[1] & 0x08);
                    }
                    else
                    {
                        input_report_key(input, BTN_EAST, data[1] & 0x01);
                        input_report_key(input, BTN_NORTH, data[1] & 0x02);
                        input_report_key(input, BTN_SOUTH, data[1] & 0x04);
                        input_report_key(input, BTN_WEST, data[1] & 0x08);
                    }
                }
                else
                {
                    input_report_key(input, BTN_SOUTH, data[1] & 0x01);
                    input_report_key(input, BTN_EAST, data[1] & 0x02);
                    input_report_key(input, BTN_WEST, data[1] & 0x04);
                    input_report_key(input, BTN_NORTH, data[1] & 0x08);
                }
            }
            break;

            case JOY_CON_MEDIUM_REPORT_ID:
            {
                u8 *stick = data + 6;
                u16 stick_horizontal = stick[0] | ((stick[1] & 0xF) << 8);
                //u16 stick_vertical = (stick[1] >> 4) | (stick[2] << 4);

                input_report_abs(input, ABS_X, stick_horizontal);

                input_report_key(input, BTN_EAST, data[5] & 0x01);
                input_report_key(input, BTN_WEST, data[5] & 0x02);
                input_report_key(input, BTN_NORTH, data[5] & 0x04);
                input_report_key(input, BTN_SOUTH, data[5] & 0x08);
            }
            break;
        }

        input_sync(input);
    }
    else
    {
        checkForSingleSelection(joycon, data, size);
        checkForCompositeSelection(joycon, data, size);
    }

    return 1;
}

static int magicmouse_setup_input(struct input_dev *input, struct hid_device *hdev)
{
    int error;
    int mt_flags = 0;

    printk("Setup Input");

    __set_bit(EV_KEY, input->evbit);
    __set_bit(BTN_LEFT, input->keybit);
    __set_bit(BTN_DPAD_UP, input->keybit);
    __set_bit(BTN_DPAD_DOWN, input->keybit);
    __set_bit(BTN_DPAD_LEFT, input->keybit);
    __set_bit(BTN_DPAD_RIGHT, input->keybit);
    __set_bit(BTN_NORTH, input->keybit);
    __set_bit(BTN_SOUTH, input->keybit);
    __set_bit(BTN_WEST, input->keybit);
    __set_bit(BTN_EAST, input->keybit);
    __set_bit(BTN_TL, input->keybit);
    __set_bit(BTN_TR, input->keybit);
    __set_bit(BTN_START, input->keybit);
    __set_bit(BTN_SELECT, input->keybit);
    
    __set_bit(EV_ABS, input->evbit);
    __set_bit(ABS_X, input->absbit);
    __set_bit(ABS_X, input->absbit);
    __set_bit(ABS_Y, input->absbit);
    __set_bit(ABS_RX, input->absbit);
    __set_bit(ABS_RY, input->absbit);

    input_set_abs_params(input, ABS_X, 800, 3200, 4, 0);
    input_set_abs_params(input, ABS_Y, 800, 3200, 4, 0);
    input_set_abs_params(input, ABS_RX, 800, 3200, 4, 0);
    input_set_abs_params(input, ABS_RY, 800, 3200, 4, 0);

    input_set_events_per_packet(input, 60);

    return 0;
}

static int magicmouse_input_mapping(struct hid_device *hdev,
        struct hid_input *hi, struct hid_field *field,
        struct hid_usage *usage, unsigned long **bit, int *max)
{
    struct joycon *msc = hid_get_drvdata(hdev);

    if (!msc->input)
        msc->input = hi->input;

    return 0;
}

static int magicmouse_input_configured(struct hid_device *hdev,
        struct hid_input *hi)

{
    struct joycon *msc = hid_get_drvdata(hdev);
    int ret;

    if (msc->input)
    {
        ret = magicmouse_setup_input(msc->input, hdev);
        if (ret) {
            hid_err(hdev, "magicmouse setup input failed (%d)\n", ret);
            /* clean msc->input to notify probe() of the failure */
            msc->input = NULL;
            return ret;
        }
    }

    return 0;
}

static int magicmouse_probe(struct hid_device *hdev,
        const struct hid_device_id *id)
{
    const u8 *feature;
    const u8 feature_mt[] = { 0xD7, 0x01 };
    const u8 feature_mt_trackpad2_usb[] = { 0x02, 0x01 };
    const u8 feature_mt_trackpad2_bt[] = { 0xF1, 0x02, 0x01 };
    u8 *buf;
    struct joycon *joycon;
    struct hid_report *report;
    int ret;
    int feature_size;

    joycon = devm_kzalloc(&hdev->dev, sizeof(*joycon), GFP_KERNEL);
    if (joycon == NULL) {
        hid_err(hdev, "can't alloc magicmouse descriptor\n");
        return -ENOMEM;
    }

    joycon->hdev = hdev;
    joycon->config = id->product ==
        USB_DEVICE_ID_NINTENDO_JOY_CON_L ? LEFT : RIGHT;

    INIT_LIST_HEAD(&joycon->list_head);

    list_add(&joycon->list_head, &joycon_list);

    hid_set_drvdata(hdev, joycon);

    ret = hid_parse(hdev);
    if (ret) {
        hid_err(hdev, "magicmouse hid parse failed\n");
        return ret;
    }

    ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
    if (ret) {
        hid_err(hdev, "magicmouse hw start failed\n");
        return ret;
    }

    //sendCommand(hdev, 0x03, (char[]){0x30}, 1);

    return 0;
err_stop_hw:
    hid_hw_stop(hdev);
    return ret;
}

static const struct hid_device_id joy_cons[] = {
    { HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
            USB_DEVICE_ID_NINTENDO_JOY_CON_L), .driver_data = 0 },
    { HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
            USB_DEVICE_ID_NINTENDO_JOY_CON_R), .driver_data = 0 },
    { }
};

MODULE_DEVICE_TABLE(hid, joy_cons);

static struct hid_driver joycon_driver = {
    .name = "joycon",
    .id_table = joy_cons,
    .probe = magicmouse_probe,
    .raw_event = magicmouse_raw_event,
    .input_mapping = magicmouse_input_mapping,
    .input_configured = magicmouse_input_configured,
};

module_hid_driver(joycon_driver);

MODULE_LICENSE("GPL");
