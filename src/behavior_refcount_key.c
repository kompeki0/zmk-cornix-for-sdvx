/*
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_refcount_key

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/endpoints.h>
#include <zmk/hid.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * 「通常キーコードのみ」前提：
 * - HID Usage Page = Keyboard/Keypad (0x07)
 * - keycode は 0..255 を想定
 *
 * param1 にキーコードが入ってくる想定（&refcount_key A みたいに使う）
 */
static uint8_t refcounts[256];

static int send_press(uint8_t keycode) {
    zmk_hid_keyboard_press(keycode);
    return zmk_endpoints_send_report(zmk_hid_get_keyboard_report());
}

static int send_release(uint8_t keycode) {
    zmk_hid_keyboard_release(keycode);
    return zmk_endpoints_send_report(zmk_hid_get_keyboard_report());
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const uint32_t keycode_u32 = binding->param1;
    if (keycode_u32 > 255) {
        LOG_ERR("refcount_key: keycode out of range: %u", keycode_u32);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    const uint8_t keycode = (uint8_t)keycode_u32;

    /* 0 -> 1 のときだけ実際に press を送る */
    if (refcounts[keycode] == 0) {
        refcounts[keycode] = 1;
        LOG_DBG("refcount_key press kc=%u rc=1", keycode);
        (void)send_press(keycode);
    } else {
        /* 飽和対策（255超えは止める） */
        if (refcounts[keycode] < 255) {
            refcounts[keycode]++;
        }
        LOG_DBG("refcount_key press kc=%u rc=%u", keycode, refcounts[keycode]);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const uint32_t keycode_u32 = binding->param1;
    if (keycode_u32 > 255) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    const uint8_t keycode = (uint8_t)keycode_u32;

    if (refcounts[keycode] == 0) {
        /* release の順序が壊れても暴れないように握りつぶす */
        LOG_WRN("refcount_key release while rc=0 kc=%u", keycode);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    refcounts[keycode]--;

    /* 1 -> 0 のときだけ実際に release を送る */
    if (refcounts[keycode] == 0) {
        LOG_DBG("refcount_key release kc=%u rc=0 (send)", keycode);
        (void)send_release(keycode);
    } else {
        LOG_DBG("refcount_key release kc=%u rc=%u", keycode, refcounts[keycode]);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_refcount_key_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

static int behavior_refcount_key_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

/* leader_key と同じ “型” の登録スタイルに合わせる */
#define REFCOUNT_INST(n)                                                                            \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_refcount_key_init, NULL, NULL, NULL, POST_KERNEL,           \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_refcount_key_driver_api);

DT_INST_FOREACH_STATUS_OKAY(REFCOUNT_INST)
