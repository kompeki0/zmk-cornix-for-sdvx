#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#include <drivers/behavior.h>
#include <zmk/hid.h>

/*
 * Refcount table:
 * index = ZMK keycode (HID "Key" usage page keycode)
 *
 * NOTE:
 * - This is intentionally "keyboard page only" for now.
 * - If you later want Consumer etc, you'll need a different key space.
 */
static uint16_t key_refcnt[256];

static inline bool is_valid_keycode(uint32_t keycode) {
    return keycode < ARRAY_SIZE(key_refcnt);
}

/*
 * We send real press/release via zmk_hid_keyboard_press/release.
 * If your tree exposes slightly different names, adjust here.
 */
static int send_press(uint32_t keycode) {
    return zmk_hid_keyboard_press((uint8_t)keycode);
}

static int send_release(uint32_t keycode) {
    return zmk_hid_keyboard_release((uint8_t)keycode);
}

static int on_binding_pressed(struct zmk_behavior_binding *binding,
                              struct zmk_behavior_binding_event event) {
    uint32_t keycode = binding->param1;

    if (!is_valid_keycode(keycode)) {
        return -EINVAL;
    }

    uint16_t before = key_refcnt[keycode];
    if (before == UINT16_MAX) {
        // Saturate; something is wrong but don't wrap.
        return 0;
    }

    key_refcnt[keycode] = before + 1;

    // First press only -> emit actual press
    if (before == 0) {
        return send_press(keycode);
    }

    return 0;
}

static int on_binding_released(struct zmk_behavior_binding *binding,
                               struct zmk_behavior_binding_event event) {
    uint32_t keycode = binding->param1;

    if (!is_valid_keycode(keycode)) {
        return -EINVAL;
    }

    uint16_t before = key_refcnt[keycode];
    if (before == 0) {
        // Release without press; ignore
        return 0;
    }

    uint16_t after = before - 1;
    key_refcnt[keycode] = after;

    // Last release only -> emit actual release
    if (after == 0) {
        return send_release(keycode);
    }

    return 0;
}

static const struct behavior_driver_api behavior_refcount_key_driver_api = {
    .binding_pressed = on_binding_pressed,
    .binding_released = on_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_refcount_key_driver_api);
