#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/hid.h>

LOG_MODULE_REGISTER(behavior_refcount_key, CONFIG_ZMK_LOG_LEVEL);

/*
 * Devicetree binding の compatible に対応させる。
 * 例: compatible = "zmk,behavior-refcount-key";
 * → DT_DRV_COMPAT は zmk_behavior_refcount_key
 */
#define DT_DRV_COMPAT zmk_behavior_refcount_key

/*
 * “まず通常キーコードのみ” 前提で、
 * keycode を 0..255 の範囲として参照カウントする。
 *
 * ※必要なら 512 などに拡張可能（ただしRAM増える）
 */
#ifndef REFCNT_KEY_MAX
#define REFCNT_KEY_MAX 256
#endif

struct behavior_refcount_key_data {
    uint8_t cnt[REFCNT_KEY_MAX];
};

static int behavior_refcount_key_press(const struct device *dev,
                                       struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    struct behavior_refcount_key_data *data = dev->data;
    const uint32_t keycode = binding->param1;

    if (keycode >= REFCNT_KEY_MAX) {
        LOG_WRN("keycode %u out of range (max %d)", (unsigned)keycode, REFCNT_KEY_MAX - 1);
        return 0;
    }

    if (data->cnt[keycode] < 0xFF) {
        data->cnt[keycode]++;
    } else {
        /* 異常系：押しっぱなし/バグでオーバーフローしないように */
        LOG_WRN("refcount overflow for keycode %u", (unsigned)keycode);
    }

    /*
     * 0→1 になった瞬間だけ “press” を送る
     */
    if (data->cnt[keycode] == 1) {
        zmk_hid_keyboard_press(keycode);
        return zmk_endpoints_send_report(ZMK_HID_USAGE_KEYBOARD);
    }

    return 0;
}

static int behavior_refcount_key_release(const struct device *dev,
                                         struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    struct behavior_refcount_key_data *data = dev->data;
    const uint32_t keycode = binding->param1;

    if (keycode >= REFCNT_KEY_MAX) {
        return 0;
    }

    /*
     * 参照が残ってる間は release を送らない。
     * 最後の1個が離されたタイミング（1→0）だけ “release” を送る。
     */
    if (data->cnt[keycode] == 0) {
        /* 二重release等の異常系は無視 */
        return 0;
    }

    data->cnt[keycode]--;

    if (data->cnt[keycode] == 0) {
        zmk_hid_keyboard_release(keycode);
        return zmk_endpoints_send_report(ZMK_HID_USAGE_KEYBOARD);
    }

    return 0;
}

static const struct behavior_driver_api behavior_refcount_key_driver_api = {
    .binding_pressed = behavior_refcount_key_press,
    .binding_released = behavior_refcount_key_release,
};

/*
 * ★重要：この compatible のノードが Devicetree 上に1つも無い時
 *        (settings_reset のビルドなど) は、デバイス定義しない。
 *
 * これで “DT_N_INST_0_... が無い” 系のエラーを回避できる。
 */
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define REFCNT_INST_DEFINE(inst)                                                            \
    static struct behavior_refcount_key_data behavior_refcount_key_data_##inst = {0};       \
    BEHAVIOR_DT_INST_DEFINE(inst,                                                           \
                            NULL, /* init */                                                \
                            NULL, /* pm */                                                  \
                            &behavior_refcount_key_data_##inst,                             \
                            NULL, /* config */                                              \
                            POST_KERNEL,                                                    \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                            \
                            &behavior_refcount_key_driver_api);

DT_INST_FOREACH_STATUS_OKAY(REFCNT_INST_DEFINE)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
