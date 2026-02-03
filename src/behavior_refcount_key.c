/*
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_refcount_key

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/endpoints.h>
#include <zmk/hid.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * シンプル版：Keyboard(Usage Page 0x07)の keycode(0-255) だけを refcount。
 * 参照カウントが 0→1 になったときに press を送信し、1→0 になったときに release を送信。
 *
 * 「同じ keycode を送るのは全部 &refcount_key 経由」なら、これで狙い通り動きます。
 */

#define KEYCODE_MAX 256

/* keycodeごとの押下参照カウント */
static atomic_t refcnt[KEYCODE_MAX];

static inline int send_press(uint8_t keycode) {
    /* ZMKのHID状態へ反映 */
    zmk_hid_keyboard_press(keycode);

    /* ホストへ送信 */
    return zmk_endpoints_send_report();
}

static inline int send_release(uint8_t keycode) {
    zmk_hid_keyboard_release(keycode);
    return zmk_endpoints_send_report();
}

static int on_refcount_key_pressed(struct zmk_behavior_binding *binding,
                                   struct zmk_behavior_binding_event event) {
    /* param1 を keycode として扱う（&refcount_key <KC> の <KC>） */
    uint8_t keycode = (uint8_t)binding->param1;

    if (keycode >= KEYCODE_MAX) {
        LOG_ERR("refcount_key: invalid keycode %d", keycode);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    /* refcount++ して、0→1 のときだけ press を実際に送る */
    int prev = atomic_inc(&refcnt[keycode]);
    if (prev == 0) {
        LOG_DBG("refcount_key: press kc=%d (cnt 0->1)", keycode);
        (void)send_press(keycode);
    } else {
        LOG_DBG("refcount_key: press kc=%d (cnt %d->%d)", keycode, prev, prev + 1);
    }

    /* ここは “透過” にせず OPAQUE 推奨（同一位置で他の処理をさせない） */
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_refcount_key_released(struct zmk_behavior_binding *binding,
                                    struct zmk_behavior_binding_event event) {
    uint8_t keycode = (uint8_t)binding->param1;

    if (keycode >= KEYCODE_MAX) {
        LOG_ERR("refcount_key: invalid keycode %d", keycode);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    int prev = atomic_get(&refcnt[keycode]);
    if (prev <= 0) {
        /* release が多重に来た、または別経路で崩れた */
        LOG_WRN("refcount_key: underflow kc=%d (cnt=%d)", keycode, prev);
        atomic_set(&refcnt[keycode], 0);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    /* refcount-- して、1→0 のときだけ release を実際に送る */
    prev = atomic_dec(&refcnt[keycode]);
    int now = prev - 1;

    if (now == 0) {
        LOG_DBG("refcount_key: release kc=%d (cnt 1->0)", keycode);
        (void)send_release(keycode);
    } else {
        LOG_DBG("refcount_key: release kc=%d (cnt %d->%d)", keycode, prev, now);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_refcount_key_driver_api = {
    .binding_pressed = on_refcount_key_pressed,
    .binding_released = on_refcount_key_released,
};

static int behavior_refcount_key_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

/*
 * ここが leader_key と同じ “DTインスタンス生成” の作法。
 * DTに node が存在する分だけ behavior デバイスが生成される。
 *
 * ※ このbehaviorは config を持たない（param1で動く）ので config=NULL でOK。
 */
#define REFCOUNT_INST(n)                                                                          \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_refcount_key_init, NULL, NULL, NULL, POST_KERNEL,         \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_refcount_key_driver_api);

DT_INST_FOREACH_STATUS_OKAY(REFCOUNT_INST)
