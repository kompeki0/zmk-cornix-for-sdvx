/*
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_refcount_key

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * encoded HID usage (binding->param1) ごとに refcount を持つ
 * - 256キー全部を配列で持つのは encoded の形式が広いのでやめる
 * - 使うキー数は多くてもせいぜい数十のはずなので固定テーブルでOK
 */
#ifndef CONFIG_ZMK_REFCOUNT_KEY_MAX_TRACKED
#define CONFIG_ZMK_REFCOUNT_KEY_MAX_TRACKED 32
#endif

struct ref_item {
    uint32_t encoded;
    uint8_t count;
};

static struct ref_item refs[CONFIG_ZMK_REFCOUNT_KEY_MAX_TRACKED];

static struct ref_item *get_or_alloc(uint32_t encoded) {
    struct ref_item *free_slot = NULL;

    for (int i = 0; i < CONFIG_ZMK_REFCOUNT_KEY_MAX_TRACKED; i++) {
        if (refs[i].encoded == encoded && refs[i].count > 0) {
            return &refs[i];
        }
        if (refs[i].count == 0 && free_slot == NULL) {
            free_slot = &refs[i];
        }
    }

    if (free_slot) {
        free_slot->encoded = encoded;
        free_slot->count = 0;
        return free_slot;
    }

    return NULL; // テーブル不足
}

static struct ref_item *find_existing(uint32_t encoded) {
    for (int i = 0; i < CONFIG_ZMK_REFCOUNT_KEY_MAX_TRACKED; i++) {
        if (refs[i].encoded == encoded && refs[i].count > 0) {
            return &refs[i];
        }
    }
    return NULL;
}

static int emit_keycode_event(uint32_t encoded, bool pressed, int64_t timestamp) {
    // kp と同じ “ZMKの正規ルート”
    return raise_zmk_keycode_state_changed_from_encoded(encoded, pressed, timestamp);
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const uint32_t encoded = binding->param1;

    struct ref_item *it = get_or_alloc(encoded);
    if (!it) {
        LOG_ERR("refcount_key: table full (increase CONFIG_ZMK_REFCOUNT_KEY_MAX_TRACKED)");
        return ZMK_BEHAVIOR_OPAQUE;
    }

    if (it->count == 0) {
        it->count = 1;
        LOG_DBG("refcount_key press encoded=0x%08X rc=1 pos=%d", encoded, event.position);
        (void)emit_keycode_event(encoded, true, event.timestamp);
    } else {
        if (it->count < 255) {
            it->count++;
        }
        LOG_DBG("refcount_key press encoded=0x%08X rc=%u pos=%d", encoded, it->count,
                event.position);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const uint32_t encoded = binding->param1;

    struct ref_item *it = find_existing(encoded);
    if (!it) {
        LOG_WRN("refcount_key release while not tracked encoded=0x%08X pos=%d", encoded,
                event.position);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    if (it->count > 0) {
        it->count--;
    }

    if (it->count == 0) {
        LOG_DBG("refcount_key release encoded=0x%08X rc=0 (emit) pos=%d", encoded, event.position);
        (void)emit_keycode_event(encoded, false, event.timestamp);
        // slot は count==0 になったので再利用可能
    } else {
        LOG_DBG("refcount_key release encoded=0x%08X rc=%u pos=%d", encoded, it->count,
                event.position);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_refcount_key_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

static int behavior_refcount_key_init(const struct device *dev) {
    ARG_UNUSED(dev);
    // refs をゼロクリア（BSSなら本来不要だけど明示してもOK）
    for (int i = 0; i < CONFIG_ZMK_REFCOUNT_KEY_MAX_TRACKED; i++) {
        refs[i] = (struct ref_item){0};
    }
    return 0;
}

#define REFCOUNT_INST(n)                                                                           \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_refcount_key_init, NULL, NULL, NULL, POST_KERNEL,          \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_refcount_key_driver_api);

DT_INST_FOREACH_STATUS_OKAY(REFCOUNT_INST)
