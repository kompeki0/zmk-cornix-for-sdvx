/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_sensor_hold_step_rotate

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/virtual_key_position.h>
#include <zmk/events/position_state_changed.h>

#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>

#include <zmk/hid.h>
#include <zmk/keys.h>

#ifndef ZMK_KEYMAP_SENSORS_LEN
#define ZMK_KEYMAP_SENSORS_LEN 0
#endif

#ifndef ZMK_KEYMAP_LAYERS_LEN
#define ZMK_KEYMAP_LAYERS_LEN 1
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * 目的:
 * - 回転中: hold-bindings を長押し（timeoutでrelease）
 * - 回転ステップをカウントし、step-group-sizeごとに step-bindings をtap発火
 * - direction-hold-mode:
 *   0=SWITCH: 逆回転でholdを切替（release→press）
 *   1=STICKY: 逆回転でもholdは維持（途切れない）
 *
 * 追加:
 * - layers: 指定レイヤー以外では動かさない
 * - quick-release: 許可外キー押下で hold を即解除（caps_wordの思想）
 *   - quick-release-allow-list: 許可する “送信キーコード” の列挙
 */

enum hold_dir {
    HOLD_DIR_NONE = 0,
    HOLD_DIR_CW = 1,
    HOLD_DIR_CCW = 2,
};

enum hold_mode {
    HOLD_MODE_SWITCH = 0,
    HOLD_MODE_STICKY = 1,
};

/* allow item: caps_word の continue-list と同じ “packed keycode” を page/id に分解して保持 */
struct allow_item {
    uint16_t page;
    uint16_t id;
};

struct behavior_sensor_hold_step_rotate_config {
    struct zmk_behavior_binding hold_cw;
    struct zmk_behavior_binding hold_ccw;
    struct zmk_behavior_binding step_cw;
    struct zmk_behavior_binding step_ccw;

    uint16_t timeout_ms;
    uint16_t step_group_size;
    uint8_t direction_hold_mode;

    /* layers restriction: 0 = no restriction */
    uint32_t layers_mask;

    /* quick-release */
    bool quick_release;
    uint8_t allow_count;
    struct allow_item allow_list[];
};

struct hold_state {
    bool inited;
    bool active;
    struct zmk_behavior_binding active_binding;

    int16_t last_position;
    uint8_t last_layer;

    uint16_t step_count;

    struct k_work_delayable release_work;
};

struct behavior_sensor_hold_step_rotate_data {
    enum hold_dir pending_dir[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    struct hold_state state[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
};

/* ---- instance list (caps_word style) ---- */
#define GET_DEV(inst) DEVICE_DT_INST_GET(inst),
static const struct device *devs[] = {DT_INST_FOREACH_STATUS_OKAY(GET_DEV)};

/* ---- helpers ---- */

static inline bool layer_allowed_mask(uint32_t mask, uint8_t layer) {
    if (mask == 0u) return true;
    if (layer >= 32) return false;
    return (mask & (1u << layer)) != 0u;
}

static inline bool layer_allowed(const struct behavior_sensor_hold_step_rotate_config *cfg,
                                 uint8_t layer) {
    return layer_allowed_mask(cfg->layers_mask, layer);
}

static int enqueue_press(struct zmk_behavior_binding_event *event,
                         struct zmk_behavior_binding binding) {
    return zmk_behavior_queue_add(event, binding, true, 0);
}

static int enqueue_release(struct zmk_behavior_binding_event *event,
                           struct zmk_behavior_binding binding) {
    return zmk_behavior_queue_add(event, binding, false, 0);
}

static int enqueue_tap(struct zmk_behavior_binding_event *event,
                       struct zmk_behavior_binding binding) {
    zmk_behavior_queue_add(event, binding, true, 0);
    return zmk_behavior_queue_add(event, binding, false, 0);
}

static bool binding_equal(struct zmk_behavior_binding a, struct zmk_behavior_binding b) {
    return (a.behavior_dev == b.behavior_dev) && (a.param1 == b.param1) && (a.param2 == b.param2);
}

static void arm_timeout(const struct behavior_sensor_hold_step_rotate_config *cfg,
                        struct hold_state *st) {
    uint16_t ms = cfg->timeout_ms ? cfg->timeout_ms : 180;
    if (ms < 1) ms = 1;
    k_work_reschedule(&st->release_work, K_MSEC(ms));
}

static void force_release_state(const struct behavior_sensor_hold_step_rotate_config *cfg,
                                struct hold_state *st,
                                bool cancel_timer) {
    ARG_UNUSED(cfg);

    if (!st->active) {
        st->step_count = 0;
        if (cancel_timer) {
            k_work_cancel_delayable(&st->release_work);
        }
        return;
    }

    struct zmk_behavior_binding_event ev = {
        .position = st->last_position,
        .layer = st->last_layer,
        .timestamp = (uint32_t)k_uptime_get(),
    };

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    ev.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    enqueue_release(&ev, st->active_binding);
    st->active = false;
    st->step_count = 0;

    if (cancel_timer) {
        k_work_cancel_delayable(&st->release_work);
    }
}

static bool is_allowed_key(const struct behavior_sensor_hold_step_rotate_config *cfg,
                           uint16_t usage_page, uint16_t usage_id) {
    /* allow-list 未指定なら「何も許可しない」（＝押したら解除） */
    if (cfg->allow_count == 0) {
        return false;
    }
    for (int i = 0; i < cfg->allow_count; i++) {
        if (cfg->allow_list[i].page == usage_page && cfg->allow_list[i].id == usage_id) {
            return true;
        }
    }
    return false;
}

/* ---- timeout handler ---- */

static void release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct hold_state *st = CONTAINER_OF(dwork, struct hold_state, release_work);

    if (!st->active) {
        return;
    }

    struct zmk_behavior_binding_event ev = {
        .position = st->last_position,
        .layer = st->last_layer,
        .timestamp = (uint32_t)k_uptime_get(),
    };

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    ev.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    enqueue_release(&ev, st->active_binding);
    st->active = false;
    st->step_count = 0;
}

/* ---- quick-release listener ---- */

static int hold_step_quick_release_listener(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev == NULL || !ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* modifier 単体は基本スルー（誤爆しがち） */
    if (is_mod(ev->usage_page, ev->keycode)) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    for (int di = 0; di < ARRAY_SIZE(devs); di++) {
        const struct device *dev = devs[di];
        const struct behavior_sensor_hold_step_rotate_config *cfg = dev->config;
        struct behavior_sensor_hold_step_rotate_data *data = dev->data;

        if (!cfg->quick_release) {
            continue;
        }

        /* 許可キーなら継続 */
        if (is_allowed_key(cfg, ev->usage_page, ev->keycode)) {
            continue;
        }

        /* 許可外キーが押された → 許可レイヤーの active hold を解除 */
        for (int si = 0; si < ZMK_KEYMAP_SENSORS_LEN; si++) {
            for (int ly = 0; ly < ZMK_KEYMAP_LAYERS_LEN; ly++) {
                if (!layer_allowed(cfg, (uint8_t)ly)) continue;

                data->pending_dir[si][ly] = HOLD_DIR_NONE;

                struct hold_state *st = &data->state[si][ly];
                if (st->active) {
                    force_release_state(cfg, st, true);
                } else {
                    st->step_count = 0;
                }
            }
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(behavior_sensor_hold_step_rotate_quick_release, hold_step_quick_release_listener);
ZMK_SUBSCRIPTION(behavior_sensor_hold_step_rotate_quick_release, zmk_keycode_state_changed);

/* ---- behavior implementation ---- */

static int accept_data(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event,
                       const struct zmk_sensor_config *sensor_config,
                       size_t channel_data_size,
                       const struct zmk_sensor_channel_data *channel_data) {
    ARG_UNUSED(binding);
    ARG_UNUSED(sensor_config);
    ARG_UNUSED(channel_data_size);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_hold_step_rotate_config *cfg = dev->config;
    struct behavior_sensor_hold_step_rotate_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    /* 非許可レイヤーでは状態を残さない */
    if (!layer_allowed(cfg, (uint8_t)event.layer)) {
        data->pending_dir[sensor_index][event.layer] = HOLD_DIR_NONE;
        data->state[sensor_index][event.layer].step_count = 0;
        return 0;
    }

    const struct sensor_value v = channel_data[0].value;

    /* val1==0ならval2をtickとみなす */
    int delta = (v.val1 == 0) ? v.val2 : v.val1;

    if (delta > 0) {
        data->pending_dir[sensor_index][event.layer] = HOLD_DIR_CW;
    } else if (delta < 0) {
        data->pending_dir[sensor_index][event.layer] = HOLD_DIR_CCW;
    } else {
        data->pending_dir[sensor_index][event.layer] = HOLD_DIR_NONE;
    }

    return 0;
}

static int process(struct zmk_behavior_binding *binding,
                   struct zmk_behavior_binding_event event,
                   enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_hold_step_rotate_config *cfg = dev->config;
    struct behavior_sensor_hold_step_rotate_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);
    struct hold_state *st = &data->state[sensor_index][event.layer];

    /* 非許可レイヤーは握りつぶし＆安全解除 */
    if (!layer_allowed(cfg, (uint8_t)event.layer)) {
        if (st->active) {
            force_release_state(cfg, st, true);
        } else if (st->inited) {
            st->step_count = 0;
        }
        data->pending_dir[sensor_index][event.layer] = HOLD_DIR_NONE;
        return ZMK_BEHAVIOR_OPAQUE;
    }

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->pending_dir[sensor_index][event.layer] = HOLD_DIR_NONE;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    enum hold_dir dir = data->pending_dir[sensor_index][event.layer];
    data->pending_dir[sensor_index][event.layer] = HOLD_DIR_NONE;

    if (dir == HOLD_DIR_NONE) {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    event.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    struct zmk_behavior_binding hold_next =
        (dir == HOLD_DIR_CW) ? cfg->hold_cw : cfg->hold_ccw;

    struct zmk_behavior_binding step_binding =
        (dir == HOLD_DIR_CW) ? cfg->step_cw : cfg->step_ccw;

    if (!st->inited) {
        k_work_init_delayable(&st->release_work, release_work_handler);
        st->inited = true;
    }

    st->last_position = event.position;
    st->last_layer = (uint8_t)event.layer;

    /* ---- step group ---- */
    uint16_t n = cfg->step_group_size ? cfg->step_group_size : 1;
    st->step_count++;
    if ((st->step_count % n) == 0) {
        enqueue_tap(&event, step_binding);
    }

    /* ---- hold ---- */
    if (!st->active) {
        st->active = true;
        st->active_binding = hold_next;
        enqueue_press(&event, st->active_binding);
        arm_timeout(cfg, st);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    if (cfg->direction_hold_mode == HOLD_MODE_SWITCH) {
        if (!binding_equal(st->active_binding, hold_next)) {
            enqueue_release(&event, st->active_binding);
            st->active_binding = hold_next;
            enqueue_press(&event, st->active_binding);
        }
    } else {
        /* STICKY: 逆回転でもholdは維持 */
    }

    arm_timeout(cfg, st);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .sensor_binding_accept_data = accept_data,
    .sensor_binding_process = process,
};

static int init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

/* ---- devicetree transform helpers ---- */

#define _BINDING_ENTRY(idx, inst)                                                                    \
    {                                                                                                \
        .behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(inst, bindings, idx)),                 \
        .param1 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(inst, bindings, idx, param1), (0),         \
                              (DT_INST_PHA_BY_IDX(inst, bindings, idx, param1))),                   \
        .param2 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(inst, bindings, idx, param2), (0),         \
                              (DT_INST_PHA_BY_IDX(inst, bindings, idx, param2))),                   \
    }

/* ---- layers mask (DT layers array -> uint32_t) ----
 * 重要: DT_INST_PROP_BY_IDX を使う（node_idを渡さない）
 */
#define _LAYER_BIT(i, inst) (1u << DT_INST_PROP_BY_IDX(inst, layers, i))

#define LAYER_MASK_FROM_INST(inst)                                                                   \
    COND_CODE_0(DT_INST_NODE_HAS_PROP(inst, layers),                                                  \
                (0u),                                                                                \
                (LISTIFY(DT_INST_PROP_LEN(inst, layers), _LAYER_BIT, (|), inst) | 0u))

/* ---- allow-list (DT quick_release_allow_list -> allow_item[]) ----
 * 重要: DT_INST_PROP_BY_IDX を使う（node_idを渡さない）
 * 値は “packed keycode” なので page/id に分解する
 */
#define _ALLOW_ITEM(i, inst)                                                                         \
    {                                                                                                \
        .page = (uint16_t)ZMK_HID_USAGE_PAGE(DT_INST_PROP_BY_IDX(inst, quick_release_allow_list, i)),\
        .id   = (uint16_t)ZMK_HID_USAGE_ID(DT_INST_PROP_BY_IDX(inst, quick_release_allow_list, i)),  \
    }

#define ALLOW_COUNT_FROM_INST(inst)                                                                  \
    COND_CODE_0(DT_INST_NODE_HAS_PROP(inst, quick_release_allow_list),                                \
                (0),                                                                                 \
                (DT_INST_PROP_LEN(inst, quick_release_allow_list)))

#define ALLOW_LIST_FROM_INST(inst)                                                                   \
    COND_CODE_0(DT_INST_NODE_HAS_PROP(inst, quick_release_allow_list),                                \
                (),                                                                                  \
                (LISTIFY(DT_INST_PROP_LEN(inst, quick_release_allow_list), _ALLOW_ITEM, (,), inst)))

/* ---- instantiate ---- */

#define INST(n)                                                                                      \
    static struct behavior_sensor_hold_step_rotate_data data_##n = {};                               \
    static const struct behavior_sensor_hold_step_rotate_config cfg_##n = {                          \
        .hold_cw  = _BINDING_ENTRY(0, n),                                                             \
        .hold_ccw = _BINDING_ENTRY(1, n),                                                             \
        .step_cw  = _BINDING_ENTRY(2, n),                                                             \
        .step_ccw = _BINDING_ENTRY(3, n),                                                             \
        .timeout_ms = DT_INST_PROP_OR(n, timeout_ms, 180),                                            \
        .step_group_size = DT_INST_PROP_OR(n, step_group_size, 5),                                   \
        .direction_hold_mode = DT_INST_PROP_OR(n, direction_hold_mode, 0),                            \
        .layers_mask = LAYER_MASK_FROM_INST(n),                                                       \
        .quick_release = DT_INST_PROP_OR(n, quick_release, 0),                                        \
        .allow_count = (uint8_t)ALLOW_COUNT_FROM_INST(n),                                             \
        .allow_list = { ALLOW_LIST_FROM_INST(n) },                                                    \
    };                                                                                               \
    BEHAVIOR_DT_INST_DEFINE(                                                                          \
        n, init, NULL, &data_##n, &cfg_##n,                                                           \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                             \
        &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
