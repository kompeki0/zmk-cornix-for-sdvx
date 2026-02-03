/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_sensor_hold_rotate

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/virtual_key_position.h>
#include <zmk/events/position_state_changed.h>

#ifndef ZMK_KEYMAP_SENSORS_LEN
#define ZMK_KEYMAP_SENSORS_LEN 0
#endif

#ifndef ZMK_KEYMAP_LAYERS_LEN
#define ZMK_KEYMAP_LAYERS_LEN 1
#endif


LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * 目的:
 * - エンコーダの step/tick 入力列を「長押し」に変換する
 * - 同方向連続: 初回だけ press, 以降はタイムアウト延長のみ
 * - 逆方向: 旧release→新press
 * - 無入力 timeout-ms: release
 *
 * NOTE:
 * - 角度積算/trigger_per_rotation などの高度な処理は捨てて「方向」だけ見ます。
 * - ほとんどの encoder sensor は回転で +/− のdeltaを返すのでそれで十分です。
 */

struct behavior_sensor_hold_rotate_config {
    struct zmk_behavior_binding cw_binding;
    struct zmk_behavior_binding ccw_binding;
    uint16_t timeout_ms;
};

enum hold_dir {
    HOLD_DIR_NONE = 0,
    HOLD_DIR_CW = 1,
    HOLD_DIR_CCW = 2,
};

struct hold_state {
    bool inited;
    bool active;
    struct zmk_behavior_binding active_binding;

    int16_t last_position;
    uint8_t last_layer;

    struct k_work_delayable release_work;
};


struct behavior_sensor_hold_rotate_data {
    // accept_data で方向を貯めておく（processで消費）
    enum hold_dir pending_dir[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    struct hold_state state[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
};

static int enqueue_press(struct zmk_behavior_binding_event *event,
                         struct zmk_behavior_binding binding) {
    return zmk_behavior_queue_add(event, binding, true, 0);
}

static int enqueue_release(struct zmk_behavior_binding_event *event,
                           struct zmk_behavior_binding binding) {
    return zmk_behavior_queue_add(event, binding, false, 0);
}

static bool binding_equal(struct zmk_behavior_binding a, struct zmk_behavior_binding b) {
    return (a.behavior_dev == b.behavior_dev) && (a.param1 == b.param1) && (a.param2 == b.param2);
}

static void arm_timeout(const struct behavior_sensor_hold_rotate_config *cfg,
                        struct hold_state *st) {
    uint16_t ms = cfg->timeout_ms ? cfg->timeout_ms : 180;
    if (ms < 1) ms = 1;
    k_work_reschedule(&st->release_work, K_MSEC(ms));
}

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

    LOG_DBG("timeout release pos=%d layer=%d", ev.position, ev.layer);
    enqueue_release(&ev, st->active_binding);
    st->active = false;
}

static int accept_data(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event,
                       const struct zmk_sensor_config *sensor_config,
                       size_t channel_data_size,
                       const struct zmk_sensor_channel_data *channel_data) {
    ARG_UNUSED(binding);
    ARG_UNUSED(sensor_config);
    ARG_UNUSED(channel_data_size);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_sensor_hold_rotate_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    const struct sensor_value v = channel_data[0].value;

    // ZMK本体の互換コメントに合わせて val1==0 なら val2 を tick とみなす
    int delta = (v.val1 == 0) ? v.val2 : v.val1;

    if (delta > 0) {
        data->pending_dir[sensor_index][event.layer] = HOLD_DIR_CW;
    } else if (delta < 0) {
        data->pending_dir[sensor_index][event.layer] = HOLD_DIR_CCW;
    } else {
        data->pending_dir[sensor_index][event.layer] = HOLD_DIR_NONE;
    }

    LOG_DBG("accept pos=%d layer=%d val1=%d val2=%d delta=%d dir=%d",
            event.position, event.layer, v.val1, v.val2, delta,
            data->pending_dir[sensor_index][event.layer]);

    return 0;
}

static int process(struct zmk_behavior_binding *binding,
                   struct zmk_behavior_binding_event event,
                   enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_hold_rotate_config *cfg = dev->config;
    struct behavior_sensor_hold_rotate_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

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

    struct zmk_behavior_binding next =
        (dir == HOLD_DIR_CW) ? cfg->cw_binding : cfg->ccw_binding;

    struct hold_state *st = &data->state[sensor_index][event.layer];

    // 初回だけ init（毎回でも害はない）
    if (!st->inited) {
        k_work_init_delayable(&st->release_work, release_work_handler);
        st->inited = true;
    }

    // timeout release 用の情報を更新
    st->last_position = event.position;
    st->last_layer = event.layer;

    if (!st->active) {
        st->active = true;
        st->active_binding = next;
        LOG_DBG("press start dir=%s", (dir == HOLD_DIR_CW) ? "cw" : "ccw");
        enqueue_press(&event, next);
        arm_timeout(cfg, st);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    if (binding_equal(st->active_binding, next)) {
        LOG_DBG("extend hold");
        arm_timeout(cfg, st);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    LOG_DBG("switch hold");
    enqueue_release(&event, st->active_binding);
    st->active_binding = next;
    enqueue_press(&event, next);
    arm_timeout(cfg, st);

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .sensor_binding_accept_data = accept_data,
    .sensor_binding_process = process,
};

#define _TRANSFORM_ENTRY(idx, node)                                                                \
    {                                                                                              \
        .behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(node, bindings, idx)),               \
        .param1 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param1), (0),       \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param1))),                  \
        .param2 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param2), (0),       \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param2))),                  \
    }

#define INST(n)                                                                                     \
    static const struct behavior_sensor_hold_rotate_config cfg_##n = {                              \
        .cw_binding  = _TRANSFORM_ENTRY(0, n),                                                      \
        .ccw_binding = _TRANSFORM_ENTRY(1, n),                                                      \
        .timeout_ms  = DT_INST_PROP_OR(n, timeout_ms, 180),                                         \
    };                                                                                              \
    static struct behavior_sensor_hold_rotate_data data_##n = {};                                   \
    BEHAVIOR_DT_INST_DEFINE(                                                                        \
        n, NULL, NULL, &data_##n, &cfg_##n,                                                         \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                           \
        &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
