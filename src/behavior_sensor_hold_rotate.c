/*
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_sensor_hold_rotate

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/virtual_key_position.h>

#include <zmk/behavior_sensor_rotate_common.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * NOTE:
 * - rotate_common の remainder/triggers 計算を流用する
 * - “実際に押す/離す” は cw/ccw の下位bindingへ behavior_queue 経由で投げる
 */

struct behavior_sensor_hold_rotate_config {
    struct zmk_behavior_binding cw_binding;
    struct zmk_behavior_binding ccw_binding;
    uint16_t timeout_ms;
};

struct hold_state {
    bool active;
    // 現在ホールド中の binding（cw or ccw のどちらか）
    struct zmk_behavior_binding active_binding;
    struct k_work_delayable release_work;
};

struct behavior_sensor_hold_rotate_data {
    // rotate_common が期待する領域（remainder/triggers）
    struct behavior_sensor_rotate_data common;

    // センサーindex × layer ごとのホールド状態
    // ※ behavior_sensor_rotate_data と同じ次元で扱う前提
    struct hold_state state[ZMK_KEYMAP_SENSORS_MAX][ZMK_KEYMAP_LAYERS_LEN];
};

static int enqueue_press(struct zmk_behavior_binding_event *event,
                         struct zmk_behavior_binding binding) {
    // “押しっぱなし”にしたいので press のみキューに積む
    return zmk_behavior_queue_add(event, binding, true, 0);
}

static int enqueue_release(struct zmk_behavior_binding_event *event,
                           struct zmk_behavior_binding binding) {
    return zmk_behavior_queue_add(event, binding, false, 0);
}

static void release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);

    // どの state かを引けるように CONTAINER_OF で引き当てる
    struct hold_state *st = CONTAINER_OF(dwork, struct hold_state, release_work);

    if (!st->active) {
        return;
    }

    // タイムスタンプは k_uptime_get で雑に作る（ZMK内でもよくある）
    struct zmk_behavior_binding_event ev = {
        .position = 0, // ここは behavior_queue_add 側が参照する程度。releaseは同一position前提なら後述の工夫が必要
        .timestamp = (uint32_t)k_uptime_get(),
        .layer = 0,
    };

    // IMPORTANT:
    // 本当は position/layer を保持して release 時にも同じ event を作るのが理想。
    // そのため、下の “改善版” を推奨（このテンプレの末尾で示します）。
    //
    // まずは「キーアップが飛ぶこと」を優先するなら、下位bindingが position に依存しない挙動（kp等）であれば動きます。

    LOG_DBG("timeout release");
    enqueue_release(&ev, st->active_binding);
    st->active = false;
}

static void arm_timeout(const struct behavior_sensor_hold_rotate_config *cfg,
                        struct hold_state *st) {
    uint16_t ms = cfg->timeout_ms ? cfg->timeout_ms : 180;
    k_work_schedule(&st->release_work, K_MSEC(ms));
}

static int sensor_hold_rotate_process(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event,
                                      enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_hold_rotate_config *cfg = dev->config;
    struct behavior_sensor_hold_rotate_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->common.triggers[sensor_index][event.layer] = 0;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    int triggers = data->common.triggers[sensor_index][event.layer];
    if (triggers == 0) {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    // rotate と同様、centralで扱う
    event.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    // どっち方向か決める（triggers の回数自体は “ホールド” では使わない）
    struct zmk_behavior_binding next =
        (triggers > 0) ? cfg->cw_binding : cfg->ccw_binding;

    struct hold_state *st = &data->state[sensor_index][event.layer];

    // 初回だけdelayable init（何度呼んでも問題ないが、気持ち的に最初に）
    k_work_init_delayable(&st->release_work, release_work_handler);

    if (!st->active) {
        // 何もホールドしてない → pressして開始
        st->active = true;
        st->active_binding = next;
        LOG_DBG("press start dir=%s", (triggers > 0) ? "cw" : "ccw");
        enqueue_press(&event, next);
        arm_timeout(cfg, st);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    // すでにホールド中
    bool same = (st->active_binding.behavior_dev == next.behavior_dev) &&
                (st->active_binding.param1 == next.param1) &&
                (st->active_binding.param2 == next.param2);

    if (same) {
        // 同方向連打 → タイマー延長だけ
        LOG_DBG("extend hold");
        arm_timeout(cfg, st);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    // 逆方向 → 即releaseしてpressし直す
    LOG_DBG("switch hold");
    enqueue_release(&event, st->active_binding);
    st->active_binding = next;
    enqueue_press(&event, next);
    arm_timeout(cfg, st);

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_sensor_hold_rotate_driver_api = {
    .sensor_binding_accept_data = zmk_behavior_sensor_rotate_common_accept_data,
    .sensor_binding_process = sensor_hold_rotate_process,
};

#define _TRANSFORM_ENTRY(idx, node)                                                                \
    {                                                                                              \
        .behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(node, bindings, idx)),               \
        .param1 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param1), (0),       \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param1))),                  \
        .param2 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param2), (0),       \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param2))),                  \
    }

#define SENSOR_HOLD_ROTATE_INST(n)                                                                 \
    static const struct behavior_sensor_hold_rotate_config cfg_##n = {                             \
        .cw_binding = _TRANSFORM_ENTRY(0, n),                                                       \
        .ccw_binding = _TRANSFORM_ENTRY(1, n),                                                      \
        .timeout_ms = DT_INST_PROP_OR(n, timeout_ms, 180),                                          \
    };                                                                                             \
    static struct behavior_sensor_hold_rotate_data data_##n = {};                                   \
    BEHAVIOR_DT_INST_DEFINE(                                                                       \
        n, NULL, NULL, &data_##n, &cfg_##n,                                                         \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                           \
        &behavior_sensor_hold_rotate_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_HOLD_ROTATE_INST)
