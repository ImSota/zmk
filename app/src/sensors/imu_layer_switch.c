#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/layers.h>
#include <zmk/sensors.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static struct sensor_value ref_orientation[3];
static const struct device *bmi270;

int imu_check_orientation_and_switch_layer() {
    struct sensor_value accel[3];
    if (sensor_sample_fetch(bmi270) < 0) {
        LOG_ERR("Sensor fetch failed");
        return -1;
    }

    if (sensor_channel_get(bmi270, SENSOR_CHAN_ACCEL_XYZ, accel) < 0) {
        LOG_ERR("Sensor read failed");
        return -1;
    }

    // 基準と現在のX軸を比較して、90度以上傾いていたらレイヤー切り替え
    float delta = accel[0].val1 - ref_orientation[0].val1;

    if (delta > 6.0 || delta < -6.0) {
        zmk_layer_activate(1);  // 1番レイヤーをアクティブに
    } else {
        zmk_layer_deactivate(1);
    }

    return 0;
}

int zmk_imu_sensor_init() {
    bmi270 = device_get_binding("BMI270");
    if (!bmi270) {
        LOG_ERR("Cannot find BMI270");
        return -1;
    }

    // 初期姿勢を記録
    if (sensor_sample_fetch(bmi270) < 0 ||
        sensor_channel_get(bmi270, SENSOR_CHAN_ACCEL_XYZ, ref_orientation) < 0) {
        LOG_ERR("Initial sample failed");
        return -1;
    }

    return 0;
}

SYS_INIT(zmk_imu_sensor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
