#include "mpconfigboard.h"

#include "commander.h"
#include "i2cdev.h"
#include "pm_esplane.h"
#include "sensors_mpu6050_spl06.h"
#include "stabilizer.h"
#include "state_estimator.h"
#include "system_int.h"

#include "esp_log.h"

static bool isOffse = 0;
static bool is_init = 0;
static uint16_t lastThrust;
static ctrlVal_t wifiCtrl; /*发送到commander姿态控制数据*/

static const char* TAG = "mpu6050_main";

static float limit(float value, float min, float max) {
    if (value > max) {
        value = max;
    } else if (value < min) {
        value = min;
    }
    return value;
}

void drone_stop(void) {
    setCommanderEmerStop(true);
    setCommanderKeyFlight(false);
    setCommanderKeyland(false);
}

void drone_landing(void) {
    setCommanderKeyFlight(false);
    setCommanderKeyland(true);
}

void drone_take_off(void) {
    setLandingDis(0);

    if (getCommanderKeyFlight() != true) {
        if (!isOffse) {
            attitude_t attitude;
            getAttitudeData(&attitude);

            wifiCtrl.trimPitch = (attitude.pitch * 0.3);
            wifiCtrl.trimRoll = (attitude.roll * 0.95);

            isOffse = 1;
        }
        setCommanderCtrlMode(1);
        setCommanderKeyFlight(true);
        setCommanderKeyland(false);
    }
}

void drone_trim(void) {
    float rol = (0) / 100.0f;
    float pit = (0) / 100.0f;

    wifiCtrl.trimRoll += limit(rol, -10.0, 10.0);
    wifiCtrl.trimPitch += limit(pit, -10.0, 10.0);
}

void drone_control(void) {
    float fTemp = 0.0f;
    fTemp = ((float)(0) / 10.0f);
    wifiCtrl.roll = limit(fTemp, -10, 10);

    // wifiCtrl.roll += wifiCtrl.trimRoll;

    fTemp = ((float)(0) / 10.0f);
    wifiCtrl.pitch = limit(fTemp, -10, 10);

    // wifiCtrl.pitch -= wifiCtrl.trimPitch;

    fTemp = ((float)(0));
    wifiCtrl.yaw = limit(fTemp, -200, 200);

    fTemp = ((float)(100) / 2.0f);
    fTemp = limit(fTemp, 3, 100);
    wifiCtrl.thrust = (uint16_t)(fTemp * 655.35f);

    if ((wifiCtrl.thrust == 32768) && lastThrust < 10000) { /*手动飞切换到定高*/
        setCommanderCtrlMode(1);
        setCommanderKeyFlight(false);
        setCommanderKeyland(false);
    } else if (wifiCtrl.thrust == 0 && lastThrust > 256) { /*定高切换成手动飞*/
        setCommanderCtrlMode(0);
        wifiCtrl.thrust = 0;
    }
    lastThrust = wifiCtrl.thrust;
    flightCtrldataCache(WIFI, wifiCtrl);
}

void read_states(void) {
    attitude_t attitude;
    getAttitudeData(&attitude);
    uint16_t bat = (uint16_t)(pmMeasureExtBatteryVoltage() * 100.0f);
    int32_t FusedHeight = (int32_t)(getFusedHeight());
    int32_t tuple[9];
    tuple[0] = attitude.roll * 100;
    tuple[1] = attitude.pitch * 100;
    tuple[2] = attitude.yaw * 100;
    tuple[3] = wifiCtrl.roll * 100;
    tuple[4] = wifiCtrl.pitch * 100;
    tuple[5] = wifiCtrl.yaw * 100;
    tuple[6] = ((wifiCtrl.thrust / 655.35) + 0.5);
    tuple[7] = bat;
    tuple[8] = FusedHeight;
}

void read_accelerometer(void) {
    Axis3i16 acc, gyro, mag;
    getSensorRawData(&acc, &gyro, &mag);
    int32_t tuple[6];
    tuple[0] = gyro.y;
    tuple[1] = gyro.z;
    tuple[2] = gyro.z;
    tuple[3] = acc.y;
    tuple[4] = acc.x;
    tuple[5] = acc.z;
}

void read_compass(void) {
    Axis3i16 acc, gyro, mag;
    int32_t tuple[3];
    getSensorRawData(&acc, &gyro, &mag);
    tuple[0] = mag.x;
    tuple[1] = mag.y;
    tuple[2] = mag.z;
}


void read_air_pressure(void) {
    float temp, press, asl;
    int16_t tuple[2];
    getPressureRawData(&temp, &press, &asl);

    tuple[0] = press * 100;
    tuple[1] = (int16_t)(temp * 100);
}

void read_cal_data(void) {
    Axis3f variance;
    int16_t tuple[3];
    readBiasVlue(&variance);

    tuple[0] = variance.x;
    tuple[1] = variance.y;
    tuple[2] = variance.z;
}

static void InitDrone(void) {
    static uint16_t lastThrust;

    if (is_init)
        return;
    systemInit();

    wifiCtrl.roll = 0;       /*roll: ±9.5 ±19.2 ±31.7*/
    wifiCtrl.pitch = 0;      /*pitch:±9.5 ±19.2 ±31.7*/
    wifiCtrl.yaw = 0;        /*yaw : ±203.2*/
    wifiCtrl.thrust = 32768; /*thrust :0~63356*/

    if (wifiCtrl.thrust == 32768 && lastThrust < 10000) /*手动飞切换到定高*/
    {
        setCommanderCtrlMode(1);
        setCommanderKeyFlight(false);
        setCommanderKeyland(false);
    } else if (wifiCtrl.thrust == 0 && lastThrust > 256) /*定高切换成手动飞*/
    {
        setCommanderCtrlMode(0);
        wifiCtrl.thrust = 0;
    }
    lastThrust = wifiCtrl.thrust;

    flightCtrldataCache(WIFI, wifiCtrl);
}

void drone_make_new() {
    if (0 == 1) {
        setCommanderFlightmode(0);
    } else {
        setCommanderFlightmode(1);
    }

    setPrintf(false);

    InitDrone();

    // drone_obj_t *drone_type;
    // drone_type = m_new_obj(drone_obj_t);
    // drone_type->base.type = &drone_drone_type;
    is_init = true;
}

void app_main() {
    // systemInit();
    // motorsTest();
    // ESP_LOGI(TAG, "systemInit [OK].\n");

    // init
    drone_make_new();
    ESP_LOGI(TAG, "init ok\n");

    ESP_LOGI(TAG, "take off\n");

    vTaskDelay(15000 / portTICK_PERIOD_MS);

    // take off
    drone_take_off();

    // controll
    // drone_control();

    vTaskDelay(3000 / portTICK_PERIOD_MS);  // 飞行3秒
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "stop\n");
    // 停止并且降落
    drone_stop();

    ESP_LOGI(TAG, "fly ok\n");
}