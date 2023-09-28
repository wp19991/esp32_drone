#include "hmc5883l.h"
#include "i2cdev.h"
#include "ledseq.h"
#include "motors.h"
#include "mpu6050.h"
#include "pm_esplane.h"
#include "spl06.h"

#include "esp_log.h"

static const char* TAG = "mpu6050_main";

void app_main() {
  // 初始化日志系统
  esp_log_level_set(TAG, ESP_LOG_INFO);

  motorsInit(15000);
  motorsTest();

  pmInit();

  ledseqInit();
  ledseqTest();

  i2cdevInit(I2C0_DEV);
  mpu6050Init(I2C0_DEV);

  //   if (mpu6050SelfTest() == true) {
  //     motorsSetRatio(0, 65535);
  //     vTaskDelay(500 / portTICK_PERIOD_MS);
  //     motorsSetRatio(0, 0);
  //   }
  mpu6050Reset();
  vTaskDelay(50 / portTICK_PERIOD_MS);
  // Activate mpu6050
  mpu6050SetSleepEnabled(false);
  // Delay until registers are reset
  vTaskDelay(100 / portTICK_PERIOD_MS);
  // Set x-axis gyro as clock source
  mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
  // Delay until clock is set and stable
  vTaskDelay(200 / portTICK_PERIOD_MS);
  // 使温度传感器
  mpu6050SetTempSensorEnabled(true);
  // 禁止中断
  mpu6050SetIntEnabled(false);
  // 将MAG和BARO连接到主I2C总线
  mpu6050SetI2CBypassEnabled(true);
  // 设置陀螺满量程
  mpu6050SetFullScaleGyroRange(0x30);
  // 设置加速度计满量程
  mpu6050SetFullScaleAccelRange(0x30);

  // Set digital low-pass bandwidth for gyro and acc
  // board ESP32_S2_DRONE_V1_2 has more vibrations, bandwidth should be lower

  // To low DLPF bandwidth might cause instability and decrease agility
  // but it works well for handling vibrations and unbalanced propellers
  // Set output rate (1): 1000 / (1 + 0) = 1000Hz
  mpu6050SetRate(0);
  mpu6050SetDLPFMode(MPU6050_DLPF_BW_42);

  motorsSetRatio(0, 10000);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  motorsSetRatio(0, 0);

  hmc5883lInit(I2C0_DEV);
  if (hmc5883lTestConnection() == true) {
    hmc5883lSetMode(QMC5883L_MODE_CONTINUOUS);  // 16bit 100Hz
    ESP_LOGI(TAG, "hmc5883l I2C connection [OK].\n");
  }

  if (SPL06Init(I2C0_DEV)) {
    ESP_LOGI(TAG, "SPL06 I2C connection [OK].\n");
  }

  int16_t axi16, ayi16, azi16;
  int16_t gxi16, gyi16, gzi16;
  float axf, ayf, azf;
  float gxf, gyf, gzf;
  float gRange, aRange;
  uint32_t scrap;

  aRange = mpu6050GetFullScaleAccelGPL();
  gRange = mpu6050GetFullScaleGyroDPL();

  // First values after startup can be read as zero. Scrap a couple to be sure.
  for (scrap = 0; scrap < 10000; scrap++) {
    mpu6050GetMotion6(&axi16, &ayi16, &azi16, &gxi16, &gyi16, &gzi16);
    // First measurement
    gxf = gxi16 * gRange;
    gyf = gyi16 * gRange;
    gzf = gzi16 * gRange;
    axf = axi16 * aRange;
    ayf = ayi16 * aRange;
    azf = azi16 * aRange;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 打印浮点数值
    ESP_LOGI(TAG,
             "Iteration %u: axi16 = %d, ayi16 = %d, azi16 = %d, gxi16 = %d, "
             "gyi16 = %d, gzi16 = %d",
             (unsigned int)scrap, axi16, ayi16, azi16, gxi16, gyi16, gzi16);
    ESP_LOGI(TAG,
             "Iteration %u: axf = %f, ayf = %f, azf = %f, gxf = %f, gyf = %f, "
             "gzf = %f",
             (unsigned int)scrap, axf, ayf, azf, gxf, gyf, gzf);
    if (ayi16 < 0) {
      ayi16 = -ayi16;
    }
    if (axi16 < 0) {
      axi16 = -axi16;
    }
    // if(azi16<0){ azi16=-azi16;}
    motorsSetRatio(0, ayi16);
    motorsSetRatio(1, axi16);
    // motorsSetRatio(2, azi16);
  }
  motorsSetRatio(0, 0);
}