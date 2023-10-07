/**
 * Crazyflie control firmware
 *
 *
 * imu.h - inertial measurement unit header file
 */
#ifndef IMU_H_
#define IMU_H_
#include <stdbool.h>
#include "filter.h"
#include "imu_types.h"

/**
 * IMU update frequency dictates the overall update frequency.
 */
#define IMU_UPDATE_FREQ   500
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)

/**
 * Set ACC_WANTED_LPF1_CUTOFF_HZ to the wanted cut-off freq in Hz.
 * The highest cut-off freq that will have any affect is fs /(2*pi).
 * E.g. fs = 350 Hz -> highest cut-off = 350/(2*pi) = 55.7 Hz -> 55 Hz
 */
#define IMU_ACC_WANTED_LPF_CUTOFF_HZ  4
/**
 * Attenuation should be between 1 to 256.
 *
 * f0 = fs / 2*pi*attenuation ->
 * attenuation = fs / 2*pi*f0
 */
#define IMU_ACC_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_ACC_WANTED_LPF_CUTOFF_HZ))
#define IMU_ACC_IIR_LPF_ATT_FACTOR  (int)(((1<<IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5)

void imu6Init(void);
bool imu6Test(void);
bool imu6ManufacturingTest(void);
void imu6Read(Axis3f* gyro, Axis3f* acc);
void imu9Read(Axis3f* gyroOut, Axis3f* accOut, Axis3f* magOut);
bool imu6IsCalibrated(void);
bool imuHasBarometer(void);
bool imuHasMangnetometer(void);



#endif /* IMU_H_ */
