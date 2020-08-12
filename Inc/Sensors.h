/*
 * Sensors.h
 *
 *  Created on: May 18, 2020
 *      Author: khmin
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

/*#if defined(MMA7455) || defined(MMA8451Q) || defined(ADXL345) || \
    defined(BMA180) || defined(BMA280) || defined(BMA020) || defined(NUNCHACK) || \
    defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(ADCACC) || \
    defined(MPU6050_AA) || defined(LSM330) || defined(NUNCHUCK)*/
void ACC_getADC ();
//#endif

/*#if defined(L3G4200D) || defined(ITG3200) || defined(MPU6050_AA) || defined(LSM330) || \
    defined(MPU3050) || defined(WMP) || defined(NUNCHUCK)*/
void Gyro_getADC ();
//#endif

/*#if MAG
uint8_t Mag_getADC();
#endif

#if defined(BMP085) || defined(MS561101BA)
uint8_t Baro_update();
#endif

#if SONAR
void Sonar_update();
#endif
*/

extern uint8_t rawADC[6];

void initSensors();

void MPU6050_Init ();
void MPU6050_Read_Accel (void);
void MPU6050_Read_Gyro (void);


#endif /* INC_SENSORS_H_ */
