/*
 * Sensors.c
 *
 *  Created on: May 18, 2020
 *      Author: khmin
 */
#include "main.h"
//#include "i2c.h"
//#include "usart.h"
//#include "gpio.h"
//#include "config.h"
//#include "def.h"
//#include "types.h"
//#include "Alarms.h"
//#include "EEPROM.h"
#include "IMU.h"
#include "ICM20602.h"

void ACC_init();

// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION)
  #define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = X; imu.accADC[PITCH]  = Y; imu.accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION)
  #define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = X; imu.gyroADC[PITCH] = Y; imu.gyroADC[YAW] = Z;}
#endif

/*** I2C address ***/
#if !defined(MPU6050_ADDRESS)
  #define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
  //#define MPU6050_ADDRESS     0x69 // address pin AD0 high (VCC)
#endif

//MPU6050 Gyro LPF setting
#if defined(MPU6050_LPF_256HZ) || defined(MPU6050_LPF_188HZ) || defined(MPU6050_LPF_98HZ) || defined(MPU6050_LPF_42HZ) || defined(MPU6050_LPF_20HZ) || defined(MPU6050_LPF_10HZ) || defined(MPU6050_LPF_5HZ)
  #if defined(MPU6050_LPF_20HZ)
    #define MPU6050_DLPF_CFG   4
  #endif
#else
  #define MPU6050_DLPF_CFG   0
#endif
//
//MPU3050 Gyro LPF setting
uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;

// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************

// ****************
// GYRO common part
// ****************
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis, tilt=0;

  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      // Reset g[axis] at start of calibration
      if (calibratingG == 512) {
        g[axis]=0;

        #if defined(GYROCALIBRATIONFAILSAFE)
            previousGyroADC[axis] = imu.gyroADC[axis];
          }
          if (calibratingG % 10 == 0) {
            if(abs(imu.gyroADC[axis] - previousGyroADC[axis]) > 8) tilt=1;
            previousGyroADC[axis] = imu.gyroADC[axis];
       #endif
      }
      // Sum up 512 readings
      g[axis] +=imu.gyroADC[axis];
      // Clear global variables for next reading
      imu.gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) {
        gyroZero[axis]=(g[axis]+256)>>9;
      #if defined(BUZZER)
        alarmArray[7] = 4;
      #else
//        blinkLED(10,15,1); //the delay causes to beep the buzzer really long
      #endif
      }
    }
    #if defined(GYROCALIBRATIONFAILSAFE)
      if(tilt) {
        calibratingG=1000;
//        LEDPIN_ON;
      } else {
        calibratingG--;
//        LEDPIN_OFF;
      }
//      Serial2.print("tilt= "); Serial2.println(tilt);
      return;
    #else
      calibratingG--;
    #endif

  }


#ifdef MMGYRO
#else
  for (axis = 0; axis < 3; axis++) {
    imu.gyroADC[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    imu.gyroADC[axis] = constrain(imu.gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
#endif
    previousGyroADC[axis] = imu.gyroADC[axis];
  }


}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 512) a[axis]=0;
      // Sum up 512 readings
      a[axis] +=imu.accADC[axis];
      // Clear global variables for next reading
      imu.accADC[axis]=0;
      global_conf.accZero[axis]=0;
    }
    // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      global_conf.accZero[ROLL]  = (a[ROLL]+256)>>9;
      global_conf.accZero[PITCH] = (a[PITCH]+256)>>9;
      global_conf.accZero[YAW]   = ((a[YAW]+256)>>9)-ACC_1G; // for nunchuk 200=1G
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
//      writeGlobalSet(1); // write accZero in EEPROM
    }
    calibratingA--;
  }
  imu.accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  imu.accADC[PITCH] -=  global_conf.accZero[PITCH];
  imu.accADC[YAW]   -=  global_conf.accZero[YAW] ;

}

// ************************************************************************************************************
// I2C Gyroscope and Accelerometer MPU6050
// ************************************************************************************************************
//#if defined(MPU6050_AA)

void Gyro_init() {
/*  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  // enable I2C bypass for AUX I2C
  #if defined(MAG)
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);           //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
  #endif*/
}

void Gyro_getADC () {
	if(ICM20602_DataReady() == 1)
	{
		ICM20602_Get3AxisGyroRawData(&ICM20602.gyro_x_raw);

		//MPU6050_Read_Gyro();
		GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])>>2 ,
                    ((rawADC[4]<<8) | rawADC[5])>>2 );
	}
	GYRO_Common();

}

void ACC_init () {
//  i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
}

void ACC_getADC () {
	if(ICM20602_DataReady() == 1)
	{
		ICM20602_Get3AxisAccRawData(&ICM20602.acc_x_raw);

		//MPU6050_Read_Accel ();
		ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>3 ,
                   ((rawADC[2]<<8) | rawADC[3])>>3 ,
                   ((rawADC[4]<<8) | rawADC[5])>>3 );
	}
	ACC_Common();

}

//#endif


void initSensors() {
//  delay(200);
//  POWERPIN_ON;
//  delay(100);
//  i2c_init();
//  delay(100);
//  if (GYRO) Gyro_init();
//  if (ACC) ACC_init();
//  MPU6050_Init ();
  f.I2C_INIT_DONE = 1;
}

