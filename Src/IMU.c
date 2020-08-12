/*#include "config.h"
#include "def.h"
#include "types.h"*/
#include "main.h"
#include "IMU.h"
#include "Sensors.h"
//#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void getEstimatedAttitude();

void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  int16_t gyroADCp[3];
  int16_t gyroADCinter[3];
  static uint32_t timeInterleave = 0;

  //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
  //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
  //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
  #if defined(NUNCHUCK)
  #else
    #if ACC
      ACC_getADC();
      getEstimatedAttitude();
    #endif
    #if GYRO
      Gyro_getADC();
    #endif
/*    for (axis = 0; axis < 3; axis++)
      gyroADCp[axis] =  imu.gyroADC[axis];
//    timeInterleave=HAL_GetTick() * 1000;	//micros();
    annexCode();
    uint8_t t=0;
//    while((uint16_t)((HAL_GetTick() * 1000)-timeInterleave)<650) t=1; //micros()//empirical, interleaving delay between 2 consecutive reads
//    if (!t) annex650_overrun_count++;
    #if GYRO
      Gyro_getADC();
    #endif
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  imu.gyroADC[axis]+gyroADCp[axis];
      // empirical, we take a weighted value of the current and the previous values
      imu.gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
      gyroADCprevious[axis] = gyroADCinter[axis]>>1;
      if (!ACC) imu.accADC[axis]=0;
    }*/
  #endif
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC
   Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. */
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 120.0f
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 700
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter
   Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 250

//****** end of advanced users settings *************
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

typedef struct fp_vector {		
  float X,Y,Z;		
} t_fp_vector_def;

typedef union {		
  float A[3];		
  t_fp_vector_def V;		
} t_fp_vector;

typedef struct int32_t_vector {
  int32_t X,Y,Z;
} t_int32_t_vector_def;

typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} t_int32_t_vector;

int16_t _atan2(int32_t y, int32_t x){
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) ){
     a = 573 * z / (1.0f + 0.28f * z * z);
   if (x<0) {
     if (y<0) a -= 1800;
     else a += 1800;
   }
  } else {
   a = 900 - 573 * z / (z * z + 0.28f);
   if (y<0) a -= 1800;
  }
  return a;
}

float InvSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
 // fp_vector v_tmp = *v;
  t_fp_vector_def v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}


static float accLPF[3] = {0, 0, ACC_1G};
static float invG; // 1/|G|

static t_fp_vector EstG;
static t_int32_t_vector EstG32;

void getEstimatedAttitude(){
  uint8_t axis;
  int32_t accMag = 0;
  float scale, deltaGyroAngle[3];
  uint8_t validAcc;
  
  static uint16_t previousT;

  uint16_t currentT = HAL_GetTick();	// * 1000;	//micros();

  scale = (currentT - previousT) * GYRO_SCALE; // GYRO_SCALE unit: radian/microsecond
  previousT = currentT;

  // Initialization
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = imu.gyroADC[axis]  * scale; // radian

    accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + imu.accADC[axis] * (1.0f/ACC_LPF_FACTOR);
    imu.accSmooth[axis] = accLPF[axis];
    
    accMag += (int32_t)imu.accSmooth[axis]*imu.accSmooth[axis] ;
  }

  rotateV(&EstG.V,deltaGyroAngle);

  accMag = accMag*100/((int32_t)ACC_1G*ACC_1G);
  validAcc = 72 < (uint16_t)accMag && (uint16_t)accMag < 133;
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  for (axis = 0; axis < 3; axis++) {
    if ( validAcc )
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + imu.accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float	
  }
  
  if ((int16_t)EstG32.A[2] > ACCZ_25deg)
    f.SMALL_ANGLES_25 = 1;
  else
    f.SMALL_ANGLES_25 = 0;

  // Attitude of the estimated vector
  int32_t sqGX_sqGZ = sq(EstG32.V.X) + sq(EstG32.V.Z);
  invG = InvSqrt(sqGX_sqGZ + sq(EstG32.V.Y));
  att.angle[ROLL]  = _atan2(EstG32.V.X , EstG32.V.Z);
  att.angle[PITCH] = _atan2(EstG32.V.Y , InvSqrt(sqGX_sqGZ)*sqGX_sqGZ);

}

#define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define BARO_TAB_SIZE   21

#define ACC_Z_DEADBAND (ACC_1G>>5) // was 40 instead of 32 now


#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }
