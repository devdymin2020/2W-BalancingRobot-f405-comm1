#ifndef DEF_H_
#define DEF_H_


// undefine stdlib's abs if encountered
//#ifdef abs
//#undef abs
//#endif

#define PI 3.1415926535897932384626433832795
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
//#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
//#define radians(deg) ((deg)*DEG_TO_RAD)
//#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
// Proc auto detection
//#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define PROMINI
//#endif
//  #define MEGA

/**************************************************************************************/
/***************             motor and servo numbers               ********************/
/**************************************************************************************/

  #define DYNBAL  0
  #define FLAP    0

  #define NUMBER_MOTOR     2

/**************************   atmega328P (Promini)  ************************************/
 /*   #define LEDPIN_PINMODE             pinMode (PC13, OUTPUT);
    #define LEDPIN_TOGGLE              digitalWrite(PC13, !digitalRead(PC13));  //PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
    #define LEDPIN_OFF                 digitalWrite(PC13, LOW); //PORTB &= ~(1<<5);
    #define LEDPIN_ON                  digitalWrite(PC13, HIGH); //PORTB |= (1<<5);

      #define BUZZERPIN_PINMODE         pinMode (PA5, OUTPUT);
        #define BUZZERPIN_ON            digitalWrite(PA5, HIGH); //PORTB |= 1;
        #define BUZZERPIN_OFF           digitalWrite(PA5, LOW); //PORTB &= ~1;
*/
  #define POWERPIN_PINMODE           ;
  #define POWERPIN_ON                ;
  #define POWERPIN_OFF               ;

//  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
//  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);

//  #define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
  #define SPEK_SERIAL_PORT           0
  //RX PIN assignment inside the port //for PORTD
  #define THROTTLEPIN                2
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     6
  #define AUX1PIN                    7
  #define AUX2PIN                    0 // optional PIN 8 or PIN 12
  #define AUX3PIN                    1 // unused 
  #define AUX4PIN                    3 // unused 
    
//  #define PCINT_PIN_COUNT            5
//  #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
//  #define PCINT_RX_PORT              PORTD
//  #define PCINT_RX_MASK              PCMSK2
//  #define PCIR_PORT_BIT              (1<<2)
//  #define RX_PC_INTERRUPT            PCINT2_vect
//  #define RX_PCINT_PIN_PORT          PIND
//  #define V_BATPIN                   PA3    // Analog PIN 3
//  #define PSENSORPIN                 PA2    // Analog PIN 2
  

/**************************************************************************************/
/***************      IMU Orientations and Sensor definitions      ********************/
/**************************************************************************************/
#if defined(GY_521)
  #define MPU6050
  #define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}
  #undef INTERNAL_I2C_PULLUPS
#endif

/**************************************************************************************/
/***************              Sensor Type definitions              ********************/
/**************************************************************************************/
//#if defined(MPU6050)
  #define ACC 1
//#endif

//#if defined(MPU6050)
  #define GYRO 1
//#endif

  #define MAG   0
  #define BARO  0
  #define GPS   0
  #define SONAR 0

#if defined(MPU6050)
  #if defined(FREEIMUv04)
//    #define ACC_1G 255
  #else
    #define ACC_1G 512
  #endif
#endif

#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)

#if defined(MPU6050)
  #define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits
#endif

/**************************************************************************************/
/***************      Multitype decleration for the GUI's          ********************/
/**************************************************************************************/
#define MULTITYPE 4 // BI

#if defined(SERIAL_SUM_PPM)
  #define RC_CHANS 12
#endif

/**************************************************************************************/
/***************               override defaults                   ********************/
/**************************************************************************************/

  /***************               pin assignments ?  ********************/
  #ifdef OVERRIDE_V_BATPIN
    #undef V_BATPIN
    #define V_BATPIN OVERRIDE_V_BATPIN
  #endif
  #ifdef OVERRIDE_BUZZERPIN_PINMODE
    #undef BUZZERPIN_PINMODE
    #undef BUZZERPIN_ON
    #undef BUZZERPIN_OFF
    #define BUZZERPIN_PINMODE OVERRIDE_BUZZERPIN_PINMODE
    #define BUZZERPIN_ON      OVERRIDE_BUZZERPIN_ON
    #define BUZZERPIN_OFF     OVERRIDE_BUZZERPIN_OFF
  #endif
 
#endif /* DEF_H_ */
