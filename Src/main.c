/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "bno080.h"
#include "quaternion.h"
#include "ICM20602.h"
//#include "LPS22HH.h"
//#include "M8N.h"
#include "FS-iA6B.h"
//#include "AT24C08.h"
#include "EEPROM.h"
#include "MSP.h"
#include "IMU.h"
#include "Sensors.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, uint8_t* p, int len)
{
//	for (int i=0; i < len; i++)
//	{
//		while(!LL_USART_IsActiveFlag_TXE(USART6));
//		LL_USART_TransmitData8(USART6, *(p+i));
//	};

	HAL_UART_Transmit(&huart6, p, len, 10);
	return len;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint8_t uart2_rx_data;
uint8_t msp_rx_buf[TX_BUFFER_SIZE];
uint8_t msp_rx_cplt_flag;

uint8_t msp_tx_buf[TX_BUFFER_SIZE];
uint8_t msp_tx_buf1[TX_BUFFER_SIZE];
uint8_t msp_tx_size = 0;
uint8_t msp_tx_cplt_flag = 0;

uint8_t g_msp_tx_buf[TX_BUFFER_SIZE];
uint8_t g_msp_tx_size = 0;
uint8_t g_msp_tx_cplt_flag = 0;

uint8_t inBuf[INBUF_SIZE];
//uint8_t checksum;
uint8_t indRX;
uint8_t cmdMSP;

uint16_t recvDelayCnt=0;
uint16_t sendDelayCnt=0;

uint8_t uart6_tx_flag = 0;

extern uint8_t uart6_rx_flag;
extern uint8_t uart6_rx_data;


extern uint8_t m8n_rx_buf[36];
extern uint8_t m8n_rx_cplt_flag;

extern uint8_t ibus_rx_buf[32];
extern uint8_t ibus_rx_flag;

extern uint8_t uart1_rx_data;

extern uint8_t tim7_1ms_flag;
extern uint8_t tim7_20ms_flag;

float pitch_in_kp;
float pitch_in_ki;
float pitch_in_kd;

float pitch_out_kp = 0;
float pitch_out_ki = 0;
float pitch_out_kd = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void annexCode(void);
int Is_iBus_Throttle_Min(void);
void ESC_Calibration(void);
int Is_iBus_Received(void);
void BNO080_Calibration(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*********** RC alias *****************/

const char pidnames[] =
  "SPEED;"
  "ANGLE;"
  //"YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  //"LEVEL;"
  "MAG;"
  "VEL;"
;

const char boxnames[] = // names for dynamic generation of config GUI
  "ARM;"
  #if ACC
    "SIMPLE;"
    "RISE;"
    "POS HOLD;"
  #endif
  #if defined(BUZZER)
    "BEEPER;"
  #endif
;

const uint8_t boxids[] = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
  #if ACC
    1, //"SIMPLE;"
    2, //"RISE;"
    3, //"POS HOLD;"
  #endif
  #if defined(BUZZER)
    13, //"BEEPER;"
  #endif
};

uint8_t cmdMSP_tbl1[8] = {100, 116, 111, 112, 115, 113, 114, 120};
uint8_t cmdMSP_tbl2[8] = {101, 102, 103, 104, 105, 254, 108, 109};
uint8_t cmd_index = 0;
uint8_t first_com_flag = 1;

uint8_t DRIVER_PIN[5] = {5,6,7,8,4};   //STEP1 (PORTD 5), STEP2 (PORTD 6), DIR1 (PORTD 7), DIR2 (PORTB 0), ENABLE

uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
uint16_t calibratingG;
int16_t  magHold,headFreeModeHold; // [-180;+180]
uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
uint8_t  rcOptions[CHECKBOXITEMS];
// **************
// gyro+acc IMU
// **************
int16_t gyroZero[3] = {0,0,0};

imu_t imu;

analog_t analog;

alt_t alt;

att_t att;

int16_t  debug[4];

flags_struct_t f;

int16_t  i2c_errors_count = 0;
int16_t  annex650_overrun_count = 0;

uint16_t intPowerTrigger1;

// ******************
// rc functions
// ******************
#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

int16_t rcData[RC_CHANS];    // interval [1000;2000]
int16_t rcSerial[8];         // interval [1000;2000] - is rcData coming from MSP
int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data
int16_t lookupPitchRollRC[5];// lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

static float accLPF[3] = {0, 0, ACC_1G};
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 120.0f
#endif

// *************************
// motor and servo functions
// *************************
//int16_t output;
int16_t motor[2];

int16_t actualMotorSpeed[2]={120,250};     // actual speed of motors
uint8_t actualMotorDir[2];       // actual direction of steppers motors

int16_t actualSpeed;     // actual speed of robot

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[2], dynD8[2];

global_conf_t global_conf;

conf_t conf;

  uint8_t alarmArray[16];           // array

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float q[4];
	float quatRadianAccuracy;
	unsigned char buf_read[16] = {0};
	unsigned char buf_write[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
	unsigned short adcVal;
	float batVolt;
	short gyro_x_offset = 11, gyro_y_offset = 5, gyro_z_offset = -2;
	float prev_BNO080_Roll = 0;
	float prev_BNO080_Pitch = 0;
	float prev_BNO080_Yaw = 0;

	unsigned char motor_arming_flag = 0;
	unsigned short iBus_SwA_Prev = 0;

	float pitch_reference;
	float pitch_p;
	float pitch_error;
	float pitch_i;
	float pitch_error_sum = 0;
	float pitch_derivative;
	float pitch_d;
	float pitch_pid;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
 // LL_TIM_EnableCounter(TIM3);

//  LL_USART_EnableIT_RXNE(USART6);
 // LL_USART_EnableIT_RXNE(UART4);
  LL_USART_EnableIT_RXNE(UART5);

  BNO080_Initialization();
  BNO080_enableRotationVector(2500);

  ICM20602_Initialization();

//  initSensors();

//  LPS22HH_Initialization();
//  M8N_Initialization();
  FSiA6B_UART5_Initialization();

/*  LL_TIM_EnableCounter(TIM5);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH4);
*/
  HAL_ADC_Start_DMA(&hadc1, &adcVal, 1);

//  HAL_UART_Receive_IT(&huart1, &uint1_rx_data, 1);
  HAL_UART_Receive_IT(&huart6, &uart6_rx_data, 1);

  LL_TIM_EnableCounter(TIM7);
  LL_TIM_EnableIT_UPDATE(TIM7);


  ICM20602_Writebyte(0x13, (gyro_x_offset*-2)>>8);
  ICM20602_Writebyte(0x14, (gyro_x_offset*-2));

  ICM20602_Writebyte(0x15, (gyro_y_offset*-2)>>8);
  ICM20602_Writebyte(0x16, (gyro_y_offset*-2));

  ICM20602_Writebyte(0x17, (gyro_z_offset*-2)>>8);
  ICM20602_Writebyte(0x18, (gyro_z_offset*-2));

  LoadDefaults();

//    char somedata[] = "this is data from the eeprom!1234567890abcdefghijklmnopqrstubwxyzABCDEFGHIJKLMNOPQRSTUVWXYaaaaaaaaaabbbbbbbbbbccccccccccddddddddddeeeeeeeeeeffffffffffgggggggggghhhhhhhhhhiiiiiiiiiijjjjjjjjjjkkkkkkkkkkllllllllllmmmmmmmmmmnnnnnnnnnnooooooooooppppppppppqqqqqqqqqqrrrrrrrrrrssssssssssttttttttttuuuuuuuuuuvvvvvvvvvvwwwwwwwwwwxxxxxxxxxxyyyyyyyyyyzzzzzzzzzzAAAAAAAAAABBBBBBBBBBCCCCCCCCCCDDDDDDDDDDEEEEEEEEEEFFFFFFFFFFGGGGGGGGGGHHHHHHHHHHIIIIIIIIIIJJJJJJJJJJKKKKKKKKKKLLLLLLLLLLMMMMMMMMMMNNNNNNNNNNOOOOOOOOOOPPPPPPPPPPQQQQQQQQQQRRRRRRRRRRSSSSSSSSSSTTTTTTTTTTUUUUUUUUUUVVVVVVVVVVWWWWWWWWWWXXXXXXXXXXYYYYYYYYYYZZZZZZZZZZ1111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990000000000!!!!!!!!!!@@@@@@@@@@##########$$$$$$$$$$%%%%%%%%%%^^^^^^^^^^&&&&&&&&&&**********(((((((((())))))))))__________++++++++++----------==========//////////,,,,,,,,,,..........;;;;;;;;;;''''''''''::::::::::<<<<<<<<<<>>>>>>>>>>??????????~~~~~~~~~~aaaaabbbbbcccccdddddeeeeefffffggggghhhhhiiiiijjjjjkkkkklllllmmmmmnnnnnooooopppppqqqqqrrrrrssssstttttuuuuuvvvvvwwwwwxxxxxyyyyyzzzzz"; // data to write
//    char buff[1200] = {0};
//
//    AT24C16_write_block(0, sizeof(somedata), (uint8_t*)&somedata[0]);
//    // Read array with bytes read from EEPROM memory.
//    AT24C16_read_block(270, 126, (uint8_t*)&buff[0]);


  //  writeGlobalSet(0);
    readGlobalSet();
  //  #ifdef MULTIPLE_CONFIGURATION_PROFILES
      global_conf.currentSet=2;
  //  #endif
    while(1) {                                                    // check settings integrity
      readEEPROM();                                               // check current setting integrity
      if(global_conf.currentSet == 0) break;                      // all checks is done
      global_conf.currentSet--;                                   // next setting for check
    }
    readGlobalSet();                              // reload global settings for get last profile number
    readEEPROM();                                 // load setting data from last used profile
  //  blinkLED(2,40,global_conf.currentSet+1);
  //  configureReceiver();

    initSensors();

   //  MPU6050_Init();
  //  previousTime = HAL_GetTick() * 1000;	//micros();
    #if defined(GIMBAL)
     calibratingA = 512;
    #endif
    calibratingG = 512;
    calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
  //  debugmsg_append_str("initialization completed\n");


//  while(Is_iBus_Received() == 0)
//  {
//	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//
//	  TIM3->PSC = 3000;
//	  HAL_Delay(200);
//	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//	  HAL_Delay(200);
//  }
//
//  if(iBus.SwC == 2000)
//  {
//	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//
//	  TIM3->PSC = 1500;
//	  HAL_Delay(200);
//	  TIM3->PSC = 2000;
//	  HAL_Delay(200);
//	  TIM3->PSC = 1500;
//	  HAL_Delay(200);
//	  TIM3->PSC = 2000;
//	  HAL_Delay(200);
//	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//	  HAL_Delay(200);
//
//	  ESC_Calibration();
//	  while(iBus.SwC != 1000)
//	  {
//		  Is_iBus_Received();
//		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//
//		  TIM3->PSC = 1500;
//		  HAL_Delay(200);
//		  TIM3->PSC = 2000;
//		  HAL_Delay(200);
//		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//	  }
//  }
//  else if(iBus.SwC == 1500)
//  {
//	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//
//	  TIM3->PSC = 1500;
//	  HAL_Delay(200);
//	  TIM3->PSC = 2000;
//	  HAL_Delay(200);
//	  TIM3->PSC = 1500;
//	  HAL_Delay(200);
//	  TIM3->PSC = 2000;
//	  HAL_Delay(200);
//	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//	  HAL_Delay(200);
//
//	  BNO080_Calibration();
//	  while(iBus.SwC != 1000)
//	  {
//		  Is_iBus_Received();
//		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//
//		  TIM3->PSC = 1500;
//		  HAL_Delay(200);
//		  TIM3->PSC = 2000;
//		  HAL_Delay(200);
//		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//	  }
//
//  }
//
//  while(Is_iBus_Throttle_Min() == 0)
//  {
//	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//
//	  TIM3->PSC = 1000;
//	  HAL_Delay(70);
//	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//	  HAL_Delay(70);
//
//  }

////  EP_PIDGain_Write(0, 1.1, 2.2, 3.3);
//  float p = 0.0, i = 0.0, d = 0.0;
//  EP_PIDGain_Read(0, &p, &i, &d);
//
//  printf("%f, %f, %f", p, i, d);
/*
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

  TIM3->PSC = 2000;
  HAL_Delay(100);
  TIM3->PSC = 1500;
  HAL_Delay(100);
  TIM3->PSC = 1000;
  HAL_Delay(100);

  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

  printf("start\n");
*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(ibus_rx_flag == 1)
	{
		ibus_rx_flag = 0;
	  if(iBus_Check_CHECKSUM(&ibus_rx_buf[0], 32) == 1)
	  {
		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);

		  iBus_Parsing(&ibus_rx_buf[0], &iBus);
//		  if(iBus_isActiveFailsafe(&iBus) == 1)
//		  {
//			  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//		  }
//		  else
//		  {
//			  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//		  }
	//			  printf("%d\t%d\t%d\t%d\t%d\t%d\n",
	//					  iBus.RH, iBus.RV, iBus.LV, iBus.LH, iBus.SwA, iBus.SwC);
	//			  HAL_Delay(100);
	  }
	}

		if (recvDelayCnt++ >= 50000)
		{
			recvDelayCnt = 0;
			HAL_UART_Receive_IT(&huart6, &uart6_rx_data, 1);
		}

		if(tim7_1ms_flag == 1)
		{
		  tim7_1ms_flag = 0;

//		  pitch_reference = (iBus.RV - 1500) * 0.1f;
//		  pitch_error = pitch_reference - BNO080_Pitch;
//		  pitch_p = pitch_error * pitch_out_kp;
//
//		  pitch_error_sum = pitch_error_sum + pitch_error * 0.001;
//		  if(motor_arming_flag == 0 || iBus.LV < 1030) pitch_error_sum = 0;
//		  pitch_i = pitch_error_sum * pitch_out_ki;
//
////		  pitch_derivative = (BNO080_Pitch - BNO080_Pitch_Prev) / 0.001;
////		  BNO080_Pitch_Prev = BNO080_Pitch;
//
//		  pitch_derivative = ICM20602.gyro_x;
//		  pitch_d = -pitch_derivative * pitch_out_kd;
//
//		  pitch_pid = pitch_p + pitch_i + pitch_d;
//

//		  for (uint8_t axis = 0; axis < 3; axis++) {
//			  accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + imu.accADC[axis] * (1.0f/ACC_LPF_FACTOR);
//			  imu.accSmooth[axis] = accLPF[axis];
//		  }

		  imu.accSmooth[ROLL]  = (short)BNO080_Roll*6.4;	//(((prev_BNO080_Roll + BNO080_Roll)/2) * 6.4);
		  imu.accSmooth[PITCH] = (short)BNO080_Pitch*5.7;	//(((prev_BNO080_Pitch + BNO080_Pitch)/2) *  5.8);
		  imu.accSmooth[YAW]   = (short)BNO080_Yaw*2.3;	//(((prev_BNO080_Yaw + BNO080_Yaw)/2) * 2);

		  prev_BNO080_Roll = BNO080_Roll;
		  prev_BNO080_Pitch = BNO080_Pitch;
		  prev_BNO080_Yaw = BNO080_Yaw;

		  att.angle[ROLL]  = (short)(BNO080_Roll * 10);
		  att.angle[PITCH]  = (short)(BNO080_Pitch * 10);
		  att.heading  = (short)(0);	//BNO080_Yaw);

		}

		if(tim7_20ms_flag == 1)
		{
			tim7_20ms_flag = 0;
			annexCode();
		}

		if(iBus.SwA == 2000 && iBus_SwA_Prev != 2000)
		{
		  if(iBus.LV < 1010)
		  {
			  motor_arming_flag = 1;
		  }
		  else
		  {
//			  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//
//			  TIM3->PSC = 1000;
//			  HAL_Delay(70);
//			  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//			  HAL_Delay(70);
		  }
		}
		iBus_SwA_Prev = iBus.SwA;

		if(iBus.SwA != 2000)
		{
		  motor_arming_flag = 0;
		}

		if (msp_rx_cplt_flag)
		{
			msp_rx_cplt_flag = 0;
			evaluateCommand();

		}
		if(msp_tx_cplt_flag)
		{
			if((g_msp_tx_size + msp_tx_size) < TX_BUFFER_SIZE)
			{
				msp_tx_cplt_flag = 0;
				memcpy(&g_msp_tx_buf[g_msp_tx_size],&msp_tx_buf[0],msp_tx_size);
				g_msp_tx_size += msp_tx_size;
				g_msp_tx_cplt_flag = 1;
			}
		}

		if(g_msp_tx_cplt_flag == 1)	//huart6.gState == HAL_UART_STATE_READY)
		{
			if(uart6_tx_flag == 0 || sendDelayCnt++ >= 10000)
			{
				uart6_tx_flag = 1;
				memcpy(&msp_tx_buf1[0],&g_msp_tx_buf[0],g_msp_tx_size);
				if(HAL_UART_Transmit_IT(&huart6, &msp_tx_buf1[0], g_msp_tx_size) != HAL_OK)
				{
				}
				g_msp_tx_size = 0;
				g_msp_tx_cplt_flag = 0;
				sendDelayCnt = 0;
			}

		}

	    computeRC();

//	    computeIMU();

	    batVolt = adcVal * 0.003619f;
	    analog.vbat = (uint8_t)(batVolt * 10);	//((adcVal<<4) / conf.vbatscale); // result is Vbatt in 0.1V steps
	  /* //printf("%d\t%.2f\n", adcVal, batVolt);
	  if(batVolt < 10.0f)
	  {
		  TIM3->PSC = 1000;
		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  }
	  else
	  {
		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  }
*/
	  if(BNO080_dataAvailable() == 1)
	  {
		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0);

		  q[0] = BNO080_getQuatI();
		  q[1] = BNO080_getQuatJ();
		  q[2] = BNO080_getQuatK();
		  q[3] = BNO080_getQuatReal();
		  quatRadianAccuracy = BNO080_getQuatRadianAccuracy();

		  Quaternion_Update(&q[0]);

		  BNO080_Roll = -BNO080_Roll;
		  BNO080_Pitch = -BNO080_Pitch;

		  imu.accADC[ROLL]  = (short)BNO080_Roll*10;	//-(short)(q[0]);
		  imu.accADC[PITCH] = (short)BNO080_Pitch*10;	//-(short)(q[1]);
		  imu.accADC[YAW]   = (short)BNO080_Yaw*10;		//(short)(q[2]);

		  //printf("%.2f\t%.2f\n", BNO080_Roll, BNO080_Pitch);
		  //printf("%d, %d, %d\n", (int)(BNO080_Roll*100), (int)(BNO080_Pitch*100),(int)(BNO080_Yaw*100));
	  }

//	  HAL_Delay(1);

	  if(ICM20602_DataReady() == 1)
	  {
		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_1);

		  ICM20602_Get3AxisGyroRawData(&ICM20602.gyro_x_raw);

		  ICM20602.gyro_x = ICM20602.gyro_x_raw * 2000.f /32768.f;
		  ICM20602.gyro_y = ICM20602.gyro_y_raw * 2000.f /32768.f;
		  ICM20602.gyro_z = ICM20602.gyro_z_raw * 2000.f /32768.f;

		  ICM20602.gyro_x = -ICM20602.gyro_x;
		  ICM20602.gyro_z = -ICM20602.gyro_z;

		  imu.gyroADC[ROLL]  = (short)(ICM20602.gyro_y * 10);	//ICM20602.gyro_y_raw;	//gyro_y;
		  imu.gyroADC[PITCH] = (short)(ICM20602.gyro_x * 10);	//ICM20602.gyro_x_raw;	//gyro_x;
		  imu.gyroADC[YAW]   = (short)(ICM20602.gyro_z * 10);	//ICM20602.gyro_z_raw;	//gyro_z;

		  imu.gyroData[ROLL]  = imu.gyroADC[ROLL];
		  imu.gyroData[PITCH] = imu.gyroADC[PITCH];
		  imu.gyroData[YAW]   = imu.gyroADC[YAW];

//		  printf("%d, %d, %d\n", ICM20602.gyro_x_raw, ICM20602.gyro_y_raw, ICM20602.gyro_z_raw);
//		  printf("%d, %d, %d\n", (int)(ICM20602.gyro_x * 100), (int)(ICM20602.gyro_y * 100), (int)(ICM20602.gyro_z * 100));
	  }

//	  if(LPS22HH_DataReady() == 1)
//	  {
//		  LPS22HH_GetPressure(&LPS22HH.pressure_raw);
//		  LPS22HH_GetTemperature(&LPS22HH.temperature_raw);
//
//		  LPS22HH.baroAlt = getAltitude2(LPS22HH.pressure_raw/4096.f, LPS22HH.temperature_raw/100.f);
//
//#define X 0.90f
//		  LPS22HH.baroAltFilt = LPS22HH.baroAltFilt * X + LPS22HH.baroAlt * (1.0f - X);
//
//		  printf("%d, %d\n", (int)(LPS22HH.baroAlt * 100), (int)(LPS22HH.baroAltFilt * 100));
//
//	  }

//	  if(m8n_rx_cplt_flag == 1)
//	  {
//		  m8n_rx_cplt_flag = 0;
//		  if(M8N_UBX_CHKSUM_Check(&m8n_rx_buf[0], 36) == 1)
//		  {
//			  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);
//			  M8N_UBX_NAV_POSLLH_Parsing(&m8n_rx_buf[0], &posllh);
//			  printf("LAT: %ld\tLON: %ld\tHeight: %ld\n", posllh.lat, posllh.lon, posllh.height);
//		  }
//	  }

//	  if(ibus_rx_flag == 1)
//  	  {
//		  ibus_rx_flag = 0;
//		  if(iBus_Check_CHECKSUM(&ibus_rx_buf[0], 32) == 1)
//		  {
//			  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);
//
//			  iBus_Parsing(&ibus_rx_buf[0], &iBus);
//			  if(iBus_isActiveFailsafe(&iBus) == 1)
//			  {
//				  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//			  }
//			  else
//			  {
//				  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//			  }
////			  printf("%d\t%d\t%d\t%d\t%d\t%d\n",
////					  iBus.RH, iBus.RV, iBus.LV, iBus.LH, iBus.SwA, iBus.SwC);
////			  HAL_Delay(100);
//		  }
//  	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void annexCode(void) { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  uint8_t axis,prop1,prop2;

  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  prop2 = 128; // prop2 was 100, is 128 now
  if (rcData[THROTTLE]>1500) { // breakpoint is fix: 1500
    if (rcData[THROTTLE]<2000) {
      prop2 -=  ((uint16_t)conf.dynThrPID*(rcData[THROTTLE]-1500)>>9); //  /512 instead of /500
    } else {
      prop2 -=  conf.dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) {
    tmp = min(abs(rcData[axis]-MIDRC),500);
    #if defined(DEADBAND)
      if (tmp>DEADBAND) { tmp -= DEADBAND; }
      else { tmp=0; }
    #endif
    if(axis!=2) { //ROLL & PITCH
      tmp2 = tmp>>7; // 500/128 = 3.9  => range [0;3]
      rcCommand[axis] = lookupPitchRollRC[tmp2] + ((tmp-(tmp2<<7)) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2])>>7);
      prop1 = 128-((uint16_t)conf.rollPitchRate*tmp>>9); // prop1 was 100, is 128 now -- and /512 instead of /500
      prop1 = (uint16_t)prop1*prop2>>7; // prop1: max is 128   prop2: max is 128   result prop1: max is 128
      dynP8[axis] = (uint16_t)conf.pid[axis].P8*prop1>>7; // was /100, is /128 now
      dynD8[axis] = (uint16_t)conf.pid[axis].D8*prop1>>7; // was /100, is /128 now
    } else {      // YAW
      rcCommand[axis] = tmp;
    }
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  tmp = (uint32_t)(tmp-MINCHECK)*2559/(2000-MINCHECK); // [MINCHECK;2000] -> [0;2559]
  tmp2 = tmp/256; // range [0;9]
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*256) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 256; // [0;2559] -> expo -> [conf.minthrottle;MAXTHROTTLE]

  if(f.HEADFREE_MODE) { //to optimize
    float radDiff = (att.heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH]*cosDiff + rcCommand[ROLL]*sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL]*cosDiff - rcCommand[PITCH]*sinDiff;
    rcCommand[PITCH] = rcCommand_PITCH;
  }
}

int Is_iBus_Throttle_Min(void)
{
	if(ibus_rx_flag == 1)
	{
		ibus_rx_flag = 0;
		if(iBus_Check_CHECKSUM(&ibus_rx_buf[0], 32) == 1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			if(iBus.LV < 1010) return 1;
		}
	}
	return 0;
}

void ESC_Calibration(void)
{
	  TIM5->CCR1 = 21000;
	  TIM5->CCR2 = 21000;
	  TIM5->CCR3 = 21000;
	  TIM5->CCR4 = 21000;
	  HAL_Delay(7000);

	  TIM5->CCR1 = 10500;
	  TIM5->CCR2 = 10500;
	  TIM5->CCR3 = 10500;
	  TIM5->CCR4 = 10500;
	  HAL_Delay(8000);
}

int Is_iBus_Received(void)
{
	if(ibus_rx_flag == 1)
	{
		ibus_rx_flag = 0;
		if(iBus_Check_CHECKSUM(&ibus_rx_buf[0], 32) == 1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			return 1;
		}
	}
	return 0;
}

void BNO080_Calibration(void)
{
	//Resets BNO080 to disable All output
	BNO080_Initialization();

	//BNO080/BNO085 Configuration
	//Enable dynamic calibration for accelerometer, gyroscope, and magnetometer
	//Enable Game Rotation Vector output
	//Enable Magnetic Field output
	BNO080_calibrateAll(); //Turn on cal for Accel, Gyro, and Mag
	BNO080_enableGameRotationVector(20000); //Send data update every 20ms (50Hz)
	BNO080_enableMagnetometer(20000); //Send data update every 20ms (50Hz)

	//Once magnetic field is 2 or 3, run the Save DCD Now command
  	printf("Calibrating BNO080. Pull up FS-i6 SWC to end calibration and save to flash\n");
  	printf("Output in form x, y, z, in uTesla\n\n");

	//while loop for calibration procedure
	//Iterates until iBus.SwC is mid point (1500)
	//Calibration procedure should be done while this loop is in iteration.
	while(iBus.SwC == 1500)
	{
		if(BNO080_dataAvailable() == 1)
		{
			//Observing the status bit of the magnetic field output
			float x = BNO080_getMagX();
			float y = BNO080_getMagY();
			float z = BNO080_getMagZ();
			unsigned char accuracy = BNO080_getMagAccuracy();

			float quatI = BNO080_getQuatI();
			float quatJ = BNO080_getQuatJ();
			float quatK = BNO080_getQuatK();
			float quatReal = BNO080_getQuatReal();
			unsigned char sensorAccuracy = BNO080_getQuatAccuracy();

			printf("%f,%f,%f,", x, y, z);
			if (accuracy == 0) printf("Unreliable\t");
			else if (accuracy == 1) printf("Low\t");
			else if (accuracy == 2) printf("Medium\t");
			else if (accuracy == 3) printf("High\t");

			printf("\t%f,%f,%f,%f,", quatI, quatI, quatI, quatReal);
			if (sensorAccuracy == 0) printf("Unreliable\n");
			else if (sensorAccuracy == 1) printf("Low\n");
			else if (sensorAccuracy == 2) printf("Medium\n");
			else if (sensorAccuracy == 3) printf("High\n");

			//Turn the LED and buzzer on when both accuracy and sensorAccuracy is high
			if(accuracy == 3 && sensorAccuracy == 3)
			{
				LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
				TIM3->PSC = 65000; //Very low frequency
				LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			}
			else
			{
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
				LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			}
		}

		Is_iBus_Received(); //Refreshes iBus Data for iBus.SwC
		HAL_Delay(100);
	}

	//Ends the loop when iBus.SwC is not mid point
	//Turn the LED and buzzer off
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	//Saves the current dynamic calibration data (DCD) to memory
	//Sends command to get the latest calibration status
	BNO080_saveCalibration();
	BNO080_requestCalibrationStatus();

	//Wait for calibration response, timeout if no response
	int counter = 100;
	while(1)
	{
		if(--counter == 0) break;
		if(BNO080_dataAvailable())
		{
			//The IMU can report many different things. We must wait
			//for the ME Calibration Response Status byte to go to zero
			if(BNO080_calibrationComplete() == 1)
			{
				printf("\nCalibration data successfully stored\n");
				LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				TIM3->PSC = 2000;
				HAL_Delay(300);
				TIM3->PSC = 1500;
				HAL_Delay(300);
				LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				HAL_Delay(1000);
				break;
			}
		}
		HAL_Delay(10);
	}
	if(counter == 0)
	{
		printf("\nCalibration data failed to store. Please try again.\n");
		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		TIM3->PSC = 1500;
		HAL_Delay(300);
		TIM3->PSC = 2000;
		HAL_Delay(300);
		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		HAL_Delay(1000);
	}

	//BNO080_endCalibration(); //Turns off all calibration
	//In general, calibration should be left on at all times. The BNO080
	//auto-calibrates and auto-records cal data roughly every 5 minutes

	//Resets BNO080 to disable Game Rotation Vector and Magnetometer
	//Enables Rotation Vector
	BNO080_Initialization();
	BNO080_enableRotationVector(2500); //Send data update every 2.5ms (400Hz)
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
//    errorCounter++;
	// try to clear all the UART status flags

	while( huart->Instance->SR &
	(UART_FLAG_CTS | UART_FLAG_LBD | UART_FLAG_RXNE |
	UART_FLAG_IDLE | UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE)
	)
	{

	__HAL_UART_CLEAR_PEFLAG(huart);

	huart->Instance->SR &= ~(UART_FLAG_CTS|UART_FLAG_LBD|UART_FLAG_RXNE);

	}

//    uint32_t error = HAL_UART_GetError(huart);
//	if(huart->Instance == USART6)
//	{
//		HAL_UART_Receive_IT(huart, &uart6_rx_data, 1);
//	}

	HAL_UART_Receive_IT(huart, &uart6_rx_data, 1);

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
	{
		uart6_tx_flag = 0;
		sendDelayCnt = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//	static unsigned char cnt = 0;
	static uint8_t offset;
	static uint8_t dataSize;
	static uint8_t chksum;
	static enum _serial_state {
	   IDLE,
	   HEADER_START,
	   HEADER_M,
	   HEADER_ARROW,
	   HEADER_SIZE,
	   HEADER_CMD,
	 } c_state;// = IDLE;


	if(huart->Instance == USART6)
	{
//		uart2_rx_flag = 1;
//		LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
//		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
		recvDelayCnt = 0;

		// regular data handling to detect and handle MSP and other data
		switch(c_state)
		{
		case IDLE:
			  c_state = (uart6_rx_data=='$') ? HEADER_START : IDLE;
			  if (c_state == IDLE) evaluateOtherData(uart6_rx_data); // evaluate all other incoming serial data
			  break;
		case HEADER_START:
			  c_state = (uart6_rx_data=='M') ? HEADER_M : IDLE;
			  break;
		case HEADER_M:
			  c_state = (uart6_rx_data=='<') ? HEADER_ARROW : IDLE;
			  break;
		case HEADER_ARROW:
			  if (uart6_rx_data > 64) {	//INBUF_SIZE) {  // now we are expecting the payload size
				c_state = IDLE;
				return;	//continue;
			  }
			  dataSize = uart6_rx_data;
			  offset = 0;
			  chksum = 0;
			  indRX = 0;
			  chksum ^= uart6_rx_data;
			  c_state = HEADER_SIZE;  // the command is to follow
			  break;
		case HEADER_SIZE:
			  cmdMSP = uart6_rx_data;
			  chksum ^= uart6_rx_data;
			  c_state = HEADER_CMD;
			  break;
		case HEADER_CMD:
			if(offset < dataSize) {
			  chksum ^= uart6_rx_data;
			  inBuf[offset++] = uart6_rx_data;
			} else {
			  if (chksum == uart6_rx_data) {  // compare calculated and transferred checksum
				  msp_rx_cplt_flag = 1;
			  }
			  c_state = IDLE;
			}
			break;
		default:
			  c_state = IDLE;
			break;

		}
		if (HAL_UART_Receive_IT(&huart6, &uart6_rx_data, 1) != HAL_OK)
		{
			HAL_UART_Receive_IT(huart, &uart6_rx_data, 1);
			return;
		}

	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
