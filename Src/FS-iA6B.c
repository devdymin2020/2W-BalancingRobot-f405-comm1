/*
 * FS-IA6B.c
 *
 *  Created on: Dec 11, 2019
 *      Author: khmin
 */

#include <FS-iA6B.h>

FSiA6B_iBus iBus;

/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

//RAW RC values will be store here
#if defined(SBUS)
#elif defined(SPEKTRUM) || defined(SERIAL_SUM_PPM)
  volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#else
#endif

#if defined(SERIAL_SUM_PPM) //Channel order for PPM SUM RX Configs
  static uint8_t rcChannel[RC_CHANS] = {SERIAL_SUM_PPM};
#endif

uint16_t readRawRC(uint8_t chan) {
uint16_t data;
#if defined(SPEKTRUM)
#else
//  uint8_t oldSREG;
//  oldSREG = SREG; cli(); // Let's disable interrupts
  data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
//  SREG = oldSREG;        // Let's restore interrupt state
#endif
return data; // We return the value correctly copied when the IRQ's where disabled
}

/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
void computeRC() {
static uint16_t rcData4Values[RC_CHANS][4], rcDataMean[RC_CHANS];
static uint8_t rc4ValuesIndex = 0;
uint8_t chan,a;
#if !defined(OPENLRSv2MULTI) // dont know if this is right here
  rc4ValuesIndex++;
  if (rc4ValuesIndex == 4) rc4ValuesIndex = 0;
  for (chan = 0; chan < RC_CHANS; chan++) {
	#if defined(FAILSAFE)
	#else
//	  rcData4Values[chan][rc4ValuesIndex] = readRawRC(chan);
	  rcData[chan] = readRawRC(chan);
	#endif
/*	#if defined(SPEKTRUM) || defined(SBUS) // no averaging for Spektrum & SBUS signal
	#else
	  rcDataMean[chan] = 0;
	  for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];
	  rcDataMean[chan]= (rcDataMean[chan]+2)>>2;
	  if ( rcDataMean[chan] < (uint16_t)rcData[chan] -3)  rcData[chan] = rcDataMean[chan]+2;
	  if ( rcDataMean[chan] > (uint16_t)rcData[chan] +3)  rcData[chan] = rcDataMean[chan]-2;
	#endif
	if (chan<8 && rcSerialCount > 0) { // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
	  rcSerialCount --;
	  if (rcSerial[chan] >900) {rcData[chan] = rcSerial[chan];} // only relevant channels are overridden
	}*/
  }
#endif
}

unsigned char iBus_Check_CHECKSUM(unsigned char* data, unsigned char len)
{
	unsigned short chksum = 0xffff;

	for(int i=0; i<len-2; i++)
	{
		chksum = chksum - data[i];
	}
	return ((chksum&0x00ff)==data[30]) && ((chksum>>8)==data[31]);
}

void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus)
{
	for (uint8_t i=0; i < RC_CHANS; i++)
	{
		rcValue[i] = (data[i*2+2] | data[i*2+3]<<8) & 0x0fff;
	}

	iBus->RH = (data[2] | data[3]<<8) & 0x0fff;
	iBus->RV = (data[4] | data[5]<<8) & 0x0fff;
	iBus->LV = (data[6] | data[7]<<8) & 0x0fff;
	iBus->LH = (data[8] | data[9]<<8) & 0x0fff;
	iBus->SwA = (data[10] | data[11]<<8) & 0x0fff;
	iBus->SwC = (data[12] | data[13]<<8) & 0x0fff;

	iBus->FailSafe = (data[13] >> 4);
}

unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus)
{
	return iBus->FailSafe != 0;
}

void FSiA6B_UART5_Initialization(void)
{
	  LL_USART_InitTypeDef USART_InitStruct = {0};

	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	  /* Peripheral clock enable */
	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);

	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	  /**UART5 GPIO Configuration
	  PC12   ------> UART5_TX
	  PD2   ------> UART5_RX
	  */
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /* UART5 interrupt Init */
	  NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	  NVIC_EnableIRQ(UART5_IRQn);

	  USART_InitStruct.BaudRate = 115200;
	  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_RX;
	  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	  LL_USART_Init(UART5, &USART_InitStruct);
	  LL_USART_ConfigAsyncMode(UART5);
	  LL_USART_Enable(UART5);
}
