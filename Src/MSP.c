/*
 * MSP.c
 *
 *  Created on: May 4, 2020
 *      Author: khmin
 */
#include <string.h>
#include "MSP.h"
#include "EEPROM.h"


extern uint8_t inBuf[INBUF_SIZE];
uint8_t checksum;
extern uint8_t indRX;
extern uint8_t cmdMSP;

extern uint8_t msp_tx_buf[TX_BUFFER_SIZE];
extern uint8_t msp_tx_size;
extern uint8_t msp_tx_cplt_flag;


#define BIND_CAPABLE 0;  //Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module.

const uint32_t capability = 0+BIND_CAPABLE;

extern uint8_t DRIVER_PIN[5];

//---------------------------------------------------------------------------
uint8_t read8()  {
  return inBuf[indRX++]&0xff;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}

void headSerialResponse(uint8_t err, uint8_t s) {
  msp_tx_buf[0] = '$';
  msp_tx_buf[1] = 'M';
  msp_tx_buf[2] = err ? '!' : '>';
  checksum = 0;
  msp_tx_buf[3] = s;
  checksum ^= s; 				// calculating checksum
  msp_tx_buf[4] = cmdMSP;
  checksum ^= cmdMSP;

  if(s == 0){
	  msp_tx_buf[5] = checksum;
	  msp_tx_size = 6;
	  msp_tx_cplt_flag = 1;
  }
  else
	  msp_tx_size = 5;

}

void payloadSerialResponse(uint8_t *cb,uint8_t siz) {

  int i = 0;

  for(i=0; i<siz; i++){
	  msp_tx_buf[5+i] = *cb++;
	  checksum ^= msp_tx_buf[5+i];
  }

  msp_tx_buf[5+i] =  checksum;
  msp_tx_size += (siz + 1);
  msp_tx_cplt_flag = 1;
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void headSerialError(uint8_t s) {//inline
  headSerialResponse(1, s);
}

void serializeNames(const char* s) {
  headSerialReply((uint8_t)strlen(s));
  payloadSerialResponse((uint8_t*)s, (uint8_t)strlen(s));		//  while(siz--) serialize8(*cb++);
}
void  s_struct(uint8_t *cb,uint8_t siz) {
  headSerialReply(siz);
  payloadSerialResponse((uint8_t*)cb, siz);		//  while(siz--) serialize8(*cb++);
}

void s_struct_w(uint8_t *cb,uint8_t siz) {
  headSerialReply(0);
  while(siz--) *cb++ = read8();
}

void evaluateCommand() {
  uint32_t tmp=0;


  switch(cmdMSP) {
   case MSP_SET_RAW_RC:
     s_struct_w((uint8_t*)&rcSerial,16);
     rcSerialCount = 50; // 1s transition
     break;
   case MSP_SET_PID:
     s_struct_w((uint8_t*)&conf.pid[0].P8,3*PIDITEMS);
     break;
   case MSP_SET_BOX:
     s_struct_w((uint8_t*)&conf.activate[0],CHECKBOXITEMS*2);
     break;
   case MSP_SET_RC_TUNING:
     s_struct_w((uint8_t*)&conf.rcRate8,7);
     break;
   #if !defined(DISABLE_SETTINGS_TAB)
   case MSP_SET_MISC:
     s_struct_w((uint8_t*)&set_misc,22);
     conf.minthrottle = set_misc.b;
     #if defined(VBAT)
       conf.vbatscale        = set_misc.i;
       conf.vbatlevel_warn1  = set_misc.j;
       conf.vbatlevel_warn2  = set_misc.k;
       conf.vbatlevel_crit   = set_misc.l;
     #endif
     break;
   case MSP_MISC:
     misc.a = intPowerTrigger1;
     misc.b = conf.minthrottle;
     misc.c = MAXTHROTTLE;
     misc.d = MINCOMMAND;

       misc.e = 0;
       misc.f = 0; misc.g =0;
       misc.h = 0;

     #ifdef VBAT
       misc.i = conf.vbatscale;
       misc.j = conf.vbatlevel_warn1;
       misc.k = conf.vbatlevel_warn2;
       misc.l = conf.vbatlevel_crit;
     #endif
     s_struct((uint8_t*)&misc,22);
       break;
   #endif

   #ifdef MULTIPLE_CONFIGURATION_PROFILES
   case MSP_SELECT_SETTING:
     if(!f.ARMED) {
       global_conf.currentSet = read8();
       if(global_conf.currentSet>2) global_conf.currentSet = 0;
       writeGlobalSet(0);
       readEEPROM();
     }
     headSerialReply(0);
     break;
   #endif
   case MSP_SET_HEAD:
     s_struct_w((uint8_t*)&magHold,2);
     break;
   case MSP_IDENT:
     id.v     = VERSION;
     id.t     = MULTITYPE;
     id.msp_v = MSP_VERSION;
     id.cap   = capability|DYNBAL<<2|FLAP<<3;
     s_struct((uint8_t*)&id,7);
     break;
   case MSP_STATUS:
     st.cycleTime        = cycleTime;
     st.i2c_errors_count = i2c_errors_count;
     st.sensor           = ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4;
     #if ACC
       if(f.SIMPLE_MODE)    tmp |= 1<<BOXSIMPLE;
       if(f.RISE_MODE)    tmp |= 1<<BOXRISE;
       if(f.POSHOLD_MODE)   tmp |= 1<<BOXPOSHOLD;
     #endif
     #if defined(BUZZER)
       if(rcOptions[BOXBEEPERON]) tmp |= 1<<BOXBEEPERON;
     #endif
     if(f.ARMED) tmp |= 1<<BOXARM;
     st.flag             = tmp;
     st.set              = global_conf.currentSet;
     s_struct((uint8_t*)&st,11);
     break;
   case MSP_RAW_IMU:
     s_struct((uint8_t*)&imu,18);
     break;
   case MSP_SERVO:
//     s_struct((uint8_t*)&servo,16);
//     s_struct((uint8_t*)&actualMotorSpeed,4);
     break;
   case MSP_SERVO_CONF:
     s_struct((uint8_t*)&conf.servoConf[0].min,56); // struct servo_conf_ is 7 bytes length: min:2 / max:2 / middle:2 / rate:1    ----     8 servo =>  8x7 = 56
     break;
   case MSP_SET_SERVO_CONF:
     s_struct_w((uint8_t*)&conf.servoConf[0].min,56);
     break;
   case MSP_MOTOR:
     motor[0] = actualMotorSpeed[0] + 1500;
     motor[1] = actualMotorSpeed[1] + 1500;
     s_struct((uint8_t*)&motor,4);
     //s_struct((uint8_t*)&actualMotorSpeed,4);
     break;
   case MSP_RC:
     s_struct((uint8_t*)&rcData,RC_CHANS*2);
     break;
   case MSP_ATTITUDE:
     s_struct((uint8_t*)&att,6);
     break;
   case MSP_ALTITUDE:
     s_struct((uint8_t*)&alt,6);
     break;
   case MSP_ANALOG:
     s_struct((uint8_t*)&analog,7);
     break;
   case MSP_RC_TUNING:
     s_struct((uint8_t*)&conf.rcRate8,7);
     break;
   case MSP_PID:
     s_struct((uint8_t*)&conf.pid[0].P8,3*PIDITEMS);
     break;
   case MSP_PIDNAMES:
     serializeNames(pidnames);
     break;
   case MSP_BOX:
     s_struct((uint8_t*)&conf.activate[0],2*CHECKBOXITEMS);
     break;
   case MSP_BOXNAMES:
     serializeNames(boxnames);
     break;
   case MSP_BOXIDS:
     headSerialReply(CHECKBOXITEMS);
   	 payloadSerialResponse((uint8_t*)&boxids[0], CHECKBOXITEMS);		//  while(siz--) serialize8(*cb++);

     break;
   case MSP_MOTOR_PINS:
     s_struct((uint8_t*)&DRIVER_PIN,5);
     break;
   case MSP_RESET_CONF:
     if(!f.ARMED) LoadDefaults();
     headSerialReply(0);
     break;
   case MSP_ACC_CALIBRATION:
     if(!f.ARMED) calibratingA=512;
     headSerialReply(0);
     break;
   case MSP_MAG_CALIBRATION:
     if(!f.ARMED) f.CALIBRATE_MAG = 1;
     headSerialReply(0);
     break;
   case MSP_EEPROM_WRITE:
     writeParams(0);
     headSerialReply(0);
     break;
   case MSP_DEBUG:
     s_struct((uint8_t*)&debug,8);
     break;
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
}

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
  #ifndef SUPPRESS_OTHER_SERIAL_COMMANDS
    switch (sr) {
    // Note: we may receive weird characters here which could trigger unwanted features during flight.
    //       this could lead to a crash easily.
    //       Please use if (!f.ARMED) where neccessary

    }
  #endif // SUPPRESS_OTHER_SERIAL_COMMANDS
}

void debugmsg_append_str(const char *str) {};
