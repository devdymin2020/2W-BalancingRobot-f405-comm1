
#ifndef EEPROM_H_
#define EEPROM_H_

#include "AT24C16.h"
#include "main.h"

#define true 1
#define false 0

void readGlobalSet();
_Bool readEEPROM();
void update_constants();
void writeGlobalSet(uint8_t b);
void writeParams(uint8_t b);
void LoadDefaults();
//void readPLog(void);
//void writePLog(void);

#endif /* EEPROM_H_ */
