/*
 * AT24C16.h
 *
 *  Created on: May 13, 2020
 *      Author: khmin
 */

#ifndef INC_AT24C16_H_
#define INC_AT24C16_H_

#define BUFFER_LENGTH         	  32
#define EEPROM__PAGE_SIZE         16
#define EEPROM__RD_BUFFER_SIZE    BUFFER_LENGTH
#define EEPROM__WR_BUFFER_SIZE    (BUFFER_LENGTH - 1)


void AT24C16_write_block(unsigned short address, unsigned short length, unsigned char* p_data);
void AT24C16_read_block(unsigned short address, unsigned short length, unsigned char* p_data);

void AT24C16_writePages(unsigned short address, unsigned char length, unsigned char* p_data);
void AT24C16_Page_Write(unsigned short address, unsigned char* data, unsigned char len);
void AT24C16_Page_Read(unsigned short address, unsigned char* data, unsigned char len);

#endif /* INC_AT24C16_H_ */
