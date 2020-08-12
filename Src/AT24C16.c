/*
 * AT24C16.c
 *
 *  Created on: May 13, 2020
 *      Author: khmin
 */

#include "i2c.h"
#include "AT24C16.h"


void AT24C16_write_block(unsigned short address, unsigned short length, unsigned char* p_data)
{
    // Write first page if not aligned.
    unsigned char notAlignedLength = 0;
    unsigned char pageOffset = address % EEPROM__PAGE_SIZE;
    if (pageOffset > 0)
    {
        notAlignedLength = EEPROM__PAGE_SIZE - pageOffset;
        AT24C16_writePages(address, notAlignedLength, p_data);
        length -= notAlignedLength;
    }

    if (length > 0)
    {
        address += notAlignedLength;
        p_data += notAlignedLength;

        // Write complete and aligned pages.
        unsigned char pageCount = length / EEPROM__PAGE_SIZE;
        for (int i = 0; i < pageCount; i++)
        {
        	AT24C16_writePages(address, EEPROM__PAGE_SIZE, p_data);
            address += EEPROM__PAGE_SIZE;
            p_data += EEPROM__PAGE_SIZE;
            length -= EEPROM__PAGE_SIZE;
        }

        if (length > 0)
        {
            // Write remaining uncomplete page.
        	AT24C16_writePages(address, EEPROM__PAGE_SIZE, p_data);
        }
    }
}

void AT24C16_read_block(unsigned short address, unsigned short length, unsigned char* p_data)
{
	unsigned char bufferCount = length / EEPROM__RD_BUFFER_SIZE;
    for (int i = 0; i < bufferCount; i++)
    {
    	unsigned short offset = i * EEPROM__RD_BUFFER_SIZE;
//        readBuffer(address + offset, EEPROM__RD_BUFFER_SIZE, p_data + offset);
        AT24C16_Page_Read(address + offset, p_data + offset, EEPROM__RD_BUFFER_SIZE);
    }

    unsigned char remainingBytes = length % EEPROM__RD_BUFFER_SIZE;
    unsigned short offset = length - remainingBytes;
//    readBuffer(address + offset, remainingBytes, p_data + offset);
    AT24C16_Page_Read(address + offset, p_data + offset, remainingBytes);
}

void AT24C16_writePages(unsigned short address, unsigned char length, unsigned char* p_data)
{
    // Write complete buffers.
	unsigned char bufferCount = length / EEPROM__WR_BUFFER_SIZE;
    for (unsigned char i = 0; i < bufferCount; i++)
    {
    	unsigned char offset = i * EEPROM__WR_BUFFER_SIZE;
//        writeBuffer(address + offset, EEPROM__WR_BUFFER_SIZE, p_data + offset);
        AT24C16_Page_Write(address + offset, p_data + offset, EEPROM__WR_BUFFER_SIZE);
    }

    // Write remaining bytes.
    unsigned char remainingBytes = length % EEPROM__WR_BUFFER_SIZE;
    unsigned char offset = length - remainingBytes;
//    writeBuffer(address + offset, remainingBytes, p_data + offset);
    AT24C16_Page_Write(address + offset, p_data + offset, remainingBytes);
}

void AT24C16_Page_Write(unsigned short address, unsigned char* data, unsigned char len)
{
	unsigned char devAddress = ((address >> 8) & 0x07) << 1 | 0xA0;
	unsigned char wordAddress = address & 0xff;

	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
	HAL_I2C_Mem_Write(&hi2c1, devAddress, wordAddress, I2C_MEMADD_SIZE_8BIT, &data[0], len, 1);
	HAL_Delay(1);
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
}

void AT24C16_Page_Read(unsigned short address, unsigned char* data, unsigned char len)
{
	unsigned char devAddress = ((address >> 8) & 0x07) << 1 | 0xA0;
	unsigned char wordAddress = address & 0xff;

	HAL_I2C_Mem_Read(&hi2c1, devAddress, wordAddress, I2C_MEMADD_SIZE_8BIT, &data[0], len, 1);

}


