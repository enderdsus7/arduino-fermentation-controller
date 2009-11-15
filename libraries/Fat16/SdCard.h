/* Arduino FAT16 Library
 * Copyright (C) 2008 by William Greiman
 *  
 * This file is part of the Arduino FAT16 Library
 *  
 * This Library is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with the Arduino Fat16 Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#ifndef SdCard_h
#define SdCard_h
#include "SdInfo.h"
//------------------------------------------------------------------------------
// define SPI pins
#if defined(__AVR_ATmega1280__) //SPI pins
// pins for Arduino Mega
#define SS   53
#define MOSI 51
#define MISO 50
#define SCK  52
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
// pins for Sanguino
#define SS   4
#define MOSI 5
#define MISO 6
#define SCK  7
#else //SPI pins
// pins for other Arduinos
#define SS   10
#define MOSI 11
#define MISO 12
#define SCK  13
#endif //SPI pins
//------------------------------------------------------------------------------
// error codes
#define SD_ERROR_CMD0             1 // Card did not go into SPI mode
#define SD_ERROR_ACMD41           2 // Card did not go ready
#define SD_ERROR_CMD24            3 // Write command not accepted
#define SD_ERROR_CMD17            4 // Read command not accepted
#define SD_ERROR_READ_TIMEOUT     5 // timeout waiting for read data
#define SD_ERROR_WRITE_RESPONSE   6 // write error occurred
#define SD_ERROR_WRITE_TIMEOUT    7 // timeout waiting for write status
#define SD_ERROR_BLOCK_ZERO_WRITE 8 // attempt to write block zero
//------------------------------------------------------------------------------
// SD command codes
#define ACMD41   0X29      //SEND OPERATING CONDITIONS
#define CMD0     0X00      //GO_IDLE_STATE - init card in spi mode if CS low
#define CMD9     0X09      //SEND_CSD - Card Specific Data
#define CMD10    0X0A      //SEND_CID - Card IDentification
#define CMD17    0X11      //READ_BLOCK
#define CMD24    0X18      //WRITE_BLOCK
#define CMD55    0X37      //APP_CMD - escape for application specific command
//------------------------------------------------------------------------------
/**
 * \class SdCard
 * \brief Hardware access class for SD flash cards
 *  
 * Supports raw access to a standard SD flash memory card.  
 *
 */
class SdCard  {
  void error(uint8_t code, uint8_t data);
  void error(uint8_t code);
  uint8_t readReg(uint8_t cmd, uint8_t *dst);
  uint8_t readTransfer(uint8_t *dst, uint16_t count);
public:
  /** Code for a SD error. See SdCard.h for definitions. */
  uint8_t errorCode;
  /** Data that may be helpful in determining the cause of an error */
  uint8_t errorData;
  uint32_t cardSize(void);
  uint8_t init(uint8_t slow = 0);
  uint8_t readBlock(uint32_t block, uint8_t *dst);
  /** Read the CID register which contains info about the card.
   *  This includes Manufacturer ID, OEM ID, product name, version,
   *  serial number, and manufacturing date. */
  uint8_t readCID(cid_t &cid) {return readReg(CMD10, (uint8_t *)&cid);}
  uint8_t writeBlock(uint32_t block, uint8_t *src);
};
#endif //SdCard_h