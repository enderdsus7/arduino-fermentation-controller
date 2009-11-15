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
#include <wiring.h>
#include "Fat16Config.h"
#include "SdCard.h"
//------------------------------------------------------------------------------
//r1 status values
#define R1_READY_STATE 0
#define R1_IDLE_STATE  1
//start data token for read or write
#define DATA_START_BLOCK      0XFE
//data response tokens for write block
#define DATA_RES_MASK         0X1F
#define DATA_RES_ACCEPTED     0X05
#define DATA_RES_CRC_ERROR    0X0B
#define DATA_RES_WRITE_ERROR  0X0D
//
// stop compiler from inlining where speed optimization is not required
#define STATIC_NOINLINE static __attribute__((noinline))
//------------------------------------------------------------------------------
// SPI static functions
//
// clock byte in
STATIC_NOINLINE uint8_t spiRec(void)
{
  SPDR = 0xff;
  while(!(SPSR & (1 << SPIF)));
  return SPDR;
}
// clock byte out
STATIC_NOINLINE void spiSend(uint8_t b)
{
  SPDR = b;
  while(!(SPSR & (1 << SPIF)));
}
// set Slave Select HIGH
STATIC_NOINLINE void spiSSHigh(void)
{
  digitalWrite(SS, HIGH);
}
// set Slave Select LOW
STATIC_NOINLINE void spiSSLow(void)
{
  digitalWrite(SS, LOW);
}
//------------------------------------------------------------------------------
static uint8_t cardCommand(uint8_t cmd, uint32_t arg)
{
  uint8_t r1;
  // some cards need extra clocks after transaction to go to ready state
  // easiest to put extra clocks before next transaction
  spiSSLow();
  spiRec();
  spiSend(cmd | 0x40);
  for (int8_t s = 24; s >= 0; s -= 8) spiSend(arg >> s);
  spiSend(cmd == CMD0 ? 0x95 : 0XFF);//must send valid CRC for CMD0
  //wait for not busy
  for (uint8_t retry = 0; (r1 = spiRec()) == 0xFF && retry != 0XFF; retry++);
  return r1;
}

//==============================================================================
// SdCard member functions
//------------------------------------------------------------------------------
#if SD_CARD_INFO_SUPPORT
/**
 * Determine the size of a standard SD flash memory card
 * \return The number of 512 byte data blocks in the card
 */ 
uint32_t SdCard::cardSize(void)
{
  uint16_t c_size;
  csd_t csd;
  if (!readReg(CMD9, (uint8_t *)&csd)) return 0;
  uint8_t read_bl_len = csd.read_bl_len;
  c_size = (csd.c_size_high << 10) | (csd.c_size_mid << 2) | csd.c_size_low;
  uint8_t c_size_mult = (csd.c_size_mult_high << 1) | csd.c_size_mult_low;
  return (uint32_t)(c_size+1) << (c_size_mult + read_bl_len - 7);
}
#endif //SD_CARD_INFO_SUPPORT
//------------------------------------------------------------------------------
void SdCard::error(uint8_t code, uint8_t data) 
{
  errorData = data; 
  error(code);
}
//------------------------------------------------------------------------------
void SdCard::error(uint8_t code) 
{
  errorCode = code; 
  spiSSHigh();
}
//------------------------------------------------------------------------------  
/**
 * Initialize a SD flash memory card.
 * 
 * \param[in] slow Set SPI Frequency F_CPU/4 if true else F_CPU/2. 
 *  
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure. 
 *
 */  
uint8_t SdCard::init(uint8_t slow)
{
  pinMode(SS, OUTPUT);
  spiSSHigh();
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  //Enable SPI, Master, clock rate F_CPU/128
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
  //must supply min of 74 clock cycles with CS high.
  for (uint8_t i = 0; i < 10; i++) spiSend(0XFF);
  spiSSLow();
  // next line prevent re-init hang by some cards (not sure why this works)
  for (uint16_t i = 0; i <= 512; i++) spiRec();
  uint8_t r = cardCommand(CMD0, 0);
  for (uint16_t retry = 0; r != R1_IDLE_STATE; retry++){
    if (retry == 10000) {
      error(SD_ERROR_CMD0, r);
      return false;
    }
    r = spiRec();
  }
  for (uint16_t retry = 0; ; retry++) {
    cardCommand(CMD55, 0);
    if ((r = cardCommand(ACMD41, 0)) == R1_READY_STATE)break;
    if (retry == 1000) {
      error(SD_ERROR_ACMD41, r);
      return false;
    }
  }
  // set SPI frequency
  SPCR &= ~((1 << SPR1) | (1 << SPR0)); // F_CPU/4
  if (!slow) SPSR |= (1 << SPI2X); // Doubled Clock Frequency to F_CPU/2
  spiSSHigh();
  return true;
}
//------------------------------------------------------------------------------
/**
 * Reads a 512 byte block from a storage device.
 *  
 * \param[in] blockNumber Logical block to be read.
 * \param[out] dst Pointer to the location that will receive the data. 
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.      
 */
uint8_t SdCard::readBlock(uint32_t blockNumber, uint8_t *dst)
{
  if (cardCommand(CMD17, blockNumber << 9)) {
    error(SD_ERROR_CMD17);
    return false;
  }
  return readTransfer(dst, 512);
}
//------------------------------------------------------------------------------
#if SD_CARD_INFO_SUPPORT
uint8_t SdCard::readReg(uint8_t cmd, uint8_t *dst)
{
  if (cardCommand(cmd, 0)) {
    spiSSHigh();
    return false;
  }
  return readTransfer(dst, 16);
}
#endif //SD_CARD_INFO_SUPPORT
//------------------------------------------------------------------------------
uint8_t SdCard::readTransfer(uint8_t *dst, uint16_t count)
{
  //wait for start of data
  for (uint16_t retry = 0; spiRec() != DATA_START_BLOCK; retry++) {
    if (retry == 0XFFFF) {
      error(SD_ERROR_READ_TIMEOUT);
      return false;
    }
  }
  //start first spi transfer
  SPDR = 0XFF;
  for (uint16_t i = 0; i < count; i++) {
    while(!(SPSR & (1 << SPIF)));
    dst[i] = SPDR;
    SPDR = 0XFF;  
  }
  // wait for first CRC byte
  while(!(SPSR & (1 << SPIF)));  
  spiRec();//second CRC byte
  spiSSHigh();
  return true;
}
//------------------------------------------------------------------------------
/**
 * Writes a 512 byte block to a storage device.
 *  
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written. 
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.      
 */   
uint8_t SdCard::writeBlock(uint32_t blockNumber, uint8_t *src)
{
  uint32_t address = blockNumber << 9;
#if SD_PROTECT_BLOCK_ZERO
  //don't allow write to first block
  if (address == 0) {
    error(SD_ERROR_BLOCK_ZERO_WRITE);
    return false;
  }
#endif //SD_PROTECT_BLOCK_ZERO
  if (cardCommand(CMD24, address)) {
    error(SD_ERROR_CMD24);
    return false;
  }
  // optimize write loop
  SPDR = DATA_START_BLOCK;
  for (uint16_t i = 0; i < 512; i++) {
    while(!(SPSR & (1 << SPIF)));
    SPDR = src[i];
  }
  while(!(SPSR & (1 << SPIF)));// wait for last data byte
  spiSend(0xFF);// dummy crc
  spiSend(0xFF);// dummy crc
  uint8_t r1 = spiRec();
  if ((r1 & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
    error(SD_ERROR_WRITE_RESPONSE, r1);
    return false;
  }
  // wait for card to complete write programming
  for (uint16_t retry = 0; spiRec() != 0XFF ; retry++) {
    if (retry == 0XFFFF) {
      error(SD_ERROR_WRITE_TIMEOUT);
      return false;
    }
  }
  spiSSHigh();
  return true;
}