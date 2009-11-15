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
#ifndef Fat16_h
#define Fat16_h
#include <string.h>
#include <avr/pgmspace.h>
#include "SdCard.h"
#include "Print.h"
#include "FatStructs.h"
#include "Fat16Config.h"
/**
 * \typedef fat_t
 *
 * \brief Type for FAT16 entry
 */
typedef uint16_t fat_t;
/**
 * \union cache16_t
 *  
 * \brief Cache buffer data type
 *
 */  
union cache16_t {
          /** Used to access cached file data blocks. */
  uint8_t data[512];
          /** Used to access cached FAT entries. */
  fat_t   fat[256];
          /** Used to access cached directory entries. */
  dir_t   dir[16];
          /** Used to access a cached Master Boot Record. */
  mbr_t   mbr;
          /** Used to access to a cached FAT16 boot sector. */
  fbs_t   fbs;       
};
// use the gnu style oflags
/** open for reading */
#define O_READ    0X01
/** same as O_READ */
#define O_RDONLY  O_READ
/** open for write */
#define O_WRITE   0X02
/** same as O_WRITE */
#define O_WRONLY  O_WRITE
/** open for reading and writing */
#define O_RDWR    (O_READ | O_WRITE)
/** mask for access modes */
#define O_ACCMODE (O_READ | O_WRITE) 
/** The file offset shall be set to the end of the file prior to each write. */
#define O_APPEND  0X04
/** synchronous writes - call sync() after each write */
#define O_SYNC    0X08
/** create the file if nonexistent */
#define O_CREAT   0X10 
/** If O_CREAT and O_EXCL are set, open() shall fail if the file exists */
#define O_EXCL    0X20 
/* truncate the file to zero length */
#define O_TRUNC   0X40
//------------------------------------------------------------------------------
/** \class Fat16
 * \brief Fat16 implements a minimal Arduino FAT16 Library
 *  
 * Fat16 does not support subdirectories or long file names. 
 */ 
#if FAT16_PRINT_SUPPORT
class Fat16 : public Print {
#else // FAT16_PRINT_SUPPORT
class Fat16 {
#endif //FAT16_PRINT_SUPPORT
  // Volume info
  static uint8_t  volumeInitialized_;  //true if volume has been initialized
  static uint8_t  fatCount_;           //number of FATs
  static uint8_t  blocksPerCluster_;   //must be power of 2, max cluster size 32K
  static uint16_t rootDirEntryCount_;  //should be 512 for FAT16
  static fat_t    blocksPerFat_;       //number of blocks in one FAT
  static fat_t    clusterCount_;       //total clusters in volume
  static uint32_t fatStartBlock_;      //start of first FAT
  static uint32_t rootDirStartBlock_;  //start of root dir
  static uint32_t dataStartBlock_;     //start of data clusters
  //block cache
#define CACHE_FOR_READ   0           //cache a block for read
#define CACHE_FOR_WRITE  1           //cache a block and set dirty for the block
  static SdCard *rawDev_;             // Device  
  static cache16_t cacheBuffer_;     //512 byte cache for storage device blocks
  static uint32_t cacheBlockNumber_; //Logical number of block in the cache
  static uint8_t cacheDirty_;        //cacheFlush() will write block if true
  static uint32_t cacheMirrorBlock_ ;// mirror block for second FAT
  // define fields in flags_
#define F_OFLAG (O_ACCMODE | O_APPEND | O_SYNC ) // should be 0XF
#define F_FILE_SIZE_DIRTY 0X80 // sync of directory entry file size required  
  uint8_t flags_;         // see above for bit definitions
  int16_t dirEntryIndex_; // index of directory entry for open file
  fat_t firstCluster_;    // first cluster of file
  uint32_t fileSize_;     // fileSize
  fat_t curCluster_;      // current cluster
  uint32_t curPosition_;  // current byte offset
  //private functions for cache
  static uint8_t blockOfCluster(uint32_t position) {
    // depends on blocks per cluster being power of two
    return (position >> 9) & (blocksPerCluster_ - 1);}
  static uint16_t cacheDataOffset(uint32_t position) {return position & 0X1FF;}
  static dir_t *cacheDirEntry(uint16_t index, uint8_t action = 0);
  static uint8_t cacheRawBlock(uint32_t blockNumber, uint8_t action = 0);
  static uint8_t cacheFlush(void);
  static void cacheSetDirty(void) {cacheDirty_ |= CACHE_FOR_WRITE;}
  static uint32_t dataBlockLba(fat_t cluster, uint8_t blockOfCluster) {
        return dataStartBlock_ + (uint32_t)(cluster - 2)*blocksPerCluster_ 
        + blockOfCluster;}
  static uint8_t fatGet(fat_t cluster, fat_t &value);
  static uint8_t fatPut(fat_t cluster, fat_t value);
  //find and cache a directory entry
  static dir_t *findDirEntry(uint16_t &entry, uint8_t *name,
                             uint8_t skip = DIR_ATT_VOLUME_ID);
  //end of chain test
  static uint8_t isEOC(fat_t cluster) {return cluster >= 0XFFF8;}
  //allocate a cluster to a file
  uint8_t addCluster(void);
  //free a cluster chain
  uint8_t freeChain(fat_t cluster);
  
//------------------------------------------------------------------------------
public:
  /*
   * Public functions
   */
  // create with file closed
  Fat16(void) : flags_(0) {}
  /** \return The current cluster number. */
  fat_t curCluster(void) {return curCluster_;}
  uint8_t close(void);
  /** \return The count of clusters in the FAT16 volume. */
  static fat_t clusterCount(void) {return clusterCount_;}
  /** \return The number of 512 byte blocks in a cluster */
  static uint8_t clusterSize(void) {return blocksPerCluster_;}
  /** \return The current file position. */
  uint32_t curPosition(void) {return curPosition_;}
  /** \return The file's size in bytes. */ 
  uint32_t fileSize(void) {return fileSize_;}   
  static uint8_t init(SdCard &dev, uint8_t part);
  /**
   *  Initialize a FAT16 volume.
   *
   *  First try partition 1 then try super floppy format.
   *
   * \param[in] dev The SdCard where the volume is located.
   *
   * \return The value one, true, is returned for success and
   * the value zero, false, is returned for failure.  reasons for
   * failure include not finding a valid FAT16 file system, a call
   * to init() after a volume has been successful initialized or 
   * an I/O error.
   *
   */
  static uint8_t init(SdCard &dev) {
  if (init(dev, 1)) return true; return init(dev, 0);}
  /**
   * Checks the file's open/closed status for this instance of Fat16. 
   * \return The value true if a file is open otherwise false;
   */     
  uint8_t isOpen(void) {return (flags_ & O_ACCMODE) != 0;}
  uint8_t open(const char *fileName, uint8_t oflag);  
  uint8_t open(uint16_t entry, uint8_t oflag);
  int16_t read(void);
  int16_t read(void *buf, uint16_t nbyte);                
  static uint8_t readDir(dir_t &dir, uint16_t &index,
                    uint8_t skip = (DIR_ATT_VOLUME_ID | DIR_ATT_DIRECTORY));
  static dir_t *readDir(uint16_t &index,
                    uint8_t skip = (DIR_ATT_VOLUME_ID | DIR_ATT_DIRECTORY));
  /** \return The number of entries in the root directory. */     
  static uint16_t rootDirEntryCount(void) {return rootDirEntryCount_;}
  uint8_t remove(void);
  /**  Seek to current position plus \a pos bytes. See Fat16::seekSet(). */
  uint8_t seekCur(uint32_t pos) {return seekSet(curPosition_ + pos);}
  /** Seek to end of file.  See Fat16::seekSet(). */
  uint8_t seekEnd(void) {return seekSet(fileSize_);}
  uint8_t seekSet(uint32_t pos);
  uint8_t sync(void);
  uint8_t truncate(uint32_t size);
  /** Fat16::writeError is set to true if an error occurs during a write(). 
   * Set Fat16::writeError to false before calling print() and/or write() and check
   * for true after calls to write() and/or print().
   */
  bool writeError;
  int16_t write(const void *buf, uint16_t nbyte);
  void write(uint8_t b);
  void write(const char *str);
  void write_P(PGM_P str);
  void writeln_P(PGM_P str);
//------------------------------------------------------------------------------
#if FAT16_DEBUG_SUPPORT
  /** For debug only.  Do not use in applications. */
  static cache16_t *dbgBufAdd(void) {return &cacheBuffer_;}
  /** For debug only.  Do not use in applications. */
  static void dbgSetDev(SdCard &dev) {rawDev_ = &dev;}
  /** For debug only.  Do not use in applications. */
  static uint8_t *dbgCacheBlock(uint32_t blockNumber) {
    return cacheRawBlock(blockNumber) ? cacheBuffer_.data : 0; }
  /** For debug only.  Do not use in applications. */
  static dir_t *dbgCacheDir(uint16_t index) {
    return cacheDirEntry(index);}
#endif //FAT16_DEBUG_SUPPORT
};
#endif//Fat16_h
