//
// Created by 0-0 mashuo on 2023/3/6.
//

#ifndef RTK_READB_H
#define RTK_READB_H

// 常量定义
#include <cstdint>
#include <cstring>
#include "GnssData.h"

#define OEM7SYNC1       0xAA    /* oem7/6/4 message start sync code 1 */
#define OEM7SYNC2       0x44    /* oem7/6/4 message start sync code 2 */
#define OEM7SYNC3       0x12    /* oem7/6/4 message start sync code 3 */
#define ID_RANGE        43      /* oem7/6/4 range measurement */
#define ID_GPSEPH       7
#define ID_BDSEPH       1696
#define MAXROWLEN       16384
#define POLYCRC32   0xEDB88320u /* CRC32 polynomial */

static uint16_t U2(uint8_t *p) ;
static uint32_t U4(uint8_t *p) ;
static int32_t  I4(uint8_t *p) ;
static float    R4(uint8_t *p) ;
static double   R8(uint8_t *p) ;

// 读取文件
static int readsys(uint8_t *buff,OBS* obs);
unsigned int crc32(const unsigned char *buff, int len);
static bool readHeader(uint8_t *buff,FILE * file,HeaderB *header);
static bool readObs(uint8_t * buff,HeaderB *header,OBS *obs);
static bool readGPSEPH(uint8_t *buff,HeaderB *header,GPSEPH *eph);
static bool readBDSEPH(uint8_t *buff,HeaderB *header,BDSEPH *eph);

// 端口读取头文件
static bool port_readHeader(uint8_t *buff, HeaderB *headerB, int &);

// 检验
void print_obs(OBS obs);
void print_eph(GPSEPH *eph);
void print_eph(BDSEPH *eph);

// 读取二进制文件并进行单点定位
bool read_B_SPP(const char *filename);

// 读取二进制文件进行RTK解算
void read_B_RTK(const char *base_file,const char *mobile_file);

// 读取单站数据
int read_station(FILE *file,uint8_t *buff,OBS *obs,GPSEPH *gpseph,BDSEPH *bdseph,HeaderB *headerB);
int port_read_station(uint8_t *buff,OBS *obs,GPSEPH *gpseph,BDSEPH *bdseph,HeaderB *headerB,int &);

// 读取配置文件
void read_setting();

// 判断是否为头文件
bool is_header(uint8_t buff[],int &);


#endif //RTK_READB_H
