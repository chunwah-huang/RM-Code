#ifndef SERIALPORT_H
#define SERIALPORT_H

#include "configure.h"

/** ------------------------------------------------------**
  @brief: 串口部分参数
*/

/** Serious **/
#define DEVICE_NAME "/dev/ttyUSB0"
/**
  @brief: 设置串口号
  @brief: 设置波特率
  @param: BANDRATE_1 波特率115200
  @param: BANDRATE_2 波特率1500000
*/

#define BUFF_LENGTH 16
#define BUFF_TEMP_LENGTH 12
/**
 * @brief 
 */

#define PARMODE_CRC_1
/**
  @brief: CRC校验
  @param: PARMODE_CRC_0  无CRC校验位
  @param: PARMODE_CRC_1  有CRC校验位
*/

/** ReceiveMode **/
#define RX_MODE_1
/** ------------------------------------------------------**/

class SerialPort
{
public:
    //串口标志量
    static int fd;
    static char g_buf[BUFF_LENGTH];
    static char g_buf_temp[BUFF_TEMP_LENGTH];

public:
    /**              函数声明            **/
    SerialPort();

    static void serialSet(int port_No);
    //自定义串口发送
    static void RMserialWrite(int x,int y,int depth,int mode = 1,int mode_select = 1);
    //延时函数，防止发送过快
    //void sleepUS(unsigned int secs);　usleep代替 @edit:4.10 hzk
    static int RMreceiveData();
    static int RMreceiveData_energy();
    static void RM_receiveData_armor(int16_t &gyroscope_data,bool &is_receive);
    static uint8_t Checksum_CRC8(char *buf,uint16_t len);
public:
    ~SerialPort();
};



const unsigned char CRC8Tab[300]=
{
    0,94,188,226,97,63,221,131,194,156,126,32,163,253,31,65,
    157,195,33,127,252,162,64,30, 95,1,227,189,62,96,130,220,
    35,125,159,193,66,28,254,160,225,191,93,3,128,222,60,98,
    190,224,2,92,223,129,99,61,124,34,192,158,29,67,161,255,
    70,24,250,164,39,121,155,197,132,218,56,102,229,187,89,7,
    219,133,103,57,186,228,6,88,25,71,165,251,120,38,196,154,
    101,59,217,135,4,90,184,230,167,249,27,69,198,152,122,36,
    248,166,68,26,153,199,37,123,58,100,134,216,91,5,231,185,
    140,210,48,110,237,179,81,15,78,16,242,172,47,113,147,205,
    17,79,173,243,112,46,204,146,211,141,111,49,178,236,14,80,
    175,241,19,77,206,144,114,44,109,51,209,143,12,82,176,238,
    50,108,142,208,83,13,239,177,240,174,76,18,145,207,45,115,
    202,148,118,40,171,245,23,73,8,86,180,234,105,55,213,139,
    87,9,235,181,54,104,138,212,149,203, 41,119,244,170,72,22,
    233,183,85,11,136,214,52,106,43,117,151,201,74,20,246,168,
    116,42,200,150,21,75,169,247,182,232,10,84,215,137,107,53
};

#endif // SERIALPORT_H
