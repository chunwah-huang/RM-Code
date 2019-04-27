#include "serialport.h"

int SerialPort::fd;
char SerialPort::g_buf[BUFF_LENGTH];
char SerialPort::g_buf_temp[BUFF_TEMP_LENGTH];
/**
 * @brief Construct a new Serial Port:: Serial Port object
 */
SerialPort::SerialPort() {
    cout << "The Serial set ......" << endl;
}

/**
 * @brief Destroy the Serial Port:: Serial Port object
 */
SerialPort::~SerialPort(void) {
    if (!close(fd))
        printf("Close Serial Port Successful\n");
}

/**
 * @brief: 初始化串口函数
 * ------------------------------------------------------
 * @param:  波特率,默认为115200
 * --------------------------------------------------------
 * @param:  char parity 是否进行奇偶校验,'Y'表示需要奇偶校验,'N'表示不需要奇偶校验
 * -------------------------------------------------------------
 * @param:  int databits 数据位的个数,默认值为8个数据位
 *----------------------------------------------------------
 * @return: bool  初始化是否成功
 * @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化
 *　　　　　   函数提供了一些常用的串口参数设置
 *           本串口类析构时会自动关闭串口,无需额外执行关闭串口
 * @author: Hzkkk
 *          Rcxxx (revised)
 */
void SerialPort::serialSet(int port_No) {
    const char * DeviceName[4] = {
        "",
        "/dev/ttyUSB0",
        "/dev/ttyUSB1",
        "/dev/ttyUSB2"
    };

    /* WARNING :  终端设备默认会设置为控制终端，因此open(O_NOCTTY不作为控制终端)
     * Terminals'll default to be set as Control Terminals
     */
    struct termios newstate;
    /*打开串口*/
    bzero( & newstate, sizeof(newstate)); //清零
    fd = open(DeviceName[port_No], O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Can't Open Serial Port\n");
    } else
        printf("Open Serial Port %s Successful\n", DeviceName[port_No]);

    /*改为阻塞模式*/
    //    if (fcntl(fd, F_SETFL, 0) < 0)
    //        printf("fcntl failed!\n");
    //    else
    //        printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
    //tcgetattr(fd, &newstate);
    /*设置发送波特率*/
    cfsetospeed( & newstate, B115200);
    cfsetispeed( & newstate, B115200);

    //本地连线, 取消控制功能 | 开始接收
    newstate.c_cflag |= CLOCAL | CREAD;
    //设置字符大小
    newstate.c_cflag &= ~CSIZE;
    //设置停止位1
    newstate.c_cflag &= ~CSTOPB;
    //设置数据位8位
    newstate.c_cflag |= CS8;
    //设置无奇偶校验位，N
    newstate.c_cflag &= ~PARENB;

    /*阻塞模式的设置*/
    newstate.c_cc[VTIME] = 0;
    newstate.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    tcsetattr(fd, TCSANOW, & newstate);
}

/**
 *  @brief: 串口数据读取函数
 *  @return: string  返回收到的字符串
 *  @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化
 *           逐字节读取并存到字符串
 *           等待0.01s后结束读取,将所得字符串返回
 *  @authors: Rcxxx
 *            Hzkkk
 */
int SerialPort::RMreceiveData() {
    char rece_buf_temp[8];
    /** ('S',1,1,'E') **/
    string rece_buf = "";
    unsigned int rece_COUNT = 0;
    unsigned int rece_First_COUNT = 0;
    //NOTE: Default 
    unsigned int case_mode = support_shooting_mode;

    do {
        memset(rece_buf_temp, 0, 8); //清空缓存
        int isRead = read(fd, rece_buf_temp, 8); //接收数据
        usleep(10);
        if (0 >= isRead && !rece_buf.empty()) { //如果这一次沒有收到数据，且未成功收到
            rece_COUNT++;
            if (rece_COUNT > 10) break;
        } else if (0 < isRead) { //如果有收到数据
            //成功收到数据,计数归0
            rece_COUNT = 0;
            //将读到的数据存入rece_buf
            rece_buf.append((char * ) rece_buf_temp, isRead);
        } else { //首次读取成功,记录第一次成功读取
            rece_First_COUNT++;
        }
    } while (rece_First_COUNT + rece_COUNT < 100); //设定串口读取次数

    //printf("Receive:%s",rece_buf);

    if (rece_buf[0] == '1') {
        case_mode = support_shooting_mode;
    } else if (rece_buf[0] == '2') {
        cout << rece_buf << endl;
        if (rece_buf[1] == '2') {
            case_mode = shoot_point_mode;
            cout << "展鸿甩锅" << endl;
        } else {
            case_mode = shoot_point_mode;
            cout << "展鸿甩锅222" << endl;
        }
    } else {
        //case_mode = support_shooting_mode;
        case_mode = energy_agency_mode;
    }


    return case_mode;
}

int SerialPort::RMreceiveData_energy() {
    char rece_buf_temp[4];
    /** ('S',1,2,'E') **/
    string rece_buf = "";
    unsigned int rece_COUNT = 0;
    unsigned int rece_First_COUNT = 0;
    int case_mode;

    do {
        memset(rece_buf_temp, 0, 8); //清空缓存
        int isRead = read(fd, rece_buf_temp, 8); //接收数据
        if (0 >= isRead && !rece_buf.empty()) { //如果这一次沒有收到数据，且未成功收到
            rece_COUNT++;
            if (rece_COUNT > 10) break;
        } else if (0 < isRead) { //如果有收到数据
            //成功收到数据,计数归0
            rece_COUNT = 0;
            //将读到的数据存入rece_buf
            rece_buf.append((char * ) rece_buf_temp, isRead);
        } else { //首次读取成功,记录第一次成功读取
            rece_First_COUNT++;
        }
    } while (rece_First_COUNT + rece_COUNT < 10); //设定串口读取次数

    cout << rece_buf << endl;
    if (rece_buf[1] == '2') {
        case_mode = shoot_point_mode;
        cout << "展鸿甩锅" << endl;
    } else {
        case_mode = shoot_point_mode;
        cout << "展鸿甩锅222" << endl;
    }
    //    else if (rece_buf[1] == '2') {
    //        case_mode =  shoot_point_quadrant;
    //    }
    //    else if (rece_buf[1] == '3') {
    //        case_mode =  Refresh_quadrant;
    //    }

    return case_mode;
}

void SerialPort::RM_receiveData_armor(int16_t & gyroscope_data, bool & is_receive) {
    is_receive = false;
    char receive[3];
    read(fd, receive, sizeof(receive));

    if (receive[0] == 'T') {
        gyroscope_data = (receive[1] << 8 | receive[2]);
        is_receive = true;
    }
}

/**
 *@brief: RM串口发送格式化函数
 *
 * @param: x 坐标的ｘ值
 * @param: y 坐标的ｙ值
 * @param: SendDataFlag 发送的标志
 *
 * @authors: Rcxxx
 *           Hzkkk
 */
void SerialPort::RMserialWrite(int x, int y, int depth, int mode, int mode_select) {
    sprintf(g_buf_temp, "%c%1d%1d%03d%03d%03d", 'S', mode, mode_select, x, y, depth);
    uint8_t CRC = Checksum_CRC8(g_buf_temp, sizeof(g_buf_temp));
    sprintf(g_buf, "%c%1d%1d%03d%03d%03d%03d%c", 'S', mode, mode_select, x, y, depth, CRC, 'E');
    write(fd, g_buf, sizeof(g_buf));
    // std::cout<<"depth: "<<depth<<endl;
    //std::cout << "g_buf: " << g_buf << std::endl;
    usleep(1);
}

/** CRC8校验函数
 *
 *  @param:  char *buf   需要检验的字符串
 *  @param:  uint16_t len 是否进行奇偶校验,'Y'表示需要奇偶校验,'N'表示不需要奇偶校验
 *
 *  @return: bool  初始化是否成功
 *  @brief:  CRC8校验 ---MAXIM x8+x5+x4+x1  多项式 POLY（Hex）:31(110001)  初始值 INIT（Hex）：00  结果异或值 XOROUT（Hex）：
 *  @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化
 *　　　　　   函数提供了一些常用的串口参数设置
 *           本串口类析构时会自动关闭串口,无需额外执行关闭串口
 */
uint8_t SerialPort::Checksum_CRC8(char * buf, uint16_t len) {
    uint8_t check = 0;

    while (len--) {
        check = CRC8Tab[check ^ ( * buf++)];
    }

    return (check) & 0x00ff;
}