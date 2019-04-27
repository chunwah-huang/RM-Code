#ifndef CONFIGURE_H
#define CONFIGURE_H

#include "CameraApi.h"
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <math.h>
#include <iostream>
#include <fcntl.h>  //文件控制定义
#include <termios.h>   //POSIX终端控制定义
#include <unistd.h>    //UNIX标准定义
#include <errno.h>     //ERROR数字定义
#include <sys/select.h>

using namespace std;
using namespace cv;

#define isopen_industry_capture 1
/**
  @brief: 是否使用工业相机
  @param: 0 使用工业相机
  @param: 1 使用普通USB相机
*/
#define capture_defult "/home/hzh/视频/camera_13.avi"
/**
  @brief: 相机的默认值
  @note: 使用普通USB相机时，Opencv的Videoture借口的值
*/

#define armor_color 1
/**
  @brief: 选择敌方阵营
  @param: 0 敌方为红色
  @param: 1 敌方为蓝色
*/

#define serialisopen 1
/**
  @brief: 是否启用串口
  @param: 0 不启用
  @param: 1 启用
*/

enum receive_serial_type{
   support_shooting_mode = 1,
   energy_agency_mode = 2,
};

enum big_chrysanthemum{
    //*quadrant_name
    first_quadrant = 1,
    second_quadrant = 2,
    third_quadrant = 3,
    fourth_quadrant = 4,
    origin = 5,

    fixed_mode = 6,
    follow_mode = 7,

    shoot_point_mode = 8,

    //*send_mode
    send_shoot_point = 10,

};

#endif // CONFIGURE_H