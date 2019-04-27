#ifndef RM_ARMORFITTED_H
#define RM_ARMORFITTED_H
#include "configure.h"
#include "serialport.h"
#include "rm_kalmanfilter.h"

/** ------------------------------------------------------**
  @brief:　轮廓匹配部分参数
*/
#define AREA_RATIO_MIN 0.4
#define AREA_RATIO_MAX 1.1
/**
  @param: AREA_RATIO_MIN  面积之比的最小阈值
  @param: AREA_RATIO_MAX  面积之比的最大阈值
*/

#define LOW_MULTIPLE 2.110
#define HIGH_MULTIPLE 28.888
/**
  @param: LOW_MULTIPLE  中点距离最小阈值的倍率参数
  @param: HIGH_MULTIPLE  中点距离最大阈值的倍率参数
*/

#define CENTER_SLPOE_MAX 0.25
/**
  @param: CENTER_SLPOE_MAX  中点间斜率的阈值
*/

/** ------------------------------------------------------**/

/** ------------------------------------------------------**
  @brief:　测距部分参数
*/
#define DISTANCE_PERIMETER_RATIO_TH 3.6550
/**
  @param: DISTANCE_PERIMETER_RATIO_TH  中心距离与半周长之比
  @note: 用来区分大小装甲
*/

#define BIG_ARMOR_TRUE_LENGTH 2300
#define SMALL_ARMOR_TRUE_LENGTH 1350
/**
  @param: big_armor_true_length  大装甲的真实宽度
  @param: Small_armor_true_length  小装甲的真实宽度
*/

/** ------------------------------------------------------**/

class RM_ArmorFitted
{
private:
    /** param initial **/
    Mat src_img;    //原图
    Mat gray_img;   //灰度图
    Mat bin_img;    //二值图
    Mat dst_img;    //输出图
    const unsigned int THRESHOLD_VALUE_BLUE = 20;
    const unsigned int THRESHOLD_VALUE_RED = 20;
    /** param initial **/
    RM_kalmanfilter kf;

    unsigned short int new_frame_none_count = 0;
    bool last_frame_has_data = false;
    bool new_frame_has_data = false;
    

    const float ELLIPSE_SIZE = 80;
    
    int x_buff = 320;
    int y_buff = 240;
    unsigned short int sendbuff_count = 0;
    const unsigned int buff_time = 5;
    bool buff_frame_data = false;
    

public:
    /**
     * @brief 执行函数部分
     */
    RM_ArmorFitted() :kf()
    {
        cout<<"Armor is ready"<<endl;
    }
    void imageProcessing(Mat frame);
    void armorFitted();

private:
    /**
     *@brief: 功能函数部分
     */
    int bufferReturn(unsigned int buff,unsigned int value,unsigned int flag);
    float centerDistance(Point p1,Point p2);
    float distancetoCamera(const RotatedRect &rect1, const RotatedRect &rect2);
    bool iscentral_region(Point point, const float ranging_results);
    bool isarmoredColorroi(const RotatedRect &rect);
    bool can_contour_match(const RotatedRect &rect1, const RotatedRect &rect2);
};

#endif // RM_ARMORFITTED_H
