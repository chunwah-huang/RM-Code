/**
 * @file rm_bigchrysanthemum.h
 * @author ZhenHua (961716598@qq.com)
 * @brief 
 * @version 0.5
 * @date 2019-04-26
 * 
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 * 
 */
#ifndef RM_BIGCHRYSANTHEMUM_H
#define RM_BIGCHRYSANTHEMUM_H

#include "configure.h"
#include "serialport.h"
/** ------------------------------------------------------**
  @brief:　神符部分参数
*/

/** ------------------------------------------------------*/

class RM_BigChrysanthemum {
    public:
    RM_BigChrysanthemum();
    void imageProcessing(Mat frame);
    void find_energy_agency();

    private:
    bool is_target_rect(RotatedRect & rect);
    bool dis_circle_ellipse(Point2f & p1, Point2f & p2);
    Point2f follow_Shoot(Point2f & track_point, Point2f & circle_point, int & mode);

    int find_condition = 0;
    enum find_condition {
        find_ellipse = 1,
        find_circle = 2,
        find_rect = 3,
    };

    int find_direction = 0;
    enum find_direction {
        good = 1,
        poor = 2,
    };

    RotatedRect last_take_rect;

    Mat src_img;
    Mat hsv_img;
    Mat mix_img_1;
    Mat mix_img_2;
    Mat bin_img_1;
    Mat bin_img_2;

    Mat element_1;
    Mat element_2;
    Mat element_3;
    Mat element_4;

    int fps_count = 0;
    int ans_count = 0;
    int good_count = 0;
    int poor_count = 0;

    double rect_w;
    double rect_h;
    double area;
    double ratio;
    double last_rect_circle;

    Point2f rect_center;
    Point2f last_send_point;
};
#endif // RM_BIGCHRYSANTHEMUM_H