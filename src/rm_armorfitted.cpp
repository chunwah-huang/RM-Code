/**
 * @file rm_armorfitted.cpp
 * @author Rcxxx_ (chuangxinr@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-03-29
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#include "rm_armorfitted.h"

/**
 * @brief 
 * @param buff 
 * @param value 
 * @param flag 
 * @return int 
 */
int RM_ArmorFitted::bufferReturn(unsigned int buff, unsigned int value, unsigned int flag) {
    unsigned int back_value;
    unsigned int max_value;
    if (flag == 0) {
        max_value = src_img.cols / 2;
    } else {
        max_value = src_img.rows / 2;
    }

    if (buff < max_value) {
        back_value = buff + value / (buff_time - sendbuff_count);
    } else {
        back_value = buff - value / (buff_time - sendbuff_count);
    }

    return back_value;
}

/**
 * @brief :get distance between two Point
 * @param p1 
 * @param p2 
 * @return float 
 */
float RM_ArmorFitted::centerDistance(Point p1, Point p2) {
    float D = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    return D;
}

/**
 * @brief :模糊单目测距函数
 * @param rect1 minAreaRect
 * @param rect2 minAreaRect
 * @return float 
 * @author: jiajia
 * @note: 该函数为模糊测距，仅为参考值，误差在　0-20cm左右
 */
float RM_ArmorFitted::distancetoCamera(const RotatedRect & rect1,const RotatedRect & rect2) {
    float Distance_to_camera;
    float rects_distance = centerDistance(rect1.center, rect2.center);
    unsigned int half_Perimeter = rect1.size.width + rect1.size.height;
    unsigned int true_length;
    float distance_Perimeter_ratio = rects_distance / (float) half_Perimeter;
    if (distance_Perimeter_ratio > 3.6550) {
        true_length = 2300; /** big armor **/
    } else {
        true_length = 1350; /** Small armor **/
    }
    //float  focal_length  = 7.00;   //普通相机焦距 6mm
    float focal_length = 10.3; //工业相机焦距 12mm
    Distance_to_camera = (true_length * focal_length) / rects_distance;
    return Distance_to_camera;
}

/**
 * @brief :HSV颜色空间检测 ROI区域
 * @param rect :minAreaRect
 * @return true :目标区域符合颜色要求
 * @return false :目标区域不符合颜色要求
 * @author: Rcxxx
 */
bool RM_ArmorFitted::isarmoredColorroi(const RotatedRect & rect) {
    bool is_color = false;
    Mat roi_img;
    Mat hsv_roi_img;
    Point2f verices[4];
    Point2f verdst[4];
    unsigned int roi_w;
    unsigned int roi_h;
    rect.points(verices);
    if (rect.size.width > rect.size.height) {
        roi_w = rect.size.height;
        roi_h = rect.size.width;
        verdst[0] = Point2f(0, roi_h);
        verdst[1] = Point2f(0, 0);
        verdst[2] = Point2f(roi_w, 0);
        verdst[3] = Point2f(roi_w, roi_h);
    } else {
        roi_w = rect.size.width;
        roi_h = rect.size.height;
        verdst[0] = Point2f(roi_w, roi_h);
        verdst[1] = Point2f(0, roi_h);
        verdst[2] = Point2f(0, 0);
        verdst[3] = Point2f(roi_w, 0);
    }

    roi_img = Mat(roi_h, roi_w, CV_8UC1);
    Mat warpMatrix = getPerspectiveTransform(verices, verdst);
    warpPerspective(src_img, roi_img, warpMatrix, roi_img.size(), INTER_LINEAR, BORDER_CONSTANT);

    cvtColor(roi_img, hsv_roi_img, COLOR_BGR2HSV);
    float H = 0.0, S = 0.0, V = 0.0;
    unsigned int flag = 0;
    for (int x = 0; x < hsv_roi_img.cols; ++x) {
        for (int y = 0; y < hsv_roi_img.rows; ++y) {
            H = hsv_roi_img.at < Vec3b > (y, x)[0];
            S = hsv_roi_img.at < Vec3b > (y, x)[1];
            V = hsv_roi_img.at < Vec3b > (y, x)[2];
            //red
            if (armor_color == 0) {
                if ((H >= 145 && H < 180) || (H >= 0 && H <= 13)) {
                    if (S >= 135 && S <= 255) {
                        if (V > 148 && V <= 255) {
                            flag += 1;
                        }
                    }
                }
            }
            //blue
            else {
                if (H >= 75 && H <= 130) {
                    if (S >= 195 && S <= 255) {
                        if (V >= 185 && V <= 255) {
                            flag += 1;
                        }
                    }
                }
            }
            if (armor_color == 0) {
                if ((flag / hsv_roi_img.cols * hsv_roi_img.rows) > 0.5) {
                    is_color = 1;
                    continue;
                }
            } else {
                if ((flag / hsv_roi_img.cols * hsv_roi_img.rows) > 0.3) {
                    is_color = 1;
                    continue;
                }
            }
        }
    }
    return is_color;
}

/**
 * @brief 
 * @param rect1 需要匹配的两个　minAreaRect
 * @param rect2 需要匹配的两个　minAreaRect
 * @return true 匹配成功
 * @return false 匹配失败
 * @authors :Rcxxx
 *          :jiajia
 */
bool RM_ArmorFitted::can_contour_match(const RotatedRect & rect1,const RotatedRect & rect2) {
    bool iscontour_matching = false;

    /** get area ratio **/
    float area_rect1 = (float) rect1.size.width * (float) rect1.size.height;
    float area_rect2 = (float) rect2.size.width * (float) rect2.size.height;
    float area_ratio = (area_rect1 > area_rect2) ? (area_rect2 / area_rect1) : (area_rect1 / area_rect2);
    /** get area ratio **/
    if (AREA_RATIO_MIN < area_ratio && area_ratio < AREA_RATIO_MAX) {
        /** exchange width and height **/
        float w1 = (rect1.size.height > rect1.size.width ? rect1.size.width : rect1.size.height);
        float h1 = (rect1.size.height > rect1.size.width ? rect1.size.height : rect1.size.width);
        float w2 = (rect2.size.height > rect2.size.width ? rect2.size.width : rect2.size.height);
        float h2 = (rect2.size.height > rect2.size.width ? rect2.size.height : rect2.size.width);
        /** exchange width and height **/

        /*- get center -*/
        float x1 = rect1.center.x;
        float y1 = rect1.center.y;
        float x2 = rect2.center.x;
        float y2 = rect2.center.y;
        /*- get center -*/
        float center_slope = fabs((y1 - y2) / (x1 - x2));

        if (center_slope <= CENTER_SLPOE_MAX || center_slope == 0) {
            float center_distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
            float low = max(w1, w2) * LOW_MULTIPLE;
            float high = max(w1, w2) * HIGH_MULTIPLE;
            if (low < center_distance && center_distance < high) {
                float max_h = h1 > h2 ? h1 : h2;
                if (center_distance > max_h && center_distance < max_h * 4) {
                    iscontour_matching = true;
                }
            }
        }
    } else {
        iscontour_matching = false;
    }
    return iscontour_matching;
}

/**
 * @brief :无效区域判断函数
 * @param point 当前点坐标值
 * @param ranging_results 测距结果，使区域大小自适应
 * @return true :目标点处于无效区域
 * @return false :目标点处于有效区域
 * @authors: Rcxxx
 */
bool RM_ArmorFitted::iscentral_region(Point point,const float ranging_results) {
    bool iscentral_region_point;
    Point2f center = (Point2f) Point(src_img.cols / 2, src_img.rows / 2);
    float ellipse_size = ELLIPSE_SIZE;
    Point2f left_focus = Point2f(center.x - (ellipse_size * 0.5 * 0.8), center.y);
    Point2f right_focus = Point2f(center.x + (ellipse_size * 0.5 * 0.8), center.y);
    float PF1 = centerDistance(point, left_focus);
    float PF2 = centerDistance(point, right_focus);
    if ((PF1 + PF2) <= ellipse_size) {
        iscentral_region_point = true;
    } else {
        iscentral_region_point = false;
    }
    ellipse(dst_img, point, Size(ellipse_size * 0.5, sqrt(pow(ellipse_size * 0.5, 2) - pow(ellipse_size * 0.5 * 0.8, 2))), 0, 0, 360, Scalar(255, 129, 0), 2, 8);
    ellipse(dst_img, center, Size(ellipse_size * 0.5, sqrt(pow(ellipse_size * 0.5, 2) - pow(ellipse_size * 0.5 * 0.8, 2))), 0, 0, 360, Scalar(255, 129, 20), 2, 8);
    return iscentral_region_point;
}

/**
 * @brief 图像预处理函数
 * @param frame 输入的原图
 */
void RM_ArmorFitted::imageProcessing(Mat frame) {
    src_img = frame;
    int threshold_Value;
    if (armor_color == 0) {
        threshold_Value = THRESHOLD_VALUE_RED;
    } else {
        threshold_Value = THRESHOLD_VALUE_BLUE;
    }
    //resize(src_img,src_img,Size(640,480),INTER_NEAREST);
    src_img.copyTo(dst_img);
    /** -------------Image Segmentation	---------------**/
    cvtColor(src_img, gray_img, COLOR_BGR2GRAY);
    threshold(gray_img, bin_img, threshold_Value, 255, THRESH_BINARY);
    medianBlur(bin_img, bin_img, 5);
    //Canny(bin_img, bin_img, 120, 240);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(bin_img, bin_img, element);
    /** -------------Image Segmentation	---------------**/
}

/**
 * @brief 识别部分执行函数
 * @authors :Rcxxx
 *          :jiajia
 */
void RM_ArmorFitted::armorFitted() {
    int x_widht = src_img.cols * 0.5; //声明数据量
    int y_height = src_img.rows * 0.5;
    float test_fuzzy_distance = 0.f;
    unsigned int Recoginition_success_flag = 0; //初始化发送类型标记 0表示发送丢失判断数据
    new_frame_has_data = false; //新帧数据标记刷新

    vector < vector < Point > > contours;
    vector < Rect > boundRect;
    vector < RotatedRect > rotateRect;
    vector < Vec4i > hierarchy;
    vector < Point2f > midPoint(2);
    vector < vector < Point2f > > midPoint_pair;
    vector < float > fuzzy_distance;
    findContours(bin_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
    for (int i = 0; i < (int) contours.size(); ++i) {
        if (contours.size() <= 1)
            break;
        Rect B_rect_i = boundingRect(contours[i]);
        RotatedRect R_rect_i = minAreaRect(contours[i]);
        if (B_rect_i.height >= B_rect_i.width) {
            boundRect.push_back(B_rect_i);
            rotateRect.push_back(R_rect_i);
        }
    }

    float distance_max = 0.f;
    for (int k1 = 0; k1 < (int) rotateRect.size(); ++k1) {
        if (rotateRect.size() <= 1)
            break;
        for (int k2 = k1 + 1; k2 < (int) rotateRect.size(); ++k2) {
            if (can_contour_match(rotateRect[k1], rotateRect[k2])) {
                float distance_temp = centerDistance(rotateRect[k1].center, rotateRect[k2].center);
                if (isarmoredColorroi(rotateRect[k1]) && isarmoredColorroi(rotateRect[k2])) {
                    /** change max distance **/
                    if (distance_temp >= distance_max) {
                        distance_max = distance_temp;
                    }
                    /** change max distance **/

                    /** get fuzzy distance **/
                    float test_fuzzy_distance;
                    test_fuzzy_distance = distancetoCamera(rotateRect[k1], rotateRect[k2]);
                    /** get fuzzy distance **/
                    rectangle(dst_img, boundRect[k1].tl(), boundRect[k1].br(), Scalar(0, 255, 255), 2, 8, 0);
                    rectangle(dst_img, boundRect[k2].tl(), boundRect[k2].br(), Scalar(0, 255, 255), 2, 8, 0);
                    midPoint[0] = rotateRect[k1].center;
                    midPoint[1] = rotateRect[k2].center;
                    midPoint_pair.push_back(midPoint);
                    fuzzy_distance.push_back(test_fuzzy_distance);
                }
            }
        }
    }

    for (int k3 = 0; k3 < (int) midPoint_pair.size(); ++k3) {
        float midPoint_distance = centerDistance(midPoint_pair[k3][0], midPoint_pair[k3][1]);
        float midPoint_slope = fabs((midPoint_pair[k3][0].y - midPoint_pair[k3][1].y) / (midPoint_pair[k3][0].x - midPoint_pair[k3][1].x));
        if (midPoint_slope < CENTER_SLPOE_MAX) {
            if (midPoint_distance >= distance_max) {
                Point2f true_center = Point2f((midPoint_pair[k3][0].x + midPoint_pair[k3][1].x) / 2, (midPoint_pair[k3][0].y + midPoint_pair[k3][1].y) / 2);
                rectangle(dst_img,
                    Point(true_center.x - int(midPoint_distance * 0.40), true_center.y - int(midPoint_distance * 0.18)),
                    Point(true_center.x + int(midPoint_distance * 0.40), true_center.y + int(midPoint_distance * 0.18)),
                    Scalar(255, 0, 255), 2, 8, 0);

                int16_t gyroscope_data = 0;
                bool is_receive_success = false;
                SerialPort::RM_receiveData_armor(gyroscope_data, is_receive_success);
                if (is_receive_success == true) {
                    cout << "gyroscope_data  " << gyroscope_data << endl;
                }
                kf.update_feedback_Param(gyroscope_data);
                Point2f new_center = kf.point_Predict(((1e-2) + 0.006666666f), true_center);
                circle(dst_img, new_center, 5, Scalar(120, 120, 150), 2, 8, 0);
                rectangle(dst_img,
                    Point(new_center.x - int(midPoint_distance * 0.47), new_center.y - int(midPoint_distance * 0.2)),
                    Point(new_center.x + int(midPoint_distance * 0.47), new_center.y + int(midPoint_distance * 0.2)),
                    Scalar(255, 255, 0), 2, 8, 0);

                x_buff = true_center.x; //更新缓存帧
                y_buff = true_center.y; //更新缓存帧

                test_fuzzy_distance = fuzzy_distance[k3];
                bool is_it_in_service_area = iscentral_region(true_center, test_fuzzy_distance);
                if (is_it_in_service_area) {
                    Recoginition_success_flag = 1; //重置标记　1 表示位于中心区
                } else {
                    Recoginition_success_flag = 2; //重置标记　2 表示正常发送数据

                    x_widht = new_center.x; //更新发送数据
                    y_height = new_center.y; //更新发送数据
                }
                buff_frame_data = true; //上一帧数据标记　1
                new_frame_none_count = 0; //重置空帧计数器

                break;
            }
        }
    }

    if (serialisopen == 1) {
        /*获取发送类型标记,选择串口发送类型*/
        switch (Recoginition_success_flag) {
            case 0:
                {
                    /** 未识别到目标时 **/
                    /*　判断上一次是否有数据成功发送　有则使用缓存,无则发送停止符*/
                    if (buff_frame_data == true) {
                        /** 没有检测到装甲板，上一帧有数据**/
                        if (sendbuff_count < buff_time) {
                            /** 检测到装甲板**/
                            int buff_reduce_value_x = abs(src_img.cols / 2 - x_buff);
                            int buff_reduce_value_y = abs(src_img.rows / 2 - y_buff);
                            unsigned int send_buff_x = bufferReturn(x_buff, buff_reduce_value_x, 0);
                            unsigned int send_buff_y = bufferReturn(y_buff, buff_reduce_value_y, 1);
                            SerialPort::RMserialWrite(send_buff_x, send_buff_y - 85, test_fuzzy_distance);
                            sendbuff_count += 1;
                        } else {
                            SerialPort::RMserialWrite(x_widht, y_height, test_fuzzy_distance);
                        }
                    } else {
                        /** 没有检测到装甲板并且上一帧无数据 **/
                        SerialPort::RMserialWrite(x_widht, y_height, test_fuzzy_distance);
                    }
                }
                break;
            case 1:
                {
                    sendbuff_count = 0; //成功发送数据时将缓存计数器置零
                    SerialPort::RMserialWrite(x_widht, y_height, test_fuzzy_distance);
                }
                break;
            case 2:
                {
                    sendbuff_count = 0;
                    SerialPort::RMserialWrite(x_widht, y_height - 85, test_fuzzy_distance);
                }
                break;
            default:
                break;
        }
    }
    imshow("dst_img", dst_img);
}