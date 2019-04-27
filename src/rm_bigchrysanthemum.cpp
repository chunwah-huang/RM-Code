#include "rm_bigchrysanthemum.h"

using namespace std;

RM_BigChrysanthemum::RM_BigChrysanthemum() {
    cout << "bigChrysanthemum is ready" << endl;
}


bool RM_BigChrysanthemum::is_target_rect(RotatedRect & rect) {
    bool is_target = false;

    if (rect.size.height > rect.size.width) {
        rect_h = rect.size.height;
        rect_w = rect.size.width;
    } else {
        rect_w = rect.size.height;
        rect_h = rect.size.width;
    }

    area = rect_w * rect_h;
    ratio = rect_w / rect_h;

    switch (find_condition) {
        case find_ellipse:
            if (3700 < area && area < 5300 && 0.42 < ratio && ratio < 0.56) {
                is_target = true;
                //cout << "area:" << area << "   " << "ratio:" << ratio << endl;
            }
            break;

        case find_circle:
            if (60 < area && area < 350 && 0.50 < ratio && ratio < 1.40) {
                is_target = true;
                //cout << "area:" << area << "   " << "ratio:" << ratio << endl;
            }
            break;

        case find_rect:
            if (500 < area && area < 2100 && 0.50 < ratio && ratio < 1.48) {
                is_target = true;
                //cout << "area:" << area << "   " << "ratio:" << ratio << endl;
            }
            break;

        default:
            break;
    }
    return is_target;
}

bool RM_BigChrysanthemum::dis_circle_ellipse(Point2f & p1, Point2f & p2) {
    bool is_circle = false;
    //cout << p1 << "     " << p2 << endl;
    float x_dis = pow(p1.x - p2.x, 2);
    float y_dis = pow(p1.y - p2.y, 2);
    float x_y_dis = sqrt(x_dis + y_dis);
    //cout << "distance:" << x_y_dis << endl;

    if (x_y_dis < 130) {
        is_circle = true;
    }
    return is_circle;
}


Point2f RM_BigChrysanthemum::follow_Shoot(Point2f & track_point, Point2f & circle_point, int & mode) {
    Point2f predict_target;
    double total;
    double adjust = 0.56;
    double theta = atan(double(track_point.y - circle_point.y) / (track_point.x - circle_point.x)) * CV_PI / 180;

    if (mode == 1) {
        total = theta + adjust;
    } else {
        total = theta - adjust;
    }

    predict_target.x = (track_point.x - circle_point.x) * cos(total) - (track_point.y - circle_point.y) * sin(total) + circle_point.x;
    predict_target.y = (track_point.x - circle_point.x) * sin(total) + (track_point.y - circle_point.y) * cos(total) + circle_point.y;

    //circle(src_img, predict_target, 3, Scalar(0, 0, 255), -1, 8); //预测

    // if (points_count.size() >= 5) {
    //     vector < Point2f > ::iterator iter = points_count.begin();
    //     points_count.erase(iter);
    //     int average_y = (points_count[1].y + points_count[2].y + points_count[3].y + points_count[4].y) / points_count.size();
    //     if(abs(average_y- points_count[1].y) > 15){
    //         cout << "jump!!!!" << endl;
    //     }

    //     points_count.size() == 5;
    // }
    // points_count.push_back(track_point);

    return predict_target;
}

/*-------------------------------------------------------------Main Code----------------------------------------------------------*/

void RM_BigChrysanthemum::imageProcessing(Mat frame) {

    resize(frame, frame, Size(640, 480));
    frame.copyTo(src_img);
    blur(frame, frame, Size(7, 7));
    //imshow("frame", frame);
    cvtColor(frame, hsv_img, COLOR_BGR2HSV);

    if (armor_color == 0) {
        inRange(hsv_img, Scalar(156, 50, 20), Scalar(180, 255, 255), mix_img_1); //red
        inRange(hsv_img, Scalar(0, 50, 20), Scalar(10, 255, 255), mix_img_2);
        bin_img_1 = mix_img_1 + mix_img_2;
    } else { //TODO:
        //inRange(hsv_img, Scalar(80, 70, 40), Scalar(124, 255, 255), bin_img_1); //blue school
        inRange(hsv_img, Scalar(80, 220, 60), Scalar(124, 255, 255), bin_img_1); //blue    rm_use
        //inRange(hsv_img, Scalar(80, 70, 90), Scalar(124, 255, 255), bin_img_1); //blue  other
    }
    // Mat element_5 = getStructuringElement(MORPH_RECT, Size(3, 3));
    // dilate(bin_img_1, bin_img_1, element_5);
    // imshow("element_5", bin_img_1);
    element_1 = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(bin_img_1, bin_img_1, MORPH_OPEN, element_1); //OPEN形态学除孤立小白噪点
    imshow("hsv_img", bin_img_1);
    // Mat element_6 = getStructuringElement(MORPH_RECT, Size(3, 3));
    // erode(bin_img_1, bin_img_1, element_6);
    // imshow("element_6", bin_img_1);
    /*----------------------------find seed point-----------------------------------*/
    Point2f seed_point(10, 10);
    bool is_black = false;
    for (int j = seed_point.y; j < frame.rows; ++j) {
        uchar * dst = bin_img_1.ptr < uchar > (j);
        for (int i = seed_point.x; i < frame.cols; ++i) {

            int value = dst[i];
            //cout << "value:" << value << endl;/*0:黑色像素点   255:白色像素点*/
            if (value == 0) {
                seed_point.x = i;
                seed_point.y = j;

                i = frame.cols;
                j = frame.rows;

                is_black = true;
            }
        }
    }

    if (is_black == true) {
        bin_img_1.copyTo(bin_img_2);

        floodFill(bin_img_2, seed_point, Scalar(255)); //漫水填充
        threshold(bin_img_2, bin_img_2, 254, 255, CV_THRESH_BINARY_INV); //漫水填充和阈值反操作后尽可能消除噪点影响
        //imshow("flood", bin_img_2);
        element_2 = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(bin_img_2, bin_img_2, MORPH_OPEN, element_2); //除孤立小白噪点
        //imshow("element_2", bin_img_2);
        element_3 = getStructuringElement(MORPH_RECT, Size(13, 13));
        dilate(bin_img_2, bin_img_2, element_3); //膨胀粘合叶片和装甲板白块
        //imshow("element_3", bin_img_2);
        element_4 = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(bin_img_2, bin_img_2, MORPH_CLOSE, element_4); //除孤立小黑噪点
        imshow("element_4", bin_img_2);
    }
}

void RM_BigChrysanthemum::find_energy_agency() {
    Point2f send_point = Point(320, 240);

    bool is_rect = false; //识别判断
    bool is_ellipse = false; //流水灯判断
    bool is_circle = false; //圆心判断
    RotatedRect surround_rect;
    RotatedRect surround_ellipse;
    Rect RECT_1;

    RotatedRect take_rect;
    take_rect.center.x = 320;
    take_rect.center.y = 240;

    RotatedRect take_ellipse;
    take_ellipse.center.x = 320;
    take_ellipse.center.y = 240;

    RotatedRect take_circle;
    take_circle.center.x = 320;
    take_circle.center.y = 240;

    vector < vector < Point >> contours_1;
    vector < vector < Point >> contours_2;
    vector < Vec4i > hierarchy_1;
    vector < Vec4i > hierarchy_2;
    findContours(bin_img_2, contours_1, hierarchy_1, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
    findContours(bin_img_1, contours_2, hierarchy_2, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));

    for (int i1 = 0; i1 < (int) contours_1.size(); ++i1) {

        surround_rect = minAreaRect(contours_1[i1]);
        find_condition = find_rect;

        if (is_target_rect(surround_rect)) {
            take_rect = surround_rect;
            is_rect = true;

            RECT_1 = surround_rect.boundingRect();
            circle(src_img, surround_rect.center, 4, Scalar(255, 0, 0), -1, 8); //识别点
            rectangle(src_img, RECT_1, Scalar(0, 0, 255), 2);
        }
    }

    double contour_area;
    double complexity;
    for (int i2 = 0; i2 < (int) contours_2.size(); ++i2) {

        contour_area = contourArea(contours_2[i2]); //轮廓面积
        complexity = pow(arcLength(contours_2[i2], true), 2) / contour_area; //复杂度

        if (contour_area < 3500) { //风车流水灯面积上限
            //cout << contour_area << endl;
            find_condition = find_ellipse;
            surround_ellipse = minAreaRect(contours_2[i2]);

            // if (800 < contour_area && 25 < complexity && complexity < 65) {//风车流水灯轮廓面积复杂度筛选//TODO:
            //     //cout << contour_area << "   " << complexity << endl;

            //     if (is_target_rect(surround_ellipse)) {//风车流水灯比例面积筛选
            //         take_ellipse = surround_ellipse;
            //         is_ellipse = true;
            //         //ellipse(src_img, surround_ellipse, Scalar(0, 0, 255), 1, 8);
            //         //circle(src_img, surround_ellipse.center, 3, Scalar(0, 255, 0), -1, 8);
            //     }
            // }

            if ((70 < contour_area && contour_area < 370) && (8 < complexity && complexity < 23)) { //风车圆心轮廓面积复杂度筛选
                //cout << contour_area << "   " << complexity << endl;
                find_condition = find_circle;

                if (is_target_rect(surround_ellipse)) { //风车圆心比例面积筛选
                    take_circle = surround_ellipse;
                    is_circle = true; //圆心通过筛选
                }
            }
        }
    }

    int rect_circlr = fastAtan2(take_rect.center.y - take_circle.center.y, take_rect.center.x - take_circle.center.x); //圆心和识别的向量
    int ellipse_circle = fastAtan2(take_ellipse.center.y - take_circle.center.y, take_ellipse.center.x - take_circle.center.x); //圆心和流水灯的向量
    int compare_angle = abs(rect_circlr - ellipse_circle); //向量差计算
    if (is_circle && is_rect /*&& is_ellipse*/) { //圆心和识别点匹配一对才进入//TODO:

        if (last_take_rect.center.x && last_rect_circle > 0) { //第二帧才进入
            //顺逆时针计算
            int ans = (take_circle.center.x - take_rect.center.x) * (last_take_rect.center.y - take_rect.center.y) - (take_circle.center.y - take_rect.center.y) * (last_take_rect.center.x - take_rect.center.x);
            int subtract = rect_circlr - last_rect_circle; //识别跳动计算

            if (abs(subtract) < 30 && ans != 0) { //排除识别跳动状况
                if (ans_count < 20) { //大神符状态打开定旋转方向的帧数
                    if (ans > 0) { //顺时针数值叠加
                        good_count++;
                    } else { //逆时针数值叠加
                        poor_count++;
                    }
                    ans_count++;
                } else if (ans_count == 20) { //叠加完成时，对比叠加量，固定往后的旋转模型状态
                    if (good_count > poor_count) {
                        find_direction = good; //顺时针
                    } else {
                        find_direction = poor; //逆时针
                    }
                    ans_count++;
                    //cout << good_count << "  " << poor_count << endl;
                }
            }

            // bool check_1 = false;//TODO:
            // if (dis_circle_ellipse(take_ellipse.center, take_circle.center)) {
            //     check_1 = true;
            // }
            // bool check_2 = false;//TODO:
            // if (compare_angle < 10) {
            //     check_2 = true;
            // }

            if (1) { //check_1 && check_2) {//TODO:
                int choose;
                switch (find_direction) {
                    case good:
                        choose = good;
                        //ellipse(src_img, take_ellipse, Scalar(0, 0, 255), 1, 8);
                        ellipse(src_img, take_circle, Scalar(0, 255, 0), 2, 8);
                        send_point = follow_Shoot(take_rect.center, take_circle.center, choose);
                        break;

                    case poor:
                        choose = poor;
                        //ellipse(src_img, take_ellipse, Scalar(0, 0, 255), 1, 8);
                        ellipse(src_img, take_circle, Scalar(0, 255, 0), 2, 8);
                        send_point = follow_Shoot(take_rect.center, take_circle.center, choose);
                        break;

                    default:
                        break;
                }
            }

            if (serialisopen == 1) {

                if (send_point.x < 0 || send_point.x > 640) {
                    if (send_point.x < 0) {
                        send_point.x = 0;
                    } else {
                        send_point.x = 640;
                    }
                    cout << "fuck_x!!!!!!!!!!!!!!!!!!!!" << endl;
                }
                if (send_point.y < 0 || send_point.y > 480) {
                    if (send_point.y < 0) {
                        send_point.y = 0;
                    } else {
                        send_point.y = 480;
                    }
                    cout << "fuck_y!!!!!!!!!!!!!!!!!!!!!!" << endl;
                }

                if ((last_send_point.x == send_point.x) && (last_send_point.y == send_point.y)) {
                    send_point.x = 320;
                    send_point.y = 240;
                }
                SerialPort::RMserialWrite(send_point.x, send_point.y, 700, 2, 1);
                circle(src_img, Point(send_point.x, send_point.y), 3, Scalar(0, 0, 255), -1, 8); //预测
                cout << "send_point:" << send_point << endl;
            }
        }
    } else {
        SerialPort::RMserialWrite(320, 240, 700, 2, 1);
        circle(src_img, Point(320, 240), 3, Scalar(0, 0, 255), -1, 8);
        cout << "send_point:" << send_point << endl;
    }
    circle(src_img, Point(320, 240), 30, Scalar(0, 125, 255), 2, 8);
    imshow("src_img", src_img);
    last_take_rect = take_rect;
    last_rect_circle = rect_circlr;
    last_send_point = send_point;
}