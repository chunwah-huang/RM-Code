#ifndef RM_KALMANFILTER_H
#define RM_KALMANFILTER_H

#include "configure.h"

#define ANTI_RANGE 5
#define DEAD_BAND 15
#define SIZE_X 640
#define SIZE_Y 480

class RM_kalmanfilter
{
public:
    RM_kalmanfilter();
    ~RM_kalmanfilter();
    Point2f point_Predict(double g_runtime,Point2d current_point);
    void update_feedback_Param(int16_t feedback_value);
    void refresh_kalman_Param(int refresh_mode,Point2d point);

private:
    Mat measurement_img;//测量矩阵
    cv::KalmanFilter kalman_img;
    Point2f last_point;
    Point2f last_predic_point;
    double runtime=(1e-2) + 0.006666666f;
    double last_v=0;
    double last_a=0;
    double feedback_v = 0;

    int lead_max_value = 0;
    float last_angular_v_change_rate = 0;
    float last_line_v_change_rate = 0;
};

#endif // RM_KALMANFILTER_H
