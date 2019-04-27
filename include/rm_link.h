#ifndef RM_LINK_H
#define RM_LINK_H

#include "configure.h"
#include "rm_videocapture.h"
#include "serialport.h"
#include "rm_armorfitted.h"
#include "rm_bigchrysanthemum.h"

class RM_Vision_Init
{
public:
    RM_Vision_Init();
    void Run();
    bool is_exit();

private:
    /** Camera Srart **/
    VideoCapture capture;
    RM_VideoCapture cap;
    /** Camera Srart **/

    /** param initial **/
    Mat src_img;
    /** param initial **/

    /** function initial **/
    RM_ArmorFitted armor;
    RM_BigChrysanthemum agency;
    /** function initial **/
};

#endif // RM_LINK_H
