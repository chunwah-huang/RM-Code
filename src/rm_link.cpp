#include "rm_link.h"

RM_Vision_Init::RM_Vision_Init():capture(capture_defult),cap(isopen_industry_capture){
}

void RM_Vision_Init::Run()
{
    /** Param Set **/
    if(cap.isindustryimgInput())
    {
        src_img = cvarrToMat(cap.iplImage,true);//这里只是进行指针转换，将IplImage转换成Mat类型
    }
    else
    {
        capture >> src_img;
    }
    resize(src_img,src_img,Size(640,480),INTER_NEAREST);
    //Receive Serial Data
    ////NOTE:Default param "case_mode"
    unsigned int receive_serial_data_No_ = SerialPort::RMreceiveData();//port.RMreceiveData();// default_mode;
    //energy_agency_mode;
    /** Change Mode　**/
    switch (receive_serial_data_No_)
    {
    /**-Support Shooting mode-**/
    case support_shooting_mode:
    {
        imshow("src_img",src_img);
        armor.imageProcessing(src_img);
        armor.armorFitted();
    }   /**-Support Shooting mode-**/
        break;
    /** Energy Agency Mode **/
    case energy_agency_mode:
    {
        imshow("src_img",src_img);
        agency.imageProcessing(src_img);
        agency.find_energy_agency();
        

    }   /** Energy Agency Mode **/
        break;
    /**-Empty mode-**/
    default:
        imshow("src_img",src_img);
        /**-Empty mode-**/
        break;
    }   /** Change Mode　**/
    cap.cameraReleasebuff();
}

bool RM_Vision_Init::is_exit()
{
    bool exit = false;
    int key = waitKey(1);
    if(char(key) == 27)
    {
        exit = true;
    }
    else
    {
        exit = false;
    }
    return exit;
}
