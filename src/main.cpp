/**
  @param:When formal use Switch to the Release in configure.h
********************************************************
  @brief:RM vision overall code
********************************************************
  @param:
********************************************************
  @authors:Rcxxx_
           jiajia
           Hzkkk
           ziYang
           ZhenHua
  */
#include "rm_link.h"

int main()
{
    /** code init **/
    RM_Vision_Init run;
    SerialPort::serialSet(1);
    for(;;)
    {
        /** run **/
        run.Run();
  
        if(run.is_exit())
        {
            break;
        }
    }
    return 1;
}

