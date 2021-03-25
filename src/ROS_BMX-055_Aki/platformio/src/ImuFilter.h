#ifndef __IMU_FILTER_H__
#define __IMU_FILTER_H__

#include<MyTimer.h>

class ImuFilter
{
    private:
        float m_init_qw ;
        float m_init_qx ;
        float m_init_qy ;
        float m_init_qz ;

        float m_qw ;
        float m_qx ;
        float m_qy ;
        float m_qz ;

        float m_delta_qw ;
        float m_delta_qx ;
        float m_delta_qy ;
        float m_delta_qz ;

        MyTimer m_timer ;

    public:
        ImuFilter() ;
        void start() ;
        void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) ;
        float getQW() ;
        float getQX() ;
        float getQY() ;
        float getQZ() ;
} ;

#endif
