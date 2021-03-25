#include <Arduino.h>
#include <ImuFilter.h>


ImuFilter::ImuFilter()
{
    m_init_qw = 1.0 ;
    m_init_qx = 0.0 ;
    m_init_qy = 0.0 ;
    m_init_qz = 0.0 ;
    m_qw = m_init_qw ;
    m_qx = m_init_qx ;
    m_qy = m_init_qy ;
    m_qz = m_init_qz ;
    m_delta_qw = 0 ;
    m_delta_qx = 0 ;
    m_delta_qy = 0 ;
    m_delta_qz = 0 ;
} ;


void ImuFilter::start()
{
    m_timer.start() ;
}


void ImuFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    // posture estimation
    m_delta_qw = -gx*m_qx -gy*m_qy -gz*m_qz ;
    m_delta_qx = +gx*m_qw +gz*m_qy -gy*m_qz ;
    m_delta_qy = +gy*m_qw -gz*m_qx +gx*m_qz ;
    m_delta_qz = +gz*m_qw +gy*m_qx -gx*m_qy ;

    // calculate control cycle
    float delta_control_time = m_timer.getSec() ;
    m_timer.reset() ;

    // integral
    m_qw += (0.5 * m_delta_qw + m_qw) * delta_control_time ;
    m_qx += (0.5 * m_delta_qx + m_qx) * delta_control_time ;
    m_qy += (0.5 * m_delta_qy + m_qy) * delta_control_time ;
    m_qz += (0.5 * m_delta_qz + m_qz) * delta_control_time ;
}


float ImuFilter::getQW()
{
    return m_qw ;
}

float ImuFilter::getQX()
{
    return m_qx ;
}

float ImuFilter::getQY()
{
    return m_qy ;
}

float ImuFilter::getQZ()
{
    return m_qz ;
}

