#include <Arduino.h>
#include <ImuFilter.h>
#include <Vector3D.h>
#include <Quaternion.h>


ImuFilter::ImuFilter()
{
    Vector3D ref_unit_vector(0,0,0) ;
    Quaternion ref_quat(ref_unit_vector,0) ;
    m_init_qw = ref_quat.getQW() ;
    m_init_qx = ref_quat.getQX() ;
    m_init_qy = ref_quat.getQY() ;
    m_init_qz = ref_quat.getQZ() ;
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
    m_qw += (0.5 * m_delta_qw ) * delta_control_time ;
    m_qx += (0.5 * m_delta_qx ) * delta_control_time ;
    m_qy += (0.5 * m_delta_qy ) * delta_control_time ;
    m_qz += (0.5 * m_delta_qz ) * delta_control_time ;

    Quaternion gyr_quat = Quaternion(m_qw, m_qx, m_qy, m_qz).normalize() ;

    Vector3D gravity_unit_vector = Vector3D(0, 0, 9.8).normalize() ;

    Vector3D   acc_unit_vector     = Vector3D(ax,ay,az).normalize() ;
    Vector3D   gyr_unit_vector     = gyr_quat.rotateVector3D(gravity_unit_vector.normalize()).normalize() ;

    Quaternion correction_quat = Quaternion(gyr_unit_vector,acc_unit_vector).normalize() ;

    gyr_quat = (gyr_quat*correction_quat).normalize();
    
    Quaternion acc_unit_quat = Quaternion(acc_unit_vector,gravity_unit_vector).normalize() ;
    Quaternion gyr_unit_quat = Quaternion(gyr_unit_vector,gravity_unit_vector).normalize() ;

     m_qw = gyr_quat.getQW() ;
     m_qx = gyr_quat.getQX() ;
     m_qy = gyr_quat.getQY() ;
     m_qz = gyr_quat.getQZ() ;
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

