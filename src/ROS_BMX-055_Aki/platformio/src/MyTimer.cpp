#include <Arduino.h>
#include <MyTimer.h>

MyTimer::MyTimer() :
    m_last_time(0),
    m_d_time(0)
{
}

void MyTimer::start()
{
    m_last_time = micros() ;
}

void MyTimer::reset()
{
    m_last_time = micros() ;
}

float MyTimer::getSec()
{
    return (( micros() - m_last_time ) / (1000000.0))  ;
}

long MyTimer::getMSec()
{
    return ( micros() - m_last_time ) / 1000.0 ;
}

long MyTimer::getUSec()
{
    return ( micros() - m_last_time ) ;
}
