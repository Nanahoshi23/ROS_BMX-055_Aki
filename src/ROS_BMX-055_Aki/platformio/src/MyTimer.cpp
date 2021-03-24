#include <Arduino.h>
#include <MyTimer.h>

MyTimer::MyTimer() :
    m_last_time(0),
    m_d_time(0)
{
}

void MyTimer::Init()
{
    m_last_time = micros() ;
}

void MyTimer::Reset()
{
    m_last_time = micros() ;
}

long MyTimer::GetTime()
{
    return ( micros() - m_last_time ) ;
}
