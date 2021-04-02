#ifndef __MY_TIMER_H__
#define __MY_TIMER_H__

class MyTimer
{
    private :
        long  m_last_time ;
        long  m_d_time ;

    public  :
        MyTimer() ;
        void start() ;
        void reset() ;
        float getSec() ;
        long getMSec() ;
        long getUSec() ;
        

} ;

#endif
