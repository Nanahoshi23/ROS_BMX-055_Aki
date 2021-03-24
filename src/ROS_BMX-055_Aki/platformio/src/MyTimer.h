#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

class MyTimer
{
    private :
        long  m_last_time ;
        long  m_d_time ;

    public  :
        MyTimer() ;
        void Init() ;
        void Reset() ;
        long GetTime() ;

} ;

#endif
