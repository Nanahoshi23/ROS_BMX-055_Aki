#ifndef __BMX_055_H__
#define __BMX_055_H__

class BMX_055
{
    private:
        const uint8_t m_addr_acc = 0x19 ; // (JP1,JP2,JP3 = Open);
        const uint8_t m_addr_gyr = 0x69 ; // (JP1,JP2,JP3 = Open);
        const uint8_t m_addr_mag = 0x13 ; // (JP1,JP2,JP3 = Open);

        float m_x_acc ;
        float m_y_acc ;
        float m_z_acc ;

        float m_x_gyr ;
        float m_y_gyr ;
        float m_z_gyr ;

        float m_x_mag ;
        float m_y_mag ;
        float m_z_mag ;

        void updateAcc() ;
        void updateGyr() ;
        void updateMag() ;

    public:
        BMX_055() ;
        void start() ;
        void update() ;
        float getXAcc() ;
        float getYAcc() ;
        float getZAcc() ;
        float getXGyr() ;
        float getYGyr() ;
        float getZGyr() ;
        float getXMag() ;
        float getYMag() ;
        float getZMag() ;
} ;

#endif
