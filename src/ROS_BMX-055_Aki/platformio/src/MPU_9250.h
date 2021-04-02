#ifndef __MPU_9250_H__
#define __MPU_9250_H__

#include <Arduino.h>
#include <Wire.h>

class MPU_9250
{
    private:
        // 読み取ったデータを格納する変数
        volatile uint8_t m_acc_gyr_tmp_data[14] ;
        volatile uint8_t m_mag_data[7] ;
        volatile uint8_t m_mag_flag_data ;

        // i2cスレーブアドレス
        const uint8_t m_mpu9250_addr = 0x68 ;
        const uint8_t m_ak8963_addr  = 0x0c ;

        // 加速度センサの読み取り設定用変数
        const uint8_t m_acc_config     = 0x19 ;
        const uint8_t m_acc_config_2g  = 0x00;  
        const uint8_t m_acc_config_4g  = 0x08; 
        const uint8_t m_acc_config_8g  = 0x10; 
        const uint8_t m_acc_config_16g = 0x18; 
        float         m_acc_range ;

        // ジャイロセンサ読み取り設定アドレス
        const uint8_t m_gyr_config         = 0x69 ;
        const uint8_t m_gyr_config_250dps  = 0x00 ;
        const uint8_t m_gyr_config_500dps  = 0x08 ;
        const uint8_t m_gyr_config_1000dps = 0x10 ;
        const uint8_t m_gyr_config_2000dps = 0x10 ;
        float         m_gyr_range ;

        // 磁気センサ読み取り設定アドレス
        const uint8_t m_mag_flag         = 0x02 ;
        const uint8_t m_mag_config       = 0x0a ;
        const uint8_t m_mag_config_8hz   = 0x12 ;
        const uint8_t m_mag_config_100hz = 0x16 ;
        float         m_mag_range ;

        // 生値
        volatile int16_t m_raw_x_acc = 0 ;
        volatile int16_t m_raw_y_acc = 0 ;
        volatile int16_t m_raw_z_acc = 0 ;
        volatile int16_t m_raw_x_gyr = 0 ;
        volatile int16_t m_raw_y_gyr = 0 ;
        volatile int16_t m_raw_z_gyr = 0 ;
        volatile int16_t m_raw_tmp   = 0 ;
        volatile int16_t m_raw_x_mag = 0 ;
        volatile int16_t m_raw_y_mag = 0 ;
        volatile int16_t m_raw_z_mag = 0 ;

        // 物理量
        volatile float m_x_acc = 0 ;
        volatile float m_y_acc = 0 ;
        volatile float m_z_acc = 0 ;
        volatile float m_x_gyr = 0 ;
        volatile float m_y_gyr = 0 ;
        volatile float m_z_gyr = 0 ;
        volatile float m_tmp   = 0 ;
        volatile float m_x_mag = 0 ;
        volatile float m_y_mag = 0 ;
        volatile float m_z_mag = 0 ;

        void updateAcc() ;
        void updateGyr() ;
        void updateMag() ;
        void updateTmp() ;

    public:
        MPU_9250() ;
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
