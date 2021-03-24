#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Wire.h>
#include <math.h>

#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>


// 加速度センサのアドレス
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Open)
// ジャイロセンサのアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Open)
// 磁気センサのアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Open)


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




class Filter
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

    public:
        Filter() ;
        void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) ;
} ;


Filter::Filter()
{
    m_init_qw = 1.0 ;
    m_init_qx = 0.0 ;
    m_init_qy = 0.0 ;
    m_init_qz = 0.0 ;
    m_qw = m_init_qw ;
    m_qx = m_init_qx ;
    m_qy = m_init_qy ;
    m_qz = m_init_qz ;
} ;


void Filter::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
}


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

BMX_055::BMX_055()
{
}


void BMX_055::updateAcc()
{
    int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(m_addr_acc);
        Wire.write((2 + i));// Select data register
        Wire.endTransmission();
        Wire.requestFrom(m_addr_acc, 1);// Request 1 byte of data
        // Read 6 bytes of data
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data to 12-bits
    m_x_acc = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (m_x_acc > 2047)  m_x_acc -= 4096;
    m_y_acc = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (m_y_acc > 2047)  m_y_acc -= 4096;
    m_z_acc = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (m_z_acc > 2047)  m_z_acc -= 4096;
    m_x_acc = m_x_acc * 0.0098; // renge +-2g
    m_y_acc = m_y_acc * 0.0098; // renge +-2g
    m_z_acc = m_z_acc * 0.0098; // renge +-2g
}

void BMX_055::updateGyr()
{
    int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(m_addr_gyr);
        Wire.write((2 + i));    // Select data register
        Wire.endTransmission();
        Wire.requestFrom(m_addr_gyr, 1);    // Request 1 byte of data
        // Read 6 bytes of data
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    m_x_gyr = (data[1] * 256) + data[0];
    if (m_x_gyr > 32767)  m_x_gyr -= 65536;
    m_y_gyr = (data[3] * 256) + data[2];
    if (m_y_gyr > 32767)  m_y_gyr -= 65536;
    m_z_gyr = (data[5] * 256) + data[4];
    if (m_z_gyr > 32767)  m_z_gyr -= 65536;

    m_x_gyr = m_x_gyr * 0.0038; //  Full scale = +/- 125 degree/s
    m_y_gyr = m_y_gyr * 0.0038; //  Full scale = +/- 125 degree/s
    m_z_gyr = m_z_gyr * 0.0038; //  Full scale = +/- 125 degree/s
}

void BMX_055::updateMag()
{
    int data[8];
    for (int i = 0; i < 8; i++)
    {
        Wire.beginTransmission(Addr_Mag);
        Wire.write((0x42 + i));    // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
        // Read 6 bytes of data
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    m_x_mag = ((data[1] <<8) | (data[0]>>3));
    if (m_x_mag > 4095)  m_x_mag -= 8192;
    m_y_mag = ((data[3] <<8) | (data[2]>>3));
    if (m_y_mag > 4095)  m_y_mag -= 8192;
    m_z_mag = ((data[5] <<8) | (data[4]>>3));
    // if (m_z_mag > 16383)  m_z_mag -= 32768;
    if (m_z_mag > 4095)  m_z_mag -= 8192;
}

void BMX_055::start()
{
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_acc);
    Wire.write(0x0F); // Select PMU_Range register
    Wire.write(0x03);   // Range = +/- 2g
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_acc);
    Wire.write(0x10);  // Select PMU_BW register
    Wire.write(0x08);  // Bandwidth = 7.81 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_acc);
    Wire.write(0x11);  // Select PMU_LPW register
    Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_gyr);
    Wire.write(0x0F);  // Select Range register
    Wire.write(0x04);  // Full scale = +/- 125 degree/s
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_gyr);
    Wire.write(0x10);  // Select Bandwidth register
    Wire.write(0x07);  // ODR = 100 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_gyr);
    Wire.write(0x11);  // Select LPM1 register
    Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x83);  // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x01);  // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x4C);  // Select Mag register
    // Wire.write(0x00);  // Normal Mode, ODR = 10 Hz   ODR: output data rate
    Wire.write(0x00);  // Normal Mode, ODR = 10 Hz   ODR: output data rate
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x4E);  // Select Mag register
    // Wire.write(0x84);  // X, Y, Z-Axis enabled
    Wire.write(0xC7);  // X, Y, Z-Axis enabled
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x51);  // Select Mag register
    Wire.write(0xFF);  // No. of Repetitions for X-Y Axis = 9
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x52);  // Select Mag register
    Wire.write(0xFF);  // No. of Repetitions for Z-Axis = 15
    Wire.endTransmission();

}

void BMX_055::update()
{
    updateAcc() ;
    updateGyr() ;
    updateMag() ;
}


float BMX_055::getXAcc()
{
    return m_x_acc ;
}


float BMX_055::getYAcc()
{
    return m_y_acc ;
}


float BMX_055::getZAcc()
{
    return m_z_acc ;
}


float BMX_055::getXGyr()
{
    return m_x_gyr ;
}


float BMX_055::getYGyr()
{
    return m_y_gyr ;
}


float BMX_055::getZGyr()
{
    return m_z_gyr ;
}


float BMX_055::getXMag()
{
    return m_x_mag ;
}


float BMX_055::getYMag()
{
    return m_y_mag ;
}


float BMX_055::getZMag()
{
    return m_z_mag ;
}





MyTimer us_timer ;
MyTimer angular_velocity_timer ;

ros::NodeHandle  nh;

sensor_msgs::Imu           imu_msg ;
sensor_msgs::MagneticField mag_msg ;

ros::Publisher imu_pub("imu/data_raw", &imu_msg);
ros::Publisher mag_pub("mag/data", &mag_msg);

BMX_055 bmx_055 ;


// サービス通信に関する新しい記述 リクエストもレスポンスも両方共文字列
// 取り出すときはどっちも.c_str()メソッド
bool dummy_offset()
{
    // ダミー関数
    return true ;
}




void setup()
{
    // I2C通信を開始
    Wire.begin() ;

    // センサーのセット
    //BMX055_Init();
    bmx_055.start() ;
    delay(300);

    // シリアル通信の準備ができたらシリアル通信を開始(ROSとの通信)
    while(!Serial);
    Serial.begin(57600);

    nh.getHardware()->setBaud(57600) ;
    nh.initNode();
    nh.advertise(imu_pub);
    nh.advertise(mag_pub);
}

void loop() {

    // BMX055_Mag() ;
    // BMX055_Gyro() ;
    // BMX055_Accl() ;
    bmx_055.update() ;


    imu_msg.header.frame_id ="dtw_robot1/imu_link";
    imu_msg.header.stamp    = nh.now();

    //imu_msg.orientation.x  = MadgwickFilter.getQX() ;
    //imu_msg.orientation.y  = MadgwickFilter.getQY() ;
    //imu_msg.orientation.z  = MadgwickFilter.getQZ() ;
    //imu_msg.orientation.w  = MadgwickFilter.getQW() ;  // wはほぼ確定

    imu_msg.angular_velocity.x = bmx_055.getXGyr() ;
    imu_msg.angular_velocity.y = bmx_055.getYGyr() ;
    imu_msg.angular_velocity.z = bmx_055.getZGyr() ; // [rad/sec]


    imu_msg.linear_acceleration.x = bmx_055.getXAcc() ;
    imu_msg.linear_acceleration.y = bmx_055.getYAcc() ;
    imu_msg.linear_acceleration.z = bmx_055.getZAcc() ;

    imu_pub.publish( &imu_msg ) ;

    mag_msg.header.frame_id ="imu_link";
    mag_msg.header.stamp    = nh.now();

    mag_msg.magnetic_field.x = bmx_055.getXMag() ;
    mag_msg.magnetic_field.y = bmx_055.getYMag() ;
    mag_msg.magnetic_field.z = bmx_055.getZMag() ;

    mag_pub.publish( &mag_msg ) ;

    // 受け取るまでロックする　多分Serial.avaiable()すればロック回避できちゃうかもしれない　別の使い方できる的な　
    nh.spinOnce();
}
