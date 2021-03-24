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

#define GRYO 1
#define ACCL 2
#define MAG 3
#define XYZ_ROTATION 4

float set_hz    = 200.8 ;    // ROSと通信してる関係上周波数をぴったり指定することはできないためフィルタに工夫を施した
float time_bias = 0 ;


// 加速度センサのパラメータ
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;

// ジャイロセンサのパラメータ
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;

// 磁気センサのパラメータ
float   xMag  = 0;
float   yMag  = 0;
float   zMag  = 0;


// 姿勢(rpy)
float  roll=0  ;
float  pitch=0 ;
float  yaw=0 ;

// 姿勢(四元数)
float result_q0 = 0.00 ;
float result_q1 = 0.00 ;
float result_q2 = 0.00 ;
float result_q3 = 0.00 ;

float last_roll  ;
float last_pitch ;
float last_yaw   ;
float now_roll   ;
float now_pitch  ;
float now_yaw    ;

// 角加速度
float angular_velocity_x ;
float angular_velocity_y ;
float angular_velocity_z ;
float angular_velocity_dt ;


//=====================================================================================//
void BMX055_Init()
{
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x0F); // Select PMU_Range register
    Wire.write(0x03);   // Range = +/- 2g
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x10);  // Select PMU_BW register
    Wire.write(0x08);  // Bandwidth = 7.81 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x11);  // Select PMU_LPW register
    Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x0F);  // Select Range register
    Wire.write(0x04);  // Full scale = +/- 125 degree/s
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x10);  // Select Bandwidth register
    Wire.write(0x07);  // ODR = 100 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x11);  // Select LPM1 register
    Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x83);  // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x01);  // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4C);  // Select Mag register
    // Wire.write(0x00);  // Normal Mode, ODR = 10 Hz   ODR: output data rate
    Wire.write(0x00);  // Normal Mode, ODR = 10 Hz   ODR: output data rate
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4E);  // Select Mag register
    // Wire.write(0x84);  // X, Y, Z-Axis enabled
    Wire.write(0xC7);  // X, Y, Z-Axis enabled
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x51);  // Select Mag register
    Wire.write(0xFF);  // No. of Repetitions for X-Y Axis = 9
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x52);  // Select Mag register
    Wire.write(0xFF);  // No. of Repetitions for Z-Axis = 15
    // Wire.write(0x16); ってなってたぞ。　15周設定なら0x0f　秋月コードが違う
    Wire.endTransmission();
}
//=====================================================================================//
void BMX055_Accl()
{
    int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(Addr_Accl);
        Wire.write((2 + i));// Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
        // Read 6 bytes of data
        // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data to 12-bits
    xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (xAccl > 2047)  xAccl -= 4096;
    yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (yAccl > 2047)  yAccl -= 4096;
    zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (zAccl > 2047)  zAccl -= 4096;
    xAccl = xAccl * 0.0098; // renge +-2g
    yAccl = yAccl * 0.0098; // renge +-2g
    zAccl = zAccl * 0.0098; // renge +-2g
}
//=====================================================================================//
void BMX055_Gyro()
{
    int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(Addr_Gyro);
        Wire.write((2 + i));    // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
        // Read 6 bytes of data
        // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    xGyro = (data[1] * 256) + data[0];
    if (xGyro > 32767)  xGyro -= 65536;
    yGyro = (data[3] * 256) + data[2];
    if (yGyro > 32767)  yGyro -= 65536;
    zGyro = (data[5] * 256) + data[4];
    if (zGyro > 32767)  zGyro -= 65536;

    xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
    yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
    zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}
//=====================================================================================//
void BMX055_Mag()
{
    int data[8];
    for (int i = 0; i < 8; i++)
    {
        Wire.beginTransmission(Addr_Mag);
        Wire.write((0x42 + i));    // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
        // Read 6 bytes of data
        // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    xMag = ((data[1] <<8) | (data[0]>>3));
    if (xMag > 4095)  xMag -= 8192;
    yMag = ((data[3] <<8) | (data[2]>>3));
    if (yMag > 4095)  yMag -= 8192;
    zMag = ((data[5] <<8) | (data[4]>>3));
    // if (zMag > 16383)  zMag -= 32768;
    if (zMag > 4095)  zMag -= 8192;
}








class MyTimer
{
    private :
        long  m_old_time ;
        long  m_d_time ;

    public  :
        MyTimer() ;
        void Init() ;
        void Reset() ;
        long GetTime() ;

} ;

MyTimer::MyTimer() :
    m_old_time(0),
    m_d_time(0)
{
}

void MyTimer::Init()
{
    m_old_time = micros() ;
}

void MyTimer::Reset()
{
    m_old_time = micros() ;
}

long MyTimer::GetTime()
{
    return ( micros() - m_old_time ) ;
}




MyTimer us_timer ;
MyTimer angular_velocity_timer ;

ros::NodeHandle  nh;

sensor_msgs::Imu           imu_msg ;
sensor_msgs::MagneticField mag_msg ;

ros::Publisher imu_pub("imu/data_raw", &imu_msg);
ros::Publisher mag_pub("mag/data", &mag_msg);

float time = 0 ;
float limit_time = 0 ;


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
    BMX055_Init();
    delay(300);

    // シリアル通信の準備ができたらシリアル通信を開始(ROSとの通信)
    while(!Serial);
    Serial.begin(57600);

    nh.getHardware()->setBaud(57600) ;
    nh.initNode();
    nh.advertise(imu_pub);
    nh.advertise(mag_pub);

    us_timer.Init() ;
    angular_velocity_timer.Init() ;

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(13, OUTPUT);

    limit_time =  1000000 *  float( 1 / set_hz ) ;

}

void loop() {

    BMX055_Mag() ;
    BMX055_Gyro() ;
    BMX055_Accl() ;

    time       =  us_timer.GetTime() ;

    if(  time > limit_time - 5000  ) 
    {
        us_timer.Reset() ;
        // ホントの制御周期を求めて代入
        float control_hz = 1000000 / time ;

        //Serial.print( "   Time:" ) ;
        //Serial.print( time ) ;
        //Serial.print( "   LimitTime:" ) ;
        //Serial.print( limit_time ) ;
        //Serial.print( "   set_control_Hz:" ) ;
        //Serial.print( control_hz, 4 ) ;
        //Serial.print( "   ControlHz:" ) ; Serial.println( 1 / MadgwickFilter.getInvSampleFreq(), 4 ) ;

        // MadgwickFilter.begin( control_hz ) ;
        // MadgwickFilter.update(xGyro,yGyro,zGyro,xAccl,yAccl,zAccl, 0.0f, 0.0f, 0.0f );

        imu_msg.header.frame_id ="dtw_robot1/imu_link";
        imu_msg.header.stamp    = nh.now();

        //imu_msg.orientation.x  = MadgwickFilter.getQX() ;
        //imu_msg.orientation.y  = MadgwickFilter.getQY() ;
        //imu_msg.orientation.z  = MadgwickFilter.getQZ() ;
        //imu_msg.orientation.w  = MadgwickFilter.getQW() ;  // wはほぼ確定

        imu_msg.angular_velocity.x = xGyro ;
        imu_msg.angular_velocity.y = yGyro ;
        imu_msg.angular_velocity.z = zGyro ; // [rad/sec]


        imu_msg.linear_acceleration.x = xAccl ;
        imu_msg.linear_acceleration.y = yAccl ;
        imu_msg.linear_acceleration.z = zAccl ;

        imu_pub.publish( &imu_msg ) ;

        mag_msg.header.frame_id ="imu_link";
        mag_msg.header.stamp    = nh.now();

        mag_msg.magnetic_field.x = xMag ;
        mag_msg.magnetic_field.y = yMag ;
        mag_msg.magnetic_field.z = zMag ;

        mag_pub.publish( &mag_msg ) ;

        // 受け取るまでロックする　多分Serial.avaiable()すればロック回避できちゃうかもしれない　別の使い方できる的な　
        nh.spinOnce();
    }
}
