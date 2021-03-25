#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Wire.h>
#include <math.h>

#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <BMX_055.h>
#include <ImuFilter.h>


// node handle
ros::NodeHandle  nh;

// sensor message
sensor_msgs::Imu           imu_msg ;
sensor_msgs::MagneticField mag_msg ;

// publisher
ros::Publisher imu_pub("imu/data_raw", &imu_msg);
ros::Publisher mag_pub("mag/data", &mag_msg);

// BMX-055
BMX_055 bmx_055 ;

// ImuFilter
ImuFilter imu_filter ;


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

    imu_filter.start() ;
}

void loop() {

    // BMX055_Mag() ;
    // BMX055_Gyro() ;
    // BMX055_Accl() ;
    bmx_055.update() ;
    imu_filter.update(bmx_055.getXGyr(),bmx_055.getYGyr(), bmx_055.getZGyr(),
                        bmx_055.getXAcc(), bmx_055.getYAcc(), bmx_055.getZAcc(),
                        bmx_055.getXMag(), bmx_055.getYMag(), bmx_055.getZMag()) ;


    imu_msg.header.frame_id ="dtw_robot1/imu_link";
    imu_msg.header.stamp    = nh.now();

    imu_msg.orientation.x  = imu_filter.getQX() ;
    imu_msg.orientation.y  = imu_filter.getQY() ;
    imu_msg.orientation.z  = imu_filter.getQZ() ;
    imu_msg.orientation.w  = imu_filter.getQW() ;  // wはほぼ確定

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
