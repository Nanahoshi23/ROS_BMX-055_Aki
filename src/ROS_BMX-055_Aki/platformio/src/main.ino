//#include <ros.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Bool.h>
//#include <Wire.h>
//#include <math.h>
//
//#include <ros/time.h>
//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/MagneticField.h>
//
//#include <BMX_055.h>
//#include <ImuFilter.h>
//
//
//// node handle
//ros::NodeHandle  nh;
//
//// sensor message
//sensor_msgs::Imu           imu_msg ;
//sensor_msgs::MagneticField mag_msg ;
//
//// publisher
//ros::Publisher imu_pub("imu/data_raw", &imu_msg);
//ros::Publisher mag_pub("mag/data", &mag_msg);
//
//// BMX-055
//BMX_055 bmx_055 ;
//
//// ImuFilter
//ImuFilter imu_filter ;
//
//
//void setup()
//{
//    // I2C通信を開始
//    Wire.begin() ;
//
//    // センサーのセット
//    //BMX055_Init();
//    bmx_055.start() ;
//    delay(300);
//
//    // シリアル通信の準備ができたらシリアル通信を開始(ROSとの通信)
//    while(!Serial);
//    Serial.begin(57600);
//
//    nh.getHardware()->setBaud(57600) ;
//    nh.initNode();
//    nh.advertise(imu_pub);
//    nh.advertise(mag_pub);
//
//    imu_filter.start() ;
//}
//
//void loop() {
//
//    // BMX055_Mag() ;
//    // BMX055_Gyro() ;
//    // BMX055_Accl() ;
//    bmx_055.update() ;
//    imu_filter.update(bmx_055.getXGyr(),bmx_055.getYGyr(), bmx_055.getZGyr(),
//                        bmx_055.getXAcc(), bmx_055.getYAcc(), bmx_055.getZAcc(),
//                        bmx_055.getXMag(), bmx_055.getYMag(), bmx_055.getZMag()) ;
//
//
//    imu_msg.header.frame_id ="dtw_robot1/imu_link";
//    imu_msg.header.stamp    = nh.now();
//
//    imu_msg.orientation.x  = imu_filter.getQX() ;
//    imu_msg.orientation.y  = imu_filter.getQY() ;
//    imu_msg.orientation.z  = imu_filter.getQZ() ;
//    imu_msg.orientation.w  = imu_filter.getQW() ;  // wはほぼ確定
//
//    imu_msg.angular_velocity.x = bmx_055.getXGyr() ;
//    imu_msg.angular_velocity.y = bmx_055.getYGyr() ;
//    imu_msg.angular_velocity.z = bmx_055.getZGyr() ; // [rad/sec]
//
//
//    imu_msg.linear_acceleration.x = bmx_055.getXAcc() ;
//    imu_msg.linear_acceleration.y = bmx_055.getYAcc() ;
//    imu_msg.linear_acceleration.z = bmx_055.getZAcc() ;
//
//    imu_pub.publish( &imu_msg ) ;
//
//    mag_msg.header.frame_id ="imu_link";
//    mag_msg.header.stamp    = nh.now();
//
//    mag_msg.magnetic_field.x = bmx_055.getXMag() ;
//    mag_msg.magnetic_field.y = bmx_055.getYMag() ;
//    mag_msg.magnetic_field.z = bmx_055.getZMag() ;
//
//    mag_pub.publish( &mag_msg ) ;
//
//    nh.spinOnce();
//}

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Wire.h>
#include <math.h>

#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <MPU_9250.h>
#include <ImuFilter.h>

// NodeHandle
ros::NodeHandle  nh;

// sensor message
sensor_msgs::Imu           imu_msg ;
sensor_msgs::MagneticField mag_msg ;

// publisher
ros::Publisher imu_pub("imu/data_raw", &imu_msg);
ros::Publisher mag_pub("mag/data", &mag_msg);

// MPU_9250
MPU_9250 mpu_9250 ;

// ImuFilter
ImuFilter imu_filter ;

void setup() {

    mpu_9250.start() ;

    // シリアル通信の準備ができたらシリアル通信を開始(ROSとの通信)
    while(!Serial);
    Serial.begin(57600);

    nh.getHardware()->setBaud(57600) ;
    nh.initNode();
    nh.advertise(imu_pub);
    nh.advertise(mag_pub);

    imu_filter.start() ;
}

void loop()
{
    mpu_9250.update() ;
    imu_filter.update(mpu_9250.getXGyr(), mpu_9250.getYGyr(), mpu_9250.getZGyr(),
            mpu_9250.getXAcc(), mpu_9250.getYAcc(), mpu_9250.getZAcc(),
            mpu_9250.getXMag(), mpu_9250.getYMag(), mpu_9250.getZMag()) ;

    char acc_x_cstr[8], acc_y_cstr[8], acc_z_cstr[8];
    dtostrf(mpu_9250.getXMag(), 6, 2, acc_x_cstr);
    dtostrf(mpu_9250.getYMag(), 6, 2, acc_y_cstr);
    dtostrf(mpu_9250.getZMag(), 6, 2, acc_z_cstr);
    String log_msg = "acc_x:" + String(acc_x_cstr) + "   acc_y:" + String(acc_y_cstr) + "   acc_z:" + String(acc_z_cstr);
    nh.loginfo(log_msg.c_str());

    imu_msg.header.frame_id ="dtw_robot1/imu_link";
    imu_msg.header.stamp    = nh.now();

    imu_msg.orientation.x  = imu_filter.getQX() ;
    imu_msg.orientation.y  = imu_filter.getQY() ;
    imu_msg.orientation.z  = imu_filter.getQZ() ;
    imu_msg.orientation.w  = imu_filter.getQW() ;  // wはほぼ確定

    imu_msg.angular_velocity.x = mpu_9250.getXGyr() ;
    imu_msg.angular_velocity.y = mpu_9250.getYGyr() ;
    imu_msg.angular_velocity.z = mpu_9250.getZGyr() ; // [rad/sec]


    imu_msg.linear_acceleration.x = mpu_9250.getXAcc() ;
    imu_msg.linear_acceleration.y = mpu_9250.getYAcc() ;
    imu_msg.linear_acceleration.z = mpu_9250.getZAcc() ;

    imu_pub.publish( &imu_msg ) ;

    mag_msg.header.frame_id ="dtw_robot1/imu_link";
    mag_msg.header.stamp    = nh.now();

    mag_msg.magnetic_field.x = mpu_9250.getXMag() ;
    mag_msg.magnetic_field.y = mpu_9250.getYMag() ;
    mag_msg.magnetic_field.z = mpu_9250.getZMag() ;

    mag_pub.publish( &mag_msg ) ;

    nh.spinOnce();

}

