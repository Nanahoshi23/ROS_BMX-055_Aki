#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher posestamped_pub;
void imu_callback(const sensor_msgs::Imu& imu_msg){
    geometry_msgs::PoseStamped ps_msg;
    ps_msg.header=imu_msg.header;
    ps_msg.pose.orientation=imu_msg.orientation;
    posestamped_pub.publish(ps_msg);    
}

//------------------main------------------

int main(int argc, char **argv)
{
    // ノードを初期化
    ros::init(argc, argv, "imu_data_to_pose");
    ros::NodeHandle nh ;

    //Publisher
    posestamped_pub = nh.advertise<geometry_msgs::PoseStamped>("imu_pose", 10);
    //Subscriber
    ros::Subscriber serial_sub = nh.subscribe("imu/data_raw", 10, imu_callback);

    ros::Rate loop_rate(20) ;
    while(ros::ok()) {
        ros::spinOnce() ;
        loop_rate.sleep() ;
    }
    return 0 ;
}
