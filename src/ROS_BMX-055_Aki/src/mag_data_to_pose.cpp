#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/MagneticField.h"

#include "tf/transform_datatypes.h"
#include <visualization_msgs/MarkerArray.h>

ros::Publisher mag_marker_pub;

void mag_callback(const sensor_msgs::MagneticField& mag_msg){
    geometry_msgs::PoseStamped ps_msg;
    ps_msg.header=mag_msg.header;
    // tf::Vector3 mag_vector(mag_msg.magnetic_field.x,mag_msg.magnetic_field.y,mag_msg.magnetic_field.z) ;
    geometry_msgs::Vector3 mag_arror ;
    mag_arror.x = 0.1 ;
    mag_arror.y = 0.1 ;
    mag_arror.z = 0.1 ;

    geometry_msgs::Point linear_start ;
    linear_start.x = 0 ;
    linear_start.y = 0 ;
    linear_start.z = 0 ;

    geometry_msgs::Point linear_end ;
    linear_end.x = mag_msg.magnetic_field.x * 0.1;
    linear_end.y = mag_msg.magnetic_field.y * 0.1;
    linear_end.z = mag_msg.magnetic_field.z * 0.1;

    // tf::Quaternion mag_quat(mag_vector, 0) ;

    // ps_msg.pose.orientation.x = mag_quat.getAxis().getX() ;
    // ps_msg.pose.orientation.y = mag_quat.getAxis().getY() ;
    // ps_msg.pose.orientation.z = mag_quat.getAxis().getZ() ;
    // ps_msg.pose.orientation.w = mag_quat.getW() ;

    // ps_msg.pose.position.x = 0  ;
    // ps_msg.pose.position.y = 0  ;
    // ps_msg.pose.position.z = 0  ;

    // posestamped_pub.publish(ps_msg);    

    visualization_msgs::Marker mag_marker;

    mag_marker.header = mag_msg.header ;
    mag_marker.ns = "mag_val_display" ;
    mag_marker.id = 0 ;
    mag_marker.lifetime = ros::Duration() ;

    mag_marker.type = visualization_msgs::Marker::ARROW ;
    mag_marker.action = visualization_msgs::Marker::ADD ;

    mag_marker.scale = mag_arror ;

    mag_marker.points.resize(2) ;
    mag_marker.points[0] = linear_start ;
    mag_marker.points[1] = linear_end ;

    mag_marker.color.r = 0.0f ;
    mag_marker.color.g = 1.0f ;
    mag_marker.color.b = 0.0f ;
    mag_marker.color.a = 1.0f ;

    mag_marker_pub.publish(mag_marker) ;
}

//------------------main------------------

int main(int argc, char **argv)
{
    // ノードを初期化
    ros::init(argc, argv, "mag_data_to_pose");
    ros::NodeHandle nh ;

    //Publisher
    mag_marker_pub = nh.advertise<visualization_msgs::Marker>("mag_marker", 10);
    //Subscriber
    ros::Subscriber serial_sub = nh.subscribe("mag/data", 10, mag_callback);

    ros::Rate loop_rate(20) ;
    while(ros::ok()) {
        ros::spinOnce() ;
        loop_rate.sleep() ;
    }
    return 0 ;
}
