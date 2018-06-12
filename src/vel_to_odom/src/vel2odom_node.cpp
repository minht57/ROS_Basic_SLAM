#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

float x = 0.00, y = 0.00, th = 0.00;
float dt;
float delta_t;
std::string frame_id;
std::string child_frame_id;
std::string cmd_vel_topic;
std::string odom_topic;
bool publish_odom;

void velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    x += vel->linear.x * delta_t * cos(th);
    y += vel->linear.x * delta_t * sin(th);
    th += -vel->angular.z * delta_t;
    ROS_INFO("x: [%.2f] y: [%.2f] th: [%.2f]", x, y, th);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel2odom_node");
    ros::NodeHandle nh("~");

    nh.param<float>("dt", dt, 0.02);
    nh.param<std::string>("child_frame_id", child_frame_id, "/base_frame");
    nh.param<std::string>("frame_id", frame_id, "/odom");
    nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "cmd_vel_echo");
    nh.param<std::string>("odom_topic", odom_topic, "odom");
    nh.param<bool>("publish_odom", publish_odom, false);


    ros::Subscriber cmd_vel_sub = nh.subscribe(cmd_vel_topic, 1000, velCallback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);
    
    tf::TransformBroadcaster broadcaster;

    ros::Rate loop_rate(20);
    geometry_msgs::TransformStamped odom_trans;
    ros::Time current_time, last_time;

    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    while(ros::ok()){
        current_time = ros::Time::now();
        delta_t = (current_time - last_time).toSec();
        /*nav_msgs::Odometry _odom;
        geometry_msgs::Point _point;
        
        _point.x = 0;//x;
        _point.y = 0;//y;
        _point.z = 0;

        _odom.header.stamp = current_time;
        _odom.child_frame_id = child_frame_id;
        _odom.header.frame_id = frame_id;
        _odom.pose.pose.position = _point;
        _odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);//th);
        
        odom_pub.publish(_odom);*/
        // ROS_INFO("OK");

        if(publish_odom)
        {
            odom_trans.header.stamp = current_time;
            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

            broadcaster.sendTransform(odom_trans);
            // ROS_INFO("OK");
        }

        loop_rate.sleep();
        last_time = current_time;
        ros::spinOnce();
    }

    return 0;
}

