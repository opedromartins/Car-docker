#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turn_left_node");
    ros::NodeHandle nh;

    ros::Publisher twist_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1);

    ros::Rate loop_rate(1);  // 1 Hz, adjust the frequency based on your requirement

    while (ros::ok())
    {
        geometry_msgs::TwistStamped twist_cmd_msg;
        
        // Set a small negative angular velocity to turn left slowly
        twist_cmd_msg.twist.angular.z = 1.0;  // You can adjust this value to control the turning speed

        // Set linear velocity to zero (assuming only steering control)
        twist_cmd_msg.twist.linear.x = 0.0;

        twist_cmd_pub.publish(twist_cmd_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
