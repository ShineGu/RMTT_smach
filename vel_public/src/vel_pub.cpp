#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

bool go = false;

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(msg->pose.position.x != 0)
        go = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_pub");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_go", 10);
    ros::Subscriber pose_sub = n.subscribe("/pose", 10, callback);
    ros::Rate loop_rate(10);
    int count = 0;

    double velx[] = {0, 0.3, -0.3};
    double vely[] = {0, 0.3, -0.3};
    double angle[] = {0, 0.4, -0.4};

    while(ros::ok())
    {
        geometry_msgs::Twist vel_msg;
        if(go) {
            if (count / 10 >= 0) {
                vel_msg.linear.x = velx[1];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[0];
            }
            if (count / 10 >= 4) {
                vel_msg.linear.x = velx[0];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[0];
            }
            if (count / 10 >= 5) {
                vel_msg.linear.x = velx[0];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[2];
            }
            if (count / 10 >= 9) {
                vel_msg.linear.x = velx[0];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[0];
            }
            if (count / 10 >= 10) {
                vel_msg.linear.x = velx[1];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[0];
            }
            if (count / 10 >= 15) {
                vel_msg.linear.x = velx[0];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[0];
            }
            if (count / 10 >= 16) {
                vel_msg.linear.x = velx[0];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[2];
            }
            if (count / 10 >= 20) {
                vel_msg.linear.x = velx[0];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[0];
            }
            if (count / 10 >= 21) {
                vel_msg.linear.x = velx[1];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[0];
            }
            if (count / 10 >= 25) {
                vel_msg.linear.x = velx[0];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[0];
            }
            if (count / 10 >= 26) {
                vel_msg.linear.x = velx[0];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[1];
            }
            if (count / 10 >= 30) {
                vel_msg.linear.x = velx[0];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[0];
            }
            if (count / 10 >= 31) {
                vel_msg.linear.x = velx[1];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[0];
            }
            if (count / 10 >= 34) {
                vel_msg.linear.x = velx[0];
                vel_msg.linear.y = vely[0];
                vel_msg.angular.z = angle[0];
            }


            vel_pub.publish(vel_msg);
            ROS_INFO("Publish velocity command[%0.2f m/s, %0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.linear.y,
                     vel_msg.angular.z);

            count++;
        }
        else
            ROS_INFO("no vel");

        loop_rate.sleep();
    }

    return 0;
}