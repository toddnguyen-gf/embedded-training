/**
 * turtle_publisher_18642.cpp
 * Skeleton code by Milda Zizyte (milda@cmu.edu)
 * 
 * You should edit this file to use the /turtle1/cmd_vel topic
 * to draw a figure-eight with the turtlebot.
 *
 * A figure eight is two adjacent circles. You can draw this by drawing a full
 * circle using constant non-zero angular and linear velocity, and then
 * negating angular velocity and drawing a second full circle.
 *
 * For extra credit, add a function to print the Turtle's (x,y) location and
 * linear and angular velocity, by reading the turtlesim/Pose message.
 * You can print using the ROS_INFO(string) function.
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // access using geometry_msgs::Twist
#include "turtlesim/Pose.h"      // access using turtlesim::Pose

// any state variables or callback/computation functions go here
void draw_straight_line(ros::Publisher vel_pub);
void draw_eight_figure(ros::Publisher vel_pub);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_publisher_18642");
  ros::NodeHandle nh;

  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  ROS_INFO("Starting up turtle_publisher_18642.");

  /* 
   * The following is code to command the turtle in a straight line, in 5 steps.
   * You should replace it with your own code to draw a figure 8.
   */
  // draw_straight_line(vel_pub);
  draw_eight_figure(vel_pub);

  return 0;
}

void draw_straight_line(ros::Publisher vel_pub)
{
  ros::Rate r(.5); // .5Hz (run every 2 seconds)
  int num_steps = 5;

  while (num_steps > 0)
  {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 1.0;
    vel_pub.publish(vel_msg);
    num_steps--;
    ros::spinOnce();
    r.sleep();
  }
}

void draw_eight_figure(ros::Publisher vel_pub)
{
  ros::Rate r(.5); // .5Hz (run every 2 seconds)
  int num_step = 8;

  // Figure 8 top
  for (int i = 0; i < num_step; i++)
  {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 1.0;
    vel_msg.angular.z = 1.0;
    vel_pub.publish(vel_msg);
    ros::spinOnce();
    r.sleep();
  }

  // Negate angular velocity
  for (int i = 0; i < num_step; i++)
  {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 1.0;
    vel_msg.angular.z = -1.0;
    vel_pub.publish(vel_msg);
    ros::spinOnce();
    r.sleep();
  }
}
