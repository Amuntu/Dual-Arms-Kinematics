#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <iostream>
using std::string;
using namespace std;

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Do something with the received Twist message
 float x = msg->linear.x;
 float y = msg->linear.y;
 float z = msg->linear.z;
 
 string s = "rosrun robot control " + to_string(x)+" "+to_string(y)+ " "+to_string(z)+"&";
 
 const char * s1 = s.c_str();
 system(s1);
 }
 
int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "my_node");

  // Create a node handle
  ros::NodeHandle nh;

  // Create a publisher for a Point message on the "/point" topic
 // ros::Publisher point_pub = nh.advertise<geometry_msgs::Twist>("/point", 10);

  // Create a subscriber for a Twist message on the "/twist" topic
  ros::Subscriber twist_sub = nh.subscribe("/point", 10, twistCallback);

  // Spin the node and wait for callbacks
  ros::spin();
  return 0;
}

// Create a Point message and publish it on the "/point" topic

