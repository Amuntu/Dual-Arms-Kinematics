#include <ros/ros.h>
#include <std_msgs/String.h>

void stringCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string str = msg->data;
  system(str.c_str());
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "string_subscriber_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<std_msgs::String>("SERIAL", 1, stringCallback);

  ros::spin();

  return 0;
}
