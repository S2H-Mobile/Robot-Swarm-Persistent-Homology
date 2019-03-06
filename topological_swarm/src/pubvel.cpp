#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <boost/functional/hash.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "publish_velocity");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  // initialize rand with current time and node name
  std::string name = ros::this_node::getName();
  const int seed = time(NULL) + boost::hash<std::string>()(name);
  srand(seed);

  ros::Rate rate(2);

  while(ros::ok()) {
    geometry_msgs::Twist msg;

    // assign random numbers in the range [-1,1] to x and y velocity
    msg.linear.x = 2 * double(rand())/double(RAND_MAX) - 1;
    msg.linear.y = 2 * double(rand())/double(RAND_MAX) - 1;

    pub.publish(msg);
    // ROS_INFO_STREAM("x= " << msg.linear.x << "y= " << msg.linear.y);
    rate.sleep();
  }
}
