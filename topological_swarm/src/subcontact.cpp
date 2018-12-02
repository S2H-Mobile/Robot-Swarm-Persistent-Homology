// This ROS node subscribes to the bumper sensor
// and stores contact events in a file.
#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>

void contactMessageReceived(const gazebo_msgs::ContactsState &msg) {
  if (!(msg.states.empty())) {
    ROS_INFO_STREAM("contact message received, is empty " << msg.states[0] << "\n");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_to_contact_message");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("bumper_sensor_state", 1000, &contactMessageReceived);
  ros::spin();
}
