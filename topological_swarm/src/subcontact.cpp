// This ROS node subscribes to the bumper sensor
// and stores contact events in a file.
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>

// list of contact events
std::vector<gazebo_msgs::ContactState> contactStates;

// handle a message from the bumper sensor
void contactMessageReceived(const gazebo_msgs::ContactsState &msg) {
  if (!(msg.states.empty())) {
    gazebo_msgs::ContactState contactState = msg.states[0];
    contactStates.assign(contactStates.size() + 1, contactState);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_to_contact_message");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("bumper_sensor_state", 1000, &contactMessageReceived);
  ros::spin();

  // TODO store the list of contact events
  std::cout << "Contact states vector contains " << contactStates.size() <<" elements.\n";
}
