#include <vector>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>
#include <geometry_msgs/Vector3.h>

class ContactEvent {

  public:
    void setContactTime(ros::Time time);
    void setContactPosition(geometry_msgs::Vector3 position);
    void print();

  private:
    ros::Time contactTime;
    geometry_msgs::Vector3 contactPosition;

};

void ContactEvent::setContactTime(ros::Time time) {
  contactTime = time;
}

void ContactEvent::setContactPosition(geometry_msgs::Vector3 position) {
  contactPosition = position;
}

void ContactEvent::print() {
  std::cout << contactTime << "\n" << contactPosition;
}
