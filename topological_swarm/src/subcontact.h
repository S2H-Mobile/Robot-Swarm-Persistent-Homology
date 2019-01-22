#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <signal.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>
#include <geometry_msgs/Vector3.h>

class ContactEvent {

  public:
    void setContactTime(ros::Time time);
    void setContactPosition(geometry_msgs::Vector3 position);
    ros::Time getContactTime();
    geometry_msgs::Vector3 getContactPosition();
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

ros::Time ContactEvent::getContactTime() {
  return contactTime;
}

geometry_msgs::Vector3 ContactEvent::getContactPosition() {
  return contactPosition;
}

void ContactEvent::print() {
  std::cout << contactTime << "\n" << contactPosition;
}
