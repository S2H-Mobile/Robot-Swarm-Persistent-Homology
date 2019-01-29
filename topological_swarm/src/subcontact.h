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
    void setMyGeometry(std::string input);
    void setOtherGeometry(std::string input);
    ros::Time getContactTime();
    geometry_msgs::Vector3 getContactPosition();
    std::string getMyGeometry();
    std::string getOtherGeometry();
    void print();

  private:
    ros::Time contactTime;
    geometry_msgs::Vector3 contactPosition;
    std::string myGeometry;
    std::string otherGeometry;

};

void ContactEvent::setContactTime(ros::Time time) {
  contactTime = time;
}

void ContactEvent::setContactPosition(geometry_msgs::Vector3 position) {
  contactPosition = position;
}

void ContactEvent::setMyGeometry(std::string input) {
  myGeometry = input;
}

void ContactEvent::setOtherGeometry(std::string input) {
  otherGeometry = input;
}

ros::Time ContactEvent::getContactTime() {
  return contactTime;
}

geometry_msgs::Vector3 ContactEvent::getContactPosition() {
  return contactPosition;
}

std::string ContactEvent::getMyGeometry() {
  return myGeometry;
}

std::string ContactEvent::getOtherGeometry() {
  return otherGeometry;
}

void ContactEvent::print() {
  std::cout << contactTime << "\n" << contactPosition;
}
