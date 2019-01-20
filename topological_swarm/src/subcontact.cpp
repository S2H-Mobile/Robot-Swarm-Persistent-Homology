// This ROS node subscribes to the bumper sensor
// and stores contact events in a file.
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

// list of contact events
std::vector<ContactEvent> contactEvents;

// handle a message from the bumper sensor
void contactMessageReceived(const gazebo_msgs::ContactsState &msg) {
  if (!(msg.states.empty())) {

    // read the data from the message
    const gazebo_msgs::ContactState contactState = msg.states[0];
    const std::string info = contactState.info;
    const geometry_msgs::Vector3 position = contactState.contact_positions[0];

    // construct a contact event
    ContactEvent event;
    event.setContactTime(ros::Time::now());
    event.setContactPosition(position);

    // append the event to the global vector
    contactEvents.push_back(event);
  }
}

void writeContactsToFile() {

  // generate file name
  const std::string name = ros::this_node::getName();

  std::cout << "Contact events vector for " << name << " contains " << contactEvents.size() <<" elements.\n";

  // TODO store the list of contact events
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_to_contact_message");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("bumper_sensor_state", 1000, &contactMessageReceived);
  ros::spin();
  writeContactsToFile();
}
