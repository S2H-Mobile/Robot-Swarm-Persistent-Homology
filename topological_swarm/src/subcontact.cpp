// This ROS node subscribes to the bumper sensor
// and stores contact events in a file.
#include "subcontact.h"

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

void writeContactsToFileSigintHandler(const int signal) {

  // create file name
  std::string nameSpace = ros::this_node::getNamespace();
  nameSpace.erase(std::remove(nameSpace.begin(), nameSpace.end(), '/'), nameSpace.end());
  const char * fileName = nameSpace.c_str();

  ROS_INFO_STREAM("Contact events vector contains " << contactEvents.size() <<" elements.\n");
  ROS_INFO_STREAM("Saving events to file \"" << fileName << "\".\n");

  // store the list of contact events
  std::ofstream file;
  file.open(fileName);
  if (file.is_open()) {
    for (std::vector<ContactEvent>::iterator it = contactEvents.begin(); it != contactEvents.end(); ++it) {
      // read contact event from vector
      ContactEvent event = *it;
      const geometry_msgs::Vector3 position = event.getContactPosition();
      const ros::Time time = event.getContactTime();
      // convert object data to string
      std::ostringstream line;
      line << "t: " << time << ", x: " << position.x << ", y: " << position.y << "\n";
      // write string to file
      file << line.str();
    }
    file.close();
  } else {
    ROS_ERROR_STREAM("Unable to open file \"" << fileName << "\" for writing.\n");
  }
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_to_contact_message", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  signal(SIGINT, writeContactsToFileSigintHandler);
  ros::Subscriber sub = nh.subscribe("bumper_sensor_state", 1000, &contactMessageReceived);
  ros::spin();
  return 0;
}
