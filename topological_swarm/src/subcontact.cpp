// This ROS node subscribes to the bumper sensor
// and stores contact events in a file.
#include "subcontact.h"

// list of contact events
std::vector<ContactEvent> contactEvents;

// handle a message from the bumper sensor
void contactMessageReceived(const gazebo_msgs::ContactsState &msg) {
  if (!(msg.states.empty())) {

    // output information on vector at regular intervals
    const int size = contactEvents.size();
    if (size < 10 | size % 10 == 0) {
      ROS_INFO_STREAM("Recording contact event #" << size <<".\n");
    }

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
  std::string fileName = ros::this_node::getName();
  fileName.erase(std::remove(fileName.begin(), fileName.end(), '/'), fileName.end());
  fileName += ".txt";

  ROS_INFO_STREAM("Saving " << contactEvents.size() << " contact events to file \"" << fileName << "\".\n");

  // store the list of contact events
  std::ofstream file;
  file.open(fileName.c_str());
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
  ros::Subscriber sub1 = nh.subscribe("agent1/bumper_sensor_state", 1000, &contactMessageReceived);
  ros::Subscriber sub2 = nh.subscribe("agent2/bumper_sensor_state", 1000, &contactMessageReceived);
  ros::spin();
  return 0;
}
