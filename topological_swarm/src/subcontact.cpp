// This ROS node subscribes to the bumper sensor
// and stores contact events in a file.
#include "subcontact.h"

// delimiters for parsing contact info string
#define DELIMITER_DOUBLE_COLON "::"
#define DELIMITER_TIMESTAMP "time:"

// set the length of timestamp token when parsing contact info
#define LENGTH_TIME_TOKEN 7

// maximum number of contact events to be recorded
#define MAX_COUNT_CONTACT_EVENT 500

// list of contact events
std::vector<ContactEvent> contactEvents;

// flag to indicate whether contact events have been written to file
bool saved = false;

std::string parseForLink(const std::string s, const std::string key);
void contactMessageReceived(const gazebo_msgs::ContactsState &msg);
void handleContactMessage(const gazebo_msgs::ContactsState &msg);
void writeContactsToFile();

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_to_contact_message");
  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe("agent1/bumper_sensor_state", 1000, &contactMessageReceived);
  ros::Subscriber sub2 = nh.subscribe("agent2/bumper_sensor_state", 1000, &contactMessageReceived);
  ros::spin();
  return 0;
}

// parse contact info for object name
std::string parseForLink(const std::string s, const std::string key) {
  const int start = s.find(key) + key.length();
  const std::string token = s.substr(start, s.length() - start);
  return token.substr(0, token.find(DELIMITER_DOUBLE_COLON));
}

// callback for bumper sensor subscriber
void contactMessageReceived(const gazebo_msgs::ContactsState &msg) {
  if (contactEvents.size() < MAX_COUNT_CONTACT_EVENT) {
    handleContactMessage(msg);
  } else if (!saved) {
    // save file one time
    writeContactsToFile();
    saved = true;
  }
}

void handleContactMessage(const gazebo_msgs::ContactsState &msg) {
  if (!(msg.states.empty())) {

    // output information on vector at regular intervals
    const int size = contactEvents.size();
    if (size < 10 | size % 10 == 0) {
      ROS_INFO_STREAM("Recording contact event #" << size <<".\n");
    }

    // read the data from the message
    const gazebo_msgs::ContactState contactState = msg.states[0];
    const geometry_msgs::Vector3 position = contactState.contact_positions[0];

    // parse info string for timestamp
    const std::string delimiter = DELIMITER_TIMESTAMP;
    const int pos = contactState.info.find(delimiter) + delimiter.length();
    const std::string token = contactState.info.substr(pos, LENGTH_TIME_TOKEN);
    const double time = std::atof(token.c_str());

    // construct a contact event
    ContactEvent event;
    event.setContactTime(time);
    event.setContactPosition(position);
    event.setMyGeometry(parseForLink(contactState.info, "my geom:"));
    event.setOtherGeometry(parseForLink(contactState.info, "other geom:"));

    // append the event to the global vector
    contactEvents.push_back(event);
  }
}

void writeContactsToFile() {
  // create file name
  std::string fileName = ros::this_node::getName();
  fileName.erase(std::remove(fileName.begin(), fileName.end(), '/'), fileName.end());
  fileName += ".txt";
  // store the list of contact events
  std::ofstream file;
  file.open(fileName.c_str());
  if (file.is_open()) {
    for (std::vector<ContactEvent>::iterator it = contactEvents.begin(); it != contactEvents.end(); ++it) {
      // read contact event from vector
      ContactEvent event = *it;
      const geometry_msgs::Vector3 position = event.getContactPosition();
      const double time = event.getContactTime();
      // convert object data to string
      std::ostringstream line;
      line << "t: " << time << ",\tx: " << position.x << ",\ty: " << position.y << ",\tm: " << event.getMyGeometry() << ",\to: " << event.getOtherGeometry() << "\n";
      // write string to file
      file << line.str();
    }
    file.close();
    ROS_INFO_STREAM("Saved " << contactEvents.size() << " contact events to file \"" << fileName << "\".\n");
  } else {
    ROS_ERROR_STREAM("Unable to open file \"" << fileName << "\" for writing.\n");
  }
}

