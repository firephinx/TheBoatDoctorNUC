#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "hebiros/AddGroupFromNamesSrv.h"

using namespace hebiros;


//Global variable and callback function used to store feedback data
sensor_msgs::JointState feedback;

void feedback_callback(sensor_msgs::JointState data) {
  feedback = data;
}


int main(int argc, char **argv) {

  //Initialize ROS node
  ros::init(argc, argv, "example_03_command_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(200);

  std::string group_name = "TheBoatDoctor";

  //Create a client which uses the service to create a group
  ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>(
    "/hebiros/add_group_from_names");

  AddGroupFromNamesSrv add_group_srv;

  //Construct a group using 3 known modules
  add_group_srv.request.group_name = group_name;
  add_group_srv.request.names = {"elbow", "wrist", "end effector"};
  add_group_srv.request.families = {"The Boat Doctor"};
  //Call the add_group_from_urdf service to create a group until it succeeds
  //Specific topics and services will now be available under this group's namespace
  while(!add_group_client.call(add_group_srv)) {}

  //Create a subscriber to receive feedback from a group
  //Register feedback_callback as a callback which runs when feedback is received
  ros::Subscriber feedback_subscriber = n.subscribe(
    "/hebiros/"+group_name+"/feedback/joint_state", 100, feedback_callback);

  //Create a publisher to send desired commands to a group
  ros::Publisher command_publisher = n.advertise<sensor_msgs::JointState>(
    "/hebiros/"+group_name+"/command/joint_state", 100);

  //Construct a JointState to command the modules
  //This may potentially contain a name, position, velocity, and effort for each module
  sensor_msgs::JointState command_msg;
  command_msg.name.push_back("The Boat Doctor/elbow");
  command_msg.name.push_back("The Boat Doctor/wrist");
  command_msg.name.push_back("The Boat Doctor/end effector");

  command_msg.position.resize(3);

  feedback.position.reserve(3);

  while(ros::ok()) {
    command_msg.position[0] = feedback.position[0];
    command_msg.position[1] = feedback.position[1];
    command_msg.position[2] = feedback.position[2];
    command_publisher.publish(command_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}









