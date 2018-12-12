#include "node_example_core.h"

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  NodeExample *node_example = new NodeExample();

  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<node_example::node_example_paramsConfig> dr_srv;
  dynamic_reconfigure::Server<node_example::node_example_paramsConfig>::CallbackType cb;
  cb = boost::bind(&NodeExample::configCallback, node_example, _1, _2);
  dr_srv.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  int a;
  int b;
  string message;
  int rate;
  string topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  // Parameters defined in the .cfg file do not need to be initialized here
  // as the dynamic_reconfigure::Server does this for you.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(40));
  private_node_handle_.param("topic", topic, string("example"));

  // Create a publisher and name the topic.
  ros::Publisher pub_message = n.advertise<node_example::node_example_data>(topic.c_str(), 10);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (n.ok())
  {
    // Publish the message.
    node_example->publishMessage(&pub_message);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()