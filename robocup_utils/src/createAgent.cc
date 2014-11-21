#include "ros/ros.h"
#include "robocup_msgs/InitAgent.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robocup_client");
  if (argc != 4)
  {
    ROS_INFO("usage: createAgent <path> <team_name> <player_number>");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<robocup_msgs::InitAgent>("/gameController/init_agent");
  robocup_msgs::InitAgent srv;
  srv.request.agent = argv[1];
  srv.request.team_name = argv[2];
  srv.request.player_number = atoi(argv[3]);
  if (client.call(srv))
  {
    ROS_INFO("Result: %d", (uint8_t)srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service init_agent");
    return 1;
  }

  return 0;
}
