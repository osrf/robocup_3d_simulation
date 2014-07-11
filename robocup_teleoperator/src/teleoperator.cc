#include "ros/ros.h"
#include "robocup_msgs/SendJoints.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robocup_teleoperator");
  if (argc != 2)
  {
    ROS_INFO("usage: teleoperator <robot_name>");
    return 1;
  }

  int jointNumber;
  double newValue;
  ros::NodeHandle n;
  std::string robot = argv[1];
  std::cout << "Connected to " << robot << std::endl;

  ros::ServiceClient client =
    n.serviceClient<robocup_msgs::SendJoints>(robot + "/send_joints");

  while (true)
  {
    // Initialize joints.
    boost::array<float, 22> jointValues;;
    for (int i = 0; i < 22; ++i)
      jointValues[i] = 0.0;

    robocup_msgs::SendJoints srv;

    for (int i = 0; i < 22; ++i)
      srv.request.joints[i] = jointValues[i];

    do
    {
      std::cout << "Select joint:" << std::endl;
      std::cout << "0.HeadYaw" << std::endl;
      std::cout << "1.HeadPitch" << std::endl;
      std::cout << "2.LHipYawPitch" << std::endl;
      std::cout << "3.LHipRoll" << std::endl;
      std::cout << "4.LHipPitch" << std::endl;
      std::cout << "5.LKneePitch" << std::endl;
      std::cout << "6.LAnklePitch" << std::endl;
      std::cout << "7.LAnkleRoll" << std::endl;
      std::cout << "8.LShoulderPitch" << std::endl;
      std::cout << "9.LShoulderRoll" << std::endl;
      std::cout << "10.LElbowYaw" << std::endl;
      std::cout << "11.LElbowRoll" << std::endl;
      std::cout << "12.RHipYawPitch" << std::endl;
      std::cout << "13.RHipRoll" << std::endl;
      std::cout << "14.RHipPitch" << std::endl;
      std::cout << "15.RKneePitch" << std::endl;
      std::cout << "16.RAnklePitch" << std::endl;
      std::cout << "17.RAnkleRoll" << std::endl;
      std::cout << "18.RShoulderPitch" << std::endl;
      std::cout << "19.RShoulderRoll" << std::endl;
      std::cout << "20.RElbowYaw" << std::endl;
      std::cout << "21.RElbowRoll" << std::endl;
      std::cout << std::endl << "Joint: ";
      std::cin >> jointNumber;
      std::cout << std::endl;
      std::cout << "Joint: [" << jointNumber << "]" << std::endl;
    }
    while ((jointNumber < 0 || jointNumber > 21));

    std::cout << "Set the new value for the joint: ";
    std::cin >> newValue;
    std::cout << std::endl;

    srv.request.joints[jointNumber] = newValue;

    if (client.call(srv))
    {
      ROS_INFO("Result: %d", (uint8_t)srv.response.result);
    }
    else
    {
      ROS_ERROR("Failed to call service init_agent");
      return 1;
    }
    std::cout << "Press <ENTER> for sending a new command.";
  }

  return 0;
}
