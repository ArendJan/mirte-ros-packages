#include <chrono>
#include <functional>
#include <speed_control.hpp> 


int main(int argc, char **argv) {
  ros::init(argc, argv, "my_robot_base_node");

  MyRobotHWInterface hw;

  ros::spin();

  return 0;
}
