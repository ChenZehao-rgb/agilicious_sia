#include <ros/ros.h>

#include "agiros/ros_pilot.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "agiros_pilot");
  agi::RosPilot pilot;

  ros::spin();
  return 0;
}
