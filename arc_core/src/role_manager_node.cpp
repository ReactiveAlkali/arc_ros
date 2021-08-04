#include "../include/RoleManager.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "role_manager_node");

  RoleManager manager;
  manager.run();

  return 0;
}
