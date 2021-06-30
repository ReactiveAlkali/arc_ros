#include "ros/ros.h"
#include "../include/KnowledgeManager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "knowledge_manager_node");

  KnowledgeManager manager;
  manager.run();

  return 0;
}
