/**
 * role_manager_test
 * DATE: 04/08/2021
 * AUTHOR: Aaron Nedelec
 * DESCRIPTION: Tests the functionality of the role manager component of the framework
 */

#include "arc_msgs/BotInfo.h"
#include "arc_msgs/WirelessAnnouncement.h"
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <unique_id/unique_id.h>

ros::Publisher team_id_pub;
ros::Publisher wireless_announcement_pub;

bool startTest(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("Testing the role manager");
  // Simulate a bot with a higher suitability for the robot's current role being added to the team
  arc_msgs::BotInfo sim_bot;
  sim_bot.robot_id = unique_id::toMsg(unique_id::fromRandom());
  sim_bot.team_id = unique_id::toMsg(unique_id::fromHexString(
        "41b39c5b-4ca0-48b9-af74-d5694f46abcc"));
  sim_bot.role = 0;
  sim_bot.suitability = std::numeric_limits<int>::max();

  // Send a wireless announcment containing this bot
  arc_msgs::WirelessAnnouncement wifi_msg;
  wifi_msg.info = sim_bot;
  wireless_announcement_pub.publish(wifi_msg);

  // Make sure the test bot has the same team ID
  team_id_pub.publish(unique_id::toMsg(unique_id::fromHexString(
          "41b39c5b-4ca0-48b9-af74-d5694f46abcc")));

  return true;
}

int main(int argc, char** argv)
{
  ROS_INFO("Setting up role manager test");
  ros::init(argc, argv, "role_manager_test");
  ros::NodeHandle handle;

  // Act as though we are the robot's wifi handler
  wireless_announcement_pub = handle.advertise<arc_msgs::WirelessAnnouncement>(
      "/arc/test_bot/wifi_handler/incoming_announcements", 1000);
  team_id_pub = handle.advertise<uuid_msgs::UniqueID>("/arc/test_bot/knowledge_manager/update_team",
      1000);
  
  ros::ServiceServer server = handle.advertiseService("start_role_manager_test", &startTest);

  ros::spin();

  return 0;
}
