/**
 * Tests the assignee part of the task assignement process.  Publishes a task request to the
 * wireless requests topic.  Upon receiving a response a task confirmation is then sent.  It then
 * waits to recieve an acknowledgement from the robot.
 */

#include "ros/ros.h"
#include "arc_msgs/TaskRequest.h"
#include "arc_msgs/WirelessRequest.h"
#include "arc_msgs/WirelessResponse.h"
#include "dynamic_reconfigure/Config.h"
#include "std_srvs/Empty.h"

ros::Publisher wireless_response_pub;
ros::Publisher wireless_request_pub;

void sendConfirmation(dynamic_reconfigure::Config taskResponse)
{
  int task_id;

  for (std::size_t i = 0; i < taskResponse.ints.size(); ++i)
  {
    if (taskResponse.ints[i].name == "task_id")
    {
      ROS_INFO("Sending task %d confirmation", taskResponse.ints[i].value);
      arc_msgs::WirelessResponse wifi_response;
      wifi_response.type = wifi_response.TASK_CONFIRMATION;

      dynamic_reconfigure::BoolParameter confirmation;
      confirmation.name = "confirmation";
      confirmation.value = true;
      wifi_response.response.bools.push_back(confirmation);

      dynamic_reconfigure::IntParameter task_id;
      task_id.name = "task_id";
      task_id.value = taskResponse.ints[i].value;
      wifi_response.response.ints.push_back(task_id);

      wireless_response_pub.publish(wifi_response);
    }
  }
}

void responseCB(arc_msgs::WirelessResponse response)
{
  if (response.type == response.TASK_RESPONSE)
  {
    ROS_INFO("Recieved task response");
    for (std::size_t i = 0; i < response.response.bools.size(); ++i)
    {
      dynamic_reconfigure::BoolParameter curr = response.response.bools[i];
      
      if (curr.name == "accepted" && curr.value)
        sendConfirmation(response.response);
    } 
  }
  else if (response.type == response.TASK_ACKNOWLEDGEMENT)
  {
    ROS_INFO("Received task acknowledgement");
  }
}

bool startTest(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  // Create a task request
  arc_msgs::TaskRequest taskRequest;
  taskRequest.task_id = 3;
  taskRequest.created = ros::Time::now();
  taskRequest.task_name = "explore";
  taskRequest.request_type = taskRequest.TYPE_COMPLETION;

  // Send the wireless request
  ROS_INFO("Sending task request");
  arc_msgs::WirelessRequest wifiRequest;
  wifiRequest.task = taskRequest;
  wireless_request_pub.publish(wifiRequest);
  
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mock_task_assigner");
  ros::NodeHandle handle;

  // Interface directly with the communication manager
  ros::Subscriber wireless_response_sub = handle.subscribe(
      "/arc/test_bot/wifi_handler/outgoing_responses", 1000, &responseCB);
  wireless_request_pub = handle.advertise<arc_msgs::WirelessRequest>(
      "/arc/test_bot/wifi_handler/incoming_requests", 1000);
  wireless_response_pub = handle.advertise<arc_msgs::WirelessResponse>(
      "/arc/test_bot/wifi_handler/incoming_responses", 1000);
  
  ros::ServiceServer server = handle.advertiseService("test_task_responses", startTest);


  ros::spin();

  return 0;
}
