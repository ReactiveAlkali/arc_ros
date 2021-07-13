/**
* CLASS: CommunicationManager
* DATE: 21/03/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Manages incoming and outgoing communication.
* After the wifi_handler processes incoming signals,
* they will be received by this node and further decomposed for the robot.
* The robot can also make broadcasts requests to this node
* with tasks or other messages which are then wrapped up as
* signals and broadcasted by the wifi_handler.
*/

#ifndef PROJECT_COMMUNICATIONMANAGER_H
#define PROJECT_COMMUNICATIONMANAGER_H
#include "ros/ros.h"
#include "arc_msgs/WirelessAnnouncement.h"
#include "arc_msgs/WirelessRequest.h"
#include "arc_msgs/WirelessResponse.h"
#include "arc_msgs/TaskRequest.h"
#include "arc_msgs/TaskResponse.h"
#include "arc_msgs/BotInfo.h"
#include "nav_msgs/Odometry.h"

class CommunicationManager {
private:
    /**
     * Subscribing to incoming signals.
     */
    ros::Subscriber incoming_announcements_sub;
    ros::Subscriber incoming_requests_sub;
    ros::Subscriber incoming_responses_sub;

    /**
     * Publishing communication signals this robot received, onto it's internal communication lines.
     * this allows the core components/nodes to focus on broadcasts only pertaining to them.
     */
    ros::Publisher outgoing_announcements_pub;
    ros::Publisher outgoing_requests_pub;
    ros::Publisher outgoing_responses_pub;

    /**
     * Subscribe to our current position
     */
    ros::Subscriber base_pose_sub; //TODO Use TF support to track position instead of base pose

    /**
     * Task requests that made it through the communication managers filter.
     */
    ros::Publisher task_requests_pub;

    /**
     * Publish bot info of other robot's to the knowledge manager
     */
    ros::Publisher bot_info_pub;

    /**
     * Retrieve information on our robot from the knowledge manager
     */
    ros::ServiceClient bot_info_client;

    /**
     * Subscribe to outgoing task responses
     */
    ros::Subscriber outgoing_task_response_sub;

    /**
     * Publish task confirmations
     */
    ros::Publisher task_confirmations_pub;

    /**
     * Keep track of the bot's current location to attach to outgoing messages
     */
    nav_msgs::Odometry recent_pose;

    ros::NodeHandle local_handle;
    ros::NodeHandle global_handle;

private:
    /**
     * List of the task names that are valid.
     */
    std::vector<std::string> valid_tasks;

    /**
     * validate incoming task requests and only allow them to pass through system if they are valid.
     * @return true if task request is valid, false otherwise
     */
    bool isTaskRequestValid(arc_msgs::TaskRequest msg);

    /**
     * Populates a wireless message with information common with all messages
     * @param msg Reference to the message to prepare
     */
    template<typename T>
    void prepareWirelessMessage(T& msg);
public:
    CommunicationManager();

    /**
     * Main loop
     */
    void run();

    /**
    * Callbacks for all of the signals.
    */
    void process_incoming_announcements_cb(arc_msgs::WirelessAnnouncement announcement);
    void process_incoming_requests_cb(arc_msgs::WirelessRequest request);
    void process_incoming_responses_cb(arc_msgs::WirelessResponse response);

    /**
     * Callbacks for sending signals
     */
    void sendTaskResponse(arc_msgs::TaskResponse response);

    /**
     * Callback to update our position
     */
    void processBasePoseCB(nav_msgs::Odometry odom);
};

#endif //PROJECT_COMMUNICATIONMANAGER_H
