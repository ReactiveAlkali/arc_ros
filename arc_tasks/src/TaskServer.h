/**
* CLASS: TaskServer
* DATE: 06/04/17
* AUTHOR: Kyle Morris
* DESCRIPTION: All task servers must provide this functionality
*/

#ifndef ARC_TASKS_TASKSERVER_H
#define ARC_TASKS_TASKSERVER_H
#include "arc_msgs/ArcTaskAction.h"
#include "arc_msgs/TaskSuitability.h"
#include "ros/ros.h"

class TaskServer {
protected:
     /**
     * Keep track of most recent goal sent to this server.
     */
    arc_msgs::ArcTaskGoal recent_goal;

    /**
     * Service server to provide suitability
     */
    ros::ServiceServer suitability_server;

    /**
     * Client to obtain the bot's attributes
     */
    ros::ServiceClient attributes_client;

private:
    /**
     * Perform any routine startup procedures when this task instance is started.
     * Load request parameters, check for existence of nodes needed for this task.
     * @parameter goal: The goal set for this task
     */
    virtual void startup(const arc_msgs::ArcTaskGoalConstPtr &goal) = 0;

    /**
     * The main state machine loop for the task.
     */
    virtual void process() = 0;

    /**
     * Ensure after this task instance is no longer of use, we've shut down everything that was required for it.
     */
    virtual void shutdown() = 0;


public:
     /**
     * Receives request to perform the task.
     */
    void goal_cb(const arc_msgs::ArcTaskGoalConstPtr &goal);

    /**
     * Handles requests to get the task's suitability
     */
    //virtual bool suitability_cb(arc_msgs::TaskSuitability::Request& req,
        //arc_msgs::TaskSuitability::Response& res);
};

#endif //ARC_TASKS_TASKSERVER_H
