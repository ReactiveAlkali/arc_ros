
#include "arc_msgs/ToggleSchema.h"
#include "TaskConfirmVictimServer.h"
#include "dynamic_reconfigure/Config.h"
#include <arc_msgs/DetectedVictims.h>
#include "boost/algorithm/string.hpp"
#include "TaskServer.h"


#define MAX_QUEUE_SIZE 1000

TaskConfirmVictimServer::TaskConfirmVictimServer() : server(global_handle, "task_guided_clean_debris", boost::bind(&TaskServer::goal_cb, this, _1), false)
{
    ros::NodeHandle local_handle("task_guided_clean_debris_server");
    ros::Timer timer = global_handle.createTimer(ros::Duration(60), &TaskConfirmVictimServer::explore_timer_cb, this, false);
    this->victim_sub = global_handle.subscribe("detect_victim_ps/found_victims", MAX_QUEUE_SIZE, &TaskConfirmVictimServer::found_victims_cb, this);
    this->base_pos_sub = global_handle.subscribe("base_pose_ground_truth", MAX_QUEUE_SIZE, &TaskConfirmVictimServer::base_pose_cb, this);
    this->pose_listener;

    this->victim_success_count = 0;

    this->local_handle = local_handle;
    //TODO: Handle pre-empt callback as well

    this->arc_base_client = global_handle.serviceClient<arc_msgs::ToggleSchema>("arc_base/toggle_schema");
    this->move_to_debris_client = global_handle.serviceClient<arc_msgs::NavigationRequest>("move_to_goal_ms/move_to_goal");
    this->abort_all_goals_client = global_handle.serviceClient<std_srvs::Trigger>("navigation_adapter/abort_goals");

    //Enable the move_to_goal schema since we will be using it throughout task.
    dynamic_reconfigure::BoolParameter move_to_goal_ms;
    arc_msgs::ToggleSchema request;
    move_to_goal_ms.name = "move_to_goal_ms";
    move_to_goal_ms.value = true;
    request.request.schema.push_back(move_to_goal_ms); //allow robot to wander around randomly
    this->arc_base_client.call(request);
    this->server.start();

}

void TaskConfirmVictimServer::base_pose_cb(const nav_msgs::Odometry &odom) {
    this->recent_pose = odom;
}

void TaskConfirmVictimServer::explore_timer_cb(const ros::TimerEvent &event) {
    //State transition
    ROS_INFO("Explore timer cb called. Started at %d and ended at %d",event.last_real, event.current_real);
    this->shutdown();
}

void TaskConfirmVictimServer::shutdown() {
    arc_msgs::ToggleSchema request;

    //disable schemas
    dynamic_reconfigure::BoolParameter schema_clean;
    schema_clean.name = "clean_debris_ms";
    schema_clean.value = false;
    request.request.schema.push_back(schema_clean);

    this->arc_base_client.call(request);

    //turn off flags
    this->result.task_id = this->recent_goal.task_id;
    this->instance_state.currently_seeking_debris = false;

    this->server.setSucceeded(this->result);
}

void TaskConfirmVictimServer::found_victims_cb(const arc_msgs::DetectedVictims &victims) {
    this->victims_found_nearby = victims;
}

std::string TaskConfirmVictimServer::stateToString(TaskConfirmVictimServer::State state) {
    std::string result = "";
    if (state == STATE_SelectVictimTarget) {
        result = "SelectDebrisTarget";
    } else if (state == STATE_SeekingVictimLocation) {
        result = "SeekingDebrisLocation";
    } else if (state == STATE_DetectingVictim) {
        result = "RemovingDebris";
    }

    return result;
}

void TaskConfirmVictimServer::StateSelectVictimTarget() {
    if(this->victim_list.victims.empty()) {
        arc_msgs::ArcTaskActionResult result;

        if(this->victim_count - this->victim_success_count == 0) { //no failed attempts
            result.result.completed = true;
        } else {
            result.result.completed = false;
        }

        this->result.final_state = stateToString(this->state).c_str();
        this->server.setSucceeded(this->result);

        this->shutdown();
    } else {
        //find victim closest to us.
        this->target_victim =this->victim_list.victims.at(0);
        this->target_victim.status = this->victim_list.victims.at(0).status;
        int nearest_debris_pos = 0; //position of the nearest debris

        for(int pos = 1; pos < this->victim_list.victims.size(); pos++) {
            arc_msgs::DetectedVictim curr = this->victim_list.victims.at(pos);
            double curr_distance_away = sqrt(pow(curr.pose.position.x,2) + pow(curr.pose.position.y,2));
            double min_distance_away = sqrt(pow(this->target_victim.pose.position.x, 2) + pow(this->target_victim.pose.position.y,2));
            ROS_DEBUG("checking debris at location %f, %f", curr.pose.position.x, curr.pose.position.y);

            //compare the distance of current debris, and the current minimum to see if we have a new min
            if(curr_distance_away < min_distance_away) {
                this->target_victim = curr;
                this->target_victim.status = curr.status;
                nearest_debris_pos = pos;
            }
        }

        this->target_pose.position = this->target_victim.pose.position;
        this->target_pose.orientation.w = 1.0;

        //remove the nearest debris from the list now so we don't look for it later. this->victim_list.debris.erase(victim_list.debris.begin()+nearest_debris_pos);

        //Now we head to where this debris is. This is giving us info about debris in position relative (in front of us)
        ROS_INFO("Planning on going to location (%f, %f) to find victim with status %d", target_pose.position.x, target_pose.position.y, this->target_victim.status);
        this->state = STATE_SeekingVictimLocation;
    }
}

void TaskConfirmVictimServer::startup(const arc_msgs::ArcTaskGoalConstPtr &goal) {
    const char REQUEST_DELIMITER = '|';
    ROS_INFO("Starting up task_guidedcleandebris.");

    try {
        //handling string parameters
        ROS_INFO("Trying to cycle through %d debris items", (int)goal->parameters.ints.size());
        for (int pos = 0; pos < goal->parameters.strs.size(); pos++) {
            dynamic_reconfigure::StrParameter curr_param = goal->parameters.strs.at(pos);

            if (curr_param.name == "victim_list") {
                int pos = 0;
                std::string curr_coordinate;
                ROS_INFO("Checking debris parameter list");

                this->victim_list.victims = parseVictimList(curr_param.value).victims;
            }
        }
    } catch(std::invalid_argument &e) {
        ROS_WARN("%s",e.what());
    }

    this->state = STATE_SelectVictimTarget;
}

arc_msgs::DetectedVictims TaskConfirmVictimServer::parseVictimList(std::string input) {
//Go through each coordinate pair "(x,y)"
    arc_msgs::DetectedVictims victim_list;
    int pos = 0;
    std::string curr_coordinate;

    while ((pos = input.find("|")) != std::string::npos) {
        curr_coordinate = input.substr(0, pos);
        curr_coordinate.erase(0, 1); //get rid of leading (
        curr_coordinate.erase(curr_coordinate.size() - 1, curr_coordinate.size()); //get rid of rightmost )

        int first_comma_pos = curr_coordinate.find(",");
        int second_comma_pos = curr_coordinate.find(",", first_comma_pos + 1);

        //extracting the actual coordinates. finally... geez c++, verbose much?
        int status = atoi(curr_coordinate.substr(0,first_comma_pos).c_str());
        double x = atof(curr_coordinate.substr(first_comma_pos + 1, second_comma_pos).c_str());
        double y = atof(curr_coordinate.substr(second_comma_pos + 1, curr_coordinate.size()).c_str());

        arc_msgs::DetectedVictim curr_victim;
        curr_victim.status = status;
        curr_victim.pose.position.x = x;
        curr_victim.pose.position.y = y;
        curr_victim.pose.orientation.w = 1.0;

        ROS_INFO("Requested Confirm Victim task with parameter (victim_status=%d, x_pos=%f, y_pos=%f)",status, x,y);
        victim_list.victims.push_back(curr_victim);

        ROS_ASSERT(x >= 0 && y >= 0);

        input.erase(0, pos + 1); //+1 because request delimiter is length 1 char.
    }

    ROS_INFO("Left with this for string %s", input.c_str());

    //assume just a single parameter here
    if(input.size()>0) {
        int first_comma_pos = input.find(",");
        int second_comma_pos = input.find(",", first_comma_pos + 1);

        //extracting the actual coordinates. finally... geez c++, verbose much?
        int status = atoi(input.substr(1,first_comma_pos).c_str());
        double x = atof(input.substr(first_comma_pos + 1, second_comma_pos).c_str());
        double y = atof(input.substr(second_comma_pos + 1, input.size()).c_str());

        arc_msgs::DetectedVictim curr_victim;
        curr_victim.status = status;
        curr_victim.pose.position.x = x;
        curr_victim.pose.position.y = y;
        curr_victim.pose.orientation.w = 1.0;

        ROS_INFO("Requested Confirm Victim task with parameter (victim_status=%d, x_pos=%f, y_pos=%f)",status, x,y);
        victim_list.victims.push_back(curr_victim);

        ROS_ASSERT(x >= 0 && y >= 0);
    }

    for(int pos=0; pos < victim_list.victims.size(); pos++) {
        ROS_INFO("Parsed the victim at (%d, %d)", (int)victim_list.victims.at(pos).pose.position.x, (int)victim_list.victims.at(pos).pose.position.y);
    }

    return victim_list;
}

bool TaskConfirmVictimServer::decodeStringParameter(std::string name, std::string value) {
    ROS_INFO("Parameter named %s with value %s", name.c_str(), value.c_str());
}

void TaskConfirmVictimServer::process() {
    ros::Rate r(10);

    //toggling of server (setSucceeded() etc, should only be done in this loop, not within state methods)
    while(ros::ok() && server.isActive()) {
        ros::spinOnce();
        if (state == STATE_SelectVictimTarget) {
            ROS_INFO_ONCE("IN STATE: SelectVictimTarget");
            StateSelectVictimTarget();
        } else if (state == STATE_SeekingVictimLocation) {
            StateSeekingVictimLocation();
        } else if (state == STATE_DetectingVictim) {
            ROS_INFO_ONCE("IN STATE: DetectingVictim.");
            StateDetectingVictim();
        }
        r.sleep();
    }
}

void TaskConfirmVictimServer::StateSeekingVictimLocation() {
    //only send a goal if we have not sent one already
    if(!this->instance_state.currently_seeking_debris) {
        //Enable the move_to_goal schema so we can move to the debris
        dynamic_reconfigure::BoolParameter move_to_goal_ms;
        arc_msgs::ToggleSchema request;
        move_to_goal_ms.name = "move_to_goal_ms";
        move_to_goal_ms.value = true;
        request.request.schema.push_back(move_to_goal_ms); //allow robot to wander around randomly
        this->arc_base_client.call(request);

        this->instance_state.currently_seeking_debris = true;
        arc_msgs::NavigationRequest req;
        req.request.pose.position.x = this->target_pose.position.x;
        req.request.pose.position.y = this->target_pose.position.y;

        ROS_INFO("Sending navigation request to location (%f, %f)", req.request.pose.position.x, req.request.pose.position.y);
        this->move_to_debris_client.call(req);

    } else { //we are heading to the victim
        //check our current distance from victim
        double curr_x = this->recent_pose.pose.pose.position.x;
        double curr_y = this->recent_pose.pose.pose.position.y;
        double distance_from_debris = sqrt(pow(curr_x - target_pose.position.x, 2) + pow(curr_y - target_pose.position.y,2));
        ROS_INFO("Currently %f meters away from the victim.", distance_from_debris);

        if(distance_from_debris < this->stopping_distance_from_victim) {
            ROS_INFO("CLOSE ENOUGH TO VICTIM TO STOP!");
            std_srvs::Trigger abort_trigger;
            this->abort_all_goals_client.call(abort_trigger);

            //now we can start cleaning
            this->state = STATE_DetectingVictim;
            this->instance_state.currently_seeking_debris = false;
        }
    }

    //check to see once we are close enough to the debris location
    ROS_INFO_ONCE("Planning on going to position (%f, %f)", this->target_pose.position.x, this->target_pose.position.y);
}

void TaskConfirmVictimServer::StateDetectingVictim() {
    //check if we found a victim within target range.
    arc_msgs::DetectedVictim victimFound;

    auto it = victims_found_nearby.victims.begin();
    while(it!=victims_found_nearby.victims.end()) {
        auto locationFound = it->pose.position;

        double potentialVictimDisplacement = sqrt(pow(locationFound.x-this->target_pose.position.x, 2) + pow(locationFound.x - this->target_pose.position.y,2));

        //if we found a victim within this area we claim we found our result
        if(potentialVictimDisplacement <= MAX_VICTIM_DISPLACMENT_THRESHOLD) {
            this->instance_state.found_debris_target = true;
            victimFound = *it;
            it = victims_found_nearby.victims.erase(it);
            break;
        }

        it++;
    }

    if(!this->instance_state.found_debris_target) {
        ROS_INFO("Moved to location (%f, %f) but could not find victim. Moving on.", (float)target_victim.pose.position.x, (float)target_victim.pose.position.y, (int)target_victim.status);
        //TODO: broadcast a failed to find victim message here.
    } else {
        if(victimFound.status==POSITIVE) {
            //TODO: Broadcast a success in finding victim message.. no need to clean. Just go back to finding next victim
            ROS_INFO("The potential victim actually is a victim! CALL RESCUE TEAM.");
        } else if(victimFound.status==NEGATIVE) {
            //TODO: Broadcast a success in finding victim message.. no need to clean. Just go back to finding next victim
            ROS_INFO("The potential victim is actually not a victim... That's good.");
        } else if(victimFound.status==POTENTIAL) {
            ROS_INFO("You shouldn't have gone out looking to confirm this victim if you lack the sensor capability... silly. Also this should never happen.");
        }
    }

    this->state = STATE_SelectVictimTarget;
}
