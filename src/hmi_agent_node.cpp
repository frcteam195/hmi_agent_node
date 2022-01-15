#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <rio_control_node/Joystick_Status.h>

#include <../../action_processor_node/include/helper/action_helper.hpp>

#define RATE (100)

ros::NodeHandle* node;
ActionHelper* action_helper;


void joystick_status_callback( const rio_control_node::Joystick_Status& joytick_status )
{

    // we will parse Joystick 0 buttons 0/1/2/3

    if( joytick_status.joysticks.size() <= 0){ return; }
    if( joytick_status.joysticks[0].buttons.size() <= 4){ return; }

    const rio_control_node::Joystick* current_joystick = &joytick_status.joysticks[0];

    if( current_joystick->buttons[0] == 1 )
    {
        std::cout << "Single Action\n";
        action_helper->req_single_action("testingActions", "singleactionhere");
    }

    if( current_joystick->buttons[1] == 1 )
    {
        std::cout << "Series Action\n";
        std::vector<std::string> actions = { "action1", "action2", "action3" };
        action_helper->req_series_action("testingActions", actions);
    }

    if( current_joystick->buttons[2] == 1 )
    {
        std::cout << "Series Action\n";
        std::vector<std::string> actions = { "action1", "action2", "action3" };
        action_helper->req_series_action("testingActions", actions);
    }

    if( current_joystick->buttons[3] == 1 )
    {
        std::cout << "CANCEL Action\n";
        action_helper->req_cancel_category("testingActions");
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hmi_agent_node");
	ros::NodeHandle n;
    ros::Rate rate(RATE);
	node = &n;

    ros::Subscriber joystick_status_sub
        = node->subscribe("/JoystickStatus", 100, joystick_status_callback);

    action_helper = new ActionHelper(node);

    while( ros::ok() )
    {

        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}
