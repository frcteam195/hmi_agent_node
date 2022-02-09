#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <rio_control_node/Joystick_Status.h>
#include <hmi_agent_node/HMI_Signals.h>
#include <action_helper/action_helper.hpp>

#define RATE (100)

ros::NodeHandle *node;
ActionHelper *action_helper;

std::map<int, uint8_t> button_clicks;

uint8_t debounce(int index, uint8_t value)
{

    if (button_clicks.count(index) == 0)
    {
        button_clicks[index] = value;
        return value;
    }

    if (button_clicks[index] != value)
    {
        button_clicks[index] = value;
        return value;
    }

    return 0;
}

void joystick_status_callback(const rio_control_node::Joystick_Status &joystick_status)
{
    static double turret_aim_degrees = 0;
    static double turret_hood_degrees = 0;
    static double turret_speed_rpm = 0;
    hmi_agent_node::HMI_Signals output_signals;

    if (joystick_status.joysticks[3].buttons[8]) //near
    {
        turret_hood_degrees = 0;
        turret_speed_rpm = 1800;
    }
    if (joystick_status.joysticks[3].buttons[9]) //near mid
    {
        turret_hood_degrees = -6;
        turret_speed_rpm = 2100;
    }
    if (joystick_status.joysticks[3].buttons[10]) //mid
    {
        turret_hood_degrees = -12;
        turret_speed_rpm = 2400;
    }
    if (joystick_status.joysticks[3].buttons[11]) //mid far
    {
        turret_hood_degrees = -18;
        turret_speed_rpm = 2700;
    }
    if (joystick_status.joysticks[3].buttons[12]) //far
    {
        turret_hood_degrees = -24;
        turret_speed_rpm = 3000;
    }

    if (joystick_status.joysticks[1].buttons[1])
    {
        if (joystick_status.joysticks[1].axes[2] < -0.25) //aim left
        {
            turret_aim_degrees += 2;
        }
        if (joystick_status.joysticks[1].axes[2] > 0.25) //aim right
        {
            turret_aim_degrees -= 2;
        }
        if (turret_aim_degrees > 180)
        {
            turret_aim_degrees = 180;
        }
        if (turret_aim_degrees < -180)
        {
            turret_aim_degrees = -180;
        }
    }

    if (joystick_status.joysticks[1].buttons[5]) //hood up
    {
        turret_hood_degrees -= 0.2;
    }
    if (joystick_status.joysticks[1].buttons[3]) //hood down
    {
        turret_hood_degrees += 0.2;
    }
    if (turret_hood_degrees > 0)
    {
        turret_hood_degrees = 0;
    }
    if (turret_hood_degrees < -25)
    {
        turret_hood_degrees = -25;
    }

    if (joystick_status.joysticks[1].buttons[4]) //speed up
    {
        turret_speed_rpm += 60;
    }
    if (joystick_status.joysticks[1].buttons[2]) //speed down
    {
        turret_speed_rpm -= 60;
    }
    if (turret_speed_rpm > 5000)
    {
        turret_speed_rpm = 5000;
    }
    if (turret_speed_rpm < 0)
    {
        turret_speed_rpm = 0;
    }

    output_signals.drivetrain_brake = joystick_status.joysticks[0].buttons[4];
    output_signals.drivetrain_fwd_back = joystick_status.joysticks[0].axes[1];
    output_signals.drivetrain_left_right = joystick_status.joysticks[0].axes[4];
    output_signals.drivetrain_quickturn = joystick_status.joysticks[0].axes[2] > .35;
    output_signals.turret_aim_degrees = turret_aim_degrees;
    output_signals.turret_hood_degrees = turret_hood_degrees;
    output_signals.turret_speed_rpm = turret_speed_rpm;

    static ros::Publisher signal_publisher = node->advertise<hmi_agent_node::HMI_Signals>("/HMISignals", 10);
    signal_publisher.publish(output_signals);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hmi_agent_node");
    ros::NodeHandle n;
    ros::Rate rate(RATE);
    node = &n;

    ros::Subscriber joystick_status_sub = node->subscribe("/JoystickStatus", 100, joystick_status_callback);

    action_helper = new ActionHelper(node);

    ros::spin();

    return 0;
}
