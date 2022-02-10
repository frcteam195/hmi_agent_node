#include "hmi_agent_node.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <rio_control_node/Joystick_Status.h>
#include <hmi_agent_node/HMI_Signals.h>
#include <action_helper/action_helper.hpp>
#include <ck_utilities/Joystick.hpp>
#define RATE (100)

ros::NodeHandle *node;
ActionHelper *action_helper;
Joystick *drive_joystick;
Joystick *arm_joystick;
Joystick *button_box_1_joystick;
Joystick *button_box_2_joystick;
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
    Joystick::update(joystick_status);
    static double turret_aim_degrees = 0;
    static double turret_hood_degrees = 0;
    static double turret_speed_rpm = 0;
    hmi_agent_node::HMI_Signals output_signals;

    if (button_box_2_joystick->getButton(8)) //near
    {
        turret_hood_degrees = 0;
        turret_speed_rpm = 1800;
    }
    if (button_box_2_joystick->getButton(9)) //near mid
    {
        turret_hood_degrees = -6;
        turret_speed_rpm = 2100;
    }
    if (button_box_2_joystick->getButton(10)) //mid
    {
        turret_hood_degrees = -12;
        turret_speed_rpm = 2400;
    }
    if (button_box_2_joystick->getButton(11)) //mid far
    {
        turret_hood_degrees = -18;
        turret_speed_rpm = 2700;
    }
    if (button_box_2_joystick->getButton(12)) //far
    {
        turret_hood_degrees = -24;
        turret_speed_rpm = 3000;
    }

    if (arm_joystick->getButton(1))
    {
        if (arm_joystick->getFilteredAxis(2, 0.25) < 0) //aim left
        {
            turret_aim_degrees += 2;
        }
        else if (arm_joystick->getFilteredAxis(2, 0.25) > 0) //aim right
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

    if (arm_joystick->getButton(5)) //hood up
    {
        turret_hood_degrees -= 0.2;
    }
    if (arm_joystick->getButton(3)) //hood down
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

    if (arm_joystick->getButton(4)) //speed up
    {
        turret_speed_rpm += 60;
    }
    if (arm_joystick->getButton(2)) //speed down
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

   
    

    output_signals.drivetrain_brake = drive_joystick->getButton(4);
    output_signals.drivetrain_fwd_back = drive_joystick->getFilteredAxis(1, 0.05);
    output_signals.drivetrain_left_right = drive_joystick->getFilteredAxis(4, 0.05);
    output_signals.drivetrain_quickturn = drive_joystick->getAxisActuated(2, 0.35);
    output_signals.turret_aim_degrees = turret_aim_degrees;
    output_signals.turret_hood_degrees = turret_hood_degrees;
    output_signals.turret_speed_rpm = turret_speed_rpm;
    output_signals.shoot_turret = arm_joystick->getButton(0);



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
    drive_joystick = new Joystick(0);
    arm_joystick = new Joystick(1);
    button_box_1_joystick = new Joystick(2);
    button_box_2_joystick = new Joystick(3);
    action_helper = new ActionHelper(node);

    ros::spin();

    return 0;
}
