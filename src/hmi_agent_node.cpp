#include "hmi_agent_node.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <rio_control_node/Joystick_Status.h>
#include <rio_control_node/Robot_Status.h>
#include <hmi_agent_node/HMI_Signals.h>
#include <action_helper/action_helper.hpp>
#include <ck_utilities/Joystick.hpp>
#include "ck_utilities/CKMath.hpp"
#include <atomic>
#define RATE (100)

ros::NodeHandle *node;
//ActionHelper *action_helper;
Joystick *drive_joystick;
Joystick *arm_joystick;
Joystick *button_box_1_joystick;
Joystick *button_box_2_joystick;
std::map<int, uint8_t> button_clicks;

enum RobotState : int
{
    DISABLED = 0,
    TELEOP = 1,
    AUTONOMOUS = 2,
    TEST = 3,
};
std::atomic<RobotState> robot_state {DISABLED};

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


    output_signals.turret_manual = arm_joystick->getButton(5);
    static constexpr double MAX_TURRET_DEG = 180;
    static constexpr double MIN_TURRET_DEG = -180;
    static constexpr double MAX_HOOD_DEG = 25;
    static constexpr double MIN_HOOD_DEG = 0;
    static constexpr double MAX_SHOOTER_RPM = 5000;
    static constexpr double MIN_SHOOTER_RPM = 0;

    if (output_signals.turret_manual)
    {
        if (arm_joystick->getFilteredAxis(2, 0.25) < 0) //aim left
        {
            turret_aim_degrees += 2;
        }
        else if (arm_joystick->getFilteredAxis(2, 0.25) > 0) //aim right
        {
            turret_aim_degrees -= 2;
        }
        
        turret_aim_degrees = std::min(std::max(turret_aim_degrees, MIN_TURRET_DEG), MAX_TURRET_DEG);



        //fix
        turret_aim_degrees = arm_joystick->getFilteredAxis(2, 0.25) / 4.0;
    }


    if (arm_joystick->getButton(8)) //hood up
    {
        turret_hood_degrees -= 0.2;
    }
    if (arm_joystick->getButton(9)) //hood down
    {
        turret_hood_degrees += 0.2;
    }
    turret_hood_degrees = std::min(std::max(turret_hood_degrees, MIN_HOOD_DEG), MAX_HOOD_DEG);


    if (arm_joystick->getButton(10)) //speed up
    {
        turret_speed_rpm += 60;
    }
    if (arm_joystick->getButton(11)) //speed down
    {
        turret_speed_rpm -= 60;
    }
    turret_speed_rpm = std::min(std::max(turret_speed_rpm, MIN_SHOOTER_RPM), MAX_SHOOTER_RPM);

   
    
    output_signals.flip_intakes = drive_joystick->getButton(0);
    output_signals.drivetrain_brake = drive_joystick->getButton(7);
    output_signals.drivetrain_fwd_back = -drive_joystick->getFilteredAxis(1, 0.12);
    double turn = drive_joystick->getFilteredAxis(0, 0.12);
    output_signals.drivetrain_left_right = ck::math::signum(turn) * std::pow(turn, 2);
    if (drive_joystick->getAxisActuated(3, 0.35))
    {
        output_signals.drivetrain_left_right *= 0.4;
        output_signals.drivetrain_fwd_back *= 0.4;
    }
    // output_signals.drivetrain_quickturn = drive_joystick->getAxisActuated(2, 0.35);
    output_signals.drivetrain_quickturn = drive_joystick->getButton(6);
    output_signals.turret_aim_degrees = turret_aim_degrees;
    output_signals.turret_hood_degrees = turret_hood_degrees;
    output_signals.turret_speed_rpm = turret_speed_rpm;
    output_signals.intake_rollers = button_box_2_joystick->getButton(2) || drive_joystick->getButton(5) || arm_joystick->getButton(0);
    output_signals.retract_intake = button_box_2_joystick->getButton(3) || output_signals.intake_rollers;
    output_signals.manual_intake = button_box_2_joystick->getButton(4) || arm_joystick->getButton(6);
    output_signals.manual_outake = button_box_2_joystick->getButton(5) || arm_joystick->getButton(2);
    output_signals.stop_climber = button_box_2_joystick->getButton(7);
    output_signals.allow_shoot = button_box_1_joystick->getButton(0) || drive_joystick->getButton(3);
    output_signals.deploy_hooks = button_box_1_joystick->getButton(5);
    output_signals.begin_climb = button_box_1_joystick->getButton(4);
    output_signals.retract_hooks = button_box_1_joystick->getButton(11);
    output_signals.forced_reset_retract_hooks = false;  //DO NOT SET THIS SIGNAL HERE TO ANYTHING OTHER THAN FALSE

    static ros::Publisher signal_publisher = node->advertise<hmi_agent_node::HMI_Signals>("/HMISignals", 10);

    if (robot_state != RobotState::AUTONOMOUS)
    {
        signal_publisher.publish(output_signals);
    }
}

void robot_status_callback(const rio_control_node::Robot_Status &robot_status)
{
    robot_state = (RobotState)robot_status.robot_state;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hmi_agent_node");
    ros::NodeHandle n;
    ros::Rate rate(RATE);
    node = &n;

    ros::Subscriber joystick_status_sub = node->subscribe("/JoystickStatus", 100, joystick_status_callback);
    ros::Subscriber robot_status_sub = node->subscribe("/RobotStatus", 1, robot_status_callback);
    drive_joystick = new Joystick(0);
    arm_joystick = new Joystick(1);
    button_box_1_joystick = new Joystick(2);
    button_box_2_joystick = new Joystick(3);
    //action_helper = new ActionHelper(node);

    ros::spin();

    return 0;
}
