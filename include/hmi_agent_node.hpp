#pragma once
#include <map>
#include <string>

enum class SubsystemCategories
{
    INVALID,
    TURRET,
    DRIVEBASE,
    CLIMBER,
    INTAKE
};

std::map<SubsystemCategories, std::string> categories = {
    {SubsystemCategories::TURRET, "Turret"},
    {SubsystemCategories::DRIVEBASE, "DriveBase"},
    {SubsystemCategories::CLIMBER, "Climber"},
    {SubsystemCategories::INTAKE, "Intake"},
};

enum class TurretActions
{
    SHOOT_TURRET,
    DEACTIVATE_AUTOMATIC,
    ACTIVATE_AUTOMATIC
};

enum class DrivebaseActions
{
    DEACTIVATE_AUTOMATIC,
    RUN_TRAJECTORY_NAME
};

enum class IntakeActions
{
    INTAKE_BALL,
    REJECT_BALL,
    DEACTIVATE_AUTOMATIC,
    ACTIVATE_AUTOMATIC
};

enum class ClimbingActions
{
    EMERGENCY_STOP,
    DEPLOY_HOOKS,
    RETRACT_HOOKS,
    ACTIVATE_AUTO_CLIMBING
};