/* Author: Zachary Kingston */

#include <robowflex_library/detail/panda.h>
#include <robowflex_library/io.h>
#include <robowflex_library/log.h>

using namespace robowflex;

const std::string  //
    PANDARobot::DEFAULT_URDF{"package://franka_description/robots/panda_arm_hand.urdf.xacro"};
const std::string  //
    PANDARobot::DEFAULT_SRDF{"package://panda_moveit_config/config/panda_arm_hand.xacro.srdf"};
const std::string  //
    PANDARobot::DEFAULT_LIMITS{"package://panda_moveit_config/config/joint_limits.yaml"};
const std::string  //
    PANDARobot::DEFAULT_KINEMATICS{"package://panda_moveit_config/config/kinematics.yaml"};
const std::string  //
    OMPL::PANDAOMPLPipelinePlanner::DEFAULT_CONFIG{
        "package://panda_moveit_config/config/ompl_planning.yaml"  //
    };

const std::string  //
    PANDARobot::RESOURCE_URDF{"package://robowflex_resources/panda/urdf/panda.urdf"};
const std::string  //
    PANDARobot::RESOURCE_SRDF{"package://robowflex_resources/panda/config/panda.srdf"};
const std::string  //
    PANDARobot::RESOURCE_LIMITS{"package://robowflex_resources/panda/config/joint_limits.yaml"};
const std::string  //
    PANDARobot::RESOURCE_KINEMATICS{"package://robowflex_resources/panda/config/kinematics.yaml"};
const std::string  //
    OMPL::PANDAOMPLPipelinePlanner::RESOURCE_CONFIG{
        "package://robowflex_resources/panda/config/ompl_planning.yaml"  //
    };

PANDARobot::PANDARobot() : Robot("panda")
{
}

bool PANDARobot::initialize()
{
    bool success = false;

    // First attempt the `robowflex_resources` package, then attempt the "actual" resource files.
    if (IO::resolvePackage(RESOURCE_URDF).empty() or IO::resolvePackage(RESOURCE_SRDF).empty())
    {
        RBX_INFO("Initializing PANDA with `franka_description`");
        success = Robot::initialize(DEFAULT_URDF, DEFAULT_SRDF, DEFAULT_LIMITS, DEFAULT_KINEMATICS);
    }
    else
    {
        RBX_INFO("Initializing PANDA with `robowflex_resources`");
        success = Robot::initialize(RESOURCE_URDF, RESOURCE_SRDF, RESOURCE_LIMITS, RESOURCE_KINEMATICS);
    }

    loadKinematics("panda_arm");

    return success;
}

OMPL::PANDAOMPLPipelinePlanner::PANDAOMPLPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : OMPLPipelinePlanner(robot, name)
{
}

bool OMPL::PANDAOMPLPipelinePlanner::initialize(const Settings &settings,
                                                const std::vector<std::string> &adapters)
{
    if (IO::resolvePackage(RESOURCE_CONFIG).empty())
        return OMPLPipelinePlanner::initialize(DEFAULT_CONFIG, settings, DEFAULT_PLUGIN, adapters);
    return OMPLPipelinePlanner::initialize(RESOURCE_CONFIG, settings, DEFAULT_PLUGIN, adapters);
}
