/* Author: Zachary Kingston */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/panda.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file fetch_test.cpp
 * A simple script that demonstrates motion planning with the PANDA robot. Here,
 * two planners are created: the default OMPL planner, and the default OMPL
 * planner but with simplified solutions disabled.
 */

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default PANDA robot.
    auto panda = std::make_shared<PANDARobot>();
    panda->initialize();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(panda);

    // Create the default planner for the PANDA.
    auto default_planner = std::make_shared<OMPL::PANDAOMPLPipelinePlanner>(panda, "default");
    default_planner->initialize();

    // Create the a planner for the PANDA, and disable simplification.
    auto simple_planner = std::make_shared<OMPL::PANDAOMPLPipelinePlanner>(panda, "simple");

    OMPL::Settings settings;
    settings.simplify_solutions = false;

    simple_planner->initialize(settings);

    // Run a motion plan for each planner.
    for (const auto &planner : {default_planner, simple_planner})
    {
        // Create a motion planning request with a pose goal.
        MotionRequestBuilder request(planner, "panda_arm");
        request.setStartConfiguration({0.0, -0.785, 0, -2.356, 0.0, 1.571, 0.785});

        RobotPose pose = RobotPose::Identity();
        pose.translate(Eigen::Vector3d{-0.268, -0.5, 0.5});
        Eigen::Quaterniond orn{0, 0, 1, 0};

        request.setGoalRegion("panda_link8", "world",           // links
                              pose, Geometry::makeSphere(0.1),  // position
                              orn, {0.01, 0.01, 0.01}           // orientation
        );

        // Do motion planning!
        planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            return 1;
    }

    return 0;
}
