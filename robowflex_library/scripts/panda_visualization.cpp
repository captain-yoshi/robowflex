/* Author: Zachary Kingston */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/panda.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file panda_visualization.cpp
 * A simple script that demonstrates how to use RViz with Robowflex with the
 * PANDA robot. See https://kavrakilab.github.io/robowflex/rviz.html for how to
 * use RViz visualization. Here, the scene, the pose goal, and motion plan
 * displayed in RViz.
 */

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default PANDA robot.
    auto panda = std::make_shared<PANDARobot>();
    panda->initialize();

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(panda);

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(panda);

    // Visualize the scene.
    rviz.updateScene(scene);

    // Create the default planner for the PANDA.
    auto planner = std::make_shared<OMPL::PANDAOMPLPipelinePlanner>(panda);
    planner->initialize();

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, "panda_arm");
    request.setStartConfiguration({0.0, -0.785, 0, -2.356, 0.0, 1.571, 0.785});

    RobotPose pose = RobotPose::Identity();
    pose.translate(Eigen::Vector3d{-0.268, -0.5, 0.5});
    Eigen::Quaterniond orn{0, 0, 1, 0};

    auto region = Geometry::makeSphere(0.1);

    request.setGoalRegion("panda_link8", "panda_link0",  // links
                          pose, region,                  // position
                          orn, {0.1, 0.1, 0.1}           // orientation
    );

    rviz.addGoalMarker("goal", request);
    rviz.updateMarkers();

    RBX_INFO("Scene and Goal displayed! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res);

    RBX_INFO("Press enter to remove goal and scene.");
    std::cin.get();

    rviz.removeMarker("goal");
    rviz.updateMarkers();

    rviz.removeScene();

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
