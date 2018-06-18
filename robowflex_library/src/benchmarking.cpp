#include <boost/math/constants/constants.hpp>

#include <rosbag/bag.h>
#include "robowflex.h"

using namespace robowflex;

Benchmarker::Benchmarker()
{
}

void Benchmarker::addBenchmarkingRequest(const std::string &name, Scene &scene, Planner &planner,
                                         MotionRequestBuilder &request)
{
    requests_.emplace(std::piecewise_construct,     //
                      std::forward_as_tuple(name),  //
                      std::forward_as_tuple(scene, planner, request));
}

void Benchmarker::benchmark(BenchmarkOutputter &output, const Options &options)
{
    unsigned int count = 0;
    const unsigned int total = requests_.size() * options.runs;

    rosbag::Bag bag;

    if (options.trajectory_output_file != "")
    {
        bag.open(options.trajectory_output_file, rosbag::bagmode::Write);
    }
    for (const auto &request : requests_)
    {
        const auto &name = request.first;
        const auto &scene = std::get<0>(request.second);
        auto &planner = std::get<1>(request.second);
        const auto &builder = std::get<2>(request.second);
        std::vector<moveit_msgs::RobotTrajectory> trajectories;

        Results results;

        for (int j = 0; j < options.runs; ++j)
        {
            ros::WallTime start;

            start = ros::WallTime::now();
            planning_interface::MotionPlanResponse response = planner.plan(scene, builder.getRequest());
            double time = (ros::WallTime::now() - start).toSec();

            results.addRun(scene, name, time, response);
            moveit_msgs::RobotTrajectory msg;
            response.trajectory_->getRobotTrajectoryMsg(msg);
            trajectories.push_back(msg);
            ROS_INFO("BENCHMARKING: [ %u / %u ] Completed", ++count, total);
        }

        // TODO: maybe I don't need to repeat the name here? not sure.
        output.writeOutput(results, name, scene, planner, builder);

        if (options.trajectory_output_file != "")
        {
            for (moveit_msgs::RobotTrajectory traj : trajectories)
            {
                bag.write(name, ros::Time::now(), traj);
            }
        }
    }
    if (options.trajectory_output_file != "")
    {
        bag.close();
    }
    output.close();
}

//void Benchmarker::writeOutput(const std::string &file, const Results &results, const Scene &scene, const Planner &planner,
//                              const MotionRequestBuilder &builder)
//{
    // metrics["time REAL"] = boost::lexical_cast<std::string>(total_time);
    // metrics["solved BOOLEAN"] = boost::lexical_cast<std::string>(solved);

    // metrics["path_" + run.description_[j] + "_correct BOOLEAN"] = boost::lexical_cast<std::string>(correct);
    // metrics["path_" + run.description_[j] + "_length REAL"] = boost::lexical_cast<std::string>(L);
    // metrics["path_" + run.description_[j] + "_clearance REAL"] = boost::lexical_cast<std::string>(clearance);
    // metrics["path_" + run.description_[j] + "_smoothness REAL"] = boost::lexical_cast<std::string>(smoothness);
    // metrics["path_" + run.description_[j] + "_time REAL"] = boost::lexical_cast<std::string>(run.processing_time_[j]);
//}

void Benchmarker::Results::addRun(const Scene &scene, const std::string &name, double time,
                                  planning_interface::MotionPlanResponse &run)
{
    double length, clearance, smoothness;
    bool correct, success = run.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
    if (success)
        computeMetric(scene, run, correct, length, clearance, smoothness);

    runs.emplace_back(name, time, success,  //
                      correct, length, clearance, smoothness);
}

void Benchmarker::Results::computeMetric(const Scene &scene, planning_interface::MotionPlanResponse &run,  //
                                         bool &correct, double &length, double &clearance, double &smoothness)
{
    correct = true;
    length = 0.0;
    clearance = 0.0;
    smoothness = 0.0;

    const robot_trajectory::RobotTrajectory &p = *run.trajectory_;
    const planning_scene::PlanningScene &s = *scene.getSceneConst();

    // compute path length
    for (std::size_t k = 1; k < p.getWayPointCount(); ++k)
        length += p.getWayPoint(k - 1).distance(p.getWayPoint(k));

    // compute correctness and clearance
    collision_detection::CollisionRequest request;
    for (std::size_t k = 0; k < p.getWayPointCount(); ++k)
    {
        collision_detection::CollisionResult result;
        s.checkCollisionUnpadded(request, result, p.getWayPoint(k));

        if (result.collision)
            correct = false;

        if (!p.getWayPoint(k).satisfiesBounds())
            correct = false;

        double d = s.distanceToCollisionUnpadded(p.getWayPoint(k));
        if (d > 0.0)  // in case of collision, distance is negative
            clearance += d;
    }
    clearance /= (double)p.getWayPointCount();

    // compute smoothness
    if (p.getWayPointCount() > 2)
    {
        double a = p.getWayPoint(0).distance(p.getWayPoint(1));
        for (std::size_t k = 2; k < p.getWayPointCount(); ++k)
        {
            // view the path as a sequence of segments, and look at the triangles it forms:
            //          s1
            //          /\          s4
            //      a  /  \ b       |
            //        /    \        |
            //       /......\_______|
            //     s0    c   s2     s3
            //

            // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
            double b = p.getWayPoint(k - 1).distance(p.getWayPoint(k));
            double cdist = p.getWayPoint(k - 2).distance(p.getWayPoint(k));
            double acosValue = (a * a + b * b - cdist * cdist) / (2.0 * a * b);
            if (acosValue > -1.0 && acosValue < 1.0)
            {
                // the smoothness is actually the outside angle of the one we compute
                double angle = (boost::math::constants::pi<double>() - acos(acosValue));

                // and we normalize by the length of the segments
                double u = 2.0 * angle;  /// (a + b);
                smoothness += u * u;
            }
            a = b;
        }
        smoothness /= (double)p.getWayPointCount();
    }
}

void JSONBenchmarkOutputter::writeOutput(const Benchmarker::Results &results, const std::string &name, const Scene &scene, const Planner &planner, const MotionRequestBuilder &build)
{
    if (not is_init)
    {
        outfile_.open(output_path_);
        outfile_ << "{";
        // TODO: output specific information about the scene and planner structs?

        is_init = true;
    }
    else
    {
        outfile_ << ",";
    }

    outfile_ << "\"" << name << "\":["; 

    for (size_t i = 0; i < results.runs.size(); i++)
    {
        Benchmarker::Results::Run run = results.runs[i];
        outfile_ << "{";

        outfile_ << "\"name\": \"" << run.name << "\",";
        outfile_ << "\"time\":" << run.time << ",";
        outfile_ << "\"success\":" << run.success << ",";
        outfile_ << "\"correct\":" << run.correct << ",";
        outfile_ << "\"length\":" << run.length << ",";
        outfile_ << "\"clearance\":";
        // Check for infinity.
        if (run.clearance == std::numeric_limits<double>::infinity())
            outfile_ <<  std::numeric_limits<double>::max() << ",";
        else
            outfile_ << run.clearance << ",";
        outfile_ << "\"smoothness\":" << run.smoothness;

        outfile_ << "}";
        // Write the command between each run.
        if (i != results.runs.size() - 1)
            outfile_ << "," << std::endl;
    }
    outfile_ << "]";
}

void JSONBenchmarkOutputter::close()
{
    outfile_ << "}" << std::endl;
    outfile_.close();
}
