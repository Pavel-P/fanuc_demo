// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>

//subscriber msgs
#include <std_msgs/String.h>

#include <stdio.h>      // stdin, stderr 
#include <stddef.h>     // NULL, sizeof, size_t 
#include <stdlib.h>     // atoi, atof 
#include <ctype.h>      // isspace
#include <math.h>
#include <unistd.h>     //usleep
#include <iostream>      //cout


typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

    trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory,
        const descartes_core::RobotModel& model,
        const std::vector<std::string>& joint_names,
        const std::vector<double>& time_delay)
{
    // Fill out information about our trajectory
    trajectory_msgs::JointTrajectory result;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = "world_frame";
    result.joint_names = joint_names;

    // For keeping track of time-so-far in the trajectory
    double time_offset = 0.0;
    // Loop through the trajectory
    int i = 0;
    for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it, ++i)
    {
        // Find nominal joint solution at this point
        std::vector<double> joints;
        it->get()->getNominalJointPose(std::vector<double>(), model, joints);

        // Fill out a ROS trajectory point
        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = joints;
        // velocity, acceleration, and effort are given dummy values
        // we'll let the controller figure them out
        pt.velocities.resize(joints.size(), 0.0);
        pt.accelerations.resize(joints.size(), 0.0);
        pt.effort.resize(joints.size(), 0.0);
        // set the time into the trajectory
        pt.time_from_start = ros::Duration(time_offset);
        // increment time
        time_offset += time_delay[i];
        ROS_WARN("time: %f\n", time_delay[i]);

        result.points.push_back(pt);
    }

    return result;
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
    // Create a Follow Joint Trajectory Action Client
    //  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_controller/follow_joint_trajectory", true);
    if (!ac.waitForServer(ros::Duration(2.0)))
    {
        ROS_ERROR("Could not connect to action server");
        return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance = ros::Duration(1.0);

    ac.sendGoal(goal);

    if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
    {
        ROS_INFO("Action server reported successful execution");
        return true;
    } else {
        ROS_WARN("Action server could not execute trajectory");
        return false;
    }
}

int main (int argc, char **argv)
{
    /* Descartes Model Initialization */

    const std::string robot_description = argv[1];
    const std::string group_name = argv[2];
    const std::string world_frame = argv[3];
    const std::string tcp_frame = argv[4];

    ros::init(argc, argv, "simple_message_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);
    if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
        ROS_INFO("Could not initialize robot model");
        exit(-1);
    }
    model->setCheckCollisions(true);
    ROS_INFO("Robot initialized successfully!");

    //descartes_planner::DensePlanner planner;
    descartes_planner::SparsePlanner planner;
    planner.initialize(model);

    // Get Joint Names
    std::vector<std::string> j_names;
    nh.getParam("controller_joint_names", j_names);

    TrajectoryVec points;
    TrajectoryVec results;
    std::vector<double> times;

    bool successful_exection = false;
    while (true)
    {
        char inbuf[1024];
        float f1, f2, f3, f4, f5, f6, f7;
        if (NULL == fgets(inbuf, sizeof(inbuf), stdin)) break;
        // skip leading whitespace
        char *ptr = inbuf;
        while (isspace(*ptr)) ptr++;

        if ('q' == *ptr)
        {
            break;
        }
        else if (7 == sscanf(ptr, "%f %f %f %f %f %f %f",
                    &f1, &f2, &f3, &f4, &f5, &f6, &f7))
        {
            using namespace descartes_core;
            using namespace descartes_trajectory;
            if (points.empty())
            {   
                //dummy point
                std::vector<double> joints;
                for (int q = 0; q < 6; q++) joints.push_back(0);
                TrajectoryPtPtr dummy_pt = TrajectoryPtPtr(new JointTrajectoryPt(joints));
                points.push_back(dummy_pt);
            }
            TrajectoryPtPtr next_point = TrajectoryPtPtr(new AxialSymmetricPt( f1, f2, f3, f4, f5, f6, M_PI/2.0, AxialSymmetricPt::Z_AXIS));
            points.push_back(next_point);
            times.push_back(f7);
            using namespace descartes_core;
            if (points.size() > 1 )
            {
                times.push_back(1.0);
                if (planner.planPath(points) && planner.getPath(results))
                {
                    trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(results, *model, j_names, times);
                    successful_exection = executeTrajectory(joint_solution);
                }
                points.clear();
                times.clear();
                if (successful_exection)
                {
                    points.push_back(results.back());
                }
                results.clear();
            }
        }
    }
}
