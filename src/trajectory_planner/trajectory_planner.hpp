#pragma once

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <kal2_trajectory_planner/a_star.hpp>
#include <kal2_trajectory_planner/types.hpp>
#include <kal2_trajectory_planner_ros_tool/TrajectoryPlannerInterface.h>



namespace kal2_trajectory_planner_ros_tool {

using namespace kal_a_star;
// std::vector<Point> linkcones;
// std::vector<Point> rightcones;


class TrajectoryPlanner_Node {
public:

    using Interface = TrajectoryPlannerInterface;
    explicit TrajectoryPlanner_Node(const ros::NodeHandle& nhPrivate);

private:

    A_star a_star_;
    Interface interface_;

    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;
    ros::Timer pathGenerationTimer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};


    void reconfigureCallback(const Interface::Config& config, uint32_t /*level*/);

    void conesCallback(const nav_msgs::Path::ConstPtr& cones);

    void pathCallback(const ros::TimerEvent&);

    bool currentVehiclePose(Pose& pose) const;
    bool conesInitialized_{false};

    void vehicletomap( std::vector<Point>& cones,  std::vector<Point>& mapposition);





};
} // namespace kal_trajectory_planner_ros_tool
