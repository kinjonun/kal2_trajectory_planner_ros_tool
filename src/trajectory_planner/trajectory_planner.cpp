#include "trajectory_planner.hpp"
#include <stdexcept>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include "conversions.hpp"

namespace kal2_trajectory_planner_ros_tool{

/**
 * Initialization
 */
  TrajectoryPlanner_Node::TrajectoryPlanner_Node(const ros::NodeHandle& nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    interface_.fromParamServer();
    reconfigureServer_.setCallback(boost::bind(&TrajectoryPlanner_Node::reconfigureCallback, this, _1, _2));

    interface_.cones_subscriber->registerCallback(&TrajectoryPlanner_Node::conesCallback, this);

    pathGenerationTimer_ =
        nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &TrajectoryPlanner_Node::pathCallback, this);


    interface_.showNodeInfo();
    interface_.logDebug("Node initialized.");


    }

void TrajectoryPlanner_Node::conesCallback(const nav_msgs::Path::ConstPtr& cones){
    std::vector<Point> conesposition = conversions::depthmsgtoconesposition(cones);

    a_star_.cones(conesposition);       //unterscheiden link recht
    conesInitialized_ = true;

}

void TrajectoryPlanner_Node::reconfigureCallback(const Interface::Config& config, uint32_t /*level*/) {
    interface_.fromConfig(config);
    // trajectoryPlanner_.setPolynomialDegree(interface_.polynomial_degree);
  }


void TrajectoryPlanner_Node::pathCallback(const ros::TimerEvent& /*timer*/) {
    if (!conesInitialized_) {
        interface_.logWarn("Cannot compute trajectory because no cones has been set yet.");
        return;
    }

    // Node startpoint(startpoint.x,startpoint.y );
    // Node targetpoint(targetpoint.x,targetpoint.y);
    // std::vector<Node*> Node_ = a_star_.aStarSearch(startpoint, targetpoint, linkcones, rightcones);


    // Pose vehiclePose;
    // if (!currentVehiclePose(vehiclePose)) {
    //     interface_.logWarn("Cannot compute trajectory because vehicle pose was not found in tf tree.");
    //     return;
    // }


    Pose transform;       //    currentPosition
    transform.translation() = Eigen::Vector2d(0.0, 0.0); // 设置平移向量 (1.0, 2.0)
    const std::vector<Point>& points = a_star_.getConesPosition();
    int num = points.size();
    double distancetocones = conversions::findClosestDistance(points);

    // if (distancetocones < 0.1 && num >1 ){
    //     std::vector<Node*> Node_ = a_star_.aStar(transform);
    //     std::vector<Position> eigenVectors = conversions::convertToEigenVector(Node_);
    //     Trajectory trajectory;
    //     conversions::vectortotrajectory(eigenVectors, trajectory);
    //     interface_.path_publisher.publish(conversions::trajectoryToPathMsg(trajectory, interface_.map_frame));
    // }else{

    // }


    std::vector<Node*> Node_ = a_star_.aStar(transform);
    conversions::print_vector(Node_);

    std::vector<Position> eigenVectors = conversions::convertToEigenVector(Node_);
    Trajectory trajectory;
    conversions::vectortotrajectory(eigenVectors, trajectory);
    interface_.path_publisher.publish(conversions::trajectoryToPathMsg(trajectory, interface_.map_frame));
}




bool TrajectoryPlanner_Node::currentVehiclePose(Pose& pose) const {

    // Find vehicle position in tf tree
    geometry_msgs::TransformStamped vehiclePoseRos;
    Eigen::Isometry3d vehiclePose3d;
    try {
        vehiclePoseRos = tfBuffer_.lookupTransform(interface_.map_frame, interface_.vehicle_frame, ros::Time(0));
    } catch (const tf2::TransformException& e) {
        return false;
    }

    // Transform to Eigen type
    vehiclePose3d = tf2::transformToEigen(vehiclePoseRos);

    // Convert to two dimensions
    pose = conversions::isometry2dFromIsometry3d(vehiclePose3d);

    return true;
}

}