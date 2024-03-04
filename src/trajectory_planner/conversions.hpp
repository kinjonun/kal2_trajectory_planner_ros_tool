#pragma once
#include <chrono>
#include <stdexcept>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <kal2_trajectory_planner/types.hpp>



namespace kal2_trajectory_planner_ros_tool::conversions {
using namespace kal_a_star;

template <typename TimeT>
inline ros::Time toRosTime(const TimeT& from) {
    ros::Time t;
    const auto nSec = std::chrono::duration_cast<std::chrono::nanoseconds>(from.time_since_epoch()).count();
    if (nSec < 0) {
        throw std::runtime_error("Invalid time given: " + std::to_string(nSec));
    }
    t.fromNSec(nSec);
    return t;
}




inline std::vector<Position> convertToEigenVector(const std::vector<Node*>& path) {
   std::vector<Position> eigenVectors;
   eigenVectors.reserve(path.size());

   if (path.empty()) {
        std::cout << "No path found!" << std::endl;
    } else {

   for (const auto& point : path) {
       Eigen::Vector2d eigenPoint(point->x, point->y);
       eigenVectors.push_back(eigenPoint);
      }
   }
   return eigenVectors;
}

inline void vectortotrajectory(const std::vector<Position>& eigenVectors, Trajectory& trajectory) {
   Time t0 = std::chrono::steady_clock::now();
   // trajectory.clear();
   for (int i = 0; i < eigenVectors.size(); i++) {
       trajectory.push_back({eigenVectors[i], t0});
   }
   // std::cout << trajectory[0].position << std::endl;

}


inline nav_msgs::Path trajectoryToPathMsg(const Trajectory& trajectory, const std::string& frameId) {

        nav_msgs::Path pathMsg;
        pathMsg.header.frame_id = frameId;
        pathMsg.header.stamp = toRosTime(trajectory.front().stamp);

        geometry_msgs::PoseStamped stampedPoseRos;
        stampedPoseRos.header.frame_id = frameId;
        stampedPoseRos.pose.orientation.w = 1;
        stampedPoseRos.pose.orientation.x = 0;
        stampedPoseRos.pose.orientation.y = 0;
        stampedPoseRos.pose.orientation.z = 0;

        for (auto const& stampedPosition : trajectory) {
            stampedPoseRos.header.stamp = toRosTime(stampedPosition.stamp);
            stampedPoseRos.pose.position.x = stampedPosition.position.x();
            stampedPoseRos.pose.position.y = stampedPosition.position.y();

            pathMsg.poses.push_back(stampedPoseRos);
        }
        std::cout << "path publish ______________ _____-----------------------------------------------------------------" << std::endl;
        return pathMsg;


}

/**
 * @brief Create Eigen::Isometry2d from x and y translation and rotation around z axis of Eigen::Isometry3d
 *
 * @param isometry3d
 * @return Eigen::Isometry2d
 */
inline Eigen::Isometry2d isometry2dFromIsometry3d(const Eigen::Isometry3d& isometry3d) {
    Eigen::Translation2d translation(isometry3d.translation().head(2));
    Eigen::Rotation2Dd rotation(isometry3d.rotation().topLeftCorner<2, 2>());

    return translation * rotation;
}


inline std::vector<Point> depthmsgtoconesposition(const nav_msgs::Path::ConstPtr& conesdepth)
{
    std::vector<Point> conesposition;
    for(auto const& depth : conesdepth->poses){
        conesposition.push_back(Point{depth.pose.position.x,depth.pose.position.z});
    }

    return conesposition;
}


inline void print_vector(const std::vector<Node*>& path) {
    unsigned long num = path.size();
    std::vector<Point> path_vector[num-1];
    int i = 0;


    if (path.empty()) {
        std::cout << "No path found!" << std::endl;
    } else {
        for (const Node* node : path) {
        path_vector[i].push_back(Point{node->x, node->y});
        }
    }


    for (const Point& point : path_vector[i]) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    }
    std::cout << std::endl;


}





inline double calculateDistance(const Point& point) {
    return std::sqrt(point.x * point.x + point.y * point.y);
}

inline double findClosestDistance(const std::vector<Point>& points) {
    double closestDistance = std::numeric_limits<double>::max();

    for (const Point& point : points) {
        double distance = calculateDistance(point);
        if (distance < closestDistance) {
            closestDistance = distance;
        }
    }

    return closestDistance;
}









} // namespace kal_trajectory_planner_ros_tool::conversions