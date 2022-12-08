#ifndef ROBOT_H
#define ROBOT_H

// Each ROS nodelet must have these.
#include <nodelet/nodelet.h>
// #include <ros/rospackage.h>
#include <ros/ros.h>

// Every nodelet must include machs which export the class as a nodelet plugin.
#include <pluginlib/class_list_macros.h>

// ROS includes for localization.
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <map_server/image_loader.h>
#include <visualization_msgs/Marker.h>

#include <vector>
#include <utility>
#include <ctime>
#include <cstdlib>

// #define CVPLOT_HEADER_ONLY

// #include "CvPlot/cvplot.h"
// #include <opencv2/opencv.hpp>

#include "matplotlibcpp.h"

#include "Utils.h"

namespace robotics_mcl {

// #define PI 3.141592

class Data {
public:
    Data(ros::NodeHandle& nh);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void mapCallback(const nav_msgs::OccupancyGrid& res);
    void cloudCallback(const geometry_msgs::PoseArray::ConstPtr& cloud);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
    
    const auto& getSensorPose() { return sensorPose_m; }
    Coord getPose() { return pose_m; }
    const auto& getScan() { return scan_m; }
    const auto& getScanRanges() { return scan_m.ranges; }
    const auto& getMap() { return map_m; }

    Coord compPose(const geometry_msgs::Pose& p) {
        tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
        tf::Matrix3x3 mat(q);
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);

        double x = p.position.x * 1.0 / map_m.resolution;
        double y = p.position.y * 1.0 / map_m.resolution;
        return robotics_mcl::Coord(x, y, yaw);
    }

    std::vector<Coord> createParticles(int num);

    void plot(const std::vector<Coord>& particles, int step);
    // void drawRviz();

private:
    ros::Subscriber odomSub_m;
    ros::Subscriber laserSub_m;
    ros::Subscriber mapSub_m;
    ros::Subscriber partCloudSub_m;
    ros::Subscriber poseSub_m;
    ros::Publisher markerPub_m;
    Coord sensorPose_m;
    Coord pose_m;
    Scan scan_m;
    Map map_m;
    std::vector<Coord> particles_m;
};

}

#endif