// Each ROS nodelet must have these.
#include <nodelet/nodelet.h>
// #include <ros/rospackage.h>
#include <ros/ros.h>

// Every nodelet must include machs which export the class as a nodelet plugin.
#include <pluginlib/class_list_macros.h>

// ROS includes for localization.
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <map_server/image_loader.h>
#include <nav_msgs/GetMap.h>

#include <vector>

#include "Data.h"
#include "Utils.h"
#include "MCL.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "robotics_mcl");
    ros::NodeHandle nh;
    robotics_mcl::Data d(nh);

	ros::Rate rate(10.0); // ROS Rate at 10Hz.
    // ros::spinOnce();
    // rate.sleep();
    // auto belief = d.createParticles(2000);
    // std::cout << d.getSensorPose() << " " << d.getPose() << std::endl;
    // robotics_mcl::MCL mcl(&d, d.getPose(), {0.2, 0.2, 0.2, 0.2}, {0.2, 0.2, 0.2}, 0.2, 10);
    // d.plot(belief, step++);
    // int step = 0;
    // const int maxSteps = 100;
	while (nh.ok()) {
        std::cout << "Estimated pose:" << d.getPose() << std::endl;
        
        // if(mcl.run(belief)) {
        //     belief = mcl.resample();
        //     std::cout << "updated" << std::endl;
        //     d.plot(belief, step++);
        // } else {
        //     std::cout << "it is not updated" << std::endl;
        // }

        ros::spinOnce();
        rate.sleep();
	}

    // ros::spin(); // Enters a loop, calling message callbacks as fast as possible.
}

// PLUGINLIB_EXPORT_CLASS(localization::EKF_MCL_Global, nodelet::Nodelet)