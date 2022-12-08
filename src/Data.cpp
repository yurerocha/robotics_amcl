#include "Data.h"

namespace robotics_mcl {

Data::Data(ros::NodeHandle& nh) {
    srand(time(NULL));

    odomSub_m = nh.subscribe("/odom", 1000, &Data::odomCallback, this); // The second arg is the queue size.
    // ros::Subscriber velSub = nh.subscribe("/mobile_base/commands/velocity", 1000, velCallback, this); // The second arg is the queue size.
    laserSub_m = nh.subscribe("/my_robot/rplidar/laser/scan", 1000, &Data::scanCallback, this);
    mapSub_m = nh.subscribe("/map", 1000, &Data::mapCallback, this);
    partCloudSub_m = nh.subscribe("/particlecloud", 1000, &Data::cloudCallback, this);
    partCloudSub_m = nh.subscribe("/amcl_pose", 1000, &Data::poseCallback, this);
    markerPub_m = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10, this);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    while(true) {
        try{
            listener.waitForTransform("carcaca","rplidar",  
                                    ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("carcaca","rplidar",  
                                    ros::Time(0), transform);
            // std::cout << listener.allFramesAsString() << std::endl;
            break;
        } catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            ROS_ERROR("%s", listener.allFramesAsString().c_str());
            ros::Duration(1.0).sleep();
        }
    }

    // Get Euler rotation angles.
    double yaw, pitch, roll;
    transform.getBasis().getEulerYPR(yaw, pitch, roll);

    // Configure the laser-range-finder.
    sensorPose_m = Coord(
        transform.getOrigin().x(),
        transform.getOrigin().y(),
        yaw
    );

    ROS_INFO_ONCE("MCL initialized!");
}

void Data::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // ROS_INFO("Seq: [%d]", msg->header.seq);
    // ROS_INFO("Odom position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    // Transform quaterniom to Euler.
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 mat(q);
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);
    // pose_m = robotics_mcl::Coord(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
}

// void velCallback(const geometry_msgs::Twist& msg) {
//     if(msg.angular.z > 0){
//         ROS_INFO("Subscriber velocities: linear=%f angular=%f", msg.linear.x, msg.angular.z);
//     }
// }

// Todo: scan_m com tamanho fixo.
void Data::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int count = msg->ranges.size();
    // int count =* PI / msg->angle_increment;
    // ROS_INFO("Scan count: %d", count);
    if(count > 0) {
        // ROS_INFO("Laser scan %s[%d]:", msg->header.frame_id.c_str(), count);
        // ROS_INFO("angle_range, %f, %f", msg->angle_min, msg->angle_max);
        // scan = std::vector<double>(count);
        // ROS_INFO("scan begin: %d %d", scan_m.len, count);
        int ri = 0;
        for(int i = 0; i < count; ++i) {
            double d = msg->ranges[i];
            // if(ri < scan_m.maxLen && d >= scan_m.minRange && d <= scan_m.maxRange) {
            //     scan_m.ranges[ri++] = d;
            // Only/=scans considered.
            if(!(i % 45) && d >= scan_m.minRange && d <= scan_m.maxRange) { 
                scan_m.ranges[ri++] = d;
            }
        }
        // ROS_INFO("scan end: %d", (int)scan_m.ranges.size());
    }
}

void Data::mapCallback(const nav_msgs::OccupancyGrid& res) {
    int w = map_m.width > 0 ? map_m.width : res.info.width;
    int h = map_m.height > 0 ? map_m.height : res.info.height;
    map_m.xmin = w;
    map_m.xmax = 0;
    map_m.ymin = h;
    map_m.ymax = 0;
    ROS_INFO("Map info: width=%d heigth=%d", w, h);
    // map_m = std::vector<std::pair<int, int>>();
    for(int i = 0; i < w; ++i) {
        for(int j = 0; j < h; ++j) {
            double d = res.data[i * w + j];
            if(d > 0) {
                // map_m.push_back(std::make_pair(j, i));
                map_m.xmin = std::min(map_m.xmin, j);
                map_m.xmax = std::max(map_m.xmax, j);
                map_m.ymin = std::min(map_m.ymin, i);
                map_m.ymax = std::max(map_m.ymax, i);
                map_m.x.push_back(j);
                map_m.y.push_back(i);
            }
        }
    }
    map_m.width = w;
    map_m.height = h;
    map_m.resolution = res.info.resolution;
}

void Data::cloudCallback(const geometry_msgs::PoseArray::ConstPtr& cloud) {
    particles_m = {};
    ROS_INFO("Particles %d", (int)particles_m.size());
    for(auto i = 0; i < cloud->poses.size(); ++i) {
        auto c = compPose(cloud->poses[i]);
        particles_m.push_back(c);
        std::cout << particles_m[i] << std::endl;
    }
    ROS_INFO("Particles %d", (int)particles_m.size());
}

void Data::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose) {
    pose_m = compPose(pose.pose.pose);
}

std::vector<Coord> Data::createParticles(int num) {
    std::vector<Coord> p(num);

    for(int i = 0; i < num; ++i) {
        int x = rand() % (map_m.xmax - map_m.xmin) + map_m.xmin;
        int y = rand() % (map_m.ymax - map_m.ymin) + map_m.ymin;
        double sign = rand() % 2 ? -1.0 : 1.0;
        double w = sign * ((rand() % 31416) / 10000.0);
        // std::cout << x << " " << y << " " << w << std::endl;
        p[i] = Coord(x, y, w);
    }

    return p;
}

void Data::plot(const std::vector<Coord>& particles, int step) {
    namespace plt = matplotlibcpp;
    double xmin = 0.0, xmax = 850.0;
    double ymin = 0.0, ymax = 850.0;

    //Graph Format
    auto name = "MCL update=" + std::to_string(step);

    if(step) {
        plt::close(); // Clear the current figure.
        // plt::clf();
    }
    // plt::figure(step + 1);
    plt::figure_size(400, 600);
    plt::xlim(xmin, xmax);
    plt::ylim(ymin, ymax);
    plt::plot(map_m.x, map_m.y, "o", {{"markersize", "1"}});
    plt::axis("equal");
    // std::cout << std::typeid(p) << std::endl;
    plt::title(name);

    auto x = std::vector<int>(particles.size());
    auto y = std::vector<int>(particles.size());
    for(int i = 0; i < particles.size(); ++i) {
        x[i] = particles[i].x;
        y[i] = particles[i].y;
    }

    plt::plot(x, y, "o", {{"markersize", "3"}});
    
    plt::show(false);
    plt::pause(0.001); 
}

/*
void Data::drawRviz() {
	visualization_msgs::Marker points, lineStrip;
	points.header.frame_id = "/my_frame";
	points.header.stamp = ros::Time::now();
	points.ns = "points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	// points.header.frame_id = lineStrip.header.frame_id = "/my_frame";
	// points.header.stamp = lineStrip.header.stamp = ros::Time::now();
	// points.ns = lineStrip.ns = "points_and_lines";
	// points.action = lineStrip.action = visualization_msgs::Marker::ADD;
	// points.pose.orientation.w = lineStrip.pose.orientation.w = 1.0;

    points.id = 0;
    // lineStrip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    // lineStrip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP markers use only the x component of scale, for the line width
    // lineStrip.scale.x = 0.1;

    // Map points are blue
    points.color.g = 1.0;
    points.color.a = 1.0;

    // Line strip is blue
    // lineStrip.color.b = 1.0;
    // lineStrip.color.a = 1.0;

    // Create the vertices for the map points
    std::cout << map_m.map.size() << std::endl;
    for (auto i = 0; i < map_m.map.size(); ++i) {
        geometry_msgs::Point p;
        p.x = map_m.map[i].first;
        p.y = map_m.map[i].second;
        p.z = 0.0;

        points.points.push_back(p);
        // lineStrip.points.push_back(p);

        // The line list needs two points for each line
        // lineList.points.push_back(p);
        // p.z += 1.0;
        // lineList.points.push_back(p);
    }

    markerPub_m.publish(points);
    // marker_pub.publish(lineStrip);
}
*/

}


// PLUGINLIB_EXPORT_CLASS(robotics_mcl::Robot, nodelet::Nodelet);