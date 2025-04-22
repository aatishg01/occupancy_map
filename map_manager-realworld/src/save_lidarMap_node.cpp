#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <csignal>  // For signal handling
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

bool shutting_down = false;  // Flag to ensure the shutdown handler is called only once
int frame_count = 0;  // Counter for the number of frames received
bool save_lidarMap;  // Flag to save the accumulated point cloud

// Function to get the current date and time for the filename
std::string getCurrentDate() {
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
    return oss.str();
}

// Global variables to store the odometry data (transformation from base_link to map)
bool odom_received = false;
pcl::PointCloud<pcl::PointXYZ>::Ptr accumulatedCloud(new pcl::PointCloud<pcl::PointXYZ>);  // Global cloud

void syncCallback(const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, ros::Publisher& cloudPub) {
    // if (!odom_received) {
    //     ROS_WARN("Odometry data not received yet.");
    //     return;
    // }

    // ROS_INFO("[Map Saver]: Latest Point Cloud Message Obtained.");

    const geometry_msgs::Pose& pose = odom_msg->pose.pose;
    
    Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);
    
    // map to base_link
    Eigen::Affine3d map2body = Eigen::Affine3d::Identity();
    map2body.translate(translation);
    map2body.rotate(rotation);

    // LiDAR offset
    Eigen::Affine3d lidar2body = Eigen::Affine3d::Identity();
    lidar2body.translate(Eigen::Vector3d(0.0, 0.0, 0.644));

    // map to LiDAR frame
    Eigen::Affine3d map2lidar = map2body * lidar2body;

    frame_count++;

    pcl::PCLPointCloud2* pclCloud2 = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::PointCloud<pcl::PointXYZ> transformedCloud;  // For the current frame

    // Convert sensor_msgs::PointCloud2 to PCL format
    pcl_conversions::toPCL(*cloud_msg, *pclCloud2);
    pcl::fromPCLPointCloud2(*pclCloud2, pclCloud);

    if (pclCloud.size() > 0) {
        // Transform the point cloud to the global map frame
        for (const auto& point : pclCloud.points) {
            Eigen::Vector3d pt(point.x, point.y, point.z);
            Eigen::Vector3d transformedPt = map2lidar * pt;

            pcl::PointXYZ transformedPoint;
            transformedPoint.x = transformedPt.x();
            transformedPoint.y = transformedPt.y();
            transformedPoint.z = transformedPt.z();
            transformedCloud.points.push_back(transformedPoint);  // Only store current frame
            if (frame_count % 20 == 0) {
                accumulatedCloud->points.push_back(transformedPoint);  // Accumulate the points
            }
        }

        // Set width, height, and is_dense fields for transformedCloud (current frame)
        transformedCloud.width = transformedCloud.points.size();
        transformedCloud.height = 1;  // Unorganized point cloud
        transformedCloud.is_dense = true;

        // Convert back to sensor_msgs::PointCloud2 for publishing (current frame)
        sensor_msgs::PointCloud2 outputCloudMsg;
        pcl::toROSMsg(transformedCloud, outputCloudMsg);
        outputCloudMsg.header.frame_id = "map";
        outputCloudMsg.header.stamp = ros::Time::now();

        // Publish the current frame point cloud to /cloud_registered
        cloudPub.publish(outputCloudMsg);
        // ROS_INFO("Published current frame point cloud.");
    }
}

// Function to save the accumulated point cloud to a single PCD file
void saveAccumulatedCloud() {
    ROS_INFO("Saving the accumulated point cloud...");
    if (accumulatedCloud->points.size() > 0) {
        // Set the correct width and height before saving
        accumulatedCloud->width = accumulatedCloud->points.size();
        accumulatedCloud->height = 1;  // Unorganized point cloud
        accumulatedCloud->is_dense = true;  // Set is_dense to true
        std::string filename = "sim_map_" + getCurrentDate() + ".pcd";
        pcl::io::savePCDFileASCII(filename, *accumulatedCloud);
        ROS_INFO("Successfully wrote accumulated point cloud to file: %s", filename.c_str());
    } else {
        ROS_WARN("No points in the accumulated cloud to save.");
    }
}

// Function to handle node shutdown
void shutdownHandler(int sig) {
    if (!shutting_down) {
        shutting_down = true;  // Ensure this handler is only run once
        ROS_INFO("Node is shutting down.");
        if (save_lidarMap) {
            saveAccumulatedCloud();
        } else {
            ROS_WARN("Skipping save operation as 'save_lidarMap' is set to false.");
        }
        ros::shutdown();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "save_lidarMap_node");
    ros::NodeHandle nh;

    // Get the parameter value
    if (!nh.getParam("save_lidarMap_node/save_lidarMap", save_lidarMap)) {
        save_lidarMap = true;
        ROS_INFO("No save_lidarMap option. Using default: true");
    } else {
        ROS_INFO("Save LiDAR Map: %s", save_lidarMap ? "true" : "false");
    }

    // Create a publisher for the transformed point cloud
    ros::Publisher cloudPub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 10);

    // Subscribe to the odometry and point cloud topics using message filters
    message_filters::Subscriber<nav_msgs::Odometry> odomSub(nh, "/odom_ugv", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub(nh, "/point_cloud_ugv", 10);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odomSub, cloudSub);

    // Bind the sync callback
    sync.registerCallback(boost::bind(&syncCallback, _1, _2, boost::ref(cloudPub)));

    // Handle shutdown signal
    signal(SIGINT, shutdownHandler);

    ROS_INFO("Node started, waiting for point clouds and odometry data...");
    ros::spin();

    return 0;
}
