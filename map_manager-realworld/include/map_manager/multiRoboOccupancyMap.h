/*
	FILE: occupancyMap,h
	-------------------------------------
	occupancy voxel map header file
*/
#ifndef MAPMANAGER_MULTIROBO_OCCUPANCYMAP
#define MAPMANAGER_MULTIROBO_OCCUPANCYMAP
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <map_manager/raycast.h>
// #include <map_manager/viewpointMsg.h>
// #include <map_manager/viewpointArray.h>
#include <map_manager/occupancyMap.h>
#include <map_manager/sharedVoxels.h>
#include <map_manager/robotStates.h>
#include <map_manager/semanticObjMsg.h>
#include <map_manager/semanticObjArrayMsg.h>
#include <map_manager/detector/uvDetector.h>
#include <map_manager/detector/kalmanFilter.h>
#include <map_manager/detector/dbscan.h>
#include <map_manager/detector/utils.h>
#include <thread>


namespace mapManager{
	class multiRoboOccMap : public occMap{
	protected:
		// ROS
		// ------------------------------------------------------------------
		ros::NodeHandle nh_;
		ros::Publisher mapSharedPub_;
		ros::Subscriber mapSharedSub_;
		ros::Publisher robotStatesPub_;
		ros::Subscriber robotStatesSub_;
		ros::Timer mapSharedPubTimer_;
		ros::Timer uDetectTimer_;
		ros::Publisher mapSharedVisPub_;
		// ros::Publisher semanticObjPub_;
		ros::Publisher uMapPub_;
		ros::Subscriber semanticObjSub_;
		ros::Timer pointCloudFilterTimer_;
		// ros::Subscriber lidarDepthImgSub_;
		// ros::Subscriber lidarRegister2ImgSub_;
		ros::Subscriber registeredMapSub_; // subscribe registered map from lidar slam
		// debug
		ros::Publisher debugPointCloudPub_;
		// ros::Timer semanticObjPubTimer_;
		// ------------------------------------------------------------------

		// PARAMS
		// ------------------------------------------------------------------
		int robot_id_;
		int robot_num_;
		std::string robotName_;
		std::vector<int> objOfInterest_;
		Eigen::Matrix4d global2Map_; // from robot team global frame to map frame 
		bool debug_ = true;
		int voxelFilterThresh_;
		double detectThresh_;
		double iouThresh_;
		// ------------------------------------------------------------------

		// SENSOR DATA
		// ------------------------------------------------------------------
		cv::Mat syncDepthImage_;
		cv::Mat lidarDepthImage_;
		cv::Mat lidarRegister2Image_;
		Eigen::Vector3d syncPosition_;
		Eigen::Matrix3d syncOrientation_;
		std::shared_ptr<mapManager::UVdetector> uvDetector_;
		std::shared_ptr<mapManager::DBSCAN> dbscan_;
		std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depthSyncSub_;
		std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> poseSyncSub_;
		std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odomSyncSub_;
		std::shared_ptr<message_filters::Subscriber<vision_msgs::Detection2DArray>> detection2DSyncSub_;
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vision_msgs::Detection2DArray, geometry_msgs::PoseStamped> depthDetectionPoseSyncPolicy;
		std::shared_ptr<message_filters::Synchronizer<depthDetectionPoseSyncPolicy>> depthDetectionPoseSync_; 
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vision_msgs::Detection2DArray, nav_msgs::Odometry> depthDetectionOdomSyncPolicy;
		std::shared_ptr<message_filters::Synchronizer<depthDetectionOdomSyncPolicy>> depthDetectionOdomSync_;
		std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> lidarDepthImgSyncSub_;
		std::shared_ptr<message_filters::Synchronizer<depthDetectionOdomSyncPolicy>> lidarDepthDetectionOdomSync_;
		// ------------------------------------------------------------------

		// STATUS DATA	
		// ------------------------------------------------------------------
		std::vector<int> readyRobotsID_; // vector recording IDs of robot being ready to share map
		bool allRobotsReady_; // all robots in the network are ready to share map
		map_manager::sharedVoxels sharedVoxels_; 
		map_manager::robotStates robotStates_;
		// ------------------------------------------------------------------

		// MAP DATA
		Eigen::Vector3d uavSize_;
		std::vector<bool> uavOccupancyInflated_;
		Eigen::Vector3d ugvLPHMin_;
		Eigen::Vector3d ugvLPHMax_;
		std::vector<int> voxelFilterCount_;
		std::vector<std::deque<int>> voxelFilterCountArray_;
		std::vector<int> voxelFilterOccupy_;
		int voxelFilterStep_;
		// ------------------------------------------------------------------

		// DETECTION DATA
		// ------------------------------------------------------------------
		int dbMinPointsCluster_;
        double dbEpsilon_;
		
		// ------------------------------------------------------------------

	public:
		multiRoboOccMap();
		multiRoboOccMap(const ros::NodeHandle& nh);
		virtual ~multiRoboOccMap() = default;
		void initMultiRoboOccMap(const ros::NodeHandle& nh);
		void initMultiRoboParam();
		void registerMultiRoboCallback();
		void registerMultiRoboPub();
		void depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
		void depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
		void pointcloudPoseCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const geometry_msgs::PoseStampedConstPtr& pose);
		void pointcloudOdomCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const nav_msgs::OdometryConstPtr& odom);
		void mapSharedPubCB(const ros::TimerEvent& );
		void mapSharedSubCB(const map_manager::sharedVoxelsConstPtr& incomeVoxels);
		// void semanticObjPubCB(const ros::TimerEvent& );
		void semanticObjSubCB(const map_manager::semanticObjArrayMsgConstPtr& objArray);
		void robotStatesSubCB(const map_manager::robotStatesConstPtr& states);
		void updateOccupancyCB(const ros::TimerEvent& );
		void pointCloudFilterCB(const ros::TimerEvent& );
		// void uDetectCB(const ros::TimerEvent& );
		// void lidarDepthImgSubCB(const sensor_msgs::ImageConstPtr& img);
		// void lidarRegister2ImgSubCB(const sensor_msgs::ImageConstPtr& img);
		

		// helper functions
		void raycastUpdate();
		bool isNearbyRobot(const Eigen::Vector3d& pos); // check if input voxel is nearby robot or not

		// override functions
		// void rawObjDetectSubCB(const vision_msgs::Detection3DArrayConstPtr& objectDetections) override {
		// 	this->objectDetections_.clear();
		// 	// filter the raw detection
		// 	this->filterRawDetection(objectDetections);

		// 	// estimate the semantic objects. This is simple version. In the future, fuse all detections in the history to get current best esitmation
		// 	this->estimateSemanticObj();

		// 	// publish semantic objects estimation
		// 	map_manager::semanticObjArrayMsg semanticObjArray;
		// 	for (auto& object : this->semanticObjects_){
		// 		// publish semantic object
		// 		map_manager::semanticObjMsg semanticObj;
		// 		semanticObj.label_id = object.label_id;
		// 		semanticObj.position = {object.position(0), object.position(1), object.position(2)};
		// 		semanticObj.size = {object.size(0), object.size(1), object.size(2)};
		// 		semanticObj.view_angles = object.view_angles;
				
		// 		semanticObjArray.semanticObjs.push_back(semanticObj);
		// 	}

		// 	// print out semantic objects
		// 	// cout << this->hint_ << ": Semantic objects: " << endl;
		// 	// for (size_t i=0; i<this->semanticObjects_.size(); ++i){
		// 	// 	cout << "Object " << i << ": " << "label_id: " << this->semanticObjects_[i].label_id << ", position: " << this->semanticObjects_[i].position.transpose() << ", size: " << this->semanticObjects_[i].size.transpose() << endl;
		// 	// }

		// }

		// overload
		// for lidar depth image
		// void rawObjDetectSubCB2D(const vision_msgs::Detection2DArrayConstPtr& objectDetections) {
		// 	// synchroneize depth image and bounding box 
		// 	ROS_INFO("2D detection recieved.");
		// 	if (this->lidarDepthImage_.empty()){
		// 		ROS_ERROR("Lidar depth image is empty. Cannot get 3D box from 2D detection.");
		// 		return;
		// 	}
		// 	this->objectDetections_.clear();
		// 	for (const auto& detection : objectDetections->detections){
		// 		// skip objects if it is not of interest
		// 		if (std::find(this->objOfInterest_.begin(), this->objOfInterest_.end(), detection.results[0].id) == this->objOfInterest_.end()){
		// 			continue;
		// 		}
		// 		// skip if confidence score is too low
		// 		if (detection.results[0].score < 0.8){
		// 			continue;
		// 		}
		// 		this->get3dFromRaw2d(detection, this->lidarDepthImage_);
		// 		// publish image with bounding box
		// 		// if (this->depthImage_.empty()){
		// 		// 	ROS_ERROR("Depth image is empty. Cannot visualize detection.");
		// 		// 	return;
		// 		// }
				
		// 		this->drawBoundingBox(detection, this->lidarDepthImage_);
		// 	}

		// 	// print out objectDetections_
		// 	for (const auto& detection : this->objectDetections_){
		// 		ROS_INFO("Object: %i, center: (%f, %f), size: (%f, %f)", detection.results[0].id, detection.bbox.center.position.x, detection.bbox.center.position.y, detection.bbox.size.x, detection.bbox.size.y);
		// 	}

		// 	// std::vector<vision_msgs::Detection3D> filteredDetections;
		// 	// this->filterBoundingBox(this->objectDetections_, filteredDetections);
		// 	// this->objectDetections_ = filteredDetections;

		// 	this->estimateSemanticObj();

		// 	// // publish semantic objects estimation
		// 	// map_manager::semanticObjArrayMsg semanticObjArray;
		// 	// for (const auto& object : this->semanticObjects_){
		// 	// 	// publish semantic object
		// 	// 	map_manager::semanticObjMsg semanticObj;
		// 	// 	semanticObj.label_id = object.label_id;
		// 	// 	semanticObj.position = {object.position(0), object.position(1), object.position(2)};
		// 	// 	semanticObj.size = {object.size(0), object.size(1), object.size(2)};
		// 	// 	semanticObjArray.semanticObjs.push_back(semanticObj);
		// 	// }

			
		// }

		// for lidar depth image
		// void get3dFromRaw2d(const vision_msgs::Detection2D& detection, cv::Mat& depthImage) override {
		// 	ROS_INFO("Get 3D box from 2D detection.");
		// 	// 1. retrive 2D detection result
		// 	int topX = int(detection.bbox.center.x) - int(detection.bbox.size_x/2.0);
		// 	int topY = int(detection.bbox.center.y) - int(detection.bbox.size_y/2.0);
		// 	int xPixelWidth = int(detection.bbox.size_x); 
		// 	int yPixelWidth = int(detection.bbox.size_y); 

		// 	if (depthImage.empty()){
		// 		ROS_ERROR("Lidar depth image is empty. Cannot get 3D box from 2D detection.");
		// 		return;
		// 	}

		// 	// transform lidarpoints into map frame
		// 	uint16_t* rowPtr;
		// 	int rows = depthImage.rows;
		// 	int cols = depthImage.cols;
		// 	double depth;
		// 	const double inv_factor = 1.0 / this->depthScale_;
		// 	int vMin = std::max(topY, this->depthFilterMargin_);
		// 	int uMin = std::max(topX, this->depthFilterMargin_);
		// 	int vMax = std::min(topY+yPixelWidth, rows-this->depthFilterMargin_);
		// 	int uMax = std::min(topX+xPixelWidth, cols-this->depthFilterMargin_);
		// 	std::vector<Eigen::Vector3d> objPoints;
		// 	Eigen::Vector3d currPointPanoCam, currPointMap;

		// 	// debug: loop for all image pixels
		// 	// vMin = 0;
		// 	// uMin = 0;
		// 	// vMax = rows;
		// 	// uMax = cols;

		// 	int y0 = 400;
		// 	double fy = 180;
		// 	int address;
		// 	Eigen::Vector3d currPointLidar;
		// 	double xMin = this->mapSize_(0); double xMax = -this->mapSize_(0);
		// 	double yMin = this->mapSize_(1); double yMax = -this->mapSize_(1);
		// 	double zMin = this->mapSize_(2); double zMax = -this->mapSize_(2);
		// 	// orginal code
		// 	for (int v=vMin; v<vMax; ++v){ // row
		// 		rowPtr = depthImage.ptr<uint16_t>(v); // NOTE: should use aligned depth image in real world experiments
		// 		rowPtr += uMin;
		// 		for (int u=uMin; u<uMax; ++u){ // column				
		// 			depth = (*rowPtr) * inv_factor;
		// 			if (depth > 10.0){ 
		// 				++rowPtr;
		// 				continue; // filter out points far away
		// 			}

		// 			// use project-back method
		// 			// get 3D point in camera frame
		// 			currPointPanoCam(0) = depth * std::cos(M_PI-2*M_PI*u/cols);
		// 			currPointPanoCam(1) = depth * std::sin(M_PI-2*M_PI*u/cols);
		// 			currPointPanoCam(2) = - depth * ((v-y0)/fy);
		// 			// point projected back from lidar-camera register image is in the pano camera frame
		// 			Eigen::Vector3d panoPosition;
		// 			panoPosition = this->rangeSensorPosition_;
		// 			panoPosition(0) = this->rangeSensorPosition_(0) - this->body2Cam_(0,3);
		// 			panoPosition(1) = this->rangeSensorPosition_(1) - this->body2Cam_(1,3);
		// 			panoPosition(2) = this->rangeSensorPosition_(2) - this->body2Cam_(2,3);
		// 			currPointMap = this->orientation_ * currPointPanoCam + panoPosition; // transform to map coordinate
		// 			if (currPointMap(2)>=this->groundHeight_+0.2){
		// 				// address = this->posToAddress(currPointMap);
		// 				// if (this->voxelFilterOccupy_[address] >= this->voxelFilterThresh_){
		// 				// 	objPoints.push_back(currPointMap);
		// 				// }
		// 				if (currPointMap(0) < xMin){
		// 					xMin = currPointMap(0);
		// 				}
		// 				if (currPointMap(0) > xMax){
		// 					xMax = currPointMap(0);
		// 				}
		// 				if (currPointMap(1) < yMin){
		// 					yMin = currPointMap(1);
		// 				}
		// 				if (currPointMap(1) > yMax){
		// 					yMax = currPointMap(1);
		// 				}
		// 				if (currPointMap(2) < zMin){
		// 					zMin = currPointMap(2);
		// 				}
		// 				if (currPointMap(2) > zMax){
		// 					zMax = currPointMap(2);
		// 				}

		// 			}
		// 			++rowPtr;

		// 			// use prebuilt map
		// 			// int imgAdr = v * cols + u;
		// 			// for (const auto& point : this->lidarCamRegister_->pixel2Lidar[imgAdr]){
		// 			// 	currPointLidar = point;
		// 			// 	Eigen::Vector3d panoPosition;
		// 			// 	panoPosition = this->rangeSensorPosition_;
		// 			// 	panoPosition(0) = this->rangeSensorPosition_(0) - this->body2Cam_(0,3);
		// 			// 	panoPosition(1) = this->rangeSensorPosition_(1) - this->body2Cam_(1,3);
		// 			// 	panoPosition(2) = this->rangeSensorPosition_(2) - this->body2Cam_(2,3);
		// 			// 	currPointMap = this->orientation_ * currPointLidar + panoPosition;
		// 			// 	if (currPointMap(2)>=this->groundHeight_+0.2){
		// 			// 		address = this->posToAddress(currPointMap);
		// 			// 		if (this->voxelFilterOccupy_[address] >= this->voxelFilterThresh_){
		// 			// 			objPoints.push_back(currPointMap);
		// 			// 		}
		// 			// 	}
		// 			// }
		// 		}
		// 	}

		// 	pcl::PointCloud<pcl::PointXYZ> output_cloud;
		// 	double tolerance = 0.0;
		// 	// Define the min and max points for the cropping box
		// 	Eigen::Vector4f min_point(xMin-tolerance, yMin-tolerance, zMin, 1.0); // Adjust these values to your needs
		// 	Eigen::Vector4f max_point(xMax+tolerance, yMax+tolerance, zMax+tolerance, 1.0);   // Adjust these values to your needs
		// 	// std::cout << "min_point: " << min_point << std::endl;
		// 	// std::cout << "max_point: " << max_point << std::endl;

		// 	pcl::CropBox<pcl::PointXYZ> crop_filter;
		// 	crop_filter.setInputCloud(this->filteredPointcloud_.makeShared());
		// 	crop_filter.setMin(min_point);
		// 	crop_filter.setMax(max_point);
		// 	crop_filter.filter(output_cloud);

		// 	// output_cloud = this->filteredPointcloud_;

		// 	// cv::Rect roi(topX, topY, xPixelWidth, yPixelWidth);
		// 	// cv::Mat roiDepthImage = depthImage(roi);
		// 	// this->uvDetector_->depth = roiDepthImage;
		// 	// this->uvDetector_->detect();

		// 	// this->uvDetector_->display_U_map();
		// 	// // this->uvDetector_->display_bird_view();
		// 	// // this->uvDetector_->display_depth();
		// 	// this->publishUvVis();

		// 	ROS_INFO("point projected");

		// 	// debug:: publish currPointPanoCam as debugPointCloud
		// 	// pcl::PointCloud<pcl::PointXYZ> debugPointCloud;
		// 	// debugPointCloud.header.frame_id = "map";
		// 	// debugPointCloud.height = 1;
		// 	// debugPointCloud.width = objPoints.size();
		// 	// ROS_INFO("objPoints size: %d", objPoints.size());
		// 	// debugPointCloud.is_dense = true;
		// 	// pcl::PointXYZ pt;
		// 	// for (const auto& p : objPoints){
				
		// 	// 	pt.x = p(0);
		// 	// 	pt.y = p(1);
		// 	// 	pt.z = p(2);
		// 	// 	debugPointCloud.points.push_back(pt);
		// 	// }
		// 	// debugPointCloud = output_cloud;
		// 	// sensor_msgs::PointCloud2 debugPointCloudMsg;
		// 	// pcl::toROSMsg(debugPointCloud, debugPointCloudMsg);
		// 	// this->debugPointCloudPub_.publish(debugPointCloudMsg);
		// 	pcl::PointCloud<pcl::PointXYZ> debugPointCloud;
		// 	output_cloud.header.frame_id = "map";
		// 	output_cloud.height = 1;
		// 	output_cloud.width = output_cloud.points.size();
		// 	std::cout << "output_cloud size: " << output_cloud.points.size() << std::endl;

		// 	sensor_msgs::PointCloud2 debugPointCloudMsg;
		// 	pcl::toROSMsg(output_cloud, debugPointCloudMsg);
		// 	this->debugPointCloudPub_.publish(debugPointCloudMsg);

		// 	// dbscan clustring
		// 	ROS_INFO("start clustering");
		// 	std::vector<std::vector<Eigen::Vector3d>> clusters;
		// 	std::vector<mapManager::Point> pointsDB;
		// 	// mapManager::eigenToDBPointVec(objPoints, pointsDB, objPoints.size());
		// 	mapManager::pclToDBPointVec(output_cloud, pointsDB, output_cloud.points.size());
		// 	this->dbscan_.reset(new mapManager::DBSCAN(this->dbMinPointsCluster_, this->dbEpsilon_, pointsDB));
		// 	this->dbscan_->run();
		// 	if (this->dbscan_->cluster_num<=0){
		// 		ROS_WARN("No cluster found in 2D detection.");
		// 		return;
		// 	}
		// 	clusters.resize(this->dbscan_->cluster_num);
		// 	for (int i=0; i<this->dbscan_->m_points.size(); ++i){
		// 		int clusterID = this->dbscan_->m_points[i].clusterID;
		// 		if (clusterID >0){
		// 			mapManager::Point point = this->dbscan_->m_points[i];
		// 			Eigen::Vector3d p = mapManager::dbPointToEigen(point);
		// 			clusters[clusterID-1].push_back(p);
		// 		}
		// 	}
		// 	int closestClusterIdx = 0;
		// 	Eigen::Vector3d closestClusterCenter = Eigen::Vector3d(100, 100, 100);
		// 	for (size_t i=0 ; i<clusters.size(); ++i){ // caculate center of each cluster
		// 		Eigen::Vector3d center = Eigen::Vector3d::Zero();
		// 		for (const auto& p : clusters[i]){
		// 			center += p;
		// 		}
		// 		center /= clusters[i].size();
		// 		if ((center - this->rangeSensorPosition_).norm() < (closestClusterCenter - this->rangeSensorPosition_).norm()){
		// 			closestClusterIdx = i;
		// 			closestClusterCenter = center;
		// 		}
		// 	}

		// 	ROS_INFO("Closest cluster index: %d", closestClusterIdx);
		// 	ROS_INFO("Closest cluster center: (%f, %f, %f)", closestClusterCenter(0), closestClusterCenter(1), closestClusterCenter(2));
		// 	ROS_INFO("clster numbers %d", clusters.size());

		// 	// get bounding box

		// 	// extract bounding box fom point cloud
		// 	xMin = this->mapSize_(0); xMax = -this->mapSize_(0);
		// 	yMin = this->mapSize_(1); yMax = -this->mapSize_(1);
		// 	zMin = this->mapSize_(2); zMax = -this->mapSize_(2);
			
		// 	Eigen::Vector3d point;
		// 	for (size_t i=0 ; i<clusters[closestClusterIdx].size(); ++i){
		// 		point = clusters[closestClusterIdx][i];
		// 		if (point(0) < xMin){
		// 			xMin = point(0);
		// 		}
		// 		if (point(0) > xMax){
		// 			xMax = point(0);
		// 		}	
		// 		if (point(1) < yMin){
		// 			yMin = point(1);
		// 		}
		// 		if (point(1) > yMax){
		// 			yMax = point(1);
		// 		}
		// 		if (point(2) < zMin){
		// 			zMin = point(2);
		// 		}
		// 		if (point(2) > zMax){
		// 			zMax = point(2);
		// 		}
		// 	}
		// 	double x = (xMin + xMax) / 2.0;
		// 	double y = (yMin + yMax) / 2.0;
		// 	double z = (zMin + zMax) / 2.0;
		// 	double xWidth = xMax - xMin;
		// 	double yWidth = yMax - yMin;
		// 	double zWidth = zMax - zMin;
		// 	assert(xWidth >= 0 and yWidth >= 0 and zWidth >= 0);
		// 	// return empty if 2d distance is too far
		// 	if (std::sqrt((x - this->rangeSensorPosition_(0))*(x - this->rangeSensorPosition_(0)) + (y - this->rangeSensorPosition_(1))*(y - this->rangeSensorPosition_(1))) > 5){
		// 		ROS_WARN("Object is too far away from robot. Skip.");
		// 		return;
		// 	}
		// 	// get 8 bouding boxes coordinates (in the map frame )                
		// 	Eigen::Vector3d p1m (x+xWidth/2.0, y+yWidth/2.0, z+zWidth/2.0);
		// 	Eigen::Vector3d p2m (x+xWidth/2.0, y+yWidth/2.0, z-zWidth/2.0);
		// 	Eigen::Vector3d p3m (x+xWidth/2.0, y-yWidth/2.0, z+zWidth/2.0);
		// 	Eigen::Vector3d p4m (x+xWidth/2.0, y-yWidth/2.0, z-zWidth/2.0);
		// 	Eigen::Vector3d p5m (x-xWidth/2.0, y+yWidth/2.0, z+zWidth/2.0);
		// 	Eigen::Vector3d p6m (x-xWidth/2.0, y+yWidth/2.0, z-zWidth/2.0);
		// 	Eigen::Vector3d p7m (x-xWidth/2.0, y-yWidth/2.0, z+zWidth/2.0);
		// 	Eigen::Vector3d p8m (x-xWidth/2.0, y-yWidth/2.0, z-zWidth/2.0);

		// 	// re-project back to image plane, calculate 2d iou with raw 2d detection
		// 	// choose top left and bottom right corner of the bounding box front face, since they generate largest 2d box
		// 	// first, project p5m and p8m from map frame to camera frame
		// 	Eigen::Vector3d panoPosition;
		// 	panoPosition = this->rangeSensorPosition_;
		// 	panoPosition(0) = this->rangeSensorPosition_(0) - this->body2Cam_(0,3);
		// 	panoPosition(1) = this->rangeSensorPosition_(1) - this->body2Cam_(1,3);
		// 	panoPosition(2) = this->rangeSensorPosition_(2) - this->body2Cam_(2,3);
		// 	std::vector<Eigen::Vector3d> pointsMap {p1m, p2m, p3m, p4m, p5m, p6m, p7m, p8m};
			

		// 	// calculate 2d coordinates
		// 	int RePrjXmin = cols; int RePrjXmax = 0; 
		// 	int RePrjYmin = rows; int RePrjYmax = 0;
		// 	for (auto pm : pointsMap){
		// 		// std::cout << "pm: " << pm << std::endl;
		// 		Eigen::Vector3d pc = this->orientation_.transpose() * (pm - panoPosition);
		// 		// std::cout << "pc: " << pc << std::endl;
		// 		int x_img = cols * (-atan2(pc(1), pc(0)) + M_PI) / (2*M_PI);
		// 		int y_img = fy * (-pc(2)/std::sqrt(pc(0)*pc(0)+pc(1)*pc(1))) + y0;
		// 		if (x_img < RePrjXmin){
		// 			RePrjXmin = x_img;
		// 		}
		// 		if (x_img > RePrjXmax){
		// 			RePrjXmax = x_img;
		// 		}
		// 		if (y_img < RePrjYmin){
		// 			RePrjYmin = y_img;
		// 		}
		// 		if (y_img > RePrjYmax){
		// 			RePrjYmax = y_img;
		// 		}
		// 		// std::cout << "x_img: " << x_img << " y_img: " << y_img << std::endl;
		// 	}

		// 	// create cv rect, calculate 2d iou
		// 	cv::Rect reprojectRect(RePrjXmin, RePrjYmin, RePrjXmax-RePrjXmin, RePrjYmax-RePrjYmin);
		// 	cv::Rect rawRect(topX, topY, xPixelWidth, yPixelWidth);
		// 	std::cout << "reprojectRect: " << RePrjXmin << " " << RePrjYmin << " "<<RePrjXmax-RePrjXmin << " "<< RePrjYmax-RePrjYmin << std::endl;
		// 	std::cout << "rawRect: " << topX << " " << topY << " " << xPixelWidth << " "<< yPixelWidth << std::endl;
		// 	cv::Mat depthNormalized = depthImage.clone();
		// 	double min, max;
		// 	cv::minMaxIdx(depthNormalized, &min, &max);
		// 	cv::convertScaleAbs(depthNormalized, depthNormalized, 255. / max);
		// 	depthNormalized.convertTo(depthNormalized, CV_8UC1);
		// 	cv::applyColorMap(depthNormalized, depthNormalized, cv::COLORMAP_BONE);
		// 	cv::rectangle(depthNormalized, reprojectRect, cv::Scalar(0, 255, 0), 5, 8, 0);
		// 	cv::rectangle(depthNormalized, rawRect, cv::Scalar(0, 0, 255), 5, 8, 0);
		// 	this->depthImgWithDetectPub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", depthNormalized).toImageMsg());
		// 	double iou = calc2DIou(reprojectRect, rawRect);
		// 	if (iou < this->iouThresh_){

		// 		ROS_WARN("IoU is too low. Skip.");
		// 		std::cout << "iou: " << iou << std::endl;
		// 		return;
		// 	}
		// 	std::cout << "iou: " << iou << std::endl;

			
		// 	vision_msgs::Detection3D detection3d;
		// 	detection3d.results = detection.results;
		// 	detection3d.bbox.center.position.x = (xMin + xMax)/2.0;
		// 	detection3d.bbox.center.position.y = (yMin + yMax)/2.0;
		// 	detection3d.bbox.center.position.z = (zMin + zMax)/2.0;
		// 	detection3d.bbox.size.x = xMax - xMin;
		// 	detection3d.bbox.size.y = yMax - yMin;
		// 	detection3d.bbox.size.z = zMax - zMin;

		// 	this->objectDetections_.push_back(detection3d);

			

		// }

		void depthDetectionOdomCB(const sensor_msgs::ImageConstPtr& img, const vision_msgs::Detection2DArrayConstPtr& objectDetections, const nav_msgs::OdometryConstPtr& odom);
		void depthDetectionPoseCB(const sensor_msgs::ImageConstPtr& img, const vision_msgs::Detection2DArrayConstPtr& objectDetections, const geometry_msgs::PoseStampedConstPtr& pose);
		void lidarDepthDetectionOdomCB(const sensor_msgs::ImageConstPtr& img, const vision_msgs::Detection2DArrayConstPtr& objectDetections, const nav_msgs::OdometryConstPtr& odom);
		// vis
		void publishUvVis();
	};

	// if the pos is within robotSize_ distance to current robot pose, return true
	inline bool multiRoboOccMap::isNearbyRobot(const Eigen::Vector3d& pos){
		// if dist from pos to current position is within 1 meter, return true
		if ((pos - this->bodyPosition_).norm() < 1){
			return true;
		}
		return false;
	}
	
}
#endif