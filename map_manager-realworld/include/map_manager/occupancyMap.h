/*
	FILE: occupancyMap,h
	-------------------------------------
	occupancy voxel map header file
*/
#ifndef MAPMANAGER_OCCUPANCYMAP
#define MAPMANAGER_OCCUPANCYMAP
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
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <map_manager/raycast.h>
#include <map_manager/objectMap.h>
#include <map_manager/detector/dbscan.h>
#include <map_manager/semanticObjMsg.h>
#include <map_manager/semanticObjArrayMsg.h>
#include <map_manager/sensorModel.h>
#include <thread>
#include <cassert>	
// #include <unordered_map>
// #include <unordered_set>
#include "utils.h"
#define M_PI 3.14159265358979323846

using std::cout; using std::endl;
namespace mapManager{

	struct semanticObject{
		int label_id;
		Eigen::Vector3d position;
		Eigen::Vector3d size;
		double view_angle_base;
		std::vector<unsigned char> view_angles;
	};

	class occMap{
	private:

	protected:
		std::string ns_;
		std::string hint_;

		// ROS
		// ------------------------------------------------------------------
		ros::NodeHandle nh_;

		// occupancy map 
		std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depthSub_;
		std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pointcloudSub_;
		std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> poseSub_;
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> depthPoseSync;
		std::shared_ptr<message_filters::Synchronizer<depthPoseSync>> depthPoseSync_;
		std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odomSub_;
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> depthOdomSync;
		std::shared_ptr<message_filters::Synchronizer<depthOdomSync>> depthOdomSync_;
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> pointcloudPoseSync;
		std::shared_ptr<message_filters::Synchronizer<pointcloudPoseSync>> pointcloudPoseSync_;
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> pointcloudOdomSync;
		std::shared_ptr<message_filters::Synchronizer<pointcloudOdomSync>> pointcloudOdomSync_;	

		// semantic object detection
		// depth image-detection-odom-sync
		std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depthSyncSub_;
		std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odomSyncSub_;
		std::shared_ptr<message_filters::Subscriber<vision_msgs::Detection2DArray>> detection2DSyncSub_;
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vision_msgs::Detection2DArray, nav_msgs::Odometry> depthDetectionOdomSyncPolicy;
		std::shared_ptr<message_filters::Synchronizer<depthDetectionOdomSyncPolicy>> depthDetectionOdomSync_;

		// lidar-odom-detection-sync

		ros::Subscriber rawObjDetectSub_; // subscribe semantic object bounding boxes
		ros::Subscriber registeredMapSub_; // subscribe registered map from lidar slam
		ros::Timer occTimer_;
		ros::Timer inflateTimer_;
		ros::Timer visTimer_;
		ros::Timer objectMapTimer_;
		ros::Publisher depthCloudPub_;
		ros::Publisher freeRegionPub_;
		ros::Publisher mapVisPub_;
		ros::Publisher inflatedMapVisPub_;
		ros::Publisher map2DPub_;
		ros::Publisher mapExploredPub_;
		ros::Publisher mapUnknownPub_;
		ros::Publisher objectVisPub_;
		image_transport::Publisher depthImgWithDetectPub_;
		ros::Publisher objectMapVisPub_;
		ros::Publisher objectDenseCloudPub_;
		ros::Publisher debugPointCloudPub_;
		image_transport::Publisher detectedDepthImgPub_;
		ros::Publisher semanticObjPub_;

		// PARAMS
		// -----------------------------------------------------------------
		double simTimeFactor_ = 1.0;
		int rangeSensorInputMode_;
		int localizationMode_;
		int sensorFusionMode_;
		std::string depthTopicName_; // depth image topic
		std::string pointcloudTopicName_; // point cloud topic
		std::string poseTopicName_;  // pose topic
		std::string odomTopicName_; // odom topic 
		std::string rawDetectionTopicName_; // raw detection topic
		std::string denseCloudTopicName_; // input dense cloud topic
		// robot size
		Eigen::Vector3d robotSize_;
		double visibleDist_;
		double invisibleRange_;
		Eigen::Vector3d collisionBoxMin_, collisionBoxMax_;

		// range sensor: depth camera or lidar
		double fx_, fy_, cx_, cy_; // depth camera intrinsics
		double fovH_, fovV_; // field of view
		double depthScale_; // value / depthScale
		double depthMinValue_, depthMaxValue_;
		int depthFilterMargin_; // depth filter margin
		int skipPixel_ = 1; // depth skip pixel, default: 1
		int imgCols_, imgRows_;
		// convention: A2B means transform the physical coordinate frame A to the physical coordinate frame B, not the pose in this coordinate frame
		Eigen::Matrix4d body2RangeSensor_; // from body frame to range sensor(depth camera or lidar) frame
		Eigen::Matrix4d body2ColorSensor_; // from body frame to color sensor(pinhole color camera or panocam) frame
		Eigen::Matrix4d map2Body_; // from map(global) frame to body frame
		nav_msgs::Odometry odom_;
		// camera aligned depth to color
        double fxC_, fyC_, cxC_, cyC_;

		// sensor models
		// std::unordered_map<std::string, std::shared_ptr<sensorModelBase>> sensorModels_;
		std::shared_ptr<sensorModelFactory> sensorManager_;

		// raycast
		double raycastMaxLength_;
		double pHitLog_, pMissLog_, pMinLog_, pMaxLog_, pOccLog_; 

		// map
		Eigen::Vector3d bodyPosition_;
		Eigen::Matrix3d bodyOrientation_;
		Eigen::Vector3d rangeSensorPosition_;
		Eigen::Vector3d colorSensorPosition_;
		double UNKNOWN_FLAG_ = 0.01;
		double mapRes_;
		double groundHeight_; // ground height in z axis
		Eigen::Vector3d mapSize_, mapSizeMin_, mapSizeMax_; // reserved min/max map size
		Eigen::Vector3i mapVoxelMin_, mapVoxelMax_; // reserved min/max map size in voxel
		Eigen::Vector3d localUpdateRange_; // self defined local update range
		double localBoundInflate_; // inflate local map for some distance
		bool cleanLocalMap_; 
		std::string prebuiltMapDir_;

		// visualization
		double maxVisHeight_;
		Eigen::Vector3d localMapSize_;
		Eigen::Vector3i localMapVoxel_; // voxel representation of local map size
		bool visGlobalMap_;
		bool verbose_;

		// semantic detection
		double objectMapDuration_;
		int viewAgIntervals_;
		double viewAgRes_;
		int dbDownsampleFactor_;
		int dbMinPointsCluster_;
        double dbEpsilon_;
		std::vector<int> objOfInterest_;
		std::vector<double> objBboxColor_;
		std::vector<std::string> objClassLabel_;
		std::map<int, std::pair<std::vector<double>, std::string>> objColorLabelMap_;
		double detectThresh_;
		double objectMapRes_;
		double minMaskScoreThresh_;
        double obsValidProbThresh_;
		double obsInvalidProbThresh_;
		mapManager::objectMapParams objectMapParams_;
		bool objMapDebug_;
		std::string debugRootDir_;
		int voxelFilterThresh_;
		double objectFilterHeight_;
		double objectTrackThresh_;
		double objectMergeThresh_;

		// DATA
		// -----------------------------------------------------------------
		// sensor data
		cv::Mat depthImage_;
		pcl::PointCloud<pcl::PointXYZ> pointcloud_;
		pcl::PointCloud<pcl::PointXYZ> registeredDenseCloud_;
		Eigen::Vector3i localBoundMin_, localBoundMax_; // sensor data range

		std::shared_ptr<mapManager::DBSCAN> dbscan_;
		

		// map data
		int projPointsNum_ = 0;
		std::vector<Eigen::Vector3d> projPoints_; // projected points from depth image
		std::vector<int> countHitMiss_;
		std::vector<int> countHit_;
		std::queue<Eigen::Vector3i> updateVoxelCache_;
		std::queue<Eigen::Vector3i> updateVoxelCacheCopy_;
		std::vector<double> occupancy_; // occupancy log data
		std::vector<bool> occupancyInflated_; // inflated occupancy data
		int raycastNum_ = 0; 
		std::vector<int> flagTraverse_, flagRayend_, flagTraverseIG_;
		std::vector<bool> isVisited_; // for lidar detection: is this voxel visited by raycasting in this iteration
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions_;
		std::deque<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> histFreeRegions_;
		Eigen::Vector3d currMapRangeMin_ = Eigen::Vector3d (0, 0, 0); 
		Eigen::Vector3d currMapRangeMax_ = Eigen::Vector3d (0, 0, 0);
		bool useFreeRegions_ = false;

		// semantic data
		std::vector<vision_msgs::Detection3D> objectDetections_; // a buffer to store raw object detection. will be transformed into semantic objects
		std::vector<semanticObject> semanticObjects_;
		

		// status
		bool occNeedUpdate_ = false;
		bool mapNeedInflate_ = false;
		bool esdfNeedUpdate_ = false; // only used in ESDFMap

		// Raycaster
		RayCaster raycaster_;

		// DETECTION DATA
		// ------------------------------------------------------------------
		std::vector<std::shared_ptr<mapManager::objectMap>> objectMapList_;
		// ------------------------------------------------------------------


	public:
		occMap(); // empty constructor
		occMap(const ros::NodeHandle& nh);
		~occMap() = default;
		void initMap(const ros::NodeHandle& nh);
		void initParam();
		void initPrebuiltMap();
		void registerCallback();
		void registerPub();
		void initCollisionFreeBox();

		// callback
		void depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
		void depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
		void pointcloudPoseCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const geometry_msgs::PoseStampedConstPtr& pose);
		void pointcloudOdomCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const nav_msgs::OdometryConstPtr& odom);
		// virtual void rawObjDetectSubCB(const vision_msgs::Detection3DArrayConstPtr& objectDetections);
		// virtual void rawObjDetectSubCB2D(const vision_msgs::Detection2DArrayConstPtr& objectDetections);
		void depthDetectionOdomCB(const sensor_msgs::ImageConstPtr& img, const vision_msgs::Detection2DArrayConstPtr& detections, const nav_msgs::OdometryConstPtr& odom);
		void updateOccupancyCB(const ros::TimerEvent& );
		virtual void inflateMapCB(const ros::TimerEvent& );
		void updateObjectMapCB(const ros::TimerEvent& );
		void denseCloudCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud );

		// core function
		// void projectDepthImage();
		// void getPointcloud();
		void raycastUpdate();
		void cleanLocalMap();
		virtual void inflateLocalMap();

		// sensor models
		// void addSensorModel();

		// user functions
		bool isOccupied(const Eigen::Vector3d& pos);
		bool isOccupied(const Eigen::Vector3i& idx); // does not count for unknown
		bool isInflatedOccupied(const Eigen::Vector3d& pos);
		bool isInflatedOccupied(const Eigen::Vector3i& idx);
		bool isInflatedOccupiedLine(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
		bool isFree(const Eigen::Vector3d& pos);
		bool isFree(const Eigen::Vector3i& idx);
		bool isInflatedFree(const Eigen::Vector3d& pos);
		bool isInflatedFreeLine(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
		bool isUnknown(const Eigen::Vector3d& pos);
		bool isUnknown(const Eigen::Vector3i& idx);
		bool isUnknown(const int address);
		void setFree(const Eigen::Vector3d& pos);
		void setFree(const Eigen::Vector3i& idx);
		void freeRegion(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
		void freeRegions(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions);
		void freeHistRegions();
		void updateFreeRegions(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions);
		double getRes();
		void getMapRange(Eigen::Vector3d& mapSizeMin, Eigen::Vector3d& mapSizeMax);
		void getMapVoxelRange(Eigen::Vector3i& mapVoxelSizeMax);
		void getCurrMapRange(Eigen::Vector3d& currRangeMin, Eigen::Vector3d& currRangeMax);
		bool castRay(const Eigen::Vector3d& start, const Eigen::Vector3d& direction, Eigen::Vector3d& end, double maxLength=5.0, bool ignoreUnknown=true);
		void getSemanticObjects(std::vector<semanticObject>& objects);
		bool isCollisionInSafeRegion(const Eigen::Vector3d& pos, const Eigen::Vector3d& size);
		int viewAngleToIndex(double viewAngle);

		// semantic objects
		// virtual void get3dFromRaw2d(const vision_msgs::Detection2D& detection, cv::Mat& depthImage); // extract 3d box from 2d, change objectDetections_ directly
		virtual bool get3dFromRaw2dSeg(const vision_msgs::Detection2D& detection, cv::Mat& depthImage, std::vector<Eigen::Vector3d>& objPoints, Eigen::Vector3d& objCenter,  const Eigen::Matrix4d& syncBody2Map); // extract 3d box from 2d by yolo-seg, change objectDetections_ directly
		void drawBoundingBox(const vision_msgs::Detection2DArray& detections, cv::Mat& img);

		// Visualziation
		void visCB(const ros::TimerEvent& );
		void publishProjPoints();
		void publishMap();
		void publishInflatedMap();
		void publish2DOccupancyGrid();
		void publishSemanticBoundingBoxes();
		void publishSemanticObjMsg();
		void publishObjectMap();
		void publishGlobalObjectDenseCloud();

		// helper functions
		double logit(double x);
		bool isInMap(const Eigen::Vector3d& pos);
		bool isInMap(const Eigen::Vector3i& idx);
		void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx);
		void indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos);
		int posToAddress(const Eigen::Vector3d& idx);
		int posToAddress(double x, double y, double z);
		int indexToAddress(const Eigen::Vector3i& idx);
		int indexToAddress(int x, int y, int z);
		void boundIndex(Eigen::Vector3i& idx);
		bool isInLocalUpdateRange(const Eigen::Vector3d& pos);
		bool isInLocalUpdateRange(const Eigen::Vector3i& idx);
		bool isInFreeRegion(const Eigen::Vector3d& pos, const std::pair<Eigen::Vector3d, Eigen::Vector3d>& freeRegion);
		bool isInFreeRegions(const Eigen::Vector3d& pos, const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions);
		bool isInFreeRegions(const Eigen::Vector3d& pos);
		bool isInHistFreeRegions(const Eigen::Vector3d& pos);
		Eigen::Vector3d adjustPointInMap(const Eigen::Vector3d& point);
		Eigen::Vector3d adjustPointRayLength(const Eigen::Vector3d& point);
		int updateOccupancyInfo(const Eigen::Vector3d& point, bool isOccupied);
		void extractPoseMatrix(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& poseMatrix);
		void extractPoseMatrix(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& poseMatrix);
		void updateSensorPose(const geometry_msgs::PoseStampedConstPtr& pose);
		void updateSensorPose(const nav_msgs::OdometryConstPtr& odom);
		void getPosition(Eigen::Vector3d& pos);
		void getLocalUpdateRange(Eigen::Vector3d& localUpdateRange);
		void getVoxelMax(Eigen::Vector3i& voxelMax);
		void getRaycastMaxLength(double& raycastMaxLength);
		void updateObjectViewAngle(mapManager::semanticObject& semanticObject);
		bool isObjectVisible(const Eigen::Vector3d& cameraPos, const semanticObject& semanticObject);
		bool checkVisibility(objectMap& objectMap);
		bool isCollisionBoxFree(const Eigen::Vector3d& pos, const double yaw); // check collision with oriented collision box
		bool isCollisionBoxFree(const Eigen::Vector3d& pos, const double yaw, const double safeDist);
		bool isCollisionBoxOccupied(const Eigen::Vector3d& pos, const double yaw); // check collision with oriented collision box
		bool isCollisionBoxOccupied(const Eigen::Vector3d& pos, const double yaw, const double safeDist);
		bool isCollisionBoxOccupied(const Eigen::Vector3d& pos, const double yaw, const double safeDist, bool debug);
	};
	// inline function
	// user function
	inline bool occMap::isOccupied(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isOccupied(idx);
	}

	inline bool occMap::isOccupied(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)){
			return true;
		}
		int address = this->indexToAddress(idx);
		return this->occupancy_[address] >= this->pOccLog_;
	}

	inline bool occMap::isInflatedOccupied(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isInflatedOccupied(idx);
	}

	inline bool occMap::isInflatedOccupied(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)){
			return true;
		}
		int address = this->indexToAddress(idx);
		return this->occupancyInflated_[address] == true;
	}

	inline bool occMap::isInflatedOccupiedLine(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2){		
		if (this->isInflatedOccupied(pos1) or this->isInflatedOccupied(pos2)){
			return true;
		}
		Eigen::Vector3d diff = pos2 - pos1;
		double dist = diff.norm();
		Eigen::Vector3d diffUnit = diff/dist;
		int stepNum = int(dist/this->mapRes_);
		Eigen::Vector3d pCheck;
		Eigen::Vector3d unitIncrement = diffUnit * this->mapRes_;
		bool isOccupied = false;
		for (int i=1; i<stepNum; ++i){
			pCheck = pos1 + i * unitIncrement;
			isOccupied = this->isInflatedOccupied(pCheck);
			if (isOccupied){
				// std::cout << "isInflatedOccupiedLine: " << pCheck(0) << " " << pCheck(1) << " " << pCheck(2) << std::endl;
				return true;
			}
		}
		return false;
	}


	inline bool occMap::isFree(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isFree(idx);
	}

	inline bool occMap::isFree(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)){
			return false;
		}
		int address = this->indexToAddress(idx);
		return (this->occupancy_[address] < this->pOccLog_) and (this->occupancy_[address] >= this->pMinLog_);
	}

	inline bool occMap::isInflatedFree(const Eigen::Vector3d& pos){
		if (not this->isInflatedOccupied(pos) and not this->isUnknown(pos) and this->isFree(pos)){
			// if (pos(0)>=1.0 and pos(0)<=3.0 and pos(1)>=-2.0 and pos(1) <= 2.0){
			// 	ROS_INFO("inside wrong piont isInflatedFree");
			// 	std::cout << pos(0) << " " << pos(1) << " " << pos(2) << std::endl;
			// 	std::cout << this->isInflatedOccupied(pos) << " " << this->isUnknown(pos) << " " << this->isFree(pos) << std::endl;
			// }
			return true;
		}
		else{
			return false;
		}
	}

	inline bool occMap::isInflatedFreeLine(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2){
		if (not this->isInflatedFree(pos1) or not this->isInflatedFree(pos2)){
			return false;
		}

		Eigen::Vector3d diff = pos2 - pos1;
		double dist = diff.norm();
		Eigen::Vector3d diffUnit = diff/dist;
		int stepNum = int(dist/this->mapRes_);
		Eigen::Vector3d pCheck;
		Eigen::Vector3d unitIncrement = diffUnit * this->mapRes_;
		bool isFree = true;
		for (int i=1; i<stepNum; ++i){
			pCheck = pos1 + i * unitIncrement;
			isFree = this->isInflatedFree(pCheck);
			if (not isFree){
				return false;
			}
		}
		return true;
	}


	inline bool occMap::isUnknown(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isUnknown(idx);
	}

	inline bool occMap::isUnknown(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)){
			return true;
		}
		int address = this->indexToAddress(idx);
		return this->occupancy_[address] < this->pMinLog_;		
	}

	inline bool occMap::isUnknown(const int address){
		if (address < 0 or address >= int(this->occupancy_.size())){
			return true;
		}
		return this->occupancy_[address] < this->pMinLog_;
	
	}

	inline void occMap::setFree(const Eigen::Vector3d& pos){
		if (not this->isInMap(pos)) return;
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		this->setFree(idx);
	}

	inline void occMap::setFree(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)) return;
		int address = this->indexToAddress(idx);
		this->occupancy_[address] = this->pMinLog_;

		// also set inflated map to free
		int xInflateSize = ceil(this->robotSize_(0)/(2*this->mapRes_));
		int yInflateSize = ceil(this->robotSize_(1)/(2*this->mapRes_));
		int zInflateSize = ceil(this->robotSize_(2)/(2*this->mapRes_));
		Eigen::Vector3i inflateIndex;
		int inflateAddress;
		const int maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		for (int ix=-xInflateSize; ix<=xInflateSize; ++ix){
			for (int iy=-yInflateSize; iy<=yInflateSize; ++iy){
				for (int iz=-zInflateSize; iz<=zInflateSize; ++iz){
					inflateIndex(0) = idx(0) + ix;
					inflateIndex(1) = idx(1) + iy;
					inflateIndex(2) = idx(2) + iz;
					inflateAddress = this->indexToAddress(inflateIndex);
					if ((inflateAddress < 0) or (inflateAddress > maxIndex)){
						continue; // those points are not in the reserved map
					} 
					this->occupancyInflated_[inflateAddress] = false;
				}
			}
		}
	}

	inline void occMap::freeRegion(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2){
		Eigen::Vector3i idx1, idx2;
		this->posToIndex(pos1, idx1);
		this->posToIndex(pos2, idx2);
		this->boundIndex(idx1);
		this->boundIndex(idx2);
		for (int xID=idx1(0); xID<=idx2(0); ++xID){
			for (int yID=idx1(1); yID<=idx2(1); ++yID){
				for (int zID=idx1(2); zID<=idx2(2); ++zID){
					this->setFree(Eigen::Vector3i (xID, yID, zID));
				}	
			}
		}
	}

	inline void occMap::freeRegions(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions){
		for (std::pair<Eigen::Vector3d, Eigen::Vector3d> freeRegion : freeRegions){
			this->freeRegion(freeRegion.first, freeRegion.second);
		}
	}

	inline void occMap::freeHistRegions(){
		for (std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions : this->histFreeRegions_){
			this->freeRegions(freeRegions);
		}
	}

	inline void occMap::updateFreeRegions(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions){
		this->freeRegions_ = freeRegions;
		if (this->histFreeRegions_.size() <= 30){
			this->histFreeRegions_.push_back(freeRegions);
		}
		else{
			this->histFreeRegions_.pop_front();
			this->histFreeRegions_.push_back(freeRegions);
		}


		if (this->histFreeRegions_.size() != 0){
			this->useFreeRegions_ = true;
		}
		else{
			this->useFreeRegions_ = false;
		}
	}


	inline double occMap::getRes(){
		return this->mapRes_;
	}

	inline void occMap::getMapRange(Eigen::Vector3d& mapSizeMin, Eigen::Vector3d& mapSizeMax){
		mapSizeMin = this->mapSizeMin_;
		mapSizeMax = this->mapSizeMax_;
	}

	inline void occMap::getMapVoxelRange(Eigen::Vector3i& mapVoxelSizeMax){
		mapVoxelSizeMax = this->mapVoxelMax_;
	}

	inline void occMap::getCurrMapRange(Eigen::Vector3d& currRangeMin, Eigen::Vector3d& currRangeMax){
		currRangeMin = this->currMapRangeMin_;
		currRangeMax = this->currMapRangeMax_;
	}

	inline bool occMap::castRay(const Eigen::Vector3d& start, const Eigen::Vector3d& direction, Eigen::Vector3d& end, double maxLength, bool ignoreUnknown){
		// return true if raycasting successfully find the endpoint, otherwise return false

		Eigen::Vector3d directionNormalized = direction/direction.norm(); // normalize the direction vector
		int num = ceil(maxLength/this->mapRes_);
		for (int i=1; i<num; ++i){
			Eigen::Vector3d point = this->mapRes_ * directionNormalized * i + start;
			if (ignoreUnknown){
				if (this->isInflatedOccupied(point)){
					end = point;
					return true;
				}
			}
			else{
				if (this->isInflatedOccupied(point) or this->isUnknown(point)){
					end = point;
					return true;
				}	
			}
		}
		end = start;
		return false;
	}
	// end of user functinos

	// helper functions
	inline double occMap::logit(double x){
		return log(x/(1-x));
	}

	inline bool occMap::isInMap(const Eigen::Vector3d& pos){
		if ((pos(0) >= this->mapSizeMin_(0)) and (pos(0) <= this->mapSizeMax_(0)) and 
			(pos(1) >= this->mapSizeMin_(1)) and (pos(1) <= this->mapSizeMax_(1)) and 
			(pos(2) >= this->mapSizeMin_(2)) and (pos(2) <= this->mapSizeMax_(2))){
			return true;
		}
		else{
			return false;
		}
	}

	inline bool occMap::isInMap(const Eigen::Vector3i& idx){
		if ((idx(0) >= this->mapVoxelMin_(0)) and (idx(0) < this->mapVoxelMax_(0)) and
		    (idx(1) >= this->mapVoxelMin_(1)) and (idx(1) < this->mapVoxelMax_(1)) and 
		    (idx(2) >= this->mapVoxelMin_(2)) and (idx(2) < this->mapVoxelMax_(2))){
			return true;
		}
		else{
			return false;
		}
	}

	inline void occMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx){
		idx(0) = floor( (pos(0) - this->mapSizeMin_(0) ) / this->mapRes_ );
		idx(1) = floor( (pos(1) - this->mapSizeMin_(1) ) / this->mapRes_ );
		idx(2) = floor( (pos(2) - this->mapSizeMin_(2) ) / this->mapRes_ );
	}

	inline void occMap::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos){
		pos(0) = (idx(0) + 0.5) * this->mapRes_ + this->mapSizeMin_(0); 
		pos(1) = (idx(1) + 0.5) * this->mapRes_ + this->mapSizeMin_(1);
		pos(2) = (idx(2) + 0.5) * this->mapRes_ + this->mapSizeMin_(2);
	}

	inline int occMap::posToAddress(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->indexToAddress(idx);
	}

	inline int occMap::posToAddress(double x, double y, double z){
		Eigen::Vector3d pos (x, y, z);
		return this->posToAddress(pos);
	}

	inline int occMap::indexToAddress(const Eigen::Vector3i& idx){
		return idx(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2) + idx(1) * this->mapVoxelMax_(2) + idx(2);
	}

	inline int occMap::indexToAddress(int x, int y, int z){
		Eigen::Vector3i idx (x, y, z);
		return this->indexToAddress(idx);
	}

	inline void occMap::boundIndex(Eigen::Vector3i& idx){
		Eigen::Vector3i temp;
		temp(0) = std::max(std::min(idx(0), this->mapVoxelMax_(0)-1), this->mapVoxelMin_(0));
		temp(1) = std::max(std::min(idx(1), this->mapVoxelMax_(1)-1), this->mapVoxelMin_(1));
		temp(2) = std::max(std::min(idx(2), this->mapVoxelMax_(2)-1), this->mapVoxelMin_(2));
		idx = temp;
	}

	inline bool occMap::isInLocalUpdateRange(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isInLocalUpdateRange(idx);
	}

	inline bool occMap::isInLocalUpdateRange(const Eigen::Vector3i& idx){
		Eigen::Vector3d rangeMin = this->bodyPosition_ - this->localUpdateRange_;
		Eigen::Vector3d rangeMax = this->bodyPosition_ + this->localUpdateRange_;
		
		Eigen::Vector3i rangeMinIdx, rangeMaxIdx;
		this->posToIndex(rangeMin, rangeMinIdx);
		this->posToIndex(rangeMax, rangeMaxIdx);

		this->boundIndex(rangeMinIdx);
		this->boundIndex(rangeMaxIdx);
		// std::cout << "rangeMinIdx: " << rangeMinIdx.transpose() << std::endl;
		// std::cout << "rangeMaxIdx: " << rangeMaxIdx.transpose() << std::endl;
		bool inRange = (idx(0) >= rangeMinIdx(0)) and (idx(0) <= rangeMaxIdx(0)) and
					   (idx(1) >= rangeMinIdx(1)) and (idx(1) <= rangeMaxIdx(1)) and
					   (idx(2) >= rangeMinIdx(2)) and (idx(2) <= rangeMaxIdx(2));
		return inRange;
	}

	inline bool occMap::isInFreeRegion(const Eigen::Vector3d& pos, const std::pair<Eigen::Vector3d, Eigen::Vector3d>& freeRegion){
		Eigen::Vector3d lowerBound = freeRegion.first;
		Eigen::Vector3d upperBound = freeRegion.second;
		if (pos(0) >= lowerBound(0) and pos(0) <= upperBound(0) and
			pos(1) >= lowerBound(1) and pos(1) <= upperBound(1) and
			pos(2) >= lowerBound(2) and pos(2) <= upperBound(2)){
			return true;
		}
		else{
			return false;
		}
	}


	inline bool occMap::isInFreeRegions(const Eigen::Vector3d& pos, const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions){
		for (std::pair<Eigen::Vector3d, Eigen::Vector3d> freeRegion : freeRegions){
			if (this->isInFreeRegion(pos, freeRegion)){
				return true;
			}
		}
		return false;
	}

	inline bool occMap::isInFreeRegions(const Eigen::Vector3d& pos){
		return this->isInFreeRegions(pos, this->freeRegions_);
	}

	inline bool occMap::isInHistFreeRegions(const Eigen::Vector3d& pos){
		for (std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions : this->histFreeRegions_){
			if (this->isInFreeRegions(pos, freeRegions)){
				return true;
			}
		}
		return false;
	}


	inline Eigen::Vector3d occMap::adjustPointInMap(const Eigen::Vector3d& point){
		Eigen::Vector3d pos = this->bodyPosition_;
		Eigen::Vector3d diff = point - pos;
		Eigen::Vector3d offsetMin = this->mapSizeMin_ - pos;
		Eigen::Vector3d offsetMax = this->mapSizeMax_ - pos;

		double minRatio = std::numeric_limits<double>::max();
		for (int i=0; i<3; ++i){ // each axis
			if (diff[i] != 0){
				double ratio1 = offsetMin[i]/diff[i];
				double ratio2 = offsetMax[i]/diff[i];
				if ((ratio1 > 0) and (ratio1 < minRatio)){
					minRatio = ratio1;
				}

				if ((ratio2 > 0) and (ratio2 < minRatio)){
					minRatio = ratio2;
				}
			}
		}

		return pos + (minRatio - 1e-3) * diff;
	}


	inline Eigen::Vector3d occMap::adjustPointRayLength(const Eigen::Vector3d& point){
		double length = (point - this->rangeSensorPosition_).norm();
		return 0.9*(point - this->rangeSensorPosition_) * (this->raycastMaxLength_/length) + this->rangeSensorPosition_;
	}

	inline int occMap::updateOccupancyInfo(const Eigen::Vector3d& point, bool isOccupied){
		Eigen::Vector3i idx;
		this->posToIndex(point, idx);
		int voxelID = this->indexToAddress(idx);
		this->countHitMiss_[voxelID] += 1;
		if (this->countHitMiss_[voxelID] == 1){
			this->updateVoxelCache_.push(idx);
		}
		if (isOccupied){ // if not adjusted set it to occupied, otherwise it is free
			this->countHit_[voxelID] += 1;
		}

		// this->isVisited_[voxelID] = true;
		return voxelID;
	}

	inline void occMap::extractPoseMatrix(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& poseMatrix){
		Eigen::Quaterniond quat;
		quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
		Eigen::Matrix3d rot = quat.toRotationMatrix();

		poseMatrix.setZero();
		poseMatrix.block<3, 3>(0, 0) = rot;
		poseMatrix(0, 3) = pose->pose.position.x; 
		poseMatrix(1, 3) = pose->pose.position.y;
		poseMatrix(2, 3) = pose->pose.position.z;
		poseMatrix(3, 3) = 1.0;
	}

	inline void occMap::extractPoseMatrix(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& poseMatrix){
		Eigen::Quaterniond quat;
		quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
		Eigen::Matrix3d rot = quat.toRotationMatrix();

		poseMatrix.setZero();
		poseMatrix.block<3, 3>(0, 0) = rot;
		poseMatrix(0, 3) = odom->pose.pose.position.x; 
		poseMatrix(1, 3) = odom->pose.pose.position.y;
		poseMatrix(2, 3) = odom->pose.pose.position.z;
		poseMatrix(3, 3) = 1.0;
	}

	inline void occMap::updateSensorPose(const geometry_msgs::PoseStampedConstPtr& pose){
		this->extractPoseMatrix(pose, this->map2Body_);
		this->bodyPosition_ = this->map2Body_.block<3, 1>(0, 3);
		this->bodyOrientation_ = this->map2Body_.block<3, 3>(0, 0);

		// get range and color sensor position
		this->sensorManager_->rangeSensorModel_->sensor2GlobalFrame(Eigen::Vector3d(0,0,0), this->rangeSensorPosition_, this->map2Body_);
		this->sensorManager_->colorSensorModel_->sensor2GlobalFrame(Eigen::Vector3d(0,0,0), this->colorSensorPosition_, this->map2Body_);
	}

	inline void occMap::updateSensorPose(const nav_msgs::OdometryConstPtr& odom){
		this->extractPoseMatrix(odom, this->map2Body_);
		this->bodyPosition_ = this->map2Body_.block<3, 1>(0, 3);
		this->bodyOrientation_ = this->map2Body_.block<3, 3>(0, 0);

		// get range and color sensor position
		this->sensorManager_->rangeSensorModel_->sensor2GlobalFrame(Eigen::Vector3d(0,0,0), this->rangeSensorPosition_, this->map2Body_);
		this->sensorManager_->colorSensorModel_->sensor2GlobalFrame(Eigen::Vector3d(0,0,0), this->colorSensorPosition_, this->map2Body_);
	}

	inline void occMap::getPosition(Eigen::Vector3d& pos){
		pos = this->bodyPosition_;
	}

	inline void occMap::getLocalUpdateRange(Eigen::Vector3d& localUpdateRange){
		localUpdateRange = this->localUpdateRange_;
	}

	inline void occMap::getVoxelMax(Eigen::Vector3i& voxelMax){
		voxelMax = this->mapVoxelMax_;
	}

	inline void occMap::getRaycastMaxLength(double& raycastMaxLength){
		raycastMaxLength = this->raycastMaxLength_;
	}

	inline void occMap::getSemanticObjects(std::vector<semanticObject>& objects){
		objects = this->semanticObjects_;
	}
	
	inline bool occMap::isCollisionInSafeRegion(const Eigen::Vector3d& pos, const Eigen::Vector3d& size){
		Eigen::Vector3d lowerBound = pos - size/2;
		Eigen::Vector3d upperBound = pos + size/2;
		for (double x=lowerBound(0); x<=upperBound(0); x+=this->mapRes_){
			for (double y=lowerBound(1); y<=upperBound(1); y+=this->mapRes_){
				for (double z=lowerBound(2); z<=upperBound(2); z+=this->mapRes_){
					if (this->isInflatedOccupied(Eigen::Vector3d(x, y, z))){
						return false;
					}
				}
			}
		}
		return true;
	}
	
	// viewAngle in [-pi, pi], from the object to the view point
	inline int occMap::viewAngleToIndex(double viewAngle){
		if (viewAngle<0){
			viewAngle += 2*M_PI;
		}
		int idx = int((viewAngle)/this->viewAgRes_);
		double mod = fmod(viewAngle, this->viewAgRes_);
		idx = mod>(this->viewAgRes_/2)?idx+1:idx;
		if (idx == this->viewAgIntervals_){
			idx = 0;
		}
		return idx;
	}
}

#endif