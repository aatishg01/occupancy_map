/*
	FILE: occupancyMap.cpp
	--------------------------------------
	function definition of occupancy map
*/
#include <map_manager/multiRoboOccupancyMap.h>

namespace mapManager{
	multiRoboOccMap::multiRoboOccMap(){
		this->ns_ = "multi_robo_occupancy_map";
		this->hint_ = "[multiRoboOccMap]";
	}

	multiRoboOccMap::multiRoboOccMap(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "multi_robo_occupancy_map";
		this->hint_ = "[multiRoboOccMap]";
		this->initMultiRoboParam();
		this->initParam();
		this->initPrebuiltMap();
		this->registerPub();
		this->registerMultiRoboPub();
		this->registerMultiRoboCallback();
		this->initCollisionFreeBox();
	}

	void multiRoboOccMap::initMultiRoboOccMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initMultiRoboParam();
		this->initParam();
		this->initPrebuiltMap();
		this->registerPub();
		this->registerMultiRoboPub();
		this->registerMultiRoboCallback();
		this->initCollisionFreeBox();
	}

	void multiRoboOccMap::initMultiRoboParam(){
		this->uavSize_ << 0.1, 0.1, 0.1;
		std::cout <<"init multi robot occ ns: " << this->ns_ << std::endl;
		// robot num
		if (not this->nh_.getParam(this->ns_ + "/robot_num", this->robot_num_)){
			this->robot_num_ = 0;
			cout << this->hint_ << ": No robot num option. Use default: 0" << endl;
		}
		else{
			cout << this->hint_ << ": robot num: " << this->robot_num_ << endl;
		}	
		this->allRobotsReady_ = false;

		// robot id
		if (not this->nh_.getParam(this->ns_ + "/robot_id", this->robot_id_)){
			this->robot_id_ = 0;
			cout << this->hint_ << ": No robot id option. Use default: 0" << endl;
		}
		else{
			cout << this->hint_ << ": robot id: " << this->robot_id_ << endl;
		}	
		this->sharedVoxels_.from_id = this->robot_id_;

		// robot name
		if (not this->nh_.getParam(this->ns_ + "/robot_name", this->robotName_)){
			this->robotName_ = "robot";
			cout << this->hint_ << ": No robot name option. Use default: robot" << endl;
		}
		else{
			cout << this->hint_ << ": robot name: " << this->robotName_ << endl;
		}

		// raw object detection topic
		if (not this->nh_.getParam(this->ns_ + "/raw_detection_topic", this->rawDetectionTopicName_)){
			this->rawDetectionTopicName_ = "/bbox3d";
			cout << this->hint_ << ": No bbox topic option. Use default: /bbox3d" << endl;
		}
		else{
			cout << this->hint_ << ": bbox topic: " << this->rawDetectionTopicName_ << endl;
		}

		// transform matrix: map to global(for multi-robot map transmission)
		std::vector<double> global2MapVec (16);
		if (not this->nh_.getParam(this->ns_ + "/global_to_map_" + std::to_string(this->robot_id_), global2MapVec)){
			ROS_ERROR("[multiRoboOccMap]: Please check global to map matrix!");
		}
		else{
			for (int i=0; i<4; ++i){
				for (int j=0; j<4; ++j){
					this->global2Map_(i, j) = global2MapVec[i * 4 + j];
				}
			}
			cout << this->hint_ << ": from global to map: " << endl;
			cout << this->global2Map_ << endl;
		}
		int reservedSize = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		// this->uavOccupancyInflated_.resize(reservedSize, false);
		// std::cout << this->hint_ << ": reserved size: " << reservedSize << std::endl;
	}

	void multiRoboOccMap::registerMultiRoboCallback(){
		if (this->rangeSensorInputMode_ == 0){
			ROS_INFO("[multiRoboOccMap]: Use depth image as input.");
			// depth callback
			this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
			if (this->localizationMode_ == 0){
				this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
				this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_, *this->poseSub_));
				this->depthPoseSync_->registerCallback(boost::bind(&multiRoboOccMap::depthPoseCB, this, _1, _2));
			}
			else if (this->localizationMode_ == 1){
				ROS_INFO("[multiRoboOccMap]: Use odometry as localization.");
				this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
				this->depthOdomSync_.reset(new message_filters::Synchronizer<depthOdomSync>(depthOdomSync(100), *this->depthSub_, *this->odomSub_));
				this->depthOdomSync_->registerCallback(boost::bind(&multiRoboOccMap::depthOdomCB, this, _1, _2));
			}
			else{
				ROS_ERROR("[multiRoboOccMap]: Invalid localization mode!");
				exit(0);
			}
		}
		else if (this->rangeSensorInputMode_ == 1){
			// pointcloud callback
			this->pointcloudSub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(this->nh_, this->pointcloudTopicName_, 50));
			if (this->localizationMode_ == 0){
				this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
				this->pointcloudPoseSync_.reset(new message_filters::Synchronizer<pointcloudPoseSync>(pointcloudPoseSync(100), *this->pointcloudSub_, *this->poseSub_));
				this->pointcloudPoseSync_->registerCallback(boost::bind(&multiRoboOccMap::pointcloudPoseCB, this, _1, _2));
			}
			else if (this->localizationMode_ == 1){
				this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
				this->pointcloudOdomSync_.reset(new message_filters::Synchronizer<pointcloudOdomSync>(pointcloudOdomSync(100), *this->pointcloudSub_, *this->odomSub_));
				this->pointcloudOdomSync_->registerCallback(boost::bind(&multiRoboOccMap::pointcloudOdomCB, this, _1, _2));
			}
			else{
				ROS_ERROR("[multiRoboOccMap]: Invalid localization mode!");
				exit(0);
			}
		}
		else{
			ROS_ERROR("[multiRoboOccMap]: Invalid sensor input mode!");
			exit(0);
		}

		// semantic map: sync semantic and sensors
		if (this->sensorFusionMode_ == 0){
			this->depthSyncSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
		}
		else{
			this->depthSyncSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, "/registered_depth_image", 50));
		}
		this->odomSyncSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
		this->detection2DSyncSub_.reset(new message_filters::Subscriber<vision_msgs::Detection2DArray>(this->nh_, this->rawDetectionTopicName_, 100));
		this->depthDetectionOdomSync_.reset(new message_filters::Synchronizer<depthDetectionOdomSyncPolicy>(depthDetectionOdomSyncPolicy(100), *this->depthSyncSub_, *this->detection2DSyncSub_, *this->odomSyncSub_));
		this->depthDetectionOdomSync_->registerCallback(boost::bind(&occMap::depthDetectionOdomCB, this, _1, _2, _3));

		// occupancy update callback
		this->occTimer_ = this->nh_.createTimer(ros::Duration(0.1), &multiRoboOccMap::updateOccupancyCB, this);

		// map inflation callback
		this->inflateTimer_ = this->nh_.createTimer(ros::Duration(0.1), &occMap::inflateMapCB, (occMap*)this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &occMap::visCB, (occMap*)this);

		// // visualization callback
		// this->mapSharedPubTimer_ = this->nh_.createTimer(ros::Duration(0.1), &multiRoboOccMap::mapSharedPubCB, this);

		// // subscrived map shared by other robots
		// this->mapSharedSub_ = this->nh_.subscribe("/shared_map", 10, &multiRoboOccMap::mapSharedSubCB, this);
		
		// // subscribe all robots states in the robot network
		// this->robotStatesSub_ = this->nh_.subscribe("/robot_states", 10, &multiRoboOccMap::robotStatesSubCB, this);

		// for semantic map: object map update callback
		this->objectMapTimer_ = this->nh_.createTimer(ros::Duration(this->objectMapDuration_), &occMap::updateObjectMapCB, (occMap*)this);
		this->registeredMapSub_ = this->nh_.subscribe("/cloud_registered", 1, &occMap::denseCloudCB, (occMap*)this);
		// for semantic map: publish the semantic object estimation
		// this->semanticObjPubTimer_ = this->nh_.createTimer(ros::Duration(0.05), &multiRoboOccMap::semanticObjPubCB, this);

		// for semantic map: subscribe the semantic object estimation
		// this->semanticObjSub_ = this->nh_.subscribe("/semantic_objects", 10, &multiRoboOccMap::semanticObjSubCB, this);

		if (this->robotName_ == "uav"){
			ROS_INFO("[OccMap]: UAV robot.");
			// subscribe global viewpoints
			// this->globalViewpointsSub_ = this->nh_.subscribe("/global_viewpoints", 1, &multiRoboOccMap::globalViewpointsCB, this);
			// for semantic map: subscribe the raw object bounding boxes detection in vision_msgs/Detection2DArray
			// this->rawObjDetectSub_ = this->nh_.subscribe(this->rawDetectionTopicName_, 10, &multiRoboOccMap::rawObjDetectSubCB, this);

			// realworld
			// this->depthSyncSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
			// if (this->localizationMode_== 1){
			// 	this->odomSyncSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
			// 	this->detection2DSyncSub_.reset(new message_filters::Subscriber<vision_msgs::Detection2DArray>(this->nh_, this->rawDetectionTopicName_, 100));
			// 	this->depthDetectionOdomSync_.reset(new message_filters::Synchronizer<depthDetectionOdomSyncPolicy>(depthDetectionOdomSyncPolicy(100), *this->depthSyncSub_, *this->detection2DSyncSub_, *this->odomSyncSub_));
			// 	this->depthDetectionOdomSync_->registerCallback(boost::bind(&multiRoboOccMap::depthDetectionOdomCB, this, _1, _2, _3));
			// }
			// else if (this->localizationMode_ == 0){
			// 	this->poseSyncSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
			// 	this->detection2DSyncSub_.reset(new message_filters::Subscriber<vision_msgs::Detection2DArray>(this->nh_, this->rawDetectionTopicName_, 100));
			// 	this->depthDetectionPoseSync_.reset(new message_filters::Synchronizer<depthDetectionPoseSyncPolicy>(depthDetectionPoseSyncPolicy(100), *this->depthSyncSub_, *this->detection2DSyncSub_, *this->poseSyncSub_));
			// 	this->depthDetectionPoseSync_->registerCallback(boost::bind(&multiRoboOccMap::depthDetectionPoseCB, this, _1, _2, _3));
			// }
		}
		else if (this->robotName_ == "ugv" or this->robotName_ == "spot"){
			ROS_INFO("[OccMap]: %s robot.", this->robotName_.c_str());
			// for semantic map: subscribe the raw object bounding boxes detection in vision_msgs/Detection3DArray
			// this->rawObjDetectSub_ = this->nh_.subscribe(this->rawDetectionTopicName_, 10, &multiRoboOccMap::rawObjDetectSubCB, this);
		}
		else{
			ROS_ERROR("[OccMap]: Invalid robot name!");
			exit(0);
		}
		
	}

	void multiRoboOccMap::depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose){
		// store current depth image
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
		if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
		}
		imgPtr->image.copyTo(this->depthImage_);

		// store current position and orientation (camera)
		this->updateSensorPose(pose);

		if (this->isInMap(this->rangeSensorPosition_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}

	void multiRoboOccMap::depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom){
		// store current depth image
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
		if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
		}
		imgPtr->image.copyTo(this->depthImage_);

		// store current position and orientation (camera)
		this->updateSensorPose(odom);

		if (this->isInMap(this->rangeSensorPosition_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}

		// std::shared_ptr<cv::Mat> depthImgPtr = std::make_shared<cv::Mat>(this->depthImage_.clone());
		// this->chachedDepthImages_.push_back(depthImgPtr);
		// this->chachedPositions_.push_back(this->rangeSensorPosition_);
		// this->cachedOrientations_.push_back(this->orientation_);
		// this->cachedTimestamps_.push_back(img->header.stamp);
		// // assert(img->header.stamp == odom->header.stamp);
		// if (this->chachedDepthImages_.size() > 30){
		// 	this->chachedDepthImages_.pop_front();
		// 	this->chachedPositions_.pop_front();
		// 	this->cachedOrientations_.pop_front();
		// 	this->cachedTimestamps_.pop_front();
		// }
		// for (size_t i=0; i<this->chachedDepthImages_.size(); ++i){
		// 	ROS_INFO("cached position: %f, %f, %f", this->chachedPositions_[i](0), this->chachedPositions_[i](1), this->chachedPositions_[i](2));
		// 	ROS_INFO("cached timestamp: %f", this->cachedTimestamps_[i].toSec());
		// }
	}

	void multiRoboOccMap::pointcloudPoseCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const geometry_msgs::PoseStampedConstPtr& pose){
		// directly get the point cloud
		pcl::PCLPointCloud2 pclPC2;
		pcl_conversions::toPCL(*pointcloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
		pcl::fromPCLPointCloud2(pclPC2, this->pointcloud_);

		// store current position and orientation (camera)
		this->updateSensorPose(pose);

		if (this->isInMap(this->rangeSensorPosition_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}

	void multiRoboOccMap::pointcloudOdomCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const nav_msgs::OdometryConstPtr& odom){
		// directly get the point cloud
		pcl::PCLPointCloud2 pclPC2;
		pcl_conversions::toPCL(*pointcloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
		pcl::fromPCLPointCloud2(pclPC2, this->pointcloud_);
		// store current position and orientation (camera)
		this->updateSensorPose(odom);
		
		if (this->isInMap(this->rangeSensorPosition_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}

	// void multiRoboOccMap::depthDetectionOdomCB(const sensor_msgs::ImageConstPtr& img, const vision_msgs::Detection2DArrayConstPtr& objectDetections, const nav_msgs::OdometryConstPtr& odom){
	// 	// ROS_INFO("depthDetectionOdomCB");
	// 	// store current depth image
	// 	cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
	// 	if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
	// 		(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
	// 	}
	// 	imgPtr->image.copyTo(this->syncDepthImage_);

	// 	// store current position and orientation (camera)
	// 	this->updateSensorPose(odom);

	// 	this->syncPosition_(0) = camPoseMatrix(0, 3);
	// 	this->syncPosition_(1) = camPoseMatrix(1, 3);
	// 	this->syncPosition_(2) = camPoseMatrix(2, 3);
	// 	this->syncOrientation_ = camPoseMatrix.block<3, 3>(0, 0);

	// 	// ROS_INFO("2D detection recieved.");
	// 	this->objectDetections_.clear();
	// 	for (const auto& detection : objectDetections->detections){
	// 		this->get3dFromRaw2d(detection, this->syncDepthImage_);
	// 		// publish image with bounding box
	// 		if (this->depthImage_.empty()){
	// 			ROS_ERROR("Depth image is empty. Cannot visualize detection.");
	// 			return;
	// 		}
	// 	}
	// 	this->drawBoundingBox(*objectDetections, this->syncDepthImage_);

	// 	std::vector<vision_msgs::Detection3D> filteredDetections;
	// 	this->filterBoundingBox(this->objectDetections_, filteredDetections);
	// 	this->objectDetections_ = filteredDetections;

	// 	this->estimateSemanticObj();

	// 	// publish semantic objects estimation
	// 	map_manager::semanticObjArrayMsg semanticObjArray;
	// 	for (const auto& object : this->semanticObjects_){
	// 		// publish semantic object
	// 		map_manager::semanticObjMsg semanticObj;
	// 		semanticObj.label_id = object.label_id;
	// 		semanticObj.position = {object.position(0), object.position(1), object.position(2)};
	// 		semanticObj.size = {object.size(0), object.size(1), object.size(2)};
	// 		semanticObj.view_angles = object.view_angles;
	// 		semanticObjArray.semanticObjs.push_back(semanticObj);
	// 	}

	// }

	// void multiRoboOccMap::depthDetectionPoseCB(const sensor_msgs::ImageConstPtr& img, const vision_msgs::Detection2DArrayConstPtr& objectDetections, const geometry_msgs::PoseStampedConstPtr& pose){
	// 	// ROS_INFO("depthDetectionPoseCB");
	// 	// store current depth image
	// 	cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
	// 	if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
	// 		(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
	// 	}
	// 	imgPtr->image.copyTo(this->syncDepthImage_);

	// 	// store current position and orientation (camera)
	// 	this->updateSensorPose(pose);

	// 	this->syncPosition_(0) = camPoseMatrix(0, 3);
	// 	this->syncPosition_(1) = camPoseMatrix(1, 3);
	// 	this->syncPosition_(2) = camPoseMatrix(2, 3);
	// 	this->syncOrientation_ = camPoseMatrix.block<3, 3>(0, 0);

	// 	// ROS_INFO("2D detection recieved.");
	// 	this->objectDetections_.clear();
	// 	for (const auto& detection : objectDetections->detections){
	// 		this->get3dFromRaw2d(detection, this->syncDepthImage_);
	// 		// publish image with bounding box
	// 		if (this->depthImage_.empty()){
	// 			ROS_ERROR("Depth image is empty. Cannot visualize detection.");
	// 			return;
	// 		}
	// 	}
	// 	this->drawBoundingBox(*objectDetections, this->syncDepthImage_);

	// 	std::vector<vision_msgs::Detection3D> filteredDetections;
	// 	this->filterBoundingBox(this->objectDetections_, filteredDetections);
	// 	this->objectDetections_ = filteredDetections;

	// 	this->estimateSemanticObj();

	// 	// publish semantic objects estimation
	// 	map_manager::semanticObjArrayMsg semanticObjArray;
	// 	for (const auto& object : this->semanticObjects_){
	// 		// publish semantic object
	// 		map_manager::semanticObjMsg semanticObj;
	// 		semanticObj.label_id = object.label_id;
	// 		semanticObj.position = {object.position(0), object.position(1), object.position(2)};
	// 		semanticObj.size = {object.size(0), object.size(1), object.size(2)};
	// 		semanticObjArray.semanticObjs.push_back(semanticObj);
	// 	}
	// }

	void multiRoboOccMap::registerMultiRoboPub(){
		
		// map_manager::sharedVoxels test;
		this->mapSharedPub_ = this->nh_.advertise<map_manager::sharedVoxels>("/shared_map", 1);

		// mapSharedVoxels visualization
		this->mapSharedVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("/shared_map_vis", 10);

		// publish current robot's states to the robot network
		this->robotStatesPub_ = this->nh_.advertise<map_manager::robotStates>("/robot_states", 10);

		// publish semantic objects
		// this->semanticObjPub_ = this->nh_.advertise<map_manager::semanticObjArrayMsg>("/semantic_objects", 10);

		// publish global viewpoints
		if (this->robotName_ == "ugv"){
			// this->globalViewpointsPub_ = this->nh_.advertise<map_manager::viewpointArray>("/global_viewpoints", 10);
		}
	}

	void multiRoboOccMap::mapSharedSubCB(const map_manager::sharedVoxelsConstPtr& incomeVoxels){
		// // tell others current robot is ready
		// map_manager::robotStates robotStatesMsg;
		// robotStatesMsg.robot_id = this->robot_id_;
		// robotStatesMsg.ready = true;
		// this->robotStatesPub_.publish(robotStatesMsg);
		
		// Do not merge map published by self
		if (this->robot_id_ == incomeVoxels->from_id){
			return;
		}

		for (size_t i=0 ; i<incomeVoxels->occupancy.size() ; ++i){
			// occupied voxel near by robot itself will not be shared
			// ROS_INFO("income voxel: %f, %f, %f", incomeVoxels->positions[i].x, incomeVoxels->positions[i].y, incomeVoxels->positions[i].z);
			if (this->isNearbyRobot(Eigen::Vector3d(incomeVoxels->positions[i].x, 
													incomeVoxels->positions[i].y, 
													incomeVoxels->positions[i].z))){
				continue;
			}
			// ROS_INFO("IN mapSharedSubCB");
			// update occ map according to income map
			int address = this->posToAddress(incomeVoxels->positions[i].x, 
										 incomeVoxels->positions[i].y, 
										 incomeVoxels->positions[i].z);
			if (incomeVoxels->occupancy[i]){
				this->occupancy_[address] = this->pMaxLog_; 
			}
			else{
				this->occupancy_[address] = (this->pMinLog_+this->pMaxLog_)/2.0;
			}
		}
		// ROS_INFO("recived map size %d", incomeVoxels->occupancy.size());
	}

	void multiRoboOccMap::mapSharedPubCB(const ros::TimerEvent& ){
		// tell others current robot is ready
		map_manager::robotStates robotStatesMsg;
		robotStatesMsg.robot_id = this->robot_id_;
		robotStatesMsg.ready = true;
		this->robotStatesPub_.publish(robotStatesMsg);
		
		// do no publish until all others are ready to subscribe new map, so we can delete prevoius shared voxels
		if (this->allRobotsReady_){
			this->mapSharedPub_.publish(this->sharedVoxels_);
			// ROS_INFO("publish map size %d", this->sharedVoxels_.occupancy.size());
			this->sharedVoxels_.occupancy.clear();
			this->sharedVoxels_.positions.clear();
		}
	}

	void multiRoboOccMap::robotStatesSubCB(const map_manager::robotStatesConstPtr& states){
		if (states->ready){
			// push robot id if it is a new robot
			if (this->readyRobotsID_.empty()){
				this->readyRobotsID_.push_back(states->robot_id);
			}
			else if (std::find(this->readyRobotsID_.begin(), this->readyRobotsID_.end(), states->robot_id)==this->readyRobotsID_.end() 
						  and this->readyRobotsID_.back()!=states->robot_id){
				this->readyRobotsID_.push_back(states->robot_id);
			}
			if (this->readyRobotsID_.size() == this->robot_num_){
				this->allRobotsReady_ = true;
			}
		}
	}

	// void multiRoboOccMap::semanticObjPubCB(const ros::TimerEvent& ){
	// 	map_manager::semanticObjArrayMsg semanticObjArray;
	// 	for (const auto& object : this->semanticObjects_){
	// 		// publish semantic object
	// 		map_manager::semanticObjMsg semanticObj;
	// 		semanticObj.label_id = object.label_id;
	// 		semanticObj.position = {object.position(0), object.position(1), object.position(2)};
	// 		semanticObj.size = {object.size(0), object.size(1), object.size(2)};
	// 		semanticObj.view_angles = object.view_angles;
	// 		semanticObjArray.semanticObjs.push_back(semanticObj);
	// 	}
	// 	this->semanticObjPub_.publish(semanticObjArray);
	// }

	void multiRoboOccMap::semanticObjSubCB(const map_manager::semanticObjArrayMsgConstPtr& objArray){
		// update semantic objects
		// this should be the current best object estimation, so always trust the incoming message
		for (const auto& incomeObject : objArray->semanticObjs){
			bool isNewObject = true;
			for (auto& object : this->semanticObjects_){
				if (incomeObject.label_id == object.label_id){
					if (std::sqrt(std::pow(incomeObject.position[0]-object.position[0],2)+
						std::pow(incomeObject.position[1]-object.position[1],2)+
						std::pow(incomeObject.position[2]-object.position[2],2)) < 1.0){
						// update the object position and size 
						object.position = Eigen::Vector3d(incomeObject.position[0], incomeObject.position[1], incomeObject.position[2]);
						object.size = Eigen::Vector3d(incomeObject.size[0], incomeObject.size[1], incomeObject.size[2]);
						// any view angle, that was set to true by any of the robots, will be set to true
						for (size_t i=0; i<object.view_angles.size(); ++i){
							if (incomeObject.view_angles[i] == 1){
								object.view_angles[i] = 1;
							}
						}
						isNewObject = false;
						break;
					}
				}
			}
			if (isNewObject){
				// add new object
				semanticObject newObject;
				newObject.label_id = incomeObject.label_id;
				newObject.position = Eigen::Vector3d(incomeObject.position[0], incomeObject.position[1], incomeObject.position[2]);
				newObject.size = Eigen::Vector3d(incomeObject.size[0], incomeObject.size[1], incomeObject.size[2]);
				newObject.view_angles = incomeObject.view_angles; // view angles are set to true by the robot that sees the object
				this->semanticObjects_.push_back(newObject);
			}
		}
	}

	void multiRoboOccMap::updateOccupancyCB(const ros::TimerEvent& ){
		if (not this->occNeedUpdate_){
			return;
		}
		// cout << "update occupancy map" << endl;
		ros::Time startTime, endTime;
		
		startTime = ros::Time::now();
		// if (this->rangeSensorInputMode_ == 0){
		// 	// project 3D points from depth map
		// 	this->projectDepthImage();
		// }
		// else if (this->rangeSensorInputMode_ == 1){
		// 	// directly get pointcloud
		// 	this->getPointcloud();
		// }
		if (this->rangeSensorInputMode_ == 0){
			// project 3D points from depth map
			auto sensorData = std::make_shared<cv::Mat>(this->depthImage_);
			this->sensorManager_->getGlobalRangeSensorInput(this->map2Body_, sensorData, this->projPoints_, this->projPointsNum_);
		}
		else if (this->rangeSensorInputMode_ == 1){
			// directly get pointcloud
			auto sensorData = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(this->pointcloud_);
			this->sensorManager_->getGlobalRangeSensorInput(this->map2Body_, sensorData, this->projPoints_, this->projPointsNum_);
		}

		// raycasting and update occupancy
		this->raycastUpdate();


		// clear local map
		if (this->cleanLocalMap_){
			this->cleanLocalMap();
		}
		
		// infalte map
		// this->inflateLocalMap();
		endTime = ros::Time::now();
		if (this->verbose_){
			cout << this->hint_ << ": Occupancy update time: " << (endTime - startTime).toSec() << " s." << endl;
		}
		this->occNeedUpdate_ = false;
		this->mapNeedInflate_ = true;
	}

	void multiRoboOccMap::raycastUpdate(){
		auto start_time = std::chrono::high_resolution_clock::now();
		if (this->projPointsNum_ == 0){
			return;
		}
		this->raycastNum_ += 1;

		// record local bound of update
		double xmin, xmax, ymin, ymax, zmin, zmax;
		xmin = xmax = this->rangeSensorPosition_(0);
		ymin = ymax = this->rangeSensorPosition_(1);
		zmin = zmax = this->rangeSensorPosition_(2);

		// iterate through each projected points, perform raycasting and update occupancy
		Eigen::Vector3d currPoint;
		bool pointAdjusted;
		int rayendVoxelID, raycastVoxelID;
		double length;
		for (int i=0; i<this->projPointsNum_; ++i){
			currPoint = this->projPoints_[i];
			if (std::isnan(currPoint(0)) or std::isnan(currPoint(1)) or std::isnan(currPoint(2))){
				continue; // nan points can happen when we are using pointcloud as input
			}

			pointAdjusted = false;
			// check whether the point is in reserved map range
			if (not this->isInMap(currPoint)){
				currPoint = this->adjustPointInMap(currPoint);
				pointAdjusted = true;
			}

			// check whether the point exceeds the maximum raycasting length
			length = (currPoint - this->rangeSensorPosition_).norm();
			if (length > this->raycastMaxLength_){
				currPoint = this->adjustPointRayLength(currPoint);
				pointAdjusted = true;
			}


			// update local bound
			if (currPoint(0) < xmin){xmin = currPoint(0);}
			if (currPoint(1) < ymin){ymin = currPoint(1);}
			if (currPoint(2) < zmin){zmin = currPoint(2);}
			if (currPoint(0) > xmax){xmax = currPoint(0);}
			if (currPoint(1) > ymax){ymax = currPoint(1);}
			if (currPoint(2) > zmax){zmax = currPoint(2);}

			// update occupancy itself update information
			rayendVoxelID = this->updateOccupancyInfo(currPoint, not pointAdjusted); // point adjusted is free, not is occupied

			// check whether the voxel has already been updated, so no raycasting needed
			// rayendVoxelID = this->posToAddress(currPoint);
			if (this->flagRayend_[rayendVoxelID] == this->raycastNum_){
				continue; // skip
			}
			else{
				this->flagRayend_[rayendVoxelID] = this->raycastNum_;
			}



			// raycasting for update occupancy
			this->raycaster_.setInput(currPoint/this->mapRes_, this->rangeSensorPosition_/this->mapRes_);
			Eigen::Vector3d rayPoint, actualPoint;
			
			while (this->raycaster_.step(rayPoint)){
				actualPoint = rayPoint;
				actualPoint(0) += 0.5;
				actualPoint(1) += 0.5;
				actualPoint(2) += 0.5;
				actualPoint *= this->mapRes_;
				// raycastVoxelID = this->updateOccupancyInfo(actualPoint, false);

				// raycastVoxelID = this->posToAddress(actualPoint);
				raycastVoxelID = this->posToAddress(actualPoint);
				if (this->flagTraverse_[raycastVoxelID] == this->raycastNum_){
					break;
				}
				else{
					this->flagTraverse_[raycastVoxelID] = this->raycastNum_;
				}
				raycastVoxelID = this->updateOccupancyInfo(actualPoint, false);

				// do not degrade the occupancy if the voxel is occupied
				// Eigen::Vector3i idx;
				// this->posToIndex(actualPoint, idx);
				// int raycastVoxelID = this->indexToAddress(idx);
				// if(this->isOccupied(idx)){
				// 	continue;
				// }
				// else{
				// 	raycastVoxelID = this->updateOccupancyInfo(actualPoint, false);
				// }

			}
		}

		// store local bound and inflate local bound (inflate is for ESDF update)
		this->posToIndex(Eigen::Vector3d (xmin, ymin, zmin), this->localBoundMin_);
		this->posToIndex(Eigen::Vector3d (xmax, ymax, zmax), this->localBoundMax_);
		this->localBoundMin_ -= int(ceil(this->localBoundInflate_/this->mapRes_)) * Eigen::Vector3i(1, 1, 0); // inflate in x y direction
		this->localBoundMax_ += int(ceil(this->localBoundInflate_/this->mapRes_)) * Eigen::Vector3i(1, 1, 0); 
		this->boundIndex(this->localBoundMin_); // since inflated, need to bound if not in reserved range
		this->boundIndex(this->localBoundMax_);


		// update occupancy in the cache
		double logUpdateValue;
		int cacheAddress, hit, miss;
		
		// this->updateFrontiers(this->updateVoxelCache_);
		// this->computeFrontiersToVisit();
		// if (this->robotName_ == "ugv") {this->updateGlobalViewpoints();}
		// this->localViewpointsFilter();

		std::queue<Eigen::Vector3i> empty;

		// vars for map sharing
		Eigen::Vector3d sharedVoxelPos;
		geometry_msgs::Point p;
		while (not this->updateVoxelCache_.empty()){
			Eigen::Vector3i cacheIdx = this->updateVoxelCache_.front();
			this->updateVoxelCache_.pop();
			cacheAddress = this->indexToAddress(cacheIdx);

			hit = this->countHit_[cacheAddress];
			miss = this->countHitMiss_[cacheAddress] - hit;

			if (hit >= miss and hit != 0){
				logUpdateValue = this->pHitLog_;
			}
			else{
				logUpdateValue = this->pMissLog_;
			}
			this->countHit_[cacheAddress] = 0; // clear hit
			this->countHitMiss_[cacheAddress] = 0; // clear hit and miss

			// check whether point is in the local update range
			if (not this->isInLocalUpdateRange(cacheIdx)){
				continue; // do not update if not in the range
			}

			if (this->useFreeRegions_){ // current used in simulation, this region will not be updated and directly set to free
				Eigen::Vector3d pos;
				this->indexToPos(cacheIdx, pos);
				if (this->isInHistFreeRegions(pos)){
					this->occupancy_[cacheAddress] = this->pMinLog_;
					continue;
				}
			}

			// update occupancy info
			if ((logUpdateValue >= 0) and (this->occupancy_[cacheAddress] >= this->pMaxLog_)){
				continue; // not increase p if max clamped
			}
			else if ((logUpdateValue <= 0) and (this->occupancy_[cacheAddress] == this->pMinLog_)){
				continue; // not decrease p if min clamped
			}
			else if ((logUpdateValue <= 0) and (this->occupancy_[cacheAddress] < this->pMinLog_)){
				this->occupancy_[cacheAddress] = this->pMinLog_; // if unknown set it free (prior), 
				// enable map sharing for ray casted free voxels
				this->indexToPos(cacheIdx,sharedVoxelPos);
				p.x = sharedVoxelPos(0);
				p.y = sharedVoxelPos(1);
				p.z = sharedVoxelPos(2);
				this->sharedVoxels_.occupancy.push_back(false);// occupancy is false, free
				this->sharedVoxels_.positions.push_back(p);
				continue;
			}

			double prevProb = this->occupancy_[cacheAddress];

			this->occupancy_[cacheAddress] = std::min(std::max(this->occupancy_[cacheAddress]+logUpdateValue, this->pMinLog_), this->pMaxLog_);

			// collect voxels whose status changed
			// note that this module only enable map sharing for endpoint voxels
			this->indexToPos(cacheIdx,sharedVoxelPos);
			p.x = sharedVoxelPos(0);
			p.y = sharedVoxelPos(1);
			p.z = sharedVoxelPos(2);
			if (this->isOccupied(cacheIdx)){
				// from unkown/free to occupied
				if (prevProb < this->pOccLog_){
					this->sharedVoxels_.occupancy.push_back(true); // occupancy is true, occupied
					this->sharedVoxels_.positions.push_back(p);
				}
			}
			else if(this->isFree(cacheIdx)){
				// from unkown/occupied to free
				// std::cout << this->hint_ << ": free voxel prev prob: " << prevProb << ", min prob: " << this->pMinLog_ << std::endl;
				if (prevProb < this->pMinLog_ or prevProb >= this->pOccLog_){
					this->sharedVoxels_.occupancy.push_back(false); // occupancy is false, free
					this->sharedVoxels_.positions.push_back(p);
				}
			}

			// update the entire map range (if it is not unknown)
			if (not this->isUnknown(cacheIdx)){
				Eigen::Vector3d cachePos;
				this->indexToPos(cacheIdx, cachePos);
				if (cachePos(0) > this->currMapRangeMax_(0)){
					this->currMapRangeMax_(0) = cachePos(0);
				}
				else if (cachePos(0) < this->currMapRangeMin_(0)){
					this->currMapRangeMin_(0) = cachePos(0);
				}

				if (cachePos(1) > this->currMapRangeMax_(1)){
					this->currMapRangeMax_(1) = cachePos(1);
				}
				else if (cachePos(1) < this->currMapRangeMin_(1)){
					this->currMapRangeMin_(1) = cachePos(1);
				}

				if (cachePos(2) > this->currMapRangeMax_(2)){
					this->currMapRangeMax_(2) = cachePos(2);
				}
				else if (cachePos(2) < this->currMapRangeMin_(2)){
					this->currMapRangeMin_(2) = cachePos(2);
				}
			}
		}
		auto end_time = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
		// std::cout << "raycast took " << duration.count() << " milliseconds." << std::endl;
	}
}
