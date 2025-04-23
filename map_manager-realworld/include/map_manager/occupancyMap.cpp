/*
	FILE: occupancyMap.cpp
	--------------------------------------
	function definition of occupancy map
*/
#include <map_manager/occupancyMap.h>
#include <mutex>
#include <pcl/filters/crop_box.h> //https://codetoflow.com/accounts/signup/
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

namespace mapManager{
	occMap::occMap(){
		this->ns_ = "occupancy_map";
		this->hint_ = "[OccMap]";
	}

	occMap::occMap(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "occupancy_map";
		this->hint_ = "[OccMap]";
		this->initParam();
		this->initPrebuiltMap();
		this->registerPub();
		this->registerCallback();
		this->initCollisionFreeBox();
	}

	void occMap::initMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initParam();
		this->initPrebuiltMap();
		this->registerPub();
		this->registerCallback();
		this->initCollisionFreeBox();
	}

	void occMap::initParam(){
		// sensor input mode
		std::cout << "range_sensor_input_mode" << std::endl;
		if (not this->nh_.getParam("range_sensor_input_mode", this->rangeSensorInputMode_)){
			this->rangeSensorInputMode_ = 0;
			cout << this->hint_ << ": No sensor input mode option. Use default: depth image" << endl;
		}
		else{
			cout << this->hint_ << ": Sensor input mode: depth image (0)/pointcloud (1). Your option: " << this->rangeSensorInputMode_ << endl;
		}		

		// choose range sensor param
		std::vector<double> depthCamParams (4); // fx, fy, cx, cy
		if (this->rangeSensorInputMode_ == 0){
			// depth image topic name
			if (not this->nh_.getParam("depth_image_topic", this->depthTopicName_)){
				this->depthTopicName_ = "/camera/depth/image_raw";
				cout << this->hint_ << ": No depth image topic name. Use default: /camera/depth/image_raw" << endl;
			}
			else{
				cout << this->hint_ << ": Depth topic: " << this->depthTopicName_ << endl;
			}

			// depth camera params
			if (not this->nh_.getParam("depth_camera_params", depthCamParams)){
				ROS_ERROR("[OccMap]: Please check depth camera params!");
				exit(0);
			}
			else{
				this->fx_ = depthCamParams[0]; this->fy_ = depthCamParams[1];
				this->cx_ = depthCamParams[2]; this->cy_ = depthCamParams[3];
				cout << this->hint_ << ": fx, fy, cx, cy: " << "["  << this->fx_ << ", " << this->fy_  << ", " << this->cx_ << ", " << this->cy_ << "]" << endl;
			}

			// depth filter margin
			if (not this->nh_.getParam("depth_filter_margin", this->depthFilterMargin_)){
				this->depthFilterMargin_ = 0;
				cout << this->hint_ << ": No depth filter margin. Use default: 0." << endl;
			}
			else{
				cout << this->hint_ << ": Depth filter margin: " << this->depthFilterMargin_ << endl;
			}

			// depth skip pixel
			if (not this->nh_.getParam("depth_skip_pixel", this->skipPixel_)){
				this->skipPixel_ = 1;
				cout << this->hint_ << ": No depth skip pixel. Use default: 1." << endl;
			}
			else{
				cout << this->hint_ << ": Depth skip pixel: " << this->skipPixel_ << endl;
			}
		}
		else if (this->rangeSensorInputMode_ == 1){
			// pointcloud topic name
			if (not this->nh_.getParam("point_cloud_topic", this->pointcloudTopicName_)){
				this->pointcloudTopicName_ = "/camera/depth/points";
				cout << this->hint_ << ": No poincloud topic name. Use default: /camera/depth/points" << endl;
			}
			else{
				cout << this->hint_ << ": Pointcloud topic: " << this->pointcloudTopicName_ << endl;
			}
		}

		// sensor fusion mode: 0 for depth and color image, 1 for lidar and pano camera
		if (not this->nh_.getParam("sensor_fusion_mode", this->sensorFusionMode_)){
			this->sensorFusionMode_ = 0;
			cout << this->hint_ << ": No sensor fusion mode option. Use default: 0" << endl;
		}
		else{
			cout << this->hint_ << ": Sensor fusion mode: " << this->sensorFusionMode_ << endl;
		}

		// choose color sensor model
		// ------------------------------------------------------------------------------------
		std::vector<double> panoCamParams (3); // y0, fy, colss
		if (this->sensorFusionMode_ == 1) // lidar-pano mode
		{
			// pano cam params
			if (not this->nh_.getParam("pano_cam_params", panoCamParams)){
				ROS_ERROR("[OccMap]: Please check panocam params!");
				exit(0);
			}
			else{
				cout << this->hint_ << ": y0, fy, cols: " << "["  << panoCamParams[0] << ", " << panoCamParams[1]  << ", " << panoCamParams[2] << "]" << endl;
			}
		}		
		// sensor fusion mode 0 use same camera as depth, no extra params
		// ------------------------------------------------------------------------------------

		// localization mode
		if (not this->nh_.getParam("localization_mode", this->localizationMode_)){
			this->localizationMode_ = 0;
			cout << this->hint_ << ": No localization mode option. Use default: pose" << endl;
		}
		else{
			cout << this->hint_ << ": Localizaiton mode: pose (0)/odom (1). Your option: " << this->localizationMode_ << endl;
		}

		if (this->localizationMode_ == 0){
			// odom topic name
			if (not this->nh_.getParam("pose_topic", this->poseTopicName_)){
				this->poseTopicName_ = "/CERLAB/quadcopter/pose";
				cout << this->hint_ << ": No pose topic name. Use default: /CERLAB/quadcopter/pose" << endl;
			}
			else{
				cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << endl;
			}			
		}

		if (this->localizationMode_ == 1){
			// pose topic name
			if (not this->nh_.getParam("odom_topic", this->odomTopicName_)){
				this->odomTopicName_ = "/CERLAB/quadcopter/odom";
				cout << this->hint_ << ": No odom topic name. Use default: /CERLAB/quadcopter/odom" << endl;
			}
			else{
				cout << this->hint_ << ": Odom topic: " << this->odomTopicName_ << endl;
			}
		}

		// raw object detection topic
		if (not this->nh_.getParam("raw_detection_topic", this->rawDetectionTopicName_)){
			this->rawDetectionTopicName_ = "/bbox3d";
			cout << this->hint_ << ": No bbox topic option. Use default: /bbox3d" << endl;
		}
		else{
			cout << this->hint_ << ": bbox topic: " << this->rawDetectionTopicName_ << endl;
		}

		// input dense cloud topic
		if (not this->nh_.getParam("dense_cloud_topic", this->denseCloudTopicName_)){
			this->denseCloudTopicName_ = "/registered_cloud";
			cout << this->hint_ << ": No dense cloud topic option. Use default: /dense_cloud" << endl;
		}
		else{
			cout << this->hint_ << ": dense cloud topic: " << this->denseCloudTopicName_ << endl;
		}

		std::vector<double> robotSizeVec (3);
		if (not this->nh_.getParam("robot_size", robotSizeVec)){
			robotSizeVec = std::vector<double>{0.5, 0.5, 0.3};
		}
		else{
			cout << this->hint_ << ": robot size: " << "[" << robotSizeVec[0]  << ", " << robotSizeVec[1] << ", "<< robotSizeVec[2] << "]" << endl;
		}
		this->robotSize_(0) = robotSizeVec[0]; this->robotSize_(1) = robotSizeVec[1]; this->robotSize_(2) = robotSizeVec[2];
		
		// collision box
		std::vector<double> collisionBoxVec (6);
		if (not this->nh_.getParam("collision_box", collisionBoxVec)){
			collisionBoxVec = std::vector<double>{0.5, 0.5, 0.3, 0.5, 0.5, 0.3};
		}
		else{
			cout << this->hint_ << ": collision box: " << "[" << collisionBoxVec[0]  << ", " << collisionBoxVec[1] << ", "<< collisionBoxVec[2] << ", " << collisionBoxVec[3] << ", " << collisionBoxVec[4] << ", " << collisionBoxVec[5] << "]" << endl;
		}
		this->collisionBoxMin_(0) = collisionBoxVec[0]; this->collisionBoxMin_(1) = collisionBoxVec[1]; this->collisionBoxMin_(2) = collisionBoxVec[2];
		this->collisionBoxMax_(0) = collisionBoxVec[3]; this->collisionBoxMax_(1) = collisionBoxVec[4]; this->collisionBoxMax_(2) = collisionBoxVec[5];
		

		// fovH and fovV
		if (not this->nh_.getParam("fovV", this->fovV_)){
			this->fovV_ = 56.0;
			cout << this->hint_ << ": No vertical field of view. Use default: 60.0." << endl;
		}
		else{
			cout << this->hint_ << ": Vertical field of view: " << this->fovV_ << endl;
		}

		// depth scale factor
		if (not this->nh_.getParam("depth_scale_factor", this->depthScale_)){
			this->depthScale_ = 1000.0;
			cout << this->hint_ << ": No depth scale factor. Use default: 1000." << endl;
		}
		else{
			cout << this->hint_ << ": Depth scale factor: " << this->depthScale_ << endl;
		}

		// depth min value
		if (not this->nh_.getParam("depth_min_value", this->depthMinValue_)){
			this->depthMinValue_ = 0.2;
			cout << this->hint_ << ": No depth min value. Use default: 0.2 m." << endl;
		}
		else{
			cout << this->hint_ << ": Depth min value: " << this->depthMinValue_ << endl;
		}

		// depth max value
		if (not this->nh_.getParam("depth_max_value", this->depthMaxValue_)){
			this->depthMaxValue_ = 5.0;
			cout << this->hint_ << ": No depth max value. Use default: 5.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Depth depth max value: " << this->depthMaxValue_ << endl;
		}

		// ------------------------------------------------------------------------------------
		// image columns
		if (not this->nh_.getParam("image_cols", this->imgCols_)){
			this->imgCols_ = 640;
			cout << this->hint_ << ": No depth image columns. Use default: 640." << endl;
		}
		else{
			cout << this->hint_ << ": Depth image columns: " << this->imgCols_ << endl;
		}

		// image rows
		if (not this->nh_.getParam("image_rows", this->imgRows_)){
			this->imgRows_ = 480;
			cout << this->hint_ << ": No depth image rows. Use default: 480." << endl;
		}
		else{
			cout << this->hint_ << ": Depth image rows: " << this->imgRows_ << endl;
		}
		this->projPoints_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
		// ------------------------------------------------------------------------------------

		// static transformations
		// ------------------------------------------------------------------------------------
		// transform matrix: body to range sensor
		std::string param_name = "body_to_range_sensor";
		std::string full_param_path = nh_.getNamespace() + "/" + param_name;
		ROS_INFO("Looking for parameter: %s", full_param_path.c_str());
		std::vector<double> body2CamVec (16);
		std::cout << nh_.getNamespace() + "/body_to_range_sensor" << std::endl;
		if (not this->nh_.getParam("body_to_range_sensor", body2CamVec)){
			// ROS_ERROR("[OccMap]: Please check body to body to range_sensor matrix!");
			ROS_ERROR("Failed to get param '%s'", full_param_path.c_str());
		}
		else{
			for (int i=0; i<4; ++i){
				for (int j=0; j<4; ++j){
					this->body2RangeSensor_(i, j) = body2CamVec[i * 4 + j];
				}
			}
			cout << this->hint_ << ": from body to range sensor: " << endl;
			cout << this->body2RangeSensor_ << endl;
		}

		// transform matrix: body to color sensor
		std::vector<double> body2ColorVec (16);
		if (not this->nh_.getParam("body_to_color_sensor", body2ColorVec)){
			ROS_ERROR("[OccMap]: Please check body to color sensor matrix!");
		}
		else{
			for (int i=0; i<4; ++i){
				for (int j=0; j<4; ++j){
					this->body2ColorSensor_(i, j) = body2ColorVec[i * 4 + j];
				}
			}
			cout << this->hint_ << ": from body to color sensor: " << endl;
			cout << this->body2ColorSensor_ << endl;
		}

		// std::vector<double> color2DepthIntrinsics (4);
		// if (not this->nh_.getParam("color_algined_to_depth_intrinsics", color2DepthIntrinsics)){
		// 	this->fxC_ = 0.0; this->fyC_ = 0.0; this->cxC_ = 0.0; this->cyC_ = 0.0;
		// 	cout << this->hint_ << ": Did not get params of camera intrinsics!, use default: 0.0" << endl;

		// }
		// else{
		// 	this->fxC_ = color2DepthIntrinsics[0];
		// 	this->fyC_ = color2DepthIntrinsics[1];
		// 	this->cxC_ = color2DepthIntrinsics[2];
		// 	this->cyC_ = color2DepthIntrinsics[3];
		// 	cout << this->hint_ << ": color aligned to depth fx, fy, cx, cy: " << "["  << this->fxC_ << ", " << this->fyC_  << ", " << this->cxC_ << ", "<< this->cyC_ << "]" << endl;
		// }

		// raycasting
		// ------------------------------------------------------------------------------------
		// Raycast max length
		if (not this->nh_.getParam("raycast_max_length", this->raycastMaxLength_)){
			this->raycastMaxLength_ = 5.0;
			cout << this->hint_ << ": No raycast max length. Use default: 5.0." << endl;
		}
		else{
			cout << this->hint_ << ": Raycast max length: " << this->raycastMaxLength_ << endl;
		}

		// p hit
		double pHit;
		if (not this->nh_.getParam("p_hit", pHit)){
			pHit = 0.70;
			cout << this->hint_ << ": No p hit. Use default: 0.70." << endl;
		}
		else{
			cout << this->hint_ << ": P hit: " << pHit << endl;
		}
		this->pHitLog_ = this->logit(pHit);

		// p miss
		double pMiss;
		if (not this->nh_.getParam("p_miss", pMiss)){
			pMiss = 0.35;
			cout << this->hint_ << ": No p miss. Use default: 0.35." << endl;
		}
		else{
			cout << this->hint_ << ": P miss: " << pMiss << endl;
		}
		this->pMissLog_ = this->logit(pMiss);

		// p min
		double pMin;
		if (not this->nh_.getParam("p_min", pMin)){
			pHit = 0.12;
			cout << this->hint_ << ": No p min. Use default: 0.12." << endl;
		}
		else{
			cout << this->hint_ << ": P min: " << pMin << endl;
		}
		this->pMinLog_ = this->logit(pMin);

		// p max
		double pMax;
		if (not this->nh_.getParam("p_max", pMax)){
			pMax = 0.97;
			cout << this->hint_ << ": No p max. Use default: 0.97." << endl;
		}
		else{
			cout << this->hint_ << ": P max: " << pMax << endl;
		}
		this->pMaxLog_ = this->logit(pMax);

		// p occ
		double pOcc;
		if (not this->nh_.getParam("p_occ", pOcc)){
			pOcc = 0.80;
			cout << this->hint_ << ": No p occ. Use default: 0.80." << endl;
		}
		else{
			cout << this->hint_ << ": P occ: " << pOcc << endl;
		}
		this->pOccLog_ = this->logit(pOcc);


		// map resolution
		if (not this->nh_.getParam("map_resolution", this->mapRes_)){
			this->mapRes_ = 0.1;
			cout << this->hint_ << ": No map resolution. Use default: 0.1." << endl;
		}
		else{
			cout << this->hint_ << ": Map resolution: " << this->mapRes_ << endl;
		}

		// ground height
		if (not this->nh_.getParam("ground_height", this->groundHeight_)){
			this->groundHeight_ = 0.0;
			cout << this->hint_ << ": No ground height. Use default: 0.0." << endl;
		}
		else{
			cout << this->hint_ << ": Ground height: " << this->groundHeight_ << endl;
		}

		// map
		// ------------------------------------------------------------------------------------
		// map size
		std::vector<double> mapSizeMaxVec(3);
		if (not this->nh_.getParam("map_size_max", mapSizeMaxVec)){
			this->mapSizeMax_ = Eigen::Vector3d(20, 20, 3);
			cout << this->hint_ << ": No map size max. Use default: [20, 20, 3]." << endl;
		}
		else{
			this->mapSizeMax_ = Eigen::Vector3d(mapSizeMaxVec[0], mapSizeMaxVec[1], mapSizeMaxVec[2]);
			cout << this->hint_ << ": Map size max: " << "[" << this->mapSizeMax_(0) << ", " << this->mapSizeMax_(1) << ", " << this->mapSizeMax_(2) << "]" << endl;
		}

		std::vector<double> mapSizeMinVec(3);
		if (not this->nh_.getParam("map_size_min", mapSizeMinVec)){
			this->mapSizeMin_ = Eigen::Vector3d(-20, -20, 0);
			cout << this->hint_ << ": No map size min. Use default: [-20, -20, 0]." << endl;
		}
		else{
			this->mapSizeMin_ = Eigen::Vector3d(mapSizeMinVec[0], mapSizeMinVec[1], mapSizeMinVec[2]);
			cout << this->hint_ << ": Map size min: " << "[" << this->mapSizeMin_(0) << ", " << this->mapSizeMin_(1) << ", " << this->mapSizeMin_(2) << "]" << endl;
		}
		this->mapVoxelMin_(0) = 0; this->mapVoxelMax_(0) = ceil((this->mapSizeMax_(0)-this->mapSizeMin_(0))/this->mapRes_);
		this->mapVoxelMin_(1) = 0; this->mapVoxelMax_(1) = ceil((this->mapSizeMax_(1)-this->mapSizeMin_(1))/this->mapRes_);
		this->mapVoxelMin_(2) = 0; this->mapVoxelMax_(2) = ceil((this->mapSizeMax_(2)-this->mapSizeMin_(2))/this->mapRes_);
		// reserve vector for variables
		int reservedSize = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		this->countHitMiss_.resize(reservedSize, 0);
		this->countHit_.resize(reservedSize, 0);
		this->occupancy_.resize(reservedSize, this->pMinLog_-this->UNKNOWN_FLAG_);
		this->occupancyInflated_.resize(reservedSize, false);
		this->flagTraverse_.resize(reservedSize, -1);
		this->flagRayend_.resize(reservedSize, -1);
		cout << this->hint_ << ": Reserved size: " << reservedSize << endl;

		// local update range
		std::vector<double> localUpdateRangeVec;
		if (not this->nh_.getParam("local_update_range", localUpdateRangeVec)){
			localUpdateRangeVec = std::vector<double>{5.0, 5.0, 3.0};
			cout << this->hint_ << ": No local update range. Use default: [5.0, 5.0, 3.0] m." << endl;
		}
		else{
			cout << this->hint_ << ": Local update range: " << "[" << localUpdateRangeVec[0] << ", " << localUpdateRangeVec[1] << ", " << localUpdateRangeVec[2] << "]" << endl;
		}
		this->localUpdateRange_(0) = localUpdateRangeVec[0]; this->localUpdateRange_(1) = localUpdateRangeVec[1]; this->localUpdateRange_(2) = localUpdateRangeVec[2];


		// local bound inflate factor
		if (not this->nh_.getParam("local_bound_inflation", this->localBoundInflate_)){
			this->localBoundInflate_ = 0.0;
			cout << this->hint_ << ": No local bound inflate. Use default: 0.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Local bound inflate: " << this->localBoundInflate_ << endl;
		}

		// whether to clean local map
		if (not this->nh_.getParam("clean_local_map", this->cleanLocalMap_)){
			this->cleanLocalMap_ = true;
			cout << this->hint_ << ": No clean local map option. Use default: true." << endl;
		}
		else{
			cout << this->hint_ << ": Clean local map option is set to: " << this->cleanLocalMap_ << endl; 
		}

		// absolute dir of prebuilt map file (.pcd)
		if (not this->nh_.getParam("prebuilt_map_directory", this->prebuiltMapDir_)){
			this->prebuiltMapDir_ = "";
			cout << this->hint_ << ": Not using prebuilt map." << endl;
		}
		else{
			cout << this->hint_ << ": the prebuilt map absolute dir is found: " << this->prebuiltMapDir_ << endl;
		}

		// local map size (visualization)
		std::vector<double> localMapSizeVec;
		if (not this->nh_.getParam("local_map_size", localMapSizeVec)){
			localMapSizeVec = std::vector<double>{10.0, 10.0, 2.0};
			cout << this->hint_ << ": No local map size. Use default: [10.0, 10.0, 3.0] m." << endl;
		}
		else{
			cout << this->hint_ << ": Local map size: " << "[" << localMapSizeVec[0] << ", " << localMapSizeVec[1] << ", " << localMapSizeVec[2] << "]" << endl;
		}
		this->localMapSize_(0) = localMapSizeVec[0]/2; this->localMapSize_(1) = localMapSizeVec[1]/2; this->localMapSize_(2) = localMapSizeVec[2]/2;
		this->localMapVoxel_(0) = int(ceil(localMapSizeVec[0]/(2*this->mapRes_))); this->localMapVoxel_(1) = int(ceil(localMapSizeVec[1]/(2*this->mapRes_))); this->localMapVoxel_(2) = int(ceil(localMapSizeVec[2]/(2*this->mapRes_)));

		// max vis height
		if (not this->nh_.getParam("max_height_visualization", this->maxVisHeight_)){
			this->maxVisHeight_ = 3.0;
			cout << this->hint_ << ": No max visualization height. Use default: 3.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Max visualization height: " << this->maxVisHeight_ << endl;
		}

		// visualize global map
		if (not this->nh_.getParam("visualize_global_map", this->visGlobalMap_)){
			this->visGlobalMap_ = false;
			cout << this->hint_ << ": No visualize map option. Use default: visualize local map." << endl;
		}
		else{
			cout << this->hint_ << ": Visualize map option. local (0)/global (1): " << this->visGlobalMap_ << endl;
		}

		// verbose
		if (not this->nh_.getParam("verbose", this->verbose_)){
			this->verbose_ = true;
			cout << this->hint_ << ": No verbose option. Use default: check update info." << endl;
		}
		else{
			if (not this->verbose_){
				cout << this->hint_ << ": Not display messages" << endl;
			}
			else{
				cout << this->hint_ << ": Display messages" << endl;
			}
		}

		// object map and semantics
		// ------------------------------------------------------------------------------------
		// object map frequency
		if (not this->nh_.getParam("object_map_duration", this->objectMapDuration_)){
			this->objectMapDuration_ = 1;
			cout << this->hint_ << ": No object map frequency option. Use default: 1" << endl;
		}
		else{
			cout << this->hint_ << ": Object map frequency: " << this->objectMapDuration_ << endl;
		}

		// view angle intervals number
		if (not this->nh_.getParam("view_angle_intervals", this->viewAgIntervals_)){
			this->viewAgIntervals_ = 8;
			cout << this->hint_ << ": No view angle intervals option. Use default: 8" << endl;
		}
		else{
			cout << this->hint_ << ": view angle intervals: " << this->viewAgIntervals_ << endl;
		}
		this->viewAgRes_ = 2 * M_PI / this->viewAgIntervals_;
		
		this->semanticObjects_.clear();

		// dbscan downsample ratio
		if (not this->nh_.getParam("dbscan_downsample_factor", this->dbDownsampleFactor_)){
			this->dbDownsampleFactor_ = 0.1;
			cout << this->hint_ << ": No DBSCAN downsample factor parameter. Use default: 0.1." << endl;
		}
		else{
			cout << this->hint_ << ": DBSCAN downsample factor is set to: " << this->dbDownsampleFactor_ << endl;
		}

		// minimum number of points in each cluster
        if (not this->nh_.getParam("dbscan_min_points_cluster", this->dbMinPointsCluster_)){
            this->dbMinPointsCluster_ = 18;
            cout << this->hint_ << ": No DBSCAN minimum point in each cluster parameter. Use default: 18." << endl;
        }
        else{
            cout << this->hint_ << ": DBSCAN Minimum point in each cluster is set to: " << this->dbMinPointsCluster_ << endl;
        }

        // search range
        if (not this->nh_.getParam("dbscan_search_range_epsilon", this->dbEpsilon_)){
            this->dbEpsilon_ = 0.3;
            cout << this->hint_ << ": No DBSCAN epsilon parameter. Use default: 0.3." << endl;
        }
        else{
            cout << this->hint_ << ": DBSCAN epsilon is set to: " << this->dbEpsilon_ << endl;
        }  

		// object of interest
		if (not this->nh_.getParam("object_of_interest", this->objOfInterest_)){
			this->objOfInterest_.push_back(1);
			cout << this->hint_ << ": No object of interest option. Use default: 1" << endl;
		}
		else{

			cout << this->hint_ << ": object of interest: " << endl;
			for (int i : this->objOfInterest_) {
				std::cout << i << ' ';
			}
			std::cout << std::endl;
		}

		// object bbox color
		if (not this->nh_.getParam("object_bbox_color", this->objBboxColor_) || this->objBboxColor_.size() / 3 != this->objOfInterest_.size()){
			cout << this->hint_ << ": No object bbox color option OR size mismatch with object of interest. Use default: grey" << endl;
			// Assign grey to all objects
			for (size_t i = 0; i < this->objOfInterest_.size(); ++i) {
				this->objBboxColor_.insert(this->objBboxColor_.end(), {0.5, 0.5, 0.5});
			}
		}
		// object class label
		if (not nh_.getParam("object_class_label", this->objClassLabel_) || this->objClassLabel_.size() != this->objOfInterest_.size()) {
			cout << this->hint_ << ": No object class label option OR size mismatch. Using default 'chair'.";
			objClassLabel_.assign(objOfInterest_.size(), "chair");
		}
		// object color label map
		this->objColorLabelMap_.clear();
		for (size_t i = 0; i < this->objOfInterest_.size(); ++i) {
			std::vector<double> color = {this->objBboxColor_[i * 3], this->objBboxColor_[i * 3 + 1], this->objBboxColor_[i * 3 + 2]};
			std::string label = this->objClassLabel_[i];

			this->objColorLabelMap_[this->objOfInterest_[i]] = std::make_pair(color, label);
		}
		for (const auto& item : this->objColorLabelMap_) {
			cout << this->hint_ << ": Object ID: " << item.first 
				<< ", Color: [" << item.second.first[0] << ", " 
				<< item.second.first[1] << ", " 
				<< item.second.first[2] << "], Label: " 
				<< item.second.second << endl;
		}

		// probability threshold for object detection
		if (not this->nh_.getParam("detect_threshold", this->detectThresh_)){
			this->detectThresh_ = 0.85;
			cout << this->hint_ << ": No detection probability threshold option. Use default: 0.85" << endl;
		}
		else{
			cout << this->hint_ << ": detection probability threshold: " << this->detectThresh_ << endl;
		}

		// visible distance
		if (not this->nh_.getParam("visible_distance", this->visibleDist_)){
			this->visibleDist_ = 5.0;
			cout << this->hint_ << ": No visible distance option. Use default: 5.0" << endl;
		}
		else{
			cout << this->hint_ << ": visible distance: " << this->visibleDist_ << endl;
		}

		// invisible range
		if (not this->nh_.getParam("invisible_range", this->invisibleRange_)){
			this->invisibleRange_ = 1.0;
			cout << this->hint_ << ": No invisible range option. Use default: 0.5" << endl;
		}
		else{
			cout << this->hint_ << ": invisible range: " << this->invisibleRange_ << endl;
		}

		// object map size vector
		std::vector<double> objectMapSizeVec;
		if (not this->nh_.getParam("object_map_size", objectMapSizeVec)){
			objectMapSizeVec = std::vector<double>{2.0, 2.0, 2.0};
			cout << this->hint_ << ": No object map size. Use default: [10.0, 10.0, 3.0] m." << endl;
		}
		else{
			cout << this->hint_ << ": Object map size: " << "[" << objectMapSizeVec[0] << ", " << objectMapSizeVec[1] << ", " << objectMapSizeVec[2] << "]" << endl;
		}
		
		// object map resolution
		if (not this->nh_.getParam("object_map_resolution", this->objectMapRes_)){
			this->objectMapRes_ = 0.1;
			cout << this->hint_ << ": No object map resolution option. Use default: 0.1" << endl;
		}
		else{
			cout << this->hint_ << ": object map resolution: " << this->objectMapRes_ << endl;
		}

		if(not this->nh_.getParam("min_mask_score", this->minMaskScoreThresh_)){
            this->minMaskScoreThresh_ = 100;
        }
        else{
            std::cout << this->hint_ << " min_mask_score: " << this->minMaskScoreThresh_ << std::endl;
        }

        if(not this->nh_.getParam("observe_valid_prob_thresh", this->obsValidProbThresh_)){
            this->obsValidProbThresh_ = 0.3;
        }
        else{
            std::cout << this->hint_ << " observe valid probability thresh: " << this->obsValidProbThresh_ << std::endl;
        }

		// invalid observe probability threshold
		if (not this->nh_.getParam("observe_invalid_prob_thresh", this->obsInvalidProbThresh_)){
			this->obsInvalidProbThresh_ = 0.3;
			cout << this->hint_ << ": No observe probability threshold option. Use default: 0.3" << endl;
		}
		else{
			cout << this->hint_ << ": observe invalid probability threshold: " << this->obsInvalidProbThresh_ << endl;
		}

		// debug
		if (not this->nh_.getParam("object_map_debug", this->objMapDebug_)){
			this->objMapDebug_ = false;
			cout << this->hint_ << ": No object map debug option. Use default: false" << endl;
		}
		else{
			cout << this->hint_ << ": object map debug option: " << this->objMapDebug_ << endl;
		}

		// debug data root dir
		if (not this->nh_.getParam("debug_root_dir", this->debugRootDir_)){
			this->debugRootDir_ = "";
			cout << this->hint_ << ": No debug root dir option. Use default: empty" << endl;
		}
		else{
			cout << this->hint_ << ": debug root dir: " << this->debugRootDir_ << endl;
		}

		// voxel filter thresh
		if (not this->nh_.getParam("voxel_filter_thresh", this->voxelFilterThresh_)){
			this->voxelFilterThresh_ = 10;
			cout << this->hint_ << ": No voxel filter threshold option. Use default: 0.5" << endl;
		}
		else{
			cout << this->hint_ << ": voxel filter threshold: " << this->voxelFilterThresh_ << endl;
		}

		// object filter thresh
		if (not this->nh_.getParam("object_filter_height", this->objectFilterHeight_)){
			this->objectFilterHeight_ = 0.5;
			cout << this->hint_ << ": No object filter height threshold option. Use default: 0.5" << endl;
		}
		else{
			cout << this->hint_ << ": object filter height threshold: " << this->objectFilterHeight_ << endl;
		}
		
		// object track threshold
		if (not this->nh_.getParam("object_track_thresh", this->objectTrackThresh_)){
			this->objectTrackThresh_ = 0.5;
			cout << this->hint_ << ": No object track threshold option. Use default: 0.5" << endl;
		}
		else{
			cout << this->hint_ << ": object track threshold: " << this->objectTrackThresh_ << endl;
		}

		// object merge threshold
		if (not this->nh_.getParam("object_merge_thresh", this->objectMergeThresh_)){
			this->objectMergeThresh_ = 0.5;
			cout << this->hint_ << ": No object merge threshold option. Use default: 0.5" << endl;
		}
		else{
			cout << this->hint_ << ": object merge threshold: " << this->objectMergeThresh_ << endl;
		}

		// init struct: objectMapParmas
		this->objectMapParams_.resolution = this->objectMapRes_;
		this->objectMapParams_.object_map_size = objectMapSizeVec;
		this->objectMapParams_.min_mask_score_thresh = this->minMaskScoreThresh_;
		this->objectMapParams_.observe_valid_prob_thresh = this->obsValidProbThresh_;
		this->objectMapParams_.observe_invalid_prob_thresh = this->obsInvalidProbThresh_;
		this->objectMapParams_.view_angle_intervals = this->viewAgIntervals_;
		this->objectMapParams_.is_debug = this->objMapDebug_;
		this->objectMapParams_.debug_root_dir = this->debugRootDir_;

		// sensor models
		// init sensor models: sensorManager_
		// ------------------------------------------------------------------------------------
		this->sensorManager_.reset(new sensorModelFactory(this->sensorFusionMode_, this->rangeSensorInputMode_));

		if (this->sensorFusionMode_ == 0){
			// create depth color camera model
			std::shared_ptr<sensorModelBase> PinholeCameraModel(new PinholeCamModel(
			this->body2RangeSensor_, 
			depthCamParams[0], depthCamParams[1], depthCamParams[2], depthCamParams[3],
			this->imgCols_, this->imgRows_, this->depthMinValue_, this->depthMaxValue_, 
			this->depthScale_, this->depthFilterMargin_, this->skipPixel_));
			
			this->sensorManager_->setRangeSensorModel(PinholeCameraModel);
			this->sensorManager_->setColorSensorModel(PinholeCameraModel);

			// body 2 range and color sensor should be the same
			assert(this->body2RangeSensor_ == this->body2ColorSensor_);
		}
		else if (this->sensorFusionMode_ == 1){
			// create lidar model
			std::shared_ptr<sensorModelBase> lidarModel(new LidarModel(this->body2RangeSensor_, this->fovV_));
			// create pano camera model
			std::shared_ptr<sensorModelBase> panoCameraModel(new PanoCamModel(this->body2ColorSensor_, panoCamParams[0], panoCamParams[1], panoCamParams[2]));
			this->sensorManager_->setRangeSensorModel(lidarModel);
			this->sensorManager_->setColorSensorModel(panoCameraModel);
		}
		else{
			ROS_ERROR("[OccMap]: Invalid sensor fusion mode!");
		}
	}

	void occMap::initPrebuiltMap(){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ> (this->prebuiltMapDir_, *cloud) == -1) //* load the file
		{
			cout << this->hint_ << ": No prebuilt map found/not using the prebuilt map." << endl;
		}
		else {
			cout << this->hint_ << "Loaded " << cloud->width * cloud->height << " data points from test_pcd.pcd with the following fields: " << endl;
			int address;
			Eigen::Vector3i pointIndex;
			Eigen::Vector3d pointPos;
			Eigen::Vector3i inflateIndex;
			int inflateAddress;

			// update occupancy info
			int xInflateSize = ceil(this->robotSize_(0)/(2*this->mapRes_));
			int yInflateSize = ceil(this->robotSize_(1)/(2*this->mapRes_));
			int zInflateSize = ceil(this->robotSize_(2)/(2*this->mapRes_));

			Eigen::Vector3d currMapRangeMin (0.0, 0.0, 0.0);
			Eigen::Vector3d currMapRangeMax (0.0, 0.0, 0.0);

			const int  maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
			for (const auto& point: *cloud)
			{
				address = this->posToAddress(point.x, point.y, point.z);
				pointPos(0) = point.x; pointPos(1) = point.y; pointPos(2) = point.z;
				this->posToIndex(pointPos, pointIndex);

				this->occupancy_[address] = this->pMaxLog_;
				// update map range
				if (pointPos(0) < currMapRangeMin(0)){
					currMapRangeMin(0) = pointPos(0);
				}

				if (pointPos(0) > currMapRangeMax(0)){
					currMapRangeMax(0) = pointPos(0);
				}

				if (pointPos(1) < currMapRangeMin(1)){
					currMapRangeMin(1) = pointPos(1);
				}

				if (pointPos(1) > currMapRangeMax(1)){
					currMapRangeMax(1) = pointPos(1);
				}

				if (pointPos(2) < currMapRangeMin(2)){
					currMapRangeMin(2) = pointPos(2);
				}

				if (pointPos(2) > currMapRangeMax(2)){
					currMapRangeMax(2) = pointPos(2);
				}

				for (int ix=-xInflateSize; ix<=xInflateSize; ++ix){
					for (int iy=-yInflateSize; iy<=yInflateSize; ++iy){
						for (int iz=-zInflateSize; iz<=zInflateSize; ++iz){
							inflateIndex(0) = pointIndex(0) + ix;
							inflateIndex(1) = pointIndex(1) + iy;
							inflateIndex(2) = pointIndex(2) + iz;
							inflateAddress = this->indexToAddress(inflateIndex);
							if ((inflateAddress < 0) or (inflateAddress > maxIndex)){
								continue; // those points are not in the reserved map
							} 
							this->occupancyInflated_[inflateAddress] = true;
						}
					}
				}
			}
			this->currMapRangeMin_ = currMapRangeMin;
			this->currMapRangeMax_ = currMapRangeMax;
		}
	}

	void occMap::registerCallback(){
		if (this->rangeSensorInputMode_ == 0){
			// depth pose callback
			this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
			if (this->localizationMode_ == 0){
				this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
				this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_, *this->poseSub_));
				this->depthPoseSync_->registerCallback(boost::bind(&occMap::depthPoseCB, this, _1, _2));
			}
			else if (this->localizationMode_ == 1){
				this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
				this->depthOdomSync_.reset(new message_filters::Synchronizer<depthOdomSync>(depthOdomSync(100), *this->depthSub_, *this->odomSub_));
				this->depthOdomSync_->registerCallback(boost::bind(&occMap::depthOdomCB, this, _1, _2));
			}
			else{
				ROS_ERROR("[OccMap]: Invalid localization mode!");
				exit(0);
			}
		}
		else if (this->rangeSensorInputMode_ == 1){
			// pointcloud callback
			this->pointcloudSub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(this->nh_, this->pointcloudTopicName_, 1));
			if (this->localizationMode_ == 0){
				this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 1));
				this->pointcloudPoseSync_.reset(new message_filters::Synchronizer<pointcloudPoseSync>(pointcloudPoseSync(100), *this->pointcloudSub_, *this->poseSub_));
				this->pointcloudPoseSync_->registerCallback(boost::bind(&occMap::pointcloudPoseCB, this, _1, _2));
			}
			else if (this->localizationMode_ == 1){
				this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 1));
				this->pointcloudOdomSync_.reset(new message_filters::Synchronizer<pointcloudOdomSync>(pointcloudOdomSync(100), *this->pointcloudSub_, *this->odomSub_));
				this->pointcloudOdomSync_->registerCallback(boost::bind(&occMap::pointcloudOdomCB, this, _1, _2));
			}
			else{
				ROS_ERROR("[OccMap]: Invalid localization mode!");
				exit(0);
			}
		}
		else{
			ROS_ERROR("[OccMap]: Invalid sensor input mode!");
			exit(0);
		}

		// semantic map: sync semantic and sensors
		if (this->sensorFusionMode_ == 0){
			this->depthSyncSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 1));
		}
		else{
			this->depthSyncSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, "/registered_depth_image", 1));
		}
		this->odomSyncSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 1));
		this->detection2DSyncSub_.reset(new message_filters::Subscriber<vision_msgs::Detection2DArray>(this->nh_, this->rawDetectionTopicName_, 1));
		this->depthDetectionOdomSync_.reset(new message_filters::Synchronizer<depthDetectionOdomSyncPolicy>(depthDetectionOdomSyncPolicy(100), *this->depthSyncSub_, *this->detection2DSyncSub_, *this->odomSyncSub_));
		this->depthDetectionOdomSync_->registerCallback(boost::bind(&occMap::depthDetectionOdomCB, this, _1, _2, _3));

		this->registeredMapSub_ = this->nh_.subscribe(this->denseCloudTopicName_, 1, &occMap::denseCloudCB, this);

		// occupancy update callback
		ROS_INFO("register occupancy update callback");
		this->occTimer_ = this->nh_.createTimer(ros::Duration(0.1), &occMap::updateOccupancyCB, this);

		// map inflation callback
		this->inflateTimer_ = this->nh_.createTimer(ros::Duration(0.1), &occMap::inflateMapCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &occMap::visCB, this);

		// object map update callback
		this->objectMapTimer_ = this->nh_.createTimer(ros::Duration(this->objectMapDuration_), &occMap::updateObjectMapCB, this);
	}

	void occMap::registerPub(){
		this->depthCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("depth_cloud", 10);
		this->freeRegionPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("free_region", 10);
		this->mapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("voxel_map", 10);
		this->inflatedMapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("inflated_voxel_map", 10);
		this->map2DPub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_map_2D", 10);
		this->mapExploredPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("explored_voxel_map",10);
		this->mapUnknownPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("unknown_voxel_map",10);
		this->objectVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("object_bounding_box", 10);
		this->objectMapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("object_map", 10);
		this->objectDenseCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("object_dense_cloud", 10);
		this->debugPointCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ +  "/debug_pointcloud", 10);
		image_transport::ImageTransport it(this->nh_);
		this->depthImgWithDetectPub_ = it.advertise("depth_image_with_detect", 10);
		this->semanticObjPub_ = this->nh_.advertise<map_manager::semanticObjArrayMsg>("/semantic_objects", 10);
	}

	void occMap::initCollisionFreeBox(){
		// for point in -1.5m to 1.5m, set unkown to free
		ROS_INFO("[OccMap]: Init collision free box.");
		Eigen::Vector3d pointPos;
		Eigen::Vector3i pointIndex;
		int address;
		for (double x=-1.5; x<=1.5; x+=this->mapRes_/2){
			for (double y=-1.5; y<=1.5; y+=this->mapRes_/2){
				for (double z=this->groundHeight_-0.3; z<=2.5; z+=this->mapRes_/2){
					pointPos(0) = x; pointPos(1) = y; pointPos(2) = z;
					// this->posToIndex(pointPos, pointIndex);
					// address = this->indexToAddress(pointIndex);
					if (this->isUnknown(pointPos)){
						// std::cout << "Set free: " << pointPos.transpose() << std::endl;
						this->setFree(pointPos);
					}
				}
			}
		}
	}

	void occMap::depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose){
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

	void occMap::depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom){
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

	}

	void occMap::pointcloudPoseCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const geometry_msgs::PoseStampedConstPtr& pose){
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

	void occMap::pointcloudOdomCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const nav_msgs::OdometryConstPtr& odom){
		// ROS_INFO("=================================in pointcloudOdomCB=========================");
		// directly get the point cloud
		pcl::PCLPointCloud2 pclPC2;
		pcl_conversions::toPCL(*pointcloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
		pcl::fromPCLPointCloud2(pclPC2, this->pointcloud_);

		// ros::Time pointcloud_time = pointcloud->header.stamp;
        // ros::Time odom_time = odom->header.stamp;
		// ROS_INFO_STREAM("PointCloud timestamp: " << pointcloud_time);
        // ROS_INFO_STREAM("Odometry timestamp: " << odom_time);

		// store current position and orientation (camera)
		this->updateSensorPose(odom);

		if (this->isInMap(this->rangeSensorPosition_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}


	void occMap::depthDetectionOdomCB(const sensor_msgs::ImageConstPtr& img, const vision_msgs::Detection2DArrayConstPtr& detections, const nav_msgs::OdometryConstPtr& odom){
		//ROS_INFO("=================================in depthDetectionOdomCB=========================");
		
		// store current depth image
		auto start = std::chrono::high_resolution_clock::now();
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        //ROS_INFO("Image encoding: %s", img->encoding.c_str());

		if (img->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            imgPtr->image.copyTo(this->depthImage_);
        } else if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
            imgPtr->image.copyTo(this->depthImage_);
        } else {
            ROS_WARN("Unexpected depth image encoding: %s", img->encoding.c_str());
            return;
        }
		imgPtr->image.copyTo(this->depthImage_);
		// get synchronized odometry
		Eigen::Matrix4d syncMap2Body;
		this->extractPoseMatrix(odom, syncMap2Body);

		// store current detections
		this->objectDetections_.clear();
		std::vector<vision_msgs::Detection2D> valid2dDetections;
		std::vector<std::vector<Eigen::Vector3d>> detectionPointCloud;
		std::vector<int> closestObjMapIdx;
		for (const auto& detection : detections->detections){
			if (std::find(this->objOfInterest_.begin(), this->objOfInterest_.end(), detection.results[0].id) == this->objOfInterest_.end()){
				continue;
			}
			// skip if confidence score is too low
			if (detection.results[0].score < this->detectThresh_){
				// ROS_INFO("low conf prob: %f", detection.results[0].score);
				continue;
			}
			std::cout << "get valid object id: " << detection.results[0].id << std::endl;
			Eigen::Vector3d objCenter;
			std::vector<Eigen::Vector3d> objPoints;
			if (this->get3dFromRaw2dSeg(detection, this->depthImage_, objPoints, objCenter, syncMap2Body)){
				std::cout << "objPoints size: " << objPoints.size() << std::endl;
				if (objPoints.size() > 0){
					detectionPointCloud.push_back(objPoints);
					valid2dDetections.push_back(detection);
				}
			}
		}
		// ROS_WARN("extracted %d objects", detectionPointCloud.size());

		// debug
		pcl::PointCloud<pcl::PointXYZ> debugPointCloud;
		debugPointCloud.header.frame_id = "map";
		debugPointCloud.height = 1;
		debugPointCloud.is_dense = true;
		pcl::PointXYZ pt;
		for (const auto& objPoints : detectionPointCloud){
			for (const auto& p : objPoints){
				pt.x = p(0);
				pt.y = p(1);
				pt.z = p(2);
				debugPointCloud.push_back(pt);
			}
		}
		debugPointCloud.width = debugPointCloud.points.size();
		sensor_msgs::PointCloud2 debugPointCloudMsg;
		pcl::toROSMsg(debugPointCloud, debugPointCloudMsg);
		this->debugPointCloudPub_.publish(debugPointCloudMsg);
		//ROS_WARN("debug point cloud published with size: %d", debugPointCloud.points.size());
		// ROS_INFO("data association");
		for (const auto& detection : this->objectDetections_){
			
			Eigen::Vector3d detectionCenter(detection.bbox.center.position.x, detection.bbox.center.position.y, detection.bbox.center.position.z);
			Eigen::Vector3d detctionSize(detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z);
			// std::cout << "detection center: " << detectionCenter.transpose() << std::endl;
			// std::cout << "detection size: " << detctionSize.transpose() << std::endl;
			// get the closest object map index
			double maxIOU = 0;
			int bestIdx = -1;
			int idx = 0;
			for (auto& objMap : this->objectMapList_){
				// if (objMap->label_ == detection.results[0].id){
				
				Eigen::Vector3d objectCenter(objMap->lastDetection_.bbox.center.position.x, objMap->lastDetection_.bbox.center.position.y, objMap->lastDetection_.bbox.center.position.z);
				Eigen::Vector3d objectSize(objMap->lastDetection_.bbox.size.x, objMap->lastDetection_.bbox.size.y, objMap->lastDetection_.bbox.size.z);
				// use object map bounding box
				// vision_msgs::Detection3D bbox;
				// objMap->getBoundingBox(bbox);
				// Eigen::Vector3d objectCenter(bbox.bbox.center.position.x, bbox.bbox.center.position.y, bbox.bbox.center.position.z);
				// Eigen::Vector3d objectSize(bbox.bbox.size.x, bbox.bbox.size.y, bbox.bbox.size.z);
				double IOU = calc3dIOO(detectionCenter, detctionSize, objectCenter, objectSize);
				// std::cout << "detection center: " << detectionCenter(0) << " " << detectionCenter(1) << " " << detectionCenter(2) << std::endl;
				// std::cout << "object last detection center: " << objectCenter(0) << " " << objectCenter(1) << " " << objectCenter(2) << std::endl;
				// std::cout << "detection size: " << detctionSize(0) << " " << detctionSize(1) << " " << detctionSize(2) << std::endl;
				// std::cout << "object size: " << objectSize(0) << " " << objectSize(1) << " " << objectSize(2) << std::endl;
				// std::cout << "IOU: " << IOU << std::endl;
				if (IOU > maxIOU and IOU > this->objectTrackThresh_){
					maxIOU = IOU;
					bestIdx = idx;
				}
				// }
				++idx;
			}
			closestObjMapIdx.push_back(bestIdx);
			
		}
		// ROS_WARN("closest object map index size: %d", closestObjMapIdx.size());

		// associate detections with object map
		// ROS_INFO("observation association");
		// remember to hanle situation where closest index is -1
		for (size_t i=0; i<detectionPointCloud.size(); ++i){
			// std::cout << "points size: " << detectionPointCloud[i].size() << std::endl;
			if (detectionPointCloud[i].size() == 0){
				continue;
			}
			bool newObj = false;
			if (closestObjMapIdx[i] == -1){
				// create new object map
				newObj = true;
				// ROS_INFO("create new object map");
			}
			int closestIdx = closestObjMapIdx[i];
			if (newObj){
				// calc view angle base
				Eigen::Vector3d center(
				this->objectDetections_[i].bbox.center.position.x, 
				this->objectDetections_[i].bbox.center.position.y, 
				this->objectDetections_[i].bbox.center.position.z);
				Eigen::Vector3d ray = center - syncMap2Body.block<3,1>(0,3);
				double viewAngleBase = atan2(ray(1), ray(0));
				if (viewAngleBase < 0){
					viewAngleBase += 2 * M_PI;
				}
				std::shared_ptr<mapManager::objectMap> objectMap = std::make_shared<mapManager::objectMap>(
				this->objectMapParams_,
				center, 
				this->objectDetections_[i].results[0].id,
				viewAngleBase,
				this->sensorManager_);
				this->objectMapList_.push_back(objectMap);
				// ROS_INFO("after create new object map, object map list size: %d", this->objectMapList_.size());
				closestIdx = this->objectMapList_.size() - 1;
			}
			for (const auto& point : detectionPointCloud[i]){
				Eigen::Vector3d posObjFrame;
				this->objectMapList_[closestIdx]->global2ObjectFrame(point, posObjFrame);
				if (this->objectMapList_[closestIdx]->isInMapRange(posObjFrame)){
					this->objectMapList_[closestIdx]->addObservation(posObjFrame);
				}
				// HANDLE ELSE: if the point is not in the object map, then create a new object map?
			}

			// update last detection
			this->objectMapList_[closestIdx]->lastDetection_ = this->objectDetections_[i];
			// associate segmentation mask
			this->objectMapList_[closestIdx]->updateMask(valid2dDetections[i].source_img, valid2dDetections[i].results[0].id, this->depthImage_, syncMap2Body);
		}
		//ROS_WARN("updated object map list size: %d", this->objectMapList_.size());

		// this->drawBoundingBox(*detections, this->depthImage_);
		Eigen::Vector3d position = syncMap2Body.block<3,1>(0,3);
		if (this->isInMap(position)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = end - start;
		// ROS_WARN("Time taken by depthDetectionOdomCB: %f", elapsed.count());
		// std::cout << "object map list size: " << this->objectMapList_.size() << std::endl;
	}

	void occMap::denseCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud){
		auto start = std::chrono::high_resolution_clock::now();
		// directly get the point cloud
		pcl::PCLPointCloud2 pclPC2;
		pcl_conversions::toPCL(*cloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
		pcl::fromPCLPointCloud2(pclPC2, this->registeredDenseCloud_);

		// Get the transform from "lidarMap" to "Map"
		static tf2_ros::Buffer tfBuffer;
		static tf2_ros::TransformListener tfListener(tfBuffer);

		geometry_msgs::TransformStamped transformStamped;
		Eigen::Affine3d LidarToMap = Eigen::Affine3d::Identity();
		// try {
		// 	transformStamped = tfBuffer.lookupTransform("map", "lidarMap", ros::Time(0), ros::Duration(1.0));
		// 	LidarToMap = tf2::transformToEigen(transformStamped.transform);
		// } catch (tf2::TransformException &ex) {
		// 	ROS_WARN("Could not transform from lidarMap to map: %s", ex.what());
		// 	return;
		// }
		
		// loop through the point cloud and check time
		int count = 0;
		
		for (size_t i=0; i<this->registeredDenseCloud_.size(); ++i){
			// registered map point from registeredMapCloud, which is a pcl point cloud
			Eigen::Vector3d pointLidarMap(this->registeredDenseCloud_.points[i].x, this->registeredDenseCloud_.points[i].y, this->registeredDenseCloud_.points[i].z);
			if (pointLidarMap.hasNaN()){
				continue;
			}

			// Transform point from "lidarMap" to "Map" frame
			Eigen::Vector3d pointGlobal = LidarToMap * pointLidarMap;

			for (const auto& objectMap : this->objectMapList_){
				objectMap->updateGlobalObjDenseCloud(pointGlobal);
				// count += 1;
			}
		}
		// ROS_INFO("update object dense cloud for %d points", count);
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = end - start;
	}

	
	void occMap::updateOccupancyCB(const ros::TimerEvent& ){
		// ROS_INFO("=================================in updateOccupancyCB=========================");
		if (not this->occNeedUpdate_){
			return;
		}
		// cout << "update occupancy map" << endl;
		ros::Time startTime, endTime;
		
		startTime = ros::Time::now();
		if (this->rangeSensorInputMode_ == 0){
			// project 3D points from depth map
			auto sensorDataPtr = std::make_shared<cv::Mat>(this->depthImage_);
			this->sensorManager_->getGlobalRangeSensorInput(this->map2Body_, sensorDataPtr , this->projPoints_, this->projPointsNum_);
		}
		else if (this->rangeSensorInputMode_ == 1){
			// directly get pointcloud
			auto sensorDataPtr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(this->pointcloud_);
			this->sensorManager_->getGlobalRangeSensorInput(this->map2Body_, sensorDataPtr, this->projPoints_, this->projPointsNum_);
		}
		
		// raycasting and update occupancy
		this->raycastUpdate();
		// this->initCollisionFreeBox();

		// clear local map
		if (this->cleanLocalMap_){
			this->cleanLocalMap();
		}

		endTime = ros::Time::now();
		if (this->verbose_){
			//cout << this->hint_ << ": Occupancy update time: " << (endTime - startTime).toSec() << " s." << endl;
		}
		this->occNeedUpdate_ = false;
		this->mapNeedInflate_ = true;
	}

	void occMap::inflateMapCB(const ros::TimerEvent& ){
		ros::Time startTime, endTime;
		startTime = ros::Time::now();
		// inflate local map:
		if (this->mapNeedInflate_){
			this->inflateLocalMap();
			this->mapNeedInflate_ = false;
			this->esdfNeedUpdate_ = true;
		}
		endTime = ros::Time::now();
		if (this->verbose_){
			//cout << this->hint_ << ": inflateMap CB time: " << (endTime - startTime).toSec() << " s." << endl;
		}
	}

	void occMap::raycastUpdate(){
		// this->isVisited_.assign(this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2), false);
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
		int rayendVoxelID;
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
				// continue;
				if (not this->isInMap(currPoint)){
					ROS_WARN("Error: adjusted point %f %f %f is not in map range", currPoint(0), currPoint(1), currPoint(2));
				}
			}

			// check whether the point exceeds the maximum raycasting length
			double length = (currPoint - this->rangeSensorPosition_).norm();
			if (length > this->raycastMaxLength_){
				// register far away point into map but do not do ray casting
				this->updateOccupancyInfo(currPoint, not pointAdjusted);
				// then do raycasting for the farthest point on the ray's direction
				currPoint = this->adjustPointRayLength(currPoint);
				pointAdjusted = true;
				// continue;
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
			rayendVoxelID = this->posToAddress(currPoint);

			if (rayendVoxelID < 0 || rayendVoxelID >= this->flagRayend_.size()) {
				// ROS_WARN("Error: rayendVoxelID out of range: %d", rayendVoxelID);
				// ROS_WARN("currPoint: %f %f %f", currPoint(0), currPoint(1), currPoint(2));
				continue;
			}

			if (this->flagRayend_[rayendVoxelID] == this->raycastNum_){
				continue; // skip
			}
			else{
				this->flagRayend_[rayendVoxelID] = this->raycastNum_;
			}



			// raycasting for update occupancy
			this->raycaster_.setInput(currPoint/this->mapRes_, this->rangeSensorPosition_/this->mapRes_);
			Eigen::Vector3d rayPoint, actualPoint;
			Eigen::Vector3i prevIdx;
			this->posToIndex(currPoint, prevIdx);
			while (this->raycaster_.step(rayPoint)){
				actualPoint = rayPoint;

				actualPoint(0) += 0.5;
				actualPoint(1) += 0.5;
				actualPoint(2) += 0.5;
				actualPoint *= this->mapRes_;

				Eigen::Vector3i idx;
				this->posToIndex(actualPoint, idx);
				int raycastVoxelID = this->indexToAddress(idx);

				if (raycastVoxelID < 0 || raycastVoxelID >= this->flagTraverse_.size()) {
					// ROS_WARN("Error: raycastVoxelID out of range: %d", raycastVoxelID);
					// ROS_WARN("actualPoint: %f %f %f", actualPoint(0), actualPoint(1), actualPoint(2));
					continue;
				}

				// raycastVoxelID = this->posToAddress(actualPoint);
				if (this->flagTraverse_[raycastVoxelID] == this->raycastNum_){
					break; // skip
				}
				else{
					this->flagTraverse_[raycastVoxelID] = this->raycastNum_;
				}

				// if(prevIdx(2) == idx(2) and this->isOccupied(idx)){
				// 	continue;
				// }

				// ACCUMULATE OCCUPANCY INFO
				// if(this->isOccupied(idx)){
				// 	continue;
				// }
				// else{
				// 	raycastVoxelID = this->updateOccupancyInfo(actualPoint, false);
				// }
				// raycastVoxelID = this->updateOccupancyInfo(actualPoint, false);

				prevIdx = idx;
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
		// this->updateVoxelCacheCopy_ = this->updateVoxelCache_;
		std::queue<Eigen::Vector3i> empty;
   		std::swap( this->updateVoxelCacheCopy_, empty );
		
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
				continue;
			}

			this->occupancy_[cacheAddress] = std::min(std::max(this->occupancy_[cacheAddress]+logUpdateValue, this->pMinLog_), this->pMaxLog_);

			if (this->isFree(cacheIdx)){
				this->updateVoxelCacheCopy_.push(cacheIdx);
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

		
	}

	void occMap::cleanLocalMap(){
		Eigen::Vector3i posIndex;
		this->posToIndex(this->rangeSensorPosition_, posIndex);
		Eigen::Vector3i innerMinBBX = posIndex - this->localMapVoxel_;
		Eigen::Vector3i innerMaxBBX = posIndex + this->localMapVoxel_;
		Eigen::Vector3i outerMinBBX = innerMinBBX - Eigen::Vector3i(5, 5, 5);
		Eigen::Vector3i outerMaxBBX = innerMaxBBX + Eigen::Vector3i(5, 5, 5);
		this->boundIndex(innerMinBBX);
		this->boundIndex(innerMaxBBX);
		this->boundIndex(outerMinBBX);
		this->boundIndex(outerMaxBBX);

		// clear x axis
		for (int y=outerMinBBX(1); y<=outerMaxBBX(1); ++y){
			for (int z=outerMinBBX(2); z<=outerMaxBBX(2); ++z){
				for (int x=outerMinBBX(0); x<=innerMinBBX(0); ++x){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int x=innerMaxBBX(0); x<=outerMaxBBX(0); ++x){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;					
				}
			}
		}

		// clear y axis
		for (int x=outerMinBBX(0); x<=outerMaxBBX(0); ++x){
			for (int z=outerMinBBX(2); z<=outerMaxBBX(2); ++z){
				for (int y=outerMinBBX(1); y<=innerMinBBX(1); ++y){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int y=innerMaxBBX(1); y<=outerMaxBBX(1); ++y){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}
			}
		}

		// clear z axis
		for (int x=outerMinBBX(0); x<=outerMaxBBX(0); ++x){
			for (int y=outerMinBBX(1); y<=outerMaxBBX(1); ++y){
				for (int z=outerMinBBX(2); z<=innerMinBBX(2); ++z){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int z=innerMaxBBX(2); z<=outerMaxBBX(2); ++z){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}
			}
		}

		this->initCollisionFreeBox();
	}

	void occMap::inflateLocalMap(){
		Eigen::Vector3i clearIndex;
		// clear previous data in current data range
		for (int x=this->localBoundMin_(0); x<=this->localBoundMax_(0); ++x){
			for (int y=this->localBoundMin_(1); y<=this->localBoundMax_(1); ++y){
				for (int z=this->localBoundMin_(2); z<=this->localBoundMax_(2); ++z){
					clearIndex(0) = x; clearIndex(1) = y; clearIndex(2) = z;
					this->occupancyInflated_[this->indexToAddress(clearIndex)] = false;
				}
			}
		}

		// inflate size
		double xy_min = std::min(this->robotSize_(0), this->robotSize_(1));

		int xInflateSize = ceil(xy_min/(2*this->mapRes_));
		int yInflateSize = ceil(xy_min/(2*this->mapRes_));
		int zInflateSize = ceil(this->robotSize_(2)/(2*this->mapRes_));

		// inflate based on current occupancy
		Eigen::Vector3i pointIndex, inflateIndex;
		int inflateAddress;
		const int  maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		for (int x=this->localBoundMin_(0); x<=this->localBoundMax_(0); ++x){
			for (int y=this->localBoundMin_(1); y<=this->localBoundMax_(1); ++y){
				for (int z=this->localBoundMin_(2); z<=this->localBoundMax_(2); ++z){
					pointIndex(0) = x; pointIndex(1) = y; pointIndex(2) = z;
					if (this->isOccupied(pointIndex)){
						for (int ix=-xInflateSize; ix<=xInflateSize; ++ix){
							for (int iy=-yInflateSize; iy<=yInflateSize; ++iy){
								for (int iz=-zInflateSize; iz<=zInflateSize; ++iz){
									inflateIndex(0) = pointIndex(0) + ix;
									inflateIndex(1) = pointIndex(1) + iy;
									inflateIndex(2) = pointIndex(2) + iz;
									inflateAddress = this->indexToAddress(inflateIndex);
									if ((inflateAddress < 0) or (inflateAddress > maxIndex)){
										continue; // those points are not in the reserved map
									} 
									this->occupancyInflated_[inflateAddress] = true;
								}
							}
						}
					}
				}
			}
		}
	}

	// void occMap::get3dFromRaw2d(const vision_msgs::Detection2D& detection, cv::Mat& depthImage,
	// 							Eigen::Vector3d& position, Eigen::Quaterniond& orientation){
	// 	// ROS_INFO("=================================in get 3d box from raw detection=========================");
	// 	// 1. retrive 2D detection result
    //     int topX = int(detection.bbox.center.x) - int(detection.bbox.size_x/2.0);
    //     int topY = int(detection.bbox.center.y) - int(detection.bbox.size_y/2.0);
    //     int xPixelWidth = int(detection.bbox.size_x); 
    //     int yPixelWidth = int(detection.bbox.size_y); 
	// 	int label_id = detection.results[0].id;

	// 	// 2. get thickness estimation (double MAD: double Median Absolute Deviation)
    //     uint16_t* rowPtr;
    //     double depth;
    //     int vMin = std::max(topY, this->depthFilterMargin_);
    //     int uMin = std::max(topX, this->depthFilterMargin_);
    //     int vMax = std::min(topY+yPixelWidth, this->imgRows_-this->depthFilterMargin_);
    //     int uMax = std::min(topX+xPixelWidth, this->imgCols_-this->depthFilterMargin_);
    //     std::vector<double> depthValues;
		
	// 	// record the depth values in the potential regions
	// 	if (depthImage.empty()){
	// 		ROS_INFO("depth image is empty");
	// 		return;
	// 	}

	// 	std::vector<Eigen::Vector3d> objPoints;
	// 	Eigen::Vector3d currPointMap;
		
    //     for (int v=vMin; v<vMax; v+=1){ // row
    //         rowPtr = depthImage.ptr<uint16_t>(v); // NOTE: should use aligned depth image in real world experiments
	// 		rowPtr += uMin;
    //         for (int u=uMin; u<uMax; u+=1){ // column
    //             depth = (*rowPtr) /this->depthScale_; // get 3D point in camera frame
	// 			if (this->sensorFusionMode_ == 0){
	// 				this->pinholeCam2Map(u, v, depth, currPointMap, position, orientation);
	// 			}
	// 			else if (this->sensorFusionMode_ == 1){
	// 				this->panoCam2Map(u, v, depth, currPointMap, 375, 219.4734627, depthImage.cols, position, orientation);// y0, fy, cols
	// 			}
	// 			else{
	// 				ROS_WARN("Unknown sensor fusion mode");
	// 				return;
	// 			}
				
	// 			// filterout ground points and unreasonable depth
    //             if (depth >= this->depthMinValue_ and depth <= this->depthMaxValue_ and currPointMap(2)>=this->groundHeight_+0.2){
    //                 depthValues.push_back(depth);
	// 				objPoints.push_back(currPointMap);
    //             }
    //             // ++rowPtr;
	// 			rowPtr += 1;
    //         }
    //     }
	// 	// std::cout << this->hint_ << "objPoints size: " << objPoints.size() << std::endl;
    //     if (depthValues.size() == 0){ // in case of out of range
    //         return;
    //     }
	// 	if (objPoints.size()==0){
	// 		return;
	// 	}

	// 	// dbscan clustring
	// 	std::vector<std::vector<Eigen::Vector3d>> clusters;
	// 	std::vector<mapManager::Point> pointsDB;
	// 	mapManager::eigenToDBPointVec(objPoints, pointsDB, objPoints.size());
	// 	// mapManager::pclToDBPointVec(objPoints, pointsDB, output_cloud.points.size());
	// 	// std::cout << this->hint_ << "pointsDB size: " << pointsDB.size() << std::endl;
	// 	this->dbscan_.reset(new mapManager::DBSCAN(this->dbMinPointsCluster_, this->dbEpsilon_, pointsDB));
	// 	this->dbscan_->run();
	// 	if (this->dbscan_->cluster_num<=0){
	// 		ROS_WARN("No cluster found in 2D detection.");
	// 		return;
	// 	}
	// 	// ROS_INFO("clustering done");
	// 	clusters.resize(this->dbscan_->cluster_num);
	// 	for (size_t i=0; i<this->dbscan_->m_points.size(); ++i){
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

	// 	// pcl::PointCloud<pcl::PointXYZ> debugPointCloud;
	// 	// debugPointCloud.header.frame_id = "map";
	// 	// debugPointCloud.height = 1;
	// 	// debugPointCloud.is_dense = true;
	// 	// pcl::PointXYZ pt;
	// 	// for (const auto& p : objPoints){
	// 	// 	pt.x = p(0);
	// 	// 	pt.y = p(1);
	// 	// 	pt.z = p(2);
	// 	// 	debugPointCloud.push_back(pt);
	// 	// }
	// 	// debugPointCloud.width = debugPointCloud.points.size();
	// 	// sensor_msgs::PointCloud2 debugPointCloudMsg;
	// 	// pcl::toROSMsg(debugPointCloud, debugPointCloudMsg);
	// 	// this->debugPointCloudPub_.publish(debugPointCloudMsg);

	// 	bool newObj = true;
	// 	for (size_t i=0 ; i<clusters[closestClusterIdx].size(); ++i){
	// 		Eigen::Vector3d point = clusters[closestClusterIdx][i];
	// 		Eigen::Vector3d posObjFrmae;
			
	// 		// which object does this point belong to
	// 		for (auto& objectMap : this->objectMapList_){
	// 			// if the point is in the object map, then add it to the object
	// 			objectMap->global2ObjectFrame(point, posObjFrmae);
	// 			if (objectMap->isInMapRange(posObjFrmae)){
	// 				objectMap->addObservation(posObjFrmae);
	// 				newObj = false;
	// 				break;
	// 			}
	// 		}
	// 		// if the point is not in any object map, then create a new object map
	// 		if (newObj){
	// 			Eigen::Vector3d ray = closestClusterCenter - this->rangeSensorPosition_;
	// 			double viewAngleBase = atan2(ray(1), ray(0));
	// 			if (viewAngleBase < 0){
	// 				viewAngleBase += 2 * M_PI;
	// 			}
	// 			// std::cout << "view angle base: " << viewAngleBase << std::endl; 
	// 			std::shared_ptr<mapManager::objectMap> objectMap = std::make_shared<mapManager::objectMap>(this->objectMapParams_, closestClusterCenter, label_id, viewAngleBase);
	// 			this->objectMapList_.push_back(objectMap);
	// 			objectMap->global2ObjectFrame(point, posObjFrmae);
	// 			objectMap->addObservation(posObjFrmae);
	// 		}
	// 	}
		
	// }

	bool occMap::get3dFromRaw2dSeg(const vision_msgs::Detection2D& detection, cv::Mat& depthImage, std::vector<Eigen::Vector3d>& objPoints, 
										Eigen::Vector3d& objCenter, const Eigen::Matrix4d& syncBody2Map){
		// ROS_INFO("=================================in get 3d box from raw detection=========================");
		auto start_time = std::chrono::high_resolution_clock::now();
		// record running time
		
		// retrive 2D detection result
        int topLeftX = int(detection.bbox.center.x) - int(detection.bbox.size_x/2.0);
        int topLeftY = int(detection.bbox.center.y) - int(detection.bbox.size_y/2.0);
        int xPixelWidth = int(detection.bbox.size_x); 
        int yPixelWidth = int(detection.bbox.size_y); 
		int label_id = detection.results[0].id;
        uint16_t* rowPtr;
        double depth;
        int vMin = std::max(topLeftY, 0);
        int uMin = std::max(topLeftX, 0);
        int vMax = std::min(topLeftY+yPixelWidth, this->imgRows_-0);
        int uMax = std::min(topLeftX+xPixelWidth, this->imgCols_-0);
        std::vector<double> depthValues;

		// extract semantic mask
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(detection.source_img, detection.source_img.encoding);
		cv::Mat mask = imgPtr->image;
		// record the depth values in the potential regions
		if (depthImage.empty()){
			ROS_INFO("depth image is empty");
			return false;
		}
		Eigen::Vector3d currPointMap;
		std::vector<Eigen::Vector3d> points;
        for (int v=vMin; v<vMax; v+=this->dbDownsampleFactor_){ // row
            rowPtr = depthImage.ptr<uint16_t>(v); // NOTE: should use aligned depth image in real world experiments
			rowPtr += uMin;
            for (int u=uMin; u<uMax; u+=this->dbDownsampleFactor_){ // column
                depth = (*rowPtr) /this->depthScale_; // get 3D point in camera frame
				// use sensor model to get 3D point
				this->sensorManager_->colorSensorRawInput2Global(syncBody2Map, Eigen::Vector3d(u, v, depth), currPointMap);
				// mask filter
				if (mask.at<uchar>(v, u) == 0){
					continue;
				}
				// filterout ground points and unreasonable depth
                if (depth >= this->depthMinValue_ and depth <= this->depthMaxValue_ and currPointMap(2)>=this->objectFilterHeight_){
                    depthValues.push_back(depth);
					points.push_back(currPointMap);
                }
                // ++rowPtr;
				rowPtr += this->dbDownsampleFactor_;
            }
        }
		// std::cout << "[occMap::get3dFromRaw2dSeg]: "<< "points size: " << points.size() << std::endl;
        if (depthValues.size() == 0){ // in case of out of range
            return false;
        }
		if (points.size()==0){
			return false;
		}
		auto retrive_end_time = std::chrono::high_resolution_clock::now();
		auto retrive_duration = std::chrono::duration_cast<std::chrono::milliseconds>(retrive_end_time - start_time);
		std::cout << "[occMap::get3dFromRaw2dSeg]: retrive duration: " << retrive_duration.count() << " ms" << std::endl;


		// dbscan clustring
		std::vector<std::vector<Eigen::Vector3d>> clusters;
		std::vector<mapManager::Point> pointsDB;
		mapManager::eigenToDBPointVec(points, pointsDB, points.size());
		if (pointsDB.size() >100000){
			ROS_WARN("Too many points in the cluster, skip this frame");
			return false;
		}
		// mapManager::pclToDBPointVec(points, pointsDB, output_cloud.points.size());
		// std::cout << this->hint_ << "pointsDB size: " << pointsDB.size() << std::endl;
		this->dbscan_.reset(new mapManager::DBSCAN(this->dbMinPointsCluster_, this->dbEpsilon_, pointsDB));
		this->dbscan_->run();
		if (this->dbscan_->cluster_num<=0){
			// ROS_WARN("No cluster found in 2D detection.");
			return false;
		}
		auto cluster_end_time = std::chrono::high_resolution_clock::now();
		auto cluster_duration = std::chrono::duration_cast<std::chrono::milliseconds>(cluster_end_time - retrive_end_time);
		// std::cout <<"[occMap::get3dFromRaw2dSeg]: cluster groups num: " << this->dbscan_->cluster_num << 
		// "cluster duration: " << cluster_duration.count() << " ms" << std::endl;
		// ROS_INFO("clustering done");
		clusters.resize(this->dbscan_->cluster_num);
		for (size_t i=0; i<this->dbscan_->m_points.size(); ++i){
			int clusterID = this->dbscan_->m_points[i].clusterID;
			if (clusterID >0){
				mapManager::Point point = this->dbscan_->m_points[i];
				Eigen::Vector3d p = mapManager::dbPointToEigen(point);
				clusters[clusterID-1].push_back(p);
			}
		}

		// find out the closest cluster
		int closestClusterIdx = 0;
		Eigen::Vector3d closestClusterCenter = Eigen::Vector3d(100, 100, 100);
		for (size_t i=0 ; i<clusters.size(); ++i){ // caculate center of each cluster
			Eigen::Vector3d center = Eigen::Vector3d::Zero();
			for (const auto& p : clusters[i]){
				center += p;
			}
			center /= clusters[i].size();
			if ((center - this->rangeSensorPosition_).norm() < (closestClusterCenter - this->rangeSensorPosition_).norm()){
				closestClusterIdx = i;
				closestClusterCenter = center;
			}
		}
		objCenter = closestClusterCenter;

        std::vector<Eigen::Vector3d> filtered_points;
       
        // After selecting the closestClusterIdx
        std::vector<Eigen::Vector3d> cluster_points = clusters[closestClusterIdx];
        if (cluster_points.size() < 10){
            // Compute centroid
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const auto& p : cluster_points) centroid += p;
            centroid /= cluster_points.size();

            // Compute standard deviation
            double sum_sq = 0;
            for (const auto& p : cluster_points) sum_sq += (p - centroid).squaredNorm();
            double stddev = sqrt(sum_sq / cluster_points.size());

            // Filter out points that are too far from the centroid
            std::vector<Eigen::Vector3d> filtered_points;
            for (const auto& p : cluster_points) {
                if ((p - centroid).norm() < 3 * stddev) {
                    filtered_points.push_back(p);
                }
            }
        } else {
            filtered_points = cluster_points;
        }

        

		auto process_end_time = std::chrono::high_resolution_clock::now();
		auto process_duration = std::chrono::duration_cast<std::chrono::milliseconds>(process_end_time - cluster_end_time);
		std::cout << "[occMap::get3dFromRaw2dSeg]: process duration: " << cluster_duration.count() << " ms" << std::endl;

		objPoints.clear();
		double xMin = 1000;
		double xMax = -1000;
		double yMin = 1000;
		double yMax = -1000;
		double zMin = 1000;
		double zMax = -1000;
        
		for (size_t i=0 ; i<filtered_points.size(); ++i){
			Eigen::Vector3d point = filtered_points[i];
			objPoints.push_back(point);
			xMin = std::min(xMin, point(0));
			xMax = std::max(xMax, point(0));
			yMin = std::min(yMin, point(1));
			yMax = std::max(yMax, point(1));
			zMin = std::min(zMin, point(2));
			zMax = std::max(zMax, point(2));
		}

		auto final_end_time = std::chrono::high_resolution_clock::now();
		auto final_duration = std::chrono::duration_cast<std::chrono::milliseconds>(final_end_time - start_time);

		// make objectDetection
		vision_msgs::Detection3D objectDetection;
		objectDetection.header.stamp = ros::Time::now();
		objectDetection.bbox.center.position.x = closestClusterCenter(0);
		objectDetection.bbox.center.position.y = closestClusterCenter(1);
		objectDetection.bbox.center.position.z = closestClusterCenter(2);
		objectDetection.bbox.size.x = xMax - xMin;
		objectDetection.bbox.size.y = yMax - yMin;
		objectDetection.bbox.size.z = zMax - zMin;
		objectDetection.results = detection.results;
		this->objectDetections_.push_back(objectDetection);
		// std::cout << "clustered obj center: " << closestClusterCenter.transpose() << std::endl;
		// std::cout << "clustered obj size: " << objectDetection.bbox.size.x << " " << objectDetection.bbox.size.y << " " << objectDetection.bbox.size.z << std::endl;
		// std::cout << "[occMap::get3dFromRaw2dSeg]: get3dfrom2dseg overall duration: " << final_duration.count() << " ms" << std::endl;
		return true;
	}

	void occMap::updateObjectViewAngle(mapManager::semanticObject& semanticObject){		
		Eigen::Vector3d ray = semanticObject.position - this->rangeSensorPosition_;
		ray.normalize();
		double viewAngle = atan2(ray(1), ray(0));
		Eigen::Vector3d camPos;
		// if(this->sensorFusionMode_ == 0){
		// 	camPos = this->rangeSensorPosition_;
		// }
		// else if (this->sensorFusionMode_ == 1){
		// 	camPos = this->rangeSensorPosition_;
		// 	camPos(2) -= 0.3;
		// }
		camPos = this->colorSensorPosition_;
		bool visible = this->isObjectVisible(camPos, semanticObject);
		if (not visible){
			return;
		}
		int viewAngleIdx = this->viewAngleToIndex(viewAngle);
		semanticObject.view_angles[viewAngleIdx] = 1;
	}

	// TODO: Use same function for checkvisibility and isObjectVisible!!!
	// helper funciton for other pkg to call to check if an object is visible from a given sample pose
	bool occMap::isObjectVisible(const Eigen::Vector3d& cameraPos, const semanticObject& semanticObject){
		// raycast from the current position to the object position
		double viewDistLowerBound = 0.5;
		double viewDistUpperBound = this->raycastMaxLength_;
		double viewDistance = 0.0;
		RayCaster raycasterTemp;
		raycasterTemp.setInput(cameraPos/this->mapRes_, semanticObject.position/this->mapRes_);
		Eigen::Vector3d rayPoint, actualPoint;
		while (raycasterTemp.step(rayPoint)){
			actualPoint = rayPoint;
			actualPoint(0) += 0.5;
			actualPoint(1) += 0.5;
			actualPoint(2) += 0.5;
			actualPoint *= this->mapRes_;
			if (this->isOccupied(actualPoint)){
				viewDistance = (actualPoint - cameraPos).norm();
				double occlusionDist = (semanticObject.position - actualPoint).norm();
				if (occlusionDist > std::max(semanticObject.size[0], semanticObject.size[1])){
					return false;
				}
			}
		}
		
		viewDistance = (semanticObject.position - cameraPos).norm();
		if (viewDistance<viewDistLowerBound or  viewDistance>viewDistUpperBound){
			// std::cout << "view distance: " << viewDistance << std::endl;
			return false;
		}
		return true;
	}

	bool occMap::checkVisibility(objectMap& objectMap){
		Eigen::Vector3d center = objectMap.position_;
		if (not this->sensorManager_->isInColorSensorFov(this->map2Body_, center)){
			return false;
		}
		
		// check if the object is visible from the observation position
        // do raycasting by occMapPtr in global occmap frame
        Eigen::Vector3d camPos, camPosObjFrame, rayEnd;
        camPos = this->colorSensorPosition_;
        // objectMap.global2ObjectFrame(camPos, camPosObjFrame);
        // rayEnd = objectMap.adjustPointInMap(camPosObjFrame);
        // objectMap.object2GlobalFrame(rayEnd, rayEnd);
        Eigen::Vector3d rayPoint, actualPoint;
        RayCaster raycaster;
        raycaster.setInput(camPos/this->mapRes_, center/this->mapRes_);
		// std::cout << "raycasting from " << rayEnd.transpose() << " to " << camPos.transpose() << std::endl;
        raycaster.step(rayPoint);
        while (raycaster.step(rayPoint)){
            actualPoint = rayPoint;
            actualPoint(0) += 0.5;
            actualPoint(1) += 0.5;
            actualPoint(2) += 0.5;
            actualPoint *= this->mapRes_;
            if (!this->isInMap(actualPoint)){
                break;
            }
            // check occlusion
            if (this->isOccupied(actualPoint)){
                // check if this point is inside the object's bounding box
				// get bounding box
				vision_msgs::Detection3D bbox;
				objectMap.getBoundingBox(bbox);
				// Convert geometry_msgs::Point to Eigen::Vector3d
				Eigen::Vector3d bboxCenter(bbox.bbox.center.position.x,
										bbox.bbox.center.position.y,
										bbox.bbox.center.position.z);

				// Convert geometry_msgs::Vector3 to Eigen::Vector3d
				Eigen::Vector3d bboxSize(bbox.bbox.size.x,
										bbox.bbox.size.y,
										bbox.bbox.size.z);
				Eigen::Vector3d bboxMax = bboxCenter + bboxSize/2.0;
				Eigen::Vector3d bboxMin = bboxCenter - bboxSize / 2.0;
				if (actualPoint(0) >= bboxMin(0) and actualPoint(0) <= bboxMax(0) and
					actualPoint(1) >= bboxMin(1) and actualPoint(1) <= bboxMax(1) and
					actualPoint(2) >= bboxMin(2) and actualPoint(2) <= bboxMax(2)){
					return true;
				}
				else{
					return false;
				}
            }
        }

        // if not occluded, this view angle is visible: update visibility list
        return true;
	}


	void occMap::visCB(const ros::TimerEvent& ){
		auto start_time = std::chrono::high_resolution_clock::now();
		this->publishProjPoints();
		this->publishMap();
		this->publishInflatedMap();
		// // this->publish2DOccupancyGrid();
		this->publishSemanticBoundingBoxes();
		this->publishSemanticObjMsg();
		this->publishObjectMap();
		this->publishGlobalObjectDenseCloud();
		auto end_time = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
		// ROS_INFO("vis took %ld milliseconds.", duration.count());
		// std::cout << "vis took " << duration.count() << " milliseconds." << std::endl;
	}


	void occMap::publishProjPoints(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> freeRegionCloud;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		for (int i=0; i<this->projPointsNum_; ++i){
			pt.x = this->projPoints_[i](0);
			pt.y = this->projPoints_[i](1);
			pt.z = this->projPoints_[i](2);
			double length = (projPoints_[i] - this->rangeSensorPosition_).norm();
			if (length > this->raycastMaxLength_){
				continue;
			}
			cloud.push_back(pt);
		}

		while (not this->updateVoxelCacheCopy_.empty()){
			Eigen::Vector3i ind = this->updateVoxelCacheCopy_.front();
			this->updateVoxelCacheCopy_.pop();
			Eigen::Vector3d pos;
			this->indexToPos(ind, pos);
			pt.x = pos(0);
			pt.y = pos(1);
			pt.z = pos(2);
			freeRegionCloud.push_back(pt);
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		freeRegionCloud.width = freeRegionCloud.points.size();
		freeRegionCloud.height = 1;
		freeRegionCloud.is_dense = true;
		freeRegionCloud.header.frame_id = "map";

		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->depthCloudPub_.publish(cloudMsg);

		sensor_msgs::PointCloud2 freeRegionCloudMsg;
		pcl::toROSMsg(freeRegionCloud, freeRegionCloudMsg);
		this->freeRegionPub_.publish(freeRegionCloudMsg);
		
	}


	void occMap::publishMap(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointCloud<pcl::PointXYZ> exploredCloud;
		pcl::PointCloud<pcl::PointXYZ> unkownCloud;

		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			minRange = this->mapSizeMin_;
			maxRange = this->mapSizeMax_;
		}
		else{
			minRange = this->bodyPosition_ - localMapSize_;
			maxRange = this->bodyPosition_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);

					// if (this->occupancy_[this->indexToAddress(pointIdx)] > this->pMinLog_){
					if (this->isOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							cloud.push_back(pt);
						}
					}

					// publish explored voxel map
					if(!this->isUnknown(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						pt.x = point(0);
						pt.y = point(1);
						pt.z = point(2);
						exploredCloud.push_back(pt);
					}
					// publish unknown voxel map
					else{
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							unkownCloud.push_back(pt);
						}
						
					}
				}
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		exploredCloud.width = exploredCloud.points.size();
		exploredCloud.height = 1;
		exploredCloud.is_dense = true;
		exploredCloud.header.frame_id = "map";

		// unkownCloud.width = unkownCloud.points.size();
		// unkownCloud.height = 1;
		// unkownCloud.is_dense = true;
		// unkownCloud.header.frame_id = "map";

		// TODO: make this cmd to be an option in cfg
		sensor_msgs::PointCloud2 cloudMsg;
		sensor_msgs::PointCloud2 exploredCloudMsg;
		// sensor_msgs::PointCloud2 unkownCloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		pcl::toROSMsg(exploredCloud, exploredCloudMsg);
		// pcl::toROSMsg(unkownCloud, unkownCloudMsg);
		this->mapVisPub_.publish(cloudMsg);
		this->mapExploredPub_.publish(exploredCloudMsg);
		// this->mapUnknownPub_.publish(unkownCloudMsg);
	}

	void occMap::publishInflatedMap(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			minRange = this->mapSizeMin_;
			maxRange = this->mapSizeMax_;
		}
		else{
			minRange = this->bodyPosition_ - localMapSize_;
			maxRange = this->bodyPosition_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);
					// if (this->occupancy_[this->indexToAddress(pointIdx)] > this->pMinLog_){
					if (this->isInflatedOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							cloud.push_back(pt);
						}
					}
				}
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->inflatedMapVisPub_.publish(cloudMsg);	
	}

	void occMap::publish2DOccupancyGrid(){
		Eigen::Vector3d minRange, maxRange;
		minRange = this->mapSizeMin_;
		maxRange = this->mapSizeMax_;
		minRange(2) = this->groundHeight_;
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		nav_msgs::OccupancyGrid mapMsg;
		for (int i=0; i<maxRangeIdx(0); ++i){
			for (int j=0; j<maxRangeIdx(1); ++j){
				mapMsg.data.push_back(0);
			}
		}

		double z = 0.5;
		int zIdx = int(z/this->mapRes_);
		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				Eigen::Vector3i pointIdx (x, y, zIdx);
				int map2DIdx = x + maxRangeIdx(1) * y;
				if (this->isUnknown(pointIdx)){
					mapMsg.data[map2DIdx] = -1;
				}
				else if (this->isOccupied(pointIdx)){
					mapMsg.data[map2DIdx] = 100;
				}
				else{
					mapMsg.data[map2DIdx] = 0;
				}
			}
		}

		mapMsg.header.frame_id = "map";
		mapMsg.header.stamp = ros::Time::now();
		mapMsg.info.resolution = this->mapRes_;
		mapMsg.info.width = maxRangeIdx(1);
		mapMsg.info.height = maxRangeIdx(0);
		mapMsg.info.origin.position.x = minRange(1);
		mapMsg.info.origin.position.y = minRange(0);
		this->map2DPub_.publish(mapMsg);		
	}

	void occMap::publishSemanticBoundingBoxes(){
		// publish bounding box as visualization marker
		int id_count = 0;
		visualization_msgs::MarkerArray semanticMarkers;
		if (this->semanticObjects_.empty()){
			return;
		}

		// ROS_INFO("semantic object size: %d", this->semanticObjects_.size());
		for (const auto& object : this->semanticObjects_){
			// print out obj centers
			// std::cout << "object center : " << object.position(0) << " " << object.position(1) << " " << object.position(2) << " size: " << object.size(0) << " " << object.size(1) << " " << object.size(2) << std::endl;

			// check if the object has been fully observed by over 3 view angles
			// int viewAngleCount = 0;
			// for (auto& viewAngle : object.view_angles){
			// 	if (viewAngle == 1){
			// 		viewAngleCount++;
			// 	}
			// }
			// std::vector<double> color = {0.0, 0.0, 0.0};
			// if (viewAngleCount >= 3){
			// 	color = {0.0, 1.0, 0.0};
			// }
			// else{
			// 	color = {0.0, 0.0, 1.0};
			// }

			// Ensure object id exists in objColorLabelMap_
			if (this->objColorLabelMap_.find(object.label_id) == this->objColorLabelMap_.end()) {
				cout << this->hint_ << ": Object ID " << object.label_id << " not found in objColorLabelMap_. Using default grey and 'Unknown' label." << endl;
				this->objColorLabelMap_[object.label_id] = {{0.5, 0.5, 0.5}, "Unknown"};
			}

			// assign different colors based on object types
			std::vector<double> color = this->objColorLabelMap_[object.label_id].first;
			std::string label = this->objColorLabelMap_[object.label_id].second;

			// if any of the object size is 0, skip this object
			if (object.size(0) == 0 or object.size(1) == 0 or object.size(2) == 0){
				// ROS_WARN("Object size is 0 for id_count: %d", id_count);
				continue;
			}

			//  bounding boxes
			visualization_msgs::Marker boxLine = createBoundingBoxMarkerArray(
											Eigen::Vector3d(object.position(0), object.position(1), object.position(2)),
											Eigen::Vector3d(object.size(0), object.size(1), object.size(2)),
											id_count,
											std::string("bounding_box"),
											color,
											1.0
										);
			semanticMarkers.markers.push_back(boxLine);

			// class labels
			// std::string label_with_id_count = label + " " + std::to_string(id_count);
			visualization_msgs::Marker textMarker = createTextMarker(
											Eigen::Vector3d(object.position(0), object.position(1), object.position(2) + 0.5),
											id_count,
											label,
											color,
											std::string("object_label"),
											1.0
										);
			semanticMarkers.markers.push_back(textMarker);

			id_count++;
		}
		this->objectVisPub_.publish(semanticMarkers);
	}

	void occMap::publishSemanticObjMsg(){
		map_manager::semanticObjArrayMsg semanticObjArray;
		for (const auto& object : this->semanticObjects_){
			// publish semantic object
			map_manager::semanticObjMsg semanticObj;
			semanticObj.label_id = object.label_id;
			semanticObj.position = {object.position(0), object.position(1), object.position(2)};
			semanticObj.size = {object.size(0), object.size(1), object.size(2)};
			semanticObj.view_angles = object.view_angles;
			semanticObjArray.semanticObjs.push_back(semanticObj);
			// std::cout << "semantic obj msg  : " << object.position(0) << " " << object.position(1) << " " << object.position(2) << " size: " << object.size(0) << " " << object.size(1) << " " << object.size(2) << std::endl;
		}
		this->semanticObjPub_.publish(semanticObjArray);
		// std::cout << "publish semantic object message" << std::endl;
	}
	
	void occMap::publishObjectMap(){
		// publish object map
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		for (auto& objectMap : this->objectMapList_){
			std::vector<Eigen::Vector3d> points;
			objectMap->getPointCloud(points);
			for (auto& point : points){
				objectMap->object2GlobalFrame(point, point);
				pt.x = point(0); pt.y = point(1); pt.z = point(2);
				cloud.push_back(pt);
			}
		}
		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";
		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->objectMapVisPub_.publish(cloudMsg);
	}

    void occMap::publishGlobalObjectDenseCloud() {
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> denseCloud;
        std::stack<Eigen::Vector3d> denseCloudStack;
        std::vector<Eigen::Vector3d> points;
        
        for (auto& objectMap : this->objectMapList_) {
            // Get class ID from object map (assuming getClassId() exists)
            int classId = objectMap->label_; 
            bool isTV = (classId == 62);  // TV class ID from your config
            
            objectMap->getPointCloud(points);
            for (const auto& point : points) {
                int address = objectMap->posToAddress(point);
                denseCloudStack = objectMap->getDenseCloudAtVoxel(address);
    
                // Bypass filter threshold for TVs
                if (isTV || denseCloudStack.size() > this->voxelFilterThresh_) {
                    while (!denseCloudStack.empty()) {
                        Eigen::Vector3d pointEigen = denseCloudStack.top();
                        pt.x = pointEigen(0); 
                        pt.y = pointEigen(1); 
                        pt.z = pointEigen(2);
                        denseCloud.push_back(pt);
                        denseCloudStack.pop();
                    }
                }
            }
            if (isTV) {
                ROS_INFO("Processing TV object with %zu points", denseCloud.points.size());
            
            }

        }
        
        // Rest of the original code remains the same
        denseCloud.width = denseCloud.points.size();
        denseCloud.height = 1;
        denseCloud.is_dense = true;
        denseCloud.header.frame_id = "map";
        sensor_msgs::PointCloud2 denseCloudMsg;
        pcl::toROSMsg(denseCloud, denseCloudMsg);
        this->objectDenseCloudPub_.publish(denseCloudMsg);
    }

	void occMap::updateObjectMapCB(const ros::TimerEvent&){
		// measure time 
		auto start_time = std::chrono::high_resolution_clock::now();
		Eigen::Matrix4d syncMap2Body = this->map2Body_;
		// update object map and bounding box
		this->semanticObjects_.clear();
		cv::Mat emptyMask = cv::Mat::zeros(this->depthImage_.size(), CV_8UC1);
		for (auto& objectMap : this->objectMapList_){
			// update mask for those with no observation
			bool isVisible = false;
			isVisible = this->checkVisibility(*objectMap);
			if (isVisible){
				// ROS_INFO("is visible");
				objectMap->updateVisibility(this->colorSensorPosition_);
				if (not objectMap->isMaskUpdated_){
					objectMap->updateEmptyMask(emptyMask, syncMap2Body);
				}
			}
			// update object map
			objectMap->updateMap();
			objectMap->updateModel();
			objectMap->reprojectionUpdate();
			objectMap->isMaskUpdated_ = false;
		}

		// merge object map if necessary
		for (auto it1 = this->objectMapList_.begin(); it1 != this->objectMapList_.end(); ++it1){
			for (auto it2 = this->objectMapList_.begin(); it2 != this->objectMapList_.end(); ){
				// not merge with itself: check position
				if ((*it1)->position_ == (*it2)->position_){
					++it2;
					continue;
				}
				// check label
				if ((*it1)->label_ != (*it2)->label_){
					++it2;
					continue;
				}
				// check distance between center of mass
				double dist = ((*it1)->centerOfMass_ - (*it2)->centerOfMass_).norm();
				if (dist > 3.0 ){
					++it2;
					continue;
				}
				// check 3d IOU
				double iou = (*it1)->get3dIOUwithOther(*it2);
				if (iou >= this->objectMergeThresh_
					// and dist<0.4
					// or (iou==-1 and dist<0.7)
					){
					// ROS_WARN("Merge object map of center %f %f %f with center %f %f %f, IOU: %f, dist: %f", (*it1)->position_(0), (*it1)->position_(1), (*it1)->position_(2), (*it2)->position_(0), (*it2)->position_(1), (*it2)->position_(2), iou, dist);
					// ROS_INFO("initTime of it1: %f seconds, %lu nanoseconds", (*it1)->initTime_.toSec(), (*it1)->initTime_.toNSec());
            		// ROS_INFO("initTime of it2: %f seconds, %lu nanoseconds", (*it2)->initTime_.toSec(), (*it2)->initTime_.toNSec());
					if ((*it1)->initTime_.toSec() < (*it2)->initTime_.toSec()){
						(*it1)->mergeWithOtherMap(*it2);
						it2 = this->objectMapList_.erase(it2); // Remove otherMap and update iterator
					}
					else{
						(*it2)->mergeWithOtherMap(*it1);
						it1 = this->objectMapList_.erase(it1); // Remove thisMap and update iterator
						break; // Exit inner loop since it1 is invalidated
					}
				} else {
					++it2;
				}
			}
		}

		// update semantic object 
		// ROS_INFO("update semantic object");
		for (auto& objectMap : this->objectMapList_){
			// get bounding box
			vision_msgs::Detection3D objectBoundingBox;
			objectMap->getBoundingBox(objectBoundingBox);
			mapManager::semanticObject semanticObj;
			semanticObj.label_id = objectBoundingBox.results[0].id;
			objectMap->object2GlobalFrame(Eigen::Vector3d(objectBoundingBox.bbox.center.position.x, 
										objectBoundingBox.bbox.center.position.y, 
										objectBoundingBox.bbox.center.position.z), 
										semanticObj.position);
			semanticObj.size(0) = objectBoundingBox.bbox.size.x;
			semanticObj.size(1) = objectBoundingBox.bbox.size.y;
			semanticObj.size(2) = objectBoundingBox.bbox.size.z;
			// std::cout << "semantic object center : " << semanticObj.position(0) << " " << semanticObj.position(1) << " " << semanticObj.position(2) << " size: " << semanticObj.size(0) << " " << semanticObj.size(1) << " " << semanticObj.size(2) << std::endl;
			semanticObj.view_angle_base = objectMap->viewAngBase_;
			std::vector<unsigned char> viewAngles(this->viewAgIntervals_, 0);
			objectMap->getViewAngles(viewAngles);
			semanticObj.view_angles = viewAngles;
			this->semanticObjects_.push_back(semanticObj);

		}
		
		auto end_time = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
		// std::cout << "update object map took " << duration.count() << " milliseconds." << std::endl;
	}

	void occMap::drawBoundingBox(const vision_msgs::Detection2DArray& detections, cv::Mat& img){
		cv::Mat depthNormalized = img;
		double min, max;
		cv::minMaxIdx(depthNormalized, &min, &max);
		cv::convertScaleAbs(depthNormalized, depthNormalized, 255. / max);
		depthNormalized.convertTo(depthNormalized, CV_8UC1);
		cv::applyColorMap(depthNormalized, depthNormalized, cv::COLORMAP_BONE);
		for (const auto& detection : detections.detections){
			int topX = detection.bbox.center.x - detection.bbox.size_x/2;
			int topY = detection.bbox.center.y - detection.bbox.size_y/2;
			cv::Rect boxVis(topX, topY, detection.bbox.size_x, detection.bbox.size_y);
			cv::rectangle(depthNormalized, boxVis, cv::Scalar(0, 255, 0), 5, 8, 0);
		}
		this->depthImgWithDetectPub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", depthNormalized).toImageMsg());
	}

	bool occMap::isCollisionBoxFree(const Eigen::Vector3d& pos, const double yaw){
		// check if the robot will collide occupied voxels
		// get robot footprint
		Eigen::Vector3d body;
		Eigen::Vector3d global;
		Eigen::Matrix3d rotMat;
		rotMat << 	cos(yaw), -sin(yaw), 0,
					sin(yaw), cos(yaw), 0,
					0, 0, 1;
		Eigen::Vector3i checkIdx;
		for (double x = -this->robotSize_(0)/2 ; x <= this->robotSize_(0)/2; x += this->mapRes_){
			for (double y = -this->robotSize_(1)/2; y <= this->robotSize_(1)/2; y += this->mapRes_){
				for (double z = 0; z <= this->robotSize_(2); z += this->mapRes_){
					// transform from body to global frame
					body(0) = x; body(1) = y; body(2) = z;
					global = rotMat * body + pos;
					this->posToIndex(global, checkIdx);
					if (not this->isFree(checkIdx)){
						return false;
					}
				}
			}
		}
		return true;
	}

	bool occMap::isCollisionBoxFree(const Eigen::Vector3d& pos, const double yaw, const double safeDist){
		// check if the robot will collide occupied voxels
		// get robot footprint
		Eigen::Vector3d body;
		Eigen::Vector3d global;
		Eigen::Matrix3d rotMat;
		rotMat << 	cos(yaw), -sin(yaw), 0,
					sin(yaw), cos(yaw), 0,
					0, 0, 1;
		Eigen::Vector3i checkIdx;
		for (double x = -this->robotSize_(0)/2-safeDist ; x <= this->robotSize_(0)/2+safeDist; x += this->mapRes_){
			for (double y = -this->robotSize_(1)/2-safeDist; y <= this->robotSize_(1)/2+safeDist; y += this->mapRes_){
				for (double z = 0; z <= this->robotSize_(2); z += this->mapRes_){
					// transform from body to global frame
					body(0) = x; body(1) = y; body(2) = z;
					global = rotMat * body + pos;
					this->posToIndex(global, checkIdx);
					if (not this->isFree(checkIdx)){
						return false;
					}
				}
			}
		}
		return true;
	}

	bool occMap::isCollisionBoxOccupied(const Eigen::Vector3d& pos, const double yaw){
		// check if the robot will collide occupied voxels
		// get robot footprint
		Eigen::Vector3d body;
		Eigen::Vector3d global;
		Eigen::Matrix3d rotMat;
		rotMat << 	cos(yaw), -sin(yaw), 0,
					sin(yaw), cos(yaw), 0,
					0, 0, 1;
		Eigen::Vector3i checkIdx;
		for (double x = this->collisionBoxMin_(0); x <= this->collisionBoxMax_(0); x += this->mapRes_){
			for (double y = this->collisionBoxMin_(1); y <= this->collisionBoxMax_(1); y += this->mapRes_){
				for (double z = this->collisionBoxMin_(2); z <= this->collisionBoxMax_(2); z += this->mapRes_){
					// transform from body to global frame
					body(0) = x; body(1) = y; body(2) = z;
					global = rotMat * body + pos;
					this->posToIndex(global, checkIdx);
					if (this->isOccupied(checkIdx)){
						return true;
					}
				}
			}
		}
		return false;
	}

	bool occMap::isCollisionBoxOccupied(const Eigen::Vector3d& pos, const double yaw, const double safeDist){
		// check if the robot will collide occupied voxels
		// get robot footprint
		Eigen::Vector3d body;
		Eigen::Vector3d global;
		Eigen::Matrix3d rotMat;
		rotMat << 	cos(yaw), -sin(yaw), 0,
					sin(yaw), cos(yaw), 0,
					0, 0, 1;
		Eigen::Vector3i checkIdx;
		for (double x = this->collisionBoxMin_(0)-safeDist; x <= this->collisionBoxMax_(0)+safeDist; x += this->mapRes_){
			for (double y = this->collisionBoxMin_(1)-safeDist; y <= this->collisionBoxMax_(1)+safeDist; y += this->mapRes_){
				for (double z = this->collisionBoxMin_(2); z <= this->collisionBoxMax_(2); z += this->mapRes_){
					// transform from body to global frame
					body(0) = x; body(1) = y; body(2) = z;
					global = rotMat * body + pos;
					this->posToIndex(global, checkIdx);
					if (this->isOccupied(checkIdx)){
						return true;
					}
				}
			}
		}
		return false;
	}

	// check if the point is in the field of view of the sensor
	// according to the sensor model, transform the point to the sensor frame,
	// then call the sensor model to check if the point is in fov
	// bool occMap::isInFov(const Eigen::Matrix4d& map2Body, const Eigen::Vector3d& point){
	// 	Eigen::Vector3d pointSensorFrame;
	// 	std::string sensorModelName = this->rangeSensorInputMode_ == 0 ? "PinholeCamera" : "Lidar";
	// 	// std::cout << "sensor model name: " << sensorModelName << std::endl;
	// 	sensorModelFactory::global2SensorFrame(point, pointSensorFrame, this->sensorModels_[sensorModelName], map2Body);
	// 	return this->sensorModels_[sensorModelName]->isInFov(pointSensorFrame);
	// }

	// check if the point is in the field of view of the sensor
	// according to the sensor model, transform the point to the sensor frame,
	// then call the sensor model to check if the point is in fov
	// bool occMap::isInColorSensorFov(const Eigen::Vector4d& map2Body, const Eigen::Vector3d& point){
		
	// 	Eigen::Vector3d pointSensorFrame;
	// 	std::string sensorModelName = this->sensorFusionMode_==0 ? "PinholeCamModel" : "PanoCamModel";
	// 	sensorModelFactory::global2SensorFrame(point, pointSensorFrame, this->sensorModels_[sensorModelName], map2Body);
	// 	return this->sensorModels_[sensorModelName]->isInFov(pointSensorFrame);
	// }

	// bool occMap::isInRangeSensorFov(const Eigen::Vector4d& map2Body, const Eigen::Vector3d& point){
	// 	Eigen::Vector3d pointSensorFrame;
	// 	std::string sensorModelName = this->rangeSensorInputMode_ == 0 ? "PinholeCamModel" : "Lidar";
	// 	sensorModelFactory::global2SensorFrame(point, pointSensorFrame, this->sensorModels_[sensorModelName], map2Body);
	// 	return this->sensorModels_[sensorModelName]->isInFov(pointSensorFrame);
	// }

}