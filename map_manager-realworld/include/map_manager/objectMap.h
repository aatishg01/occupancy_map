/*
objectMap.h
--------------------------------------
object map header file
*/

#ifndef MAPMANAGER_OBJECTMAP_H
#define MAPMANAGER_OBJECTMAP_H
#include <map_manager/raycast.h>
#include <Eigen/Eigen>
#include <vector>
#include <queue>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <stack>
#include "utils.h"

#include <map_manager/sensorModel.h>

namespace mapManager{

    struct objectObservation{
        // header
        ros::Time stamp_;
        // Eigen::Vector3d observe_position_;
        // Eigen::Matrix3d observe_orientation_;
        Eigen::Matrix4d observation_map_2_body_;
        cv::Mat instance_mask_;
        double mask_score_;
        int observed_num_;
        int detected_num_;
    };

    struct objectMapParams{
        double resolution;
        std::vector<double> object_map_size;
		double min_mask_score_thresh;
		double observe_valid_prob_thresh;
        double observe_invalid_prob_thresh;
        int view_angle_intervals;
        bool is_debug;
        std::string debug_root_dir;
	};
        
    class objectMap{
        public: 
        Eigen::Vector3d position_; // current position
        Eigen::Vector3d centerOfMass_; // center of mass
        int label_; // object label
        std::unordered_map<int, int> labels_probability_;
        ros::Time initTime_;// time of initialization
        ros::Time modelTime_;// time of first occupancy model update
        bool isMaskUpdated_ = false;
        double viewAngBase_; // 0-2pi
        vision_msgs::Detection3D lastDetection_;
        std::shared_ptr<sensorModelFactory> sensorManager_;

        protected:

        // Raycaster
        RayCaster raycaster_;

        // map params
        double UNKNOWN_FLAG_ = 0.01;
        double pHit_ = 0.7;
        double pMiss_ = 0.35;
        double pMin_ = 0.12;
        double pMax_ = 0.97;
        double pOcc_ = 0.8;
        double pHitLog_;
        double pMissLog_;
        double pMinLog_;
        double pMaxLog_;
        double pOccLog_;
        double mapRes_;
        Eigen::Vector3d mapSizeMin_, mapSizeMax_; // reserved min/max map size
        Eigen::Vector3i mapVoxelMin_, mapVoxelMax_; // reserved min/max map size in voxel
        int viewAgIntervals_;
        double minMaskScoreThresh_;
        double obsValidProbThresh_;
        double obsInvalidProbThresh_;
        bool isDebug_;
        std::string debugRootDir_;

        // map data
        std::vector<double> occupancy_;
        std::vector<double> semanticProb_;
        
        // map containers
        std::vector<Eigen::Vector3d> observedPoints; // point cloud of object observation in current time stamp
        std::vector<int> countHitMiss_;
        std::vector<int> countHit_;
        // std::vector<bool> flagTraverse_;
        // std::vector<bool> flagRayend_;
        std::queue<int> updateObjVoxelCache_;
        int raycastNum_ = 0;

        // object data
        vision_msgs::Detection3D boundingBox_;
        std::vector<Eigen::Vector3d> pointCloud_;
        std::vector<std::stack<Eigen::Vector3d>> globalCloudVoxelList_;
        // std::vector<cv::Mat*> instanceMaskPtrList_;
        // Eigen::Vector3d observationPosition_;
        // Eigen::Matrix3d observationOrientation_;
        std::vector<unsigned char> viewAngles_;
        std::vector<unsigned char> isVisibleList_;
        std::vector<mapManager::objectObservation> observationList_;

        public:
        objectMap(objectMapParams params, Eigen::Vector3d center, int label, double viewAngBase, std::shared_ptr<sensorModelFactory> sensorManager);
        ~objectMap();
        void updateMap();
        int updateObjVoxelInfo(Eigen::Vector3d& point, bool hit);
        void updateModel();
        void reprojectionUpdate();
        void updateGlobalObjDenseCloud(Eigen::Vector3d& pointGlobal);
        void mergeWithOtherMap(std::shared_ptr<objectMap> otherMap);
        void recenterMap();

        // tools
        double logit(double x);
        void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx);
        void indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos);
        int posToAddress(const Eigen::Vector3d& pos);
        int posToAddress(double x, double y, double z);
        int indexToAddress(const Eigen::Vector3i& idx);
        int indexToAddress(int x, int y, int z);
        void global2ObjectFrame(const Eigen::Vector3d& posGlobal, Eigen::Vector3d& posLocal);
        void object2GlobalFrame(const Eigen::Vector3d& posLocal, Eigen::Vector3d& posGlobal);
        bool belongs2ObjectMap(const Eigen::Vector3d& posGlobal); // check if the point in global map belongs to the object map
        bool isInMapRange(const Eigen::Vector3d& pos); // check if the point in current map is in the range of the object map
        bool isOccupied(const Eigen::Vector3d& pos); // check if the point is occupied
        bool isOccupied(const Eigen::Vector3i& idx); // check if the point is occupied
        bool isOccupied(int address); // check if the point is occupied
        Eigen::Vector3d adjustPointInMap(const Eigen::Vector3d& point);
        double calcMaskScore(const cv::Mat& mask, const cv::Mat& lidarDepthImg);
        double get3dIOUwithOther(std::shared_ptr<objectMap> otherMap);
        Eigen::Vector3d regulateWithOrigin(const Eigen::Vector3d& pos);

        // user function
        void addObservation(const Eigen::Vector3d& points);
        void updateMask(const sensor_msgs::Image& mask, const int label, const cv::Mat& lidarDpethImg, Eigen::Matrix4d observation_map_2_body);
        void updateEmptyMask(const cv::Mat& mask, Eigen::Matrix4d observation_map_2_body);
        void updateVisibility(const Eigen::Vector3d& observationPosition);
        int calcViewAgIdx(const Eigen::Vector3d& observationPosition);
        double viewAngIdx2Ang(int idx);
        int viewAng2Idx(double angle);
        void getMapIdxRange(Eigen::Vector3i& maxIdx);
        void getBoundingBox(vision_msgs::Detection3D& boundingBox);
        void getPointCloud(std::vector<Eigen::Vector3d>& pointCloud);
        void getViewAngles(std::vector<unsigned char>& viewAngles);
        std::stack<Eigen::Vector3d> getDenseCloudAtVoxel(int address);
    };

    inline double objectMap::logit(double x){
        return log(x/(1-x));
    }

    inline void objectMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx){
		idx(0) = floor( (pos(0) - this->mapSizeMin_(0) ) / this->mapRes_ );
		idx(1) = floor( (pos(1) - this->mapSizeMin_(1) ) / this->mapRes_ );
		idx(2) = floor( (pos(2) - this->mapSizeMin_(2) ) / this->mapRes_ );
	}

	inline void objectMap::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos){
		pos(0) = (idx(0) + 0.5) * this->mapRes_ + this->mapSizeMin_(0); 
		pos(1) = (idx(1) + 0.5) * this->mapRes_ + this->mapSizeMin_(1);
		pos(2) = (idx(2) + 0.5) * this->mapRes_ + this->mapSizeMin_(2);
	}

	inline int objectMap::posToAddress(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->indexToAddress(idx);
	}

	inline int objectMap::posToAddress(double x, double y, double z){
		Eigen::Vector3d pos (x, y, z);
		return this->posToAddress(pos);
	}

	inline int objectMap::indexToAddress(const Eigen::Vector3i& idx){
		return idx(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2) + idx(1) * this->mapVoxelMax_(2) + idx(2);
	}

	inline int objectMap::indexToAddress(int x, int y, int z){
		Eigen::Vector3i idx (x, y, z);
		return this->indexToAddress(idx);
	}

    inline void objectMap::global2ObjectFrame(const Eigen::Vector3d& posGlobal, Eigen::Vector3d& posLocal){
        posLocal = posGlobal - this->position_;
    }

    inline void objectMap::object2GlobalFrame(const Eigen::Vector3d& posLocal, Eigen::Vector3d& posGlobal){
        posGlobal = posLocal + this->position_;
    }

    inline bool objectMap::belongs2ObjectMap(const Eigen::Vector3d& posGlobal){
        Eigen::Vector3d posLocal = posGlobal - this->position_;
        return this->isInMapRange(posLocal);
    }

    inline bool objectMap::isInMapRange(const Eigen::Vector3d& pos){
        if (pos(0) < this->mapSizeMin_(0) || pos(0) > this->mapSizeMax_(0) || pos(1) < this->mapSizeMin_(1) || pos(1) > this->mapSizeMax_(1) || pos(2) < this->mapSizeMin_(2) || pos(2) > this->mapSizeMax_(2)){
            return false;
        }
        return true;
    }

    inline bool objectMap::isOccupied(const Eigen::Vector3d& pos){
        int address = this->posToAddress(pos);
        return this->isOccupied(address);
    }

    inline bool objectMap::isOccupied(const Eigen::Vector3i& idx){
        int address = this->indexToAddress(idx);
        return this->isOccupied(address);
    }

    inline bool objectMap::isOccupied(int address){
        if (this->occupancy_[address] > this->pOccLog_){
            return true;
        }
        return false;
    }

    inline Eigen::Vector3d objectMap::regulateWithOrigin(const Eigen::Vector3d& pos){
        Eigen::Vector3d newPos = pos;
        // regulate to point with resolution of mapRes_
        for (int i=0; i<3; ++i){
            newPos(i) = floor(newPos(i) / this->mapRes_) * this->mapRes_;
        }
        return newPos;
    }

    inline void objectMap::getMapIdxRange(Eigen::Vector3i& maxIdx){
        maxIdx = this->mapVoxelMax_;
    }

    inline void objectMap::getBoundingBox(vision_msgs::Detection3D& boundingBox){
        boundingBox = this->boundingBox_;
    }

    inline void objectMap::getPointCloud(std::vector<Eigen::Vector3d>& pointCloud){
        pointCloud = this->pointCloud_;
    }

    inline void objectMap::getViewAngles(std::vector<unsigned char>& viewAngles){
        viewAngles = this->viewAngles_;
    }

    inline int objectMap::updateObjVoxelInfo(Eigen::Vector3d& point, bool hit){
        int address = this->posToAddress(point);

        if (hit){
            this->countHit_[address]++;
        }
        this->countHitMiss_[address]++;
        if (this->countHitMiss_[address] == 1){
            this->updateObjVoxelCache_.push(address);
        }
        
        return address;
        // double prob = (double)this->countHit_[address] / (double)(this->countHit_[address] + this->countHitMiss_[address]);
        // this->occupancy_[address] = prob;
    }

    inline void objectMap::addObservation(const Eigen::Vector3d& point){
        this->observedPoints.push_back(point);
    }

    inline Eigen::Vector3d objectMap::adjustPointInMap(const Eigen::Vector3d& point){
		Eigen::Vector3d diff = point;
		Eigen::Vector3d offsetMin = this->mapSizeMin_;
		Eigen::Vector3d offsetMax = this->mapSizeMax_;

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

		return  (minRatio - 1e-3) * diff;
	}

    inline void objectMap::updateVisibility(const Eigen::Vector3d& observationPosition){
        
        int view_idx = this->calcViewAgIdx(observationPosition);
        this->isVisibleList_[view_idx] = 1;
    }

    inline void objectMap::updateGlobalObjDenseCloud(Eigen::Vector3d& pointGlobal){
        Eigen::Vector3d pointLocal;
        this->global2ObjectFrame(pointGlobal, pointLocal);
        if (not this->isInMapRange(pointLocal)){
            // ROS_ERROR("pointLocal %f %f %f out of object map range, pointGlobal %f %f %f", pointLocal(0), pointLocal(1), pointLocal(2), pointGlobal(0), pointGlobal(1), pointGlobal(2));
            return;
        }
        int address = this->posToAddress(pointLocal);
        if (this->globalCloudVoxelList_.empty()){
            ROS_ERROR("globalCloudVoxelList_ is empty");
        }
        if (address >= this->globalCloudVoxelList_.size()){
            ROS_ERROR("address %lu is out of range %lu", address, this->globalCloudVoxelList_.size());
            std::cout << "voxelmapsize: " << this->mapVoxelMax_ << " " << this->mapVoxelMin_ << std::endl;
            std::cout << "map size min: " << this->mapSizeMin_ << " " << this->mapSizeMax_ << std::endl;
            std::cout << "pointLocal: " << pointLocal << std::endl;
        }
        this->globalCloudVoxelList_[address].push(pointGlobal);
        // pop if the stack size is larger than 50
        if (this->globalCloudVoxelList_[address].size() > 400){
            this->globalCloudVoxelList_[address].pop();
        }
    }

    inline std::stack<Eigen::Vector3d> objectMap::getDenseCloudAtVoxel(int address){
        return this->globalCloudVoxelList_[address];
    }
}

#endif