/*
    FILE: multiRoboDynamicMap.cpp
    --------------------------------------
    dynamic map for multiple robot map sharing
*/
#include <ros/ros.h>
#include <map_manager/multiRoboDynamicMap.h>

namespace mapManager{
    multiRoboDynamicMap::multiRoboDynamicMap(){
        this->ns_ = "multi_robo_dynamic_map";
        this->hint_ = "[multiRoboDynamicMap]";
    }

    multiRoboDynamicMap::multiRoboDynamicMap(const ros::NodeHandle& nh){
        this->ns_ = "multi_robo_dynamic_map";
        this->hint_ = "[multiRoboDynamicMap]";
        this->initMultiRoboDynamicMap(nh);
    }

    void multiRoboDynamicMap::initMultiRoboDynamicMap(const ros::NodeHandle& nh){
        this->nh_ = nh;
        this->initMultiRoboOccMap(nh);
        // this->detector_.reset(new mapManager::dynamicDetector (this->nh_));
        // this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.033), &multiRoboDynamicMap::freeMapCB, this);
    }

    void multiRoboDynamicMap::freeMapCB(const ros::TimerEvent&){
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
        std::vector<mapManager::box3D> dynamicBBoxes;
        this->detector_->getDynamicObstacles(dynamicBBoxes);
        for (mapManager::box3D ob:dynamicBBoxes){
            Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-2*this->mapRes_-this->robotSize_(0)/2 - 0.2, ob.y-ob.y_width/2-2*this->mapRes_-this->robotSize_(1)/2 - 0.2, 0.0);
            Eigen::Vector3d upperBound (ob.x+ob.x_width/2+2*this->mapRes_+this->robotSize_(0)/2 + 0.2, ob.y+ob.y_width/2+2*this->mapRes_+this->robotSize_(1)/2 - 0.2, ob.z+ob.z_width+2*this->mapRes_+this->robotSize_(2)/2);
            freeRegions.push_back(std::make_pair(lowerBound, upperBound));
        }
        this->updateFreeRegions(freeRegions);
        // this->freeHistRegions();
    }

    void multiRoboDynamicMap::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclePos,
                                                 std::vector<Eigen::Vector3d>& obstacleVel,
                                                 std::vector<Eigen::Vector3d>& obstacleSize){
        std::vector<mapManager::box3D> dynamicBBoxes;
        this->detector_->getDynamicObstacles(dynamicBBoxes);
        for (size_t i=0 ; i<dynamicBBoxes.size() ; ++i){
            Eigen::Vector3d pos(dynamicBBoxes[i].x, dynamicBBoxes[i].y, dynamicBBoxes[i].z);
            Eigen::Vector3d vel(dynamicBBoxes[i].Vx, dynamicBBoxes[i].Vy, 0);
            Eigen::Vector3d size(dynamicBBoxes[i].x_width, dynamicBBoxes[i].y_width, dynamicBBoxes[i].z_width);
            obstaclePos.push_back(pos);
            obstacleVel.push_back(vel);
            obstacleSize.push_back(size);
        }
    }
}
