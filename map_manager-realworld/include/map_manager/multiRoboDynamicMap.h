/*
    FILE: multiRoboDynamicMap.h
    --------------------------------------
    header file multiRoboDynamicMap
*/

#ifndef MAPMANAGER_MULTIROBODYNAMICMAP_H
#define MAPMANAGER_MULTIROBODYNAMICMAP_H

#include <map_manager/multiRoboOccupancyMap.h>
#include <map_manager/detector/dynamicDetector.h>

namespace mapManager{

    class multiRoboDynamicMap : public multiRoboOccMap{
    private:

    protected:
        // std::string ns_;
		// std::string hint_;
        std::shared_ptr<mapManager::dynamicDetector> detector_;
        ros::Timer freeMapTimer_;


    public:
        multiRoboDynamicMap();
        multiRoboDynamicMap(const ros::NodeHandle& nh);
        void initMultiRoboDynamicMap(const ros::NodeHandle& nh);

        // dynamic clean 
        void freeMapCB(const ros::TimerEvent&);

        // user function
        void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclePos, 
                                 std::vector<Eigen::Vector3d>& obstaclesVel, 
                                 std::vector<Eigen::Vector3d>& obstacleSize);
    };

}

# endif