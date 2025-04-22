/*
    FILE: dbscan.h
    ------------------
    helper class header for dbscan
*/
#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;
namespace mapManager{
    typedef struct Point_
    {
        float x, y, z;  // X, Y, Z position
        int clusterID;  // clustered ID
    }Point;

    class DBSCAN {
    public:    
        DBSCAN(unsigned int minPts, float eps, vector<Point> points){
            m_minPoints = minPts;
            m_epsilon = eps;
            m_points = points;
            m_pointSize = points.size();
            cluster_num = 0;
        }
        ~DBSCAN(){}

        int run();
        vector<int> calculateCluster(Point point);
        int expandCluster(Point point, int clusterID);
        inline double calculateDistance(const Point& pointCore, const Point& pointTarget);

        int getTotalPointSize() {return m_pointSize;}
        int getMinimumClusterSize() {return m_minPoints;}
        int getEpsilonSize() {return m_epsilon;}

    public:
        vector<Point> m_points;
        int cluster_num;
        
    private:    
        unsigned int m_pointSize;
        unsigned int m_minPoints;
        float m_epsilon;
    };

    inline mapManager::Point eigenToDBPoint(const Eigen::Vector3d& p){
        mapManager::Point pDB;
        pDB.x = p(0);
        pDB.y = p(1);
        pDB.z = p(2);
        pDB.clusterID = -1;
        return pDB;
    }

    inline Eigen::Vector3d dbPointToEigen(const mapManager::Point& pDB){
        Eigen::Vector3d p;
        p(0) = pDB.x;
        p(1) = pDB.y;
        p(2) = pDB.z;
        return p;
    }

    inline void eigenToDBPointVec(const std::vector<Eigen::Vector3d>& points, std::vector<mapManager::Point>& pointsDB, int size){
        for (int i=0; i<size; ++i){
            Eigen::Vector3d p = points[i];
            mapManager::Point pDB = eigenToDBPoint(p);
            pointsDB.push_back(pDB);
        }
    }

    inline void pclToDBPointVec(const pcl::PointCloud<pcl::PointXYZ>& points, std::vector<mapManager::Point>& pointsDB, int size){
        pcl::PointXYZ p;
        mapManager::Point pDB;
        for (int i=0; i<size; ++i){
            p = points[i];
            pDB.x = p.x;
            pDB.y = p.y;
            pDB.z = p.z;
            pDB.clusterID = -1;
            pointsDB.push_back(pDB);
        }
    }
}

#endif // DBSCAN_H
