/*
	FILE: visualizationUtils.h
	--------------------------------------
	include helper functions for visualization
*/

#ifndef MAPMANAGER_UTILS_H
#define MAPMANAGER_UTILS_H

// #include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace mapManager{
    inline double calc3dIOU(const Eigen::Vector3d& center1, const Eigen::Vector3d& size1, const Eigen::Vector3d& center2, const Eigen::Vector3d& size2){
		Eigen::Vector3d lowerBound1 = center1 - size1/2;
		Eigen::Vector3d upperBound1 = center1 + size1/2;
		Eigen::Vector3d lowerBound2 = center2 - size2/2;
		Eigen::Vector3d upperBound2 = center2 + size2/2;

		Eigen::Vector3d lowerBound = lowerBound1.cwiseMax(lowerBound2);
		Eigen::Vector3d upperBound = upperBound1.cwiseMin(upperBound2);

		Eigen::Vector3d interSize = upperBound - lowerBound;
		if (interSize(0) < 0 or interSize(1) < 0 or interSize(2) < 0){
			return 0.0;
		}

        // any size of the box is zero
        if (size1(0) <= 0 or size1(1) <= 0 or size1(2) <= 0 or size2(0) <= 0 or size2(1) <= 0 or size2(2) <= 0){
            return -1;
        }

		double interVolume = interSize(0) * interSize(1) * interSize(2);
		double unionVolume = size1(0) * size1(1) * size1(2) + size2(0) * size2(1) * size2(2) - interVolume;
		return interVolume/unionVolume;
	}

    // Intersection Over Object
    inline double calc3dIOO(const Eigen::Vector3d& center1, const Eigen::Vector3d& size1, const Eigen::Vector3d& center2, const Eigen::Vector3d& size2){
        Eigen::Vector3d lowerBound1 = center1 - size1/2;
		Eigen::Vector3d upperBound1 = center1 + size1/2;
		Eigen::Vector3d lowerBound2 = center2 - size2/2;
		Eigen::Vector3d upperBound2 = center2 + size2/2;

		Eigen::Vector3d lowerBound = lowerBound1.cwiseMax(lowerBound2);
		Eigen::Vector3d upperBound = upperBound1.cwiseMin(upperBound2);

		Eigen::Vector3d interSize = upperBound - lowerBound;
		if (interSize(0) < 0 or interSize(1) < 0 or interSize(2) < 0){
			return 0.0;
		}

        // any size of the box is zero
        if (size1(0) <= 0 or size1(1) <= 0 or size1(2) <= 0 or size2(0) <= 0 or size2(1) <= 0 or size2(2) <= 0){
            return -1;
        }

		double interVolume = interSize(0) * interSize(1) * interSize(2);
		double volume1 = size1(0) * size1(1) * size1(2);
        double volume2 = size2(0) * size2(1) * size2(2);
        return std::max(interVolume/volume1, interVolume/volume2);
    }
    
    // // create a marker for a point
    // visualization_msgs::Marker createPointMarker(const Eigen::Vector3d& point, 
    //                                             const std::string& frame_id, 
    //                                             const std::string& ns, 
    //                                             const int id, 
    //                                             const double size, 
    //                                             const std::vector<double>& color, 
    //                                             const double alpha);

    // // create a marker for a line
    // visualization_msgs::Marker createLineMarker(const Eigen::Vector3d& start, 
    //                                             const Eigen::Vector3d& end, 
    //                                             const std::string& frame_id, 
    //                                             const std::string& ns, 
    //                                             const int id, 
    //                                             const double size, 
    //                                             const std::vector<double>& color, 
    //                                             const double alpha);

    // // create a marker for a line strip
    // visualization_msgs::Marker createLineStripMarker(const std::vector<Eigen::Vector3d>& points, 
    //                                                 const std::string& frame_id, 
    //                                                 const std::string& ns, 
    //                                                 const int id, 
    //                                                 const double size, 
    //                                                 const std::vector<double>& color, 
    //                                                 const double alpha);

    // // create a marker for a line list
    // visualization_msgs::Marker createLineListMarker(const std::vector<Eigen::Vector3d>& points, 
    //                                                 const std::string& frame_id, 
    //                                                 const std::string& ns, 
    //                                                 const int id, 
    //                                                 const double size, 
    //                                                 const std::vector<double>& color, 
    //                                                 const double alpha);

    // create a marker for a sphere
    inline visualization_msgs::Marker createSphereMarker(const Eigen::Vector3d& center,   
                                                const int id,
                                                const std::string& ns,
                                                const std::vector<double>& color, 
                                                const double alpha,
                                                const double radius,
                                                const std::string& frame_id="map"
                                                ){
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = center(0);
        marker.pose.position.y = center(1);
        marker.pose.position.z = center(2);
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.scale.z = radius;
        marker.color.a = alpha;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        return marker;
    }

    // create a marker for text msg
    inline visualization_msgs::Marker createTextMarker(const Eigen::Vector3d& center, 
                                                const int id,
                                                const std::string& text, 
                                                const std::vector<double>& color, 
                                                const std::string& ns,
                                                const double alpha,
                                                const std::string& frame_id="map"
                                                ){
        visualization_msgs::Marker marker;
        marker.lifetime = ros::Duration(0.2);
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = center(0);
        marker.pose.position.y = center(1);
        marker.pose.position.z = center(2);
        marker.scale.z = 0.3;
        marker.color.a = alpha;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.text = text;
        return marker;
    }

    // create a marker for an arrow
    inline visualization_msgs::Marker createArrowMarker(const Eigen::Vector3d& position, 
                                                const Eigen::Quaterniond& pose_quat, // quat in Eigen is (w, x, y, z)
                                                const int id,
                                                const std::string& ns, 
                                                const std::vector<double>& color, 
                                                const std::vector<double>& scale, // scale is (length, width, hieght) along arrow heading
                                                const double alpha,
                                                const std::string& frame_id="map"
                                                ){
        visualization_msgs::Marker marker;
        marker.lifetime = ros::Duration(0.1);
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = position(0);
        marker.pose.position.y = position(1);
        marker.pose.position.z = position(2);
        marker.pose.orientation.x = pose_quat.x();
        marker.pose.orientation.y = pose_quat.y();
        marker.pose.orientation.z = pose_quat.z();
        marker.pose.orientation.w = pose_quat.w();
        marker.scale.x = scale[0];
        marker.scale.y = scale[1];
        marker.scale.z = scale[2];
        marker.color.a = alpha;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        return marker;
    }

    // create bounding boxes
    inline visualization_msgs::Marker createBoundingBoxMarkerArray(const Eigen::Vector3d& center, 
                                                const Eigen::Vector3d& size, 
                                                // const Eigen::Quaterniond& pose_quat, 
                                                const int id,
                                                const std::string& ns, 
                                                const std::vector<double>& color, 
                                                const double alpha,
                                                const std::string& frame_id="map"
                                                ){
        // define properties of the bounding box
        visualization_msgs::Marker line;
        line.header.frame_id = "map";
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.ns = "box3D";  
        line.scale.x = 0.06;
        line.color.r = color[0];
        line.color.g = color[1];
        line.color.b = color[2];
        line.color.a = alpha;
        line.lifetime = ros::Duration(0.2);

        // get the center of the bounding box
        double x = center(0);
        double y = center(1);
        double z = center(2);
        double x_width = size(0);
        double y_width = size(1);
        double z_width = size(2);

        // Initialize the quaternion to identity
        line.pose.orientation.x = 0.0;
        line.pose.orientation.y = 0.0;
        line.pose.orientation.z = 0.0;
        line.pose.orientation.w = 1.0;

        // define the vertices of the bounding box
        std::vector<geometry_msgs::Point> verts;
        geometry_msgs::Point p;
        // vertice 0
        p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
        verts.push_back(p);

        // vertice 1
        p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
        verts.push_back(p);

        // vertice 2
        p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
        verts.push_back(p);

        // vertice 3
        p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
        verts.push_back(p);

        // vertice 4
        p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
        verts.push_back(p);

        // vertice 5
        p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
        verts.push_back(p);

        // vertice 6
        p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
        verts.push_back(p);

        // vertice 7
        p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
        verts.push_back(p);

        // int vert_idx[12][2] = {
        //     {0,1},
        //     {1,2},
        //     {2,3},
        //     {0,3},
        //     {0,4},
        //     {1,5},
        //     {3,7},
        //     {2,6},
        //     {4,5},
        //     {5,6},
        //     {4,7},
        //     {6,7}
        // };
        
        // for (size_t i=0;i<12;i++){
        //     line.points.push_back(verts[vert_idx[i][0]]);
        //     line.points.push_back(verts[vert_idx[i][1]]);
        // }

         // Add the lines between the vertices to form the edges of the bounding box
        for (int i = 0; i < 4; ++i) {
            line.points.push_back(verts[i]);
            line.points.push_back(verts[(i + 1) % 4]);
            line.points.push_back(verts[i + 4]);
            line.points.push_back(verts[(i + 1) % 4 + 4]);
            line.points.push_back(verts[i]);
            line.points.push_back(verts[i + 4]);
        }
        
        
        line.id = id;
        return line;
    }
}


#endif
