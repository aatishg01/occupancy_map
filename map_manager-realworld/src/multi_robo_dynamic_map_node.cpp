#include <ros/ros.h>
#include <map_manager/multiRoboDynamicMap.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "multi_robo_dynamic_map_node");
	ros::NodeHandle nh;

	mapManager::multiRoboDynamicMap m;
	m.initMultiRoboDynamicMap(nh);

	ros::spin();

	return 0;
}