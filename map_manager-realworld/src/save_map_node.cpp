#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <map_manager/semanticObjArrayMsg.h>
#include <fstream>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <ctime>
#include <iomanip>
#include <sstream>

std::string getCurrentDate() {
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
    return oss.str();
}

const std::string filename = getCurrentDate() + "_base";

void mapSaverCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    std::cout << "[Map Saver]: Latest Map Message Obtained." << std::endl;
    
    pcl::PCLPointCloud2* pclCloud2 = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ> pclCloud;

    pcl_conversions::toPCL(*cloud_msg, *pclCloud2);
    pcl::fromPCLPointCloud2( *pclCloud2, pclCloud);
    if (pclCloud.size() > 0) {
        pcl::io::savePCDFileASCII (filename + ".pcd", pclCloud);
        // pcl::io::savePCDFileASCII ("./" + getCurrentDate() + "_map.pcd", pclCloud);
        ROS_INFO("Successfully wrote to file: %s.pcd", filename.c_str());
    }
}

// Define a structure for semantic objects
struct SemanticObject {
    int label_id;
    Eigen::Vector3d position;
    Eigen::Vector3d size;
};

// Define a class for the subscriber node
class SemanticObjectsSubscriber {
public:
    SemanticObjectsSubscriber() {
        sub_ = nh_.subscribe("/semantic_objects", 10, &SemanticObjectsSubscriber::callback, this);
        // loadFromFile();
    }

    ~SemanticObjectsSubscriber() {
        saveToFile();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::unordered_map<int, std::vector<SemanticObject>> semanticObjects_;
    
    void callback(const map_manager::semanticObjArrayMsg::ConstPtr& objArray) {
        semanticObjects_.clear(); // Clear the current data to store the latest

        for (const auto& incomeObject : objArray->semanticObjs) {
            SemanticObject obj;
            obj.label_id = incomeObject.label_id;
            obj.position = Eigen::Vector3d(incomeObject.position[0], incomeObject.position[1], incomeObject.position[2]);
            obj.size = Eigen::Vector3d(incomeObject.size[0], incomeObject.size[1], incomeObject.size[2]);

            // Add the object to the vector for the corresponding label_id
            semanticObjects_[incomeObject.label_id].push_back(obj);
        }

        saveToFile();
    }

    void saveToFile() {
        nlohmann::json j;
        for (const auto& [label_id, objs] : semanticObjects_) {
            for (const auto& obj : objs) {
                nlohmann::json obj_json;
                obj_json["position"] = {obj.position[0], obj.position[1], obj.position[2]};
                obj_json["size"] = {obj.size[0], obj.size[1], obj.size[2]};
                j[std::to_string(label_id)].push_back(obj_json);
            }
        }

        std::string filePath = filename + ".json";
        std::ofstream file(filePath, std::ofstream::out | std::ofstream::trunc);
        if (file.is_open()) {
            file << j.dump(4);
            file.close();
            ROS_INFO("Successfully wrote to file: %s", filePath.c_str());
        } else {
            ROS_ERROR("Unable to open file for writing: %s", filePath.c_str());
        }
    }

    void loadFromFile() {
        std::string filePath = filename + ".json";
        std::ifstream file(filePath);
        nlohmann::json j;

        try {
            file >> j;
        } catch (const nlohmann::json::parse_error& e) {
            ROS_WARN("Failed to parse JSON file: %s. Starting with an empty dataset.", e.what());
            return;
        }

        file.close();

        for (const auto& [key, value] : j.items()) {
            int label_id = std::stoi(key);
            for (const auto& obj_json : value) {
                SemanticObject obj;
                obj.label_id = label_id;
                obj.position = Eigen::Vector3d(obj_json["position"][0], obj_json["position"][1], obj_json["position"][2]);
                obj.size = Eigen::Vector3d(obj_json["size"][0], obj_json["size"][1], obj_json["size"][2]);
                semanticObjects_[label_id].push_back(obj);
            }
        }
    }
};

int main(int argc, char** argv){
	ros::init(argc, argv, "save_map_node");
	ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::param::set("/use_sim_time", false);
	// subscribe the map data
    mapSub = nh.subscribe("/spot/multi_robo_occupancy_map/voxel_map", 10, mapSaverCB);
    SemanticObjectsSubscriber sos;
	ros::spin();
	return 0;
}