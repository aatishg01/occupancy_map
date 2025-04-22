#include "sensorModel.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace mapManager {

    // LidarModel implementation
    LidarModel::LidarModel(const Eigen::Matrix4d& transform, const double fovV) 
    : body_to_sensor_transform_(transform), fovV_(fovV) {}

    void LidarModel::rawInput2SensorFrame(const Eigen::Vector3d& rawInput, Eigen::Vector3d& posSensor) const {
        // For Lidar, raw input is already in the sensor frame, so do nothing
        posSensor = rawInput;
    }

    void LidarModel::sensorFrame2RawInput(const Eigen::Vector3d& posSensor, Eigen::Vector3d& rawInput) const {
        // For Lidar, raw input is already in the sensor frame, so do nothing
        rawInput = posSensor;
    }

    void LidarModel::global2SensorFrame(const Eigen::Vector3d& posGlobal, Eigen::Vector3d& posSensor, const Eigen::Matrix4d& globalToBody) const {
        posSensor = (body_to_sensor_transform_.inverse() * (globalToBody.inverse() * posGlobal.homogeneous())).head<3>();
    }

    void LidarModel::sensor2GlobalFrame(const Eigen::Vector3d& posSensor, Eigen::Vector3d& posGlobal, const Eigen::Matrix4d& globalToBody) const {
        posGlobal = (globalToBody * body_to_sensor_transform_ * posSensor.homogeneous()).head<3>();
        // std::cout << "Lidar" << std::endl;
        // std::cout << "globalToBody: " << globalToBody << std::endl;
        // std::cout << "body_to_sensor_transform_: " << body_to_sensor_transform_ << std::endl;
        // std::cout << "posSensor: " << posSensor << std::endl;
        // std::cout << "posGlobal: " << posGlobal << std::endl;
    }

    // implement the getGlobalPointcloud function
    void LidarModel::getGlobalPointcloud(const Eigen::Matrix4d& global2Body, const std::shared_ptr<void>& rawInput, std::vector<Eigen::Vector3d>& output, int& inputCount) const {
        inputCount = 0;
		output.clear();
        // extract body position
        Eigen::Vector3d position = global2Body.block<3, 1>(0, 3);
        auto cloud = std::static_pointer_cast<pcl::PointCloud<pcl::PointXYZ>>(rawInput);
		// this->projPointcloud_.clear();
		Eigen::Vector3d currPointRangeSensor, currPointMap;
		for (size_t i=0; i<cloud->size(); ++i){
			currPointRangeSensor(0) = cloud->points[i].x;
			currPointRangeSensor(1) = cloud->points[i].y;
			currPointRangeSensor(2) = cloud->points[i].z;
			// Transform point from sensor frame to global frame using sensorModelFactory
            this->sensor2GlobalFrame(currPointRangeSensor, currPointMap, global2Body);
			
			// remove lidar detection of robot itself
			if (abs(currPointMap(0)-position(0))<=0.5 
				and currPointMap(2)<=0.2
				and abs(currPointMap(1)-position(1))<=0.3){
				continue;
			}
			output.push_back(currPointMap);
			inputCount++;
		}
    }

    // For Lidar, the field of view is 360 degrees for horizontal, but we can limit the vertical field of view
    // calculate the vertical angle between the point and the x-forward axis
    bool LidarModel::isInFov(const Eigen::Vector3d& posSensor) const {
        double angle = std::atan2(posSensor(2), posSensor(0));
        return (std::abs(angle) < fovV_ / 2);
    }

    std::string LidarModel::getSensorType() const {
        return "Lidar";
    }

    Eigen::Matrix4d LidarModel::getStaticTransform() const {
        return body_to_sensor_transform_;
    }

    // PanoCamModel implementation
    PanoCamModel::PanoCamModel(const Eigen::Matrix4d& transform, const double y0, const double fy, const int cols)
        : body_to_sensor_transform_(transform), y0_(y0), fy_(fy), cols_(cols) {}

    void PanoCamModel::rawInput2SensorFrame(const Eigen::Vector3d& rawInput, Eigen::Vector3d& posSensor) const {
        int u = static_cast<int>(rawInput.x());
        int v = static_cast<int>(rawInput.y());
        double depth = rawInput.z();
        posSensor(0) = depth * std::cos(M_PI - 2 * M_PI * u / cols_);
        posSensor(1) = depth * std::sin(M_PI - 2 * M_PI * u / cols_);
        posSensor(2) = -depth * ((v - y0_) / fy_);
    }

    void PanoCamModel::sensorFrame2RawInput(const Eigen::Vector3d& posSensor, Eigen::Vector3d& rawInput) const {
        double depth = std::sqrt(posSensor(0) * posSensor(0) + posSensor(1) * posSensor(1));
        double u = (-std::atan2(posSensor(1), posSensor(0)) + M_PI) * cols_ / (2 * M_PI);
        double v = y0_ - fy_ * posSensor(2) / depth;
        rawInput = Eigen::Vector3d(u, v, depth);
    }

    void PanoCamModel::sensor2GlobalFrame(const Eigen::Vector3d& posSensor, Eigen::Vector3d& posGlobal, const Eigen::Matrix4d& globalToBody) const {
        posGlobal = (globalToBody * body_to_sensor_transform_ * posSensor.homogeneous()).head<3>();
        // std::cout << "pano" << std::endl;
        // std::cout << "globalToBody: " << globalToBody << std::endl;
        // std::cout << "body_to_sensor_transform_: " << body_to_sensor_transform_ << std::endl;
        // std::cout << "posSensor: " << posSensor << std::endl;
        // std::cout << "posGlobal: " << posGlobal << std::endl;
    }

    void PanoCamModel::global2SensorFrame(const Eigen::Vector3d& posGlobal, Eigen::Vector3d& posSensor, const Eigen::Matrix4d& globalToBody) const {
        posSensor = (body_to_sensor_transform_.inverse() * (globalToBody.inverse() * posGlobal.homogeneous())).head<3>();
    }

    // implement the getGlobalPointcloud function
    void PanoCamModel::getGlobalPointcloud(const Eigen::Matrix4d& global2Body, const std::shared_ptr<void>& rawInput, std::vector<Eigen::Vector3d>& output, int& inputCount) const {
        // TO DO: implement this function
    }

    bool PanoCamModel::isInFov(const Eigen::Vector3d& posSensor) const {
        Eigen::Vector3d posRawInput;
        // chech range
        if (std::sqrt(posSensor(0) * posSensor(0) + posSensor(1) * posSensor(1)) > 2.5) {
            return false;
        }
        sensorFrame2RawInput(posSensor, posRawInput);
        int x_lower_limit = int(cols_/2-validRange_/2);
        int x_upper_limit = int(cols_/2+validRange_/2);
        return (posRawInput.x() >= x_lower_limit) && (posRawInput.x() < x_upper_limit) && (posRawInput.y() >= 0) && (posRawInput.y() < 600);
    }

    std::string PanoCamModel::getSensorType() const {
        return "PanoCam";
    }

    Eigen::Matrix4d PanoCamModel::getStaticTransform() const {
        return body_to_sensor_transform_;
    }

    // PinholeCamModel implementation
    PinholeCamModel::PinholeCamModel(const Eigen::Matrix4d& transform, 
                                    const double fx, const double fy, const double cx, const double cy, 
                                    const int cols, const int rows, const double depthMinValue, const double depthMaxValue,
                                    const double depthScale, const int depthFilterMargin, const int skipPixel)
    : body_to_sensor_transform_(transform), fx_(fx), fy_(fy), cx_(cx), cy_(cy), 
        cols_(cols), rows_(rows), depthMinValue_(depthMinValue), depthMaxValue_(depthMaxValue),
        depthScale_(depthScale), depthFilterMargin_(depthFilterMargin), skipPixel_(skipPixel) {}


    void PinholeCamModel::rawInput2SensorFrame(const Eigen::Vector3d& rawInput, Eigen::Vector3d& posSensor) const {
        // Assuming rawInput is in the image frame with z as depth
        double u = rawInput.x();
        double v = rawInput.y();
        double depth = rawInput.z();

        posSensor(1) = -(u - cx_) * depth / fx_;
        posSensor(2) = -(v - cy_) * depth / fy_;
        posSensor(0) = depth;
    }

    void PinholeCamModel::sensorFrame2RawInput(const Eigen::Vector3d& posSensor, Eigen::Vector3d& rawInput) const {
        double u = fx_ * (-posSensor(1)) / posSensor(0) + cx_;
        double v = fy_ * (-posSensor(2)) / posSensor(0) + cy_;
        rawInput(0) = u;
        rawInput(1) = v;
        rawInput(2) = posSensor(0);
    }

    void PinholeCamModel::global2SensorFrame(const Eigen::Vector3d& posGlobal, Eigen::Vector3d& posSensor, const Eigen::Matrix4d& globalToBody) const {
        posSensor = (body_to_sensor_transform_.inverse() * (globalToBody.inverse() * posGlobal.homogeneous())).head<3>();
    }

    void PinholeCamModel::sensor2GlobalFrame(const Eigen::Vector3d& posSensor, Eigen::Vector3d& posGlobal, const Eigen::Matrix4d& globalToBody) const {
        // posGlobal = bodyToGlobal * body_to_sensor_transform_ * posSensor.homogeneous();
        posGlobal = (globalToBody * body_to_sensor_transform_ * posSensor.homogeneous()).head<3>();
    }

    // implement the getGlobalPointcloud function
    void PinholeCamModel::getGlobalPointcloud(const Eigen::Matrix4d& global2Body, const std::shared_ptr<void>& rawInput, std::vector<Eigen::Vector3d>& output, int& inputCount) const {
        auto depthImage = std::static_pointer_cast<cv::Mat>(rawInput);
        // Check if depthImage is null
        if (!depthImage) {
            return;
        }
        inputCount = 0;
        output.clear();

		int cols = depthImage->cols;
		int rows = depthImage->rows;
		Eigen::Vector3d currPointRangeSensor, currPointMap;
		double depth;
		const double inv_factor = 1.0 / this->depthScale_;
		// iterate through each pixel in the depth image
		for (int v=this->depthFilterMargin_; v<rows-this->depthFilterMargin_; v=v+this->skipPixel_){ // row
			const uint16_t* rowPtr = depthImage->ptr<uint16_t>(v) + depthFilterMargin_;
			for (int u=this->depthFilterMargin_; u<cols-this->depthFilterMargin_; u=u+this->skipPixel_){ // column
				depth = (*rowPtr) * inv_factor;
				if (*rowPtr == 0) {
					depth = this->depthMaxValue_ + 0.1;
				} else if (depth < this->depthMinValue_) {
					continue;
				} else if (depth > this->depthMaxValue_ ) {
					continue;
				}

				rowPtr =  rowPtr + this->skipPixel_;

				// get 3D point in camera frame
				rawInput2SensorFrame(Eigen::Vector3d(u, v, depth), currPointRangeSensor);
                sensor2GlobalFrame(currPointRangeSensor, currPointMap, global2Body);

				// store current point
				output.push_back(currPointMap);
				inputCount++;
			}
		} 
    }

    bool PinholeCamModel::isInFov(const Eigen::Vector3d& posSensor) const {
        Eigen::Vector3d posRawInput;
        sensorFrame2RawInput(posSensor, posRawInput);
        return (posRawInput.x() >= 0) && (posRawInput.x() < cols_) && (posRawInput.y() >= 0) && (posRawInput.y() < rows_);
    }

    std::string PinholeCamModel::getSensorType() const {
        return "PinholeCam";
    }

    Eigen::Matrix4d PinholeCamModel::getStaticTransform() const {
        return body_to_sensor_transform_;
    }

    // sensorModelFactory implementation
    // std::shared_ptr<sensorModelBase> sensorModelFactory::createSensorModel(const std::string& sensorType, const Eigen::Matrix4d& transform, const cv::Mat& intrinsic, const double y0, const double fy, const int cols) {
    //     if (sensorType == "Lidar") {
    //         return std::make_shared<LidarModel>(transform, fovV);
    //     } else if (sensorType == "PanoCam") {
    //         return std::make_shared<PanoCamModel>(transform, y0, fy, cols);
    //     } else if (sensorType == "PinholeCam") {
    //         return std::make_shared<PinholeCamModel>(transform, intrinsic.at<double>(0, 2), intrinsic.at<double>(1, 2), intrinsic.at<double>(0, 0), intrinsic.at<double>(1, 1));
    //     } else {
    //         throw std::invalid_argument("Unknown sensor type");
    //     }
    // }

    // constructor
    sensorModelFactory::sensorModelFactory(int sensorFusionMode, int rangeSensorInputMode) {
        this->sensorFusionMode_ = sensorFusionMode;
        this->rangeSensorInputMode_ = rangeSensorInputMode;
    }

    void sensorModelFactory::sensor2GlobalFrame(const Eigen::Vector3d& posSensor, Eigen::Vector3d& posGlobal, const std::shared_ptr<sensorModelBase>& sensorModel, const Eigen::Matrix4d& globalToBody) {
        posGlobal = (globalToBody * (sensorModel->getStaticTransform() * posSensor.homogeneous())).head<3>();
    }  

    void sensorModelFactory::global2SensorFrame(const Eigen::Vector3d& posGlobal, Eigen::Vector3d& posSensor, const std::shared_ptr<sensorModelBase>& sensorModel, const Eigen::Matrix4d& globalToBody) {
        posSensor = (sensorModel->getStaticTransform().inverse() * (globalToBody.inverse() * posGlobal.homogeneous())).head<3>();
    }

    void sensorModelFactory::sensorToSensorFrame(const Eigen::Vector3d& posSourceSensor, Eigen::Vector3d& posTargetSensor, const std::shared_ptr<sensorModelBase>& sourceSensorModel, const std::shared_ptr<sensorModelBase>& targetSensorModel) {
        posTargetSensor = (targetSensorModel->getStaticTransform() * (sourceSensorModel->getStaticTransform().inverse() * posSourceSensor.homogeneous())).head<3>();
    }

    void sensorModelFactory::setRangeSensorModel(const std::shared_ptr<sensorModelBase>& model) {
        this->rangeSensorModel_ = model;
    }

    void sensorModelFactory::setColorSensorModel(const std::shared_ptr<sensorModelBase>& model) {
        this->colorSensorModel_ = model;
    }

    bool sensorModelFactory::isInRangeSensorFov(const Eigen::Matrix4d& globalToBody, const Eigen::Vector3d& posGlobal){
        Eigen::Vector3d posSensor;
        global2SensorFrame(posGlobal, posSensor, this->rangeSensorModel_, globalToBody);
        return this->rangeSensorModel_->isInFov(posSensor);
    }

    bool sensorModelFactory::isInColorSensorFov(const Eigen::Matrix4d& globalToBody, const Eigen::Vector3d& posGlobal){
        Eigen::Vector3d posSensor;
        global2SensorFrame(posGlobal, posSensor, this->colorSensorModel_, globalToBody);
        return this->colorSensorModel_->isInFov(posSensor);
    }

    void sensorModelFactory::getGlobalRangeSensorInput(const Eigen::Matrix4d& global2Body, const std::shared_ptr<void>& rawInput, std::vector<Eigen::Vector3d>& output, int& inputCount) {
        this->rangeSensorModel_->getGlobalPointcloud(global2Body, rawInput, output, inputCount);
    }

    void sensorModelFactory::global2RangeSensorRawInput(const Eigen::Matrix4d& global2Body, const Eigen::Vector3d& posGlobal, Eigen::Vector3d& rawInput){
        Eigen::Vector3d posSensor;
        global2SensorFrame(posGlobal, posSensor, this->rangeSensorModel_, global2Body);
        this->rangeSensorModel_->sensorFrame2RawInput(posSensor, rawInput);
    }

    void sensorModelFactory::global2ColorSensorRawInput(const Eigen::Matrix4d& global2Body, const Eigen::Vector3d& posGlobal, Eigen::Vector3d& rawInput){
        Eigen::Vector3d posSensor;
        global2SensorFrame(posGlobal, posSensor, this->colorSensorModel_, global2Body);
        this->colorSensorModel_->sensorFrame2RawInput(posSensor, rawInput);
    }

    void sensorModelFactory::rangeSensorRawInput2Global(const Eigen::Matrix4d& global2Body, const Eigen::Vector3d& rawInput, Eigen::Vector3d& posGlobal){
        Eigen::Vector3d posSensor;
        this->rangeSensorModel_->rawInput2SensorFrame(rawInput, posSensor);
        sensor2GlobalFrame(posSensor, posGlobal, this->rangeSensorModel_, global2Body);
    }

    void sensorModelFactory::colorSensorRawInput2Global(const Eigen::Matrix4d& global2Body, const Eigen::Vector3d& rawInput, Eigen::Vector3d& posGlobal){
        Eigen::Vector3d posSensor;
        this->colorSensorModel_->rawInput2SensorFrame(rawInput, posSensor);
        sensor2GlobalFrame(posSensor, posGlobal, this->colorSensorModel_, global2Body);
    }
}