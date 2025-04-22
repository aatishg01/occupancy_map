/*
FILE: sensorModel.h
--------------------------------------
sensor model header file
*/

// convention: A2B means transform the physical coordinate frame A to the physical coordinate frame B, 
// not the pose in this coordinate frame

#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>

namespace mapManager {

    class sensorModelBase {
    public:
        virtual ~sensorModelBase() {}

        // Pure virtual functions to be implemented by derived classes
        virtual void rawInput2SensorFrame(const Eigen::Vector3d& rawInput, Eigen::Vector3d& posSensor) const = 0;
        virtual void sensorFrame2RawInput(const Eigen::Vector3d& posSensor, Eigen::Vector3d& rawInput) const = 0;
        virtual void sensor2GlobalFrame(const Eigen::Vector3d& posSensor, Eigen::Vector3d& posGlobal, const Eigen::Matrix4d& globalToBody) const = 0;
        virtual void global2SensorFrame(const Eigen::Vector3d& posGlobal, Eigen::Vector3d& posSensor, const Eigen::Matrix4d& globalToBody) const = 0;
        virtual void getGlobalPointcloud(const Eigen::Matrix4d& global2Body, const std::shared_ptr<void>& rawInput, std::vector<Eigen::Vector3d>& output, int& inputCount) const = 0;
        virtual bool isInFov(const Eigen::Vector3d& posSensor) const = 0;
        virtual std::string getSensorType() const = 0;
        virtual Eigen::Matrix4d getStaticTransform() const = 0; // Get the static transform from body to sensor frame
    };

    class LidarModel : public sensorModelBase {
    public:
        LidarModel(const Eigen::Matrix4d& transform, const double fovV);
        ~LidarModel() {}

        void rawInput2SensorFrame(const Eigen::Vector3d& rawInput, Eigen::Vector3d& posSensor) const override;
        void sensorFrame2RawInput(const Eigen::Vector3d& posSensor, Eigen::Vector3d& rawInput) const override;
        void global2SensorFrame(const Eigen::Vector3d& posGlobal, Eigen::Vector3d& posSensor, const Eigen::Matrix4d& globalToBody) const override;
        void sensor2GlobalFrame(const Eigen::Vector3d& posSensor, Eigen::Vector3d& posGlobal, const Eigen::Matrix4d& globalToBody) const override;
        void getGlobalPointcloud(const Eigen::Matrix4d& global2Body, const std::shared_ptr<void>& rawInput, std::vector<Eigen::Vector3d>& output, int& inputCount) const override;
        bool isInFov(const Eigen::Vector3d& posSensor) const override;
        std::string getSensorType() const override;
        Eigen::Matrix4d getStaticTransform() const override;

    private:
        Eigen::Matrix4d body_to_sensor_transform_;
        double fovV_;
    };

    class PanoCamModel : public sensorModelBase {
    public:
        PanoCamModel(const Eigen::Matrix4d& transform, const double y0, const double fy, const int cols);
        ~PanoCamModel() {}

        void rawInput2SensorFrame(const Eigen::Vector3d& rawInput, Eigen::Vector3d& posSensor) const override;
        void sensorFrame2RawInput(const Eigen::Vector3d& posSensor, Eigen::Vector3d& rawInput) const override;
        void global2SensorFrame(const Eigen::Vector3d& posGlobal, Eigen::Vector3d& posSensor, const Eigen::Matrix4d& globalToBody) const override;
        void sensor2GlobalFrame(const Eigen::Vector3d& posSensor, Eigen::Vector3d& posGlobal, const Eigen::Matrix4d& globalToBody) const override;
        void getGlobalPointcloud(const Eigen::Matrix4d& global2Body, const std::shared_ptr<void>& rawInput, std::vector<Eigen::Vector3d>& output, int& inputCount) const override;
        bool isInFov(const Eigen::Vector3d& posSensor) const override;
        std::string getSensorType() const override;
        Eigen::Matrix4d getStaticTransform() const override;

    private:
        Eigen::Matrix4d body_to_sensor_transform_;
        double y0_;
        double fy_;
        int cols_;
        int validRange_ = 900;
    };

    class PinholeCamModel : public sensorModelBase {
    public:
        PinholeCamModel(const Eigen::Matrix4d& transform, 
                        const double fx, const double fy, const double cx, const double cy, 
                        const int cols, const int rows, const double depthMinValue, const double depthMaxValue,
                        const double depthScale, const int depthFilterMargin, const int skipPixel);
        ~PinholeCamModel() {}

        void rawInput2SensorFrame(const Eigen::Vector3d& rawInput, Eigen::Vector3d& posSensor) const override;
        void sensorFrame2RawInput(const Eigen::Vector3d& posSensor, Eigen::Vector3d& rawInput) const override;
        void global2SensorFrame(const Eigen::Vector3d& posGlobal, Eigen::Vector3d& posSensor, const Eigen::Matrix4d& globalToBody) const override;
        void sensor2GlobalFrame(const Eigen::Vector3d& posSensor, Eigen::Vector3d& posGlobal, const Eigen::Matrix4d& globalToBody) const override;
        void getGlobalPointcloud(const Eigen::Matrix4d& global2Body, const std::shared_ptr<void>& rawInput, std::vector<Eigen::Vector3d>& output, int& inputCount) const override;
        bool isInFov(const Eigen::Vector3d& posSensor) const override;
        std::string getSensorType() const override;
        Eigen::Matrix4d getStaticTransform() const override;

    private:
        Eigen::Matrix4d body_to_sensor_transform_;
        double cx_;
        double cy_;
        double fx_;
        double fy_;
        int cols_;
        int rows_;
        double depthMinValue_;
        double depthMaxValue_;
        double depthScale_;
        int depthFilterMargin_;
        int skipPixel_;
    };

    class sensorModelFactory {
    public:
        std::shared_ptr<sensorModelBase> rangeSensorModel_;
        std::shared_ptr<sensorModelBase> colorSensorModel_;
    
    public:
        sensorModelFactory(int sensorFusionMode, int rangeSensorInputMode);
        ~sensorModelFactory() {}
        // static std::shared_ptr<sensorModelBase> createSensorModel(const std::string& sensorType, const Eigen::Matrix4d& transform, const cv::Mat& intrinsic = cv::Mat(), const double y0 = 0.0, const double fy = 0.0, const int cols = 0);
        static void sensor2GlobalFrame(const Eigen::Vector3d& posSensor, Eigen::Vector3d& posGlobal, const std::shared_ptr<sensorModelBase>& sensorModel, const Eigen::Matrix4d& globalToBody);
        static void global2SensorFrame(const Eigen::Vector3d& posGlobal, Eigen::Vector3d& posSensor, const std::shared_ptr<sensorModelBase>& sensorModel, const Eigen::Matrix4d& globalToBody);
        static void sensorToSensorFrame(const Eigen::Vector3d& posSourceSensor, Eigen::Vector3d& posTargetSensor, const std::shared_ptr<sensorModelBase>& sourceSensorModel, const std::shared_ptr<sensorModelBase>& targetSensorModel);

        void setRangeSensorModel(const std::shared_ptr<sensorModelBase>& model);
        void setColorSensorModel(const std::shared_ptr<sensorModelBase>& model);
        std::shared_ptr<sensorModelBase> getRangeSensorModel();
        std::shared_ptr<sensorModelBase> getColorSensorModel();

        // user tool functions
        bool isInRangeSensorFov(const Eigen::Matrix4d& globalToBody, const Eigen::Vector3d& posGlobal);
        bool isInColorSensorFov(const Eigen::Matrix4d& globalToBody, const Eigen::Vector3d& posGlobal);
        // bool projInputPc2Global(const Eigen::Matrix4d& globalToBody, const Eigen::vector<Eigen::Vector3d>& rawInput);
        void getGlobalRangeSensorInput(const Eigen::Matrix4d& global2Body, const std::shared_ptr<void>& rawInput, std::vector<Eigen::Vector3d>& output, int& inputCount);
        void global2RangeSensorRawInput(const Eigen::Matrix4d& global2Body, const Eigen::Vector3d& posGlobal, Eigen::Vector3d& rawInput);
        void global2ColorSensorRawInput(const Eigen::Matrix4d& global2Body, const Eigen::Vector3d& posGlobal, Eigen::Vector3d& rawInput);
        void rangeSensorRawInput2Global(const Eigen::Matrix4d& global2Body, const Eigen::Vector3d& rawInput, Eigen::Vector3d& posGlobal);
        void colorSensorRawInput2Global(const Eigen::Matrix4d& global2Body, const Eigen::Vector3d& rawInput, Eigen::Vector3d& posGlobal);

    private:
        int sensorFusionMode_;
        int rangeSensorInputMode_;

        // std::shared_ptr<sensorModelBase> rangeSensorModel_;
        // std::shared_ptr<sensorModelBase> colorSensorModel_;
    };

}

#endif // SENSOR_MODEL_H