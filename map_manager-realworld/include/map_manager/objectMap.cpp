/*
objectMap.cpp
----------------
source code for object map 
*/

#include <map_manager/objectMap.h>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>

const uint16_t CV_16UC1_MAX = std::numeric_limits<uint16_t>::max();
    
namespace mapManager{
    objectMap::objectMap(objectMapParams params, Eigen::Vector3d center, int label, double viewAngBase,
                        std::shared_ptr<sensorModelFactory> sensorManager)
    {
        // attributes
        this->initTime_ = ros::Time::now();
        this->modelTime_ = ros::Time::now();

        this->position_ = center;
        this->centerOfMass_ = center;
        this->label_ = label;
        this->viewAngBase_ = viewAngBase;

        // init params
        this->mapRes_ = params.resolution;
        this->minMaskScoreThresh_ = params.min_mask_score_thresh;
        this->obsValidProbThresh_ = params.observe_valid_prob_thresh;
        this->obsInvalidProbThresh_ = params.observe_invalid_prob_thresh;
        this->viewAgIntervals_ = params.view_angle_intervals;
        this->isDebug_ = params.is_debug;
        this->debugRootDir_ = params.debug_root_dir;

        this->pHitLog_ = logit(pHit_);
        this->pMissLog_ = logit(pMiss_);
        this->pMinLog_ = logit(pMin_);
        this->pMaxLog_ = logit(pMax_);
        this->pOccLog_ = logit(pOcc_);
        Eigen::Vector3d mapSize(params.object_map_size[0], params.object_map_size[1], params.object_map_size[2]);
        this->mapSizeMin_ << -mapSize(0)/2, -mapSize(1)/2, -mapSize(2)/2;
        this->mapSizeMax_ <<  mapSize(0)/2,  mapSize(1)/2,  mapSize(2)/2;

        // regulate to align with global origin
        Eigen::Vector3d mapSizeMinGlobal, mapSizeMaxGlobal;
        this->object2GlobalFrame(this->mapSizeMin_, mapSizeMinGlobal);
        this->object2GlobalFrame(this->mapSizeMax_, mapSizeMaxGlobal);
        mapSizeMaxGlobal = this->regulateWithOrigin(mapSizeMaxGlobal);
        mapSizeMinGlobal = this->regulateWithOrigin(mapSizeMinGlobal);
        this->global2ObjectFrame(mapSizeMaxGlobal, this->mapSizeMax_);
        this->global2ObjectFrame(mapSizeMinGlobal, this->mapSizeMin_);
        mapSize = this->mapSizeMax_ - this->mapSizeMin_;

        // min max for voxel
        this->mapVoxelMin_(0) = 0; this->mapVoxelMax_(0) = ceil(mapSize[0]/this->mapRes_);
        this->mapVoxelMin_(1) = 0; this->mapVoxelMax_(1) = ceil(mapSize[1]/this->mapRes_);
        this->mapVoxelMin_(2) = 0; this->mapVoxelMax_(2) = ceil(mapSize[2]/this->mapRes_);
        int reservedSize = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);

        

        // map data containers
        this->countHitMiss_.resize(reservedSize, 0);
        this->countHit_.resize(reservedSize, 0);
        this->occupancy_.resize(reservedSize, this->pMinLog_-this->UNKNOWN_FLAG_);
        this->globalCloudVoxelList_.resize(reservedSize);
        // this->flagTraverse_.resize(reservedSize, false);
        // this->flagRayend_.resize(reservedSize, false);

        // init view angles
        this->viewAngles_.resize(this->viewAgIntervals_, 0);
        this->isVisibleList_.resize(this->viewAgIntervals_, 0);
        // init object observations
        this->observationList_.resize(this->viewAgIntervals_);

        this->sensorManager_ = sensorManager;
    }

    objectMap::~objectMap()
    {
    }

    void objectMap::mergeWithOtherMap(std::shared_ptr<objectMap> otherMap){
        // std::cout << "this : " << this->position_.transpose()<< ", pointcloud size: " << this->pointCloud_.size() << std::endl;
        // std::cout << "other : " << otherMap->position_.transpose() << ", pointcloud size: " << otherMap->pointCloud_.size() << std::endl;
        ROS_INFO("initTime of this: %f seconds, %lu nanoseconds", this->initTime_.toSec(), this->initTime_.toNSec());
        ROS_INFO("initTime of other: %f seconds, %lu nanoseconds", otherMap->initTime_.toSec(), otherMap->initTime_.toNSec());
        // merge initTime
        if (this->initTime_.toSec() < otherMap->initTime_.toSec()){
            this->initTime_ = otherMap->initTime_;
            this->modelTime_ = otherMap->modelTime_;
        }
        // merge occupancy
        // loop throught all the index of othermap
        for (int x = 0; x < otherMap->mapVoxelMax_(0); x++){
            for (int y = 0; y < otherMap->mapVoxelMax_(1); y++){
                for (int z = 0; z < otherMap->mapVoxelMax_(2); z++){
                    // associate other address and this address
                    Eigen::Vector3i otherIdx(x, y, z);
                    int otherAdr = otherMap->indexToAddress(otherIdx);
                    Eigen::Vector3d otherPos;
                    Eigen::Vector3d thisPos;
                    otherMap->indexToPos(otherIdx, otherPos);
                    otherMap->object2GlobalFrame(otherPos, otherPos);
                    this->global2ObjectFrame(otherPos, thisPos);
                    this->posToIndex(thisPos, otherIdx);
                    Eigen::Vector3d thisPosGlobal;
                    this->object2GlobalFrame(thisPos, thisPosGlobal);
                    if (not this->isInMapRange(thisPos)){
                        // if (otherMap->isOccupied(otherAdr)){
                        //     std::cout << "other pos: " << otherPos.transpose() << "is occupied but out of range" << std::endl;
                        // }
                        continue;
                    }
                    
                    int thisAdr = this->posToAddress(thisPos);
                    // update map data containers
                    if (this->occupancy_[thisAdr] < otherMap->occupancy_[otherAdr]){
                        this->occupancy_[thisAdr] = otherMap->occupancy_[otherAdr];
                    }
                    // this->occupancy_[thisAdr] = otherMap->occupancy_[otherAdr];
                    // this->countHitMiss_[thisAdr] += otherMap->countHitMiss_[otherAdr];
                    // this->countHit_[thisAdr] += otherMap->countHit_[otherAdr];
                    // merge unseen point cloud voxel only
                    if (this->globalCloudVoxelList_[thisAdr].size() ==0){
                        this->globalCloudVoxelList_[thisAdr] = otherMap->globalCloudVoxelList_[otherAdr];
                    }
                    // this->globalCloudVoxelList_[thisAdr] = otherMap->globalCloudVoxelList_[otherAdr];
                    
                    
                }
            }
        }
        // update observations
        // loop through othermap view angles
        for (int i = 0; i < otherMap->viewAgIntervals_; i++){
            // transform other's view angle to this view angle
            double otherViewAng = otherMap->viewAngIdx2Ang(i);
            int thisViewAngIdx = this->viewAng2Idx(otherViewAng);
            // if other has observation, update this observation
            if (otherMap->viewAngles_[i] == 1){
                this->viewAngles_[thisViewAngIdx] = 1;
            }
            if (otherMap->isVisibleList_[i] == 1){
                this->isVisibleList_[thisViewAngIdx] = 1;
            }
            objectObservation otherObs = otherMap->observationList_[i];
            this->observationList_[thisViewAngIdx].observed_num_ += otherObs.observed_num_;
            this->observationList_[thisViewAngIdx].detected_num_ += otherObs.detected_num_;
            double positiveProb;
            if (this->observationList_[thisViewAngIdx].observed_num_ > 0){
                positiveProb = this->observationList_[thisViewAngIdx].detected_num_/this->observationList_[thisViewAngIdx].observed_num_;
            }
            else{
                positiveProb = 0;
            }
            if (positiveProb > this->obsValidProbThresh_){
                // if overall positive, at leat one of them is positive. choose the one with higher mask score
                if (this->observationList_[thisViewAngIdx].mask_score_ < otherObs.mask_score_){
                    this->observationList_[thisViewAngIdx] = otherObs;
                }
            }
            else{
                // if overall negative, at least one of them is negative. choose the one with lower mask score, which must be empty mask
                // MAY NOT BE TRUE! lower one may not be empty mask
                if (this->observationList_[thisViewAngIdx].mask_score_ > otherObs.mask_score_){
                    this->observationList_[thisViewAngIdx] = otherObs;
                }
            }
        }

        // update last detection
        if (this->lastDetection_.header.stamp.toSec() < otherMap->lastDetection_.header.stamp.toSec()){
            this->lastDetection_ = otherMap->lastDetection_;
        }

        std::cout << "merged time : " << this->lastDetection_.header.stamp.toSec() << ", pointcloud size: " << this->pointCloud_.size() << std::endl;
    }

    void objectMap::recenterMap(){
        // shift the map to the center of mass, shift all the points in the map
        Eigen::Vector3d shift = this->centerOfMass_ - this->position_;
        int reservedSize = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
        std::vector<double> occupancyTemp(reservedSize, this->pMinLog_-this->UNKNOWN_FLAG_);
        std::vector<std::stack<Eigen::Vector3d>> globalCloudVoxelListTemp(reservedSize);
        std::vector<int> countHitTemp(reservedSize, 0);
        std::vector<int> countHitMissTemp(reservedSize, 0);
        for (int x = 0; x < this->mapVoxelMax_(0); x++){
            for (int y = 0; y < this->mapVoxelMax_(1); y++){
                for (int z = 0; z < this->mapVoxelMax_(2); z++){
                    Eigen::Vector3i idx(x, y, z);
                    int adr = this->indexToAddress(idx);
                    Eigen::Vector3d posLocal;
                    this->indexToPos(idx, posLocal);
                    posLocal = posLocal - shift;
                    // is in map
                    if (not this->isInMapRange(posLocal)){
                        continue;
                    }
                    this->posToIndex(posLocal, idx);
                    int newAdr = this->indexToAddress(idx);
                    occupancyTemp[newAdr] = this->occupancy_[adr];
                    globalCloudVoxelListTemp[newAdr] = this->globalCloudVoxelList_[adr];
                    countHitTemp[newAdr] = this->countHit_[adr];
                    countHitMissTemp[newAdr] = this->countHitMiss_[adr];
                }
            }
        }
        this->occupancy_ = occupancyTemp;
        this->globalCloudVoxelList_ = globalCloudVoxelListTemp;
        this->countHit_ = countHitTemp;
        this->countHitMiss_ = countHitMissTemp;

        this->position_ = this->centerOfMass_;
    }

    void objectMap::updateMap(){
        this->raycastNum_++;
        int rayendVoxelId;
        for (auto& point : this->observedPoints){
            this->updateObjVoxelInfo(point, true);
        }

        // update prob values
        int hit, miss;
        double logUpdateValue;
        while (this->updateObjVoxelCache_.size() > 0){
            int voxelAdr = this->updateObjVoxelCache_.front();
            this->updateObjVoxelCache_.pop();
            hit = this->countHit_[voxelAdr];
            miss = this->countHitMiss_[voxelAdr] - hit;
            if (hit >= miss and hit != 0){
                logUpdateValue = this->pHitLog_;
            }
            else{
                logUpdateValue = this->pMissLog_;
            }
            this->countHit_[voxelAdr] = 0; // clear hit
            this->countHitMiss_[voxelAdr] = 0; // clear hit and miss
            // update occupancy info
            if ((logUpdateValue >= 0) and (this->occupancy_[voxelAdr] >= this->pMaxLog_)){
                continue; // not increase p if max clamped
            }
            else if ((logUpdateValue <= 0) and (this->occupancy_[voxelAdr] == this->pMinLog_)){
                continue; // not decrease p if min clamped
            }
            else if ((logUpdateValue <= 0) and (this->occupancy_[voxelAdr] < this->pMinLog_)){
                this->occupancy_[voxelAdr] = this->pMinLog_; // if unknown set it free (prior), 
                continue;
            }
            this->occupancy_[voxelAdr] = std::min(std::max(this->occupancy_[voxelAdr]+logUpdateValue, this->pMinLog_), this->pMaxLog_);
        }
        this->observedPoints.clear();

        // recenter map if center bias is too large
        if ((this->centerOfMass_ - this->position_).norm() > 0.5){
            ROS_WARN("[objectMap]: center bias %f is too large, recenter map from %f %f %f to %f %f %f", 
            (this->centerOfMass_ - this->position_).norm(), this->position_(0), this->position_(1), this->position_(2), this->centerOfMass_(0), this->centerOfMass_(1), this->centerOfMass_(2));
            this->recenterMap();
        }

        // update label by choosing the most probable label in labels_probability_
        int maxLabel = -1;
        double maxProb = -1;
        for (const auto& labelProb : this->labels_probability_){
            if (labelProb.second > maxProb){
                maxProb = labelProb.second;
                maxLabel = labelProb.first;
            }
        }
        this->label_ = maxLabel;

    }

    // Updates the internal model of the object map: computes point cloud, bounding box, and updates dense cloud.
// Member variables referenced here are initialized in the objectMap constructor (see objectMap.cpp).
    void objectMap::updateModel(){
        // 1. Clear the existing point cloud for this object.
        this->pointCloud_.clear();

        // 2. Get the maximum index range of the voxel grid (mapVoxelMax_ is set in constructor).
        Eigen::Vector3i maxIdx;
        this->getMapIdxRange(maxIdx);

        // 3. Prepare variables for position and bounding box calculation.
        Eigen::Vector3d pos;
        Eigen::Vector3d minPos, maxPos;
        minPos << 1000, 1000, 1000;
        maxPos << -1000, -1000, -1000;
        bool hasValidVoxel = false;

        // 4. Iterate over all voxels in the object-local grid.
        for (int i = 0; i < maxIdx(0); i++){
            for (int j = 0; j < maxIdx(1); j++){
                for (int k = 0; k < maxIdx(2); k++){
                    // 5. If the voxel is occupied (occupancy_ set in constructor, isOccupied() checks threshold):
                    int adr = this->indexToAddress(Eigen::Vector3i(i, j, k));
                    if (this->isOccupied(Eigen::Vector3i(i, j, k)) && !this->globalCloudVoxelList_[adr].empty()){
                        // a. Convert voxel index to 3D position (indexToPos()).
                        this->indexToPos(Eigen::Vector3i(i, j, k), pos);

                        // b. Update bounding box min/max coordinates.
                        minPos(0) = std::min(minPos(0), pos(0));
                        minPos(1) = std::min(minPos(1), pos(1));
                        minPos(2) = std::min(minPos(2), pos(2));
                        maxPos(0) = std::max(maxPos(0), pos(0));
                        maxPos(1) = std::max(maxPos(1), pos(1));
                        maxPos(2) = std::max(maxPos(2), pos(2));

                        // c. Add this position to the object's point cloud.
                        this->pointCloud_.push_back(pos);
                        hasValidVoxel = true;

                        // d. Update the center of mass (in global coordinates).
                        Eigen::Vector3d posGlobal;
                        this->object2GlobalFrame(pos, posGlobal); // object2GlobalFrame() is a coordinate transform.
                        this->centerOfMass_ += posGlobal;
                    }
                }
            }
        }

        // 6. Compute the average center of mass (avoid divide by zero).
        this->centerOfMass_ /= (this->pointCloud_.size()+1);

        // 7. Generate the bounding box using the min/max coordinates.
        vision_msgs::BoundingBox3D bbox;
        bbox.center.position.x = (minPos(0) + maxPos(0)) / 2;
        bbox.center.position.y = (minPos(1) + maxPos(1)) / 2;
        bbox.center.position.z = (minPos(2) + maxPos(2)) / 2;
        bbox.size.x = maxPos(0) - minPos(0);
        bbox.size.y = maxPos(1) - minPos(1);
        bbox.size.z = maxPos(2) - minPos(2);
        // 8. If no valid voxels, set bounding box size to zero.
        if (!hasValidVoxel){
            bbox.size.x = 0;
            bbox.size.y = 0;
            bbox.size.z = 0;
        }
        this->boundingBox_.bbox = bbox;

        // 9. Update the bounding box hypothesis with the object label (label_ set in constructor).
        vision_msgs::ObjectHypothesisWithPose objHypo;
        objHypo.id = this->label_;
        if (this->boundingBox_.results.size() == 0){
            this->boundingBox_.results.push_back(objHypo);
        }
        else{
            this->boundingBox_.results[0] = objHypo;
        }

        // 10. If this is the first update since initialization and there are points, update the model time.
        if (this->initTime_ == this->modelTime_ && this->pointCloud_.size() > 0){
            this->modelTime_ = ros::Time::now();
        }

        // 11. Compute the bounding box min/max corners in 3D space.
        Eigen::Vector3d bboxMin, bboxMax;
        bboxMin(0) = this->boundingBox_.bbox.center.position.x - this->boundingBox_.bbox.size.x/2;
        bboxMin(1) = this->boundingBox_.bbox.center.position.y - this->boundingBox_.bbox.size.y/2;
        bboxMin(2) = this->boundingBox_.bbox.center.position.z - this->boundingBox_.bbox.size.z/2;
        bboxMax(0) = this->boundingBox_.bbox.center.position.x + this->boundingBox_.bbox.size.x/2;
        bboxMax(1) = this->boundingBox_.bbox.center.position.y + this->boundingBox_.bbox.size.y/2;
        bboxMax(2) = this->boundingBox_.bbox.center.position.z + this->boundingBox_.bbox.size.z/2;

        // 12. Convert bounding box corners to voxel indices.
        Eigen::Vector3i voxelMin, voxelMax;
        this->posToIndex(bboxMin, voxelMin);
        this->posToIndex(bboxMax, voxelMax);

        // 13. Loop through all voxels in the bounding box.
        int count = 0;
        int adr;
        for (int x = voxelMin(0); x < voxelMax(0); x++){
            for (int y = voxelMin(1); y < voxelMax(1); y++){
                for (int z = voxelMin(2); z < voxelMax(2); z++){
                    Eigen::Vector3i idx(x, y, z);
                    adr = this->indexToAddress(idx); // indexToAddress() maps 3D index to 1D array index.
                    // 14. If this voxel is occupied:
                    if (this->isOccupied(adr)){
                        // For all 26 neighbors (including self), aggregate dense cloud points from unoccupied neighbors.
                        for (int i = -1; i <= 1; i++){
                            for (int j = -1; j <= 1; j++){
                                for (int k = -1; k <= 1; k++){
                                    Eigen::Vector3i neighborIdx(x+i, y+j, z+k);
                                    int neighborAdr = this->indexToAddress(neighborIdx);
                                    Eigen::Vector3d neighborPos;
                                    this->indexToPos(neighborIdx, neighborPos);

                                    // Only consider neighbors inside the bounding box.
                                    if (neighborPos(0) < bboxMin(0) || neighborPos(0) > bboxMax(0) ||
                                        neighborPos(1) < bboxMin(1) || neighborPos(1) > bboxMax(1) ||
                                        neighborPos(2) < bboxMin(2) || neighborPos(2) > bboxMax(2)){
                                        continue;
                                    }
                                    // Skip invalid addresses.
                                    if (neighborAdr < 0 || neighborAdr >= this->globalCloudVoxelList_.size()){
                                        continue;
                                    }
                                    // If neighbor is not occupied, copy its dense cloud stack to this voxel's stack.
                                    if (!this->isOccupied(neighborAdr)){
                                        // Copy all points from the neighbor's stack into this voxel's stack, up to 2000 points.
                                        std::vector<Eigen::Vector3d> tempPoints;
                                        tempPoints.reserve(globalCloudVoxelList_[neighborAdr].size());
                                        std::stack<Eigen::Vector3d> tempStack = globalCloudVoxelList_[neighborAdr];
                                        while (!tempStack.empty()) {
                                            if (globalCloudVoxelList_[adr].size() > 2000){
                                                break;
                                            }
                                            globalCloudVoxelList_[adr].push(tempStack.top());
                                            tempStack.pop();
                                            count++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        // At this point, pointCloud_, boundingBox_, and dense cloud for each voxel are updated.
    }

    void objectMap::reprojectionUpdate(){
        // print out map center and view angles
        //TODO： check isVisibileList. Do not update if not visible!!!!!!!!
        for (int i = 0; i < this->viewAgIntervals_; i++){
            if (this->viewAngles_[i] == 0){
                continue;
            }
            cv::Mat instanceMask = this->observationList_[i].instance_mask_;
            cv::Mat instanceMaskVisCopy = instanceMask.clone();
            if (instanceMask.empty()){
                continue;
            }
            
            Eigen::Matrix4d observation_map_2_body = this->observationList_[i].observation_map_2_body_;
            Eigen::Vector3d camPos, camPosObjFrame;
            this->sensorManager_->colorSensorModel_->sensor2GlobalFrame(Eigen::Vector3d(0,0,0), camPos, observation_map_2_body);
            this->global2ObjectFrame(camPos, camPosObjFrame);
            // reprojection update
            for (const auto& point : this->pointCloud_){
                // reproject current model's point cloud into image frame to generate reprojection mask
                // transform from object frame to global frame
                Eigen::Vector3d posGlobalFrame;
                Eigen::Vector3d coordImgFrame; // u, v, depth(virtual) in color sensor raw input frame
                
                this->object2GlobalFrame(point, posGlobalFrame);
                this->sensorManager_->global2ColorSensorRawInput(observation_map_2_body, posGlobalFrame, coordImgFrame);

                // reduce the occupancy probablity if:
                // 1. reproject pixel is out of the instance mask, reduce the occupancy probablity
                // 2. the point in model is not occluded by other objects
                // remember do raycasting in object frame!
                int address = -1;
                if (coordImgFrame(0) <= 0 || coordImgFrame(0) > instanceMask.cols ||
                    coordImgFrame(1) <= 0 || coordImgFrame(1) > instanceMask.rows) {
                    // ROS_WARN("[objectMap]: reprojection out of mask range");
                    continue;
                }
                else{
                    // debug
                    // std::cout << "reproject global frame: " << point.transpose() << std::endl;
                    // std::cout << "coordImgFrame: " << coordImgFrame.transpose() << " posGlobalFrame: " << posGlobalFrame.transpose() << "object map center: " << this->position_.transpose() << std::endl;                    
                }
                if (instanceMask.at<uchar>(coordImgFrame(1), coordImgFrame(0)) == 0){
                    // reduce occupancy probablity
                    address = this->posToAddress(point);
                    // Only reduce if confidence is low or observations are few
                    if (this->occupancy_[address] < this->pOccLog_ + this->pHitLog_) {
                        this->occupancy_[address] = std::max(this->occupancy_[address] - this->pOccLog_/4, this->pMinLog_);
                    }
                }
                else{
                    this->raycaster_.setInput(point/this->mapRes_, camPosObjFrame/this->mapRes_);
                    Eigen::Vector3d rayPoint, actualPoint;
                    bool increment = true;
                    // raycast: self occluded object voxel should not increment its probablity just because of be in the mask
                    // skip itself because it must be occupied
                    this->raycaster_.step(rayPoint);
                    while (this->raycaster_.step(rayPoint)){
                        actualPoint = rayPoint;
                        actualPoint(0) += 0.5;
                        actualPoint(1) += 0.5;
                        actualPoint(2) += 0.5;
                        actualPoint *= this->mapRes_;
                        if (!this->isInMapRange(actualPoint)){ // end of map
                            break;
                        }
                        if(this->isOccupied(actualPoint)){ // find occlusion, stop reprojection update
                            increment = false;
                            break;
                        }
                    }
                    address = this->posToAddress(point);
                    if (increment){
                        this->occupancy_[address] = std::min(this->occupancy_[address] + this->pOccLog_/2, this->pMaxLog_);
                    }
                }
                instanceMaskVisCopy.at<uchar>(coordImgFrame(1), coordImgFrame(0)) = 185;
            }

            // debug: save the instance mask for each view
            if (this->isDebug_){
                std::string maskDir = this->debugRootDir_  
                + std::to_string(this->modelTime_.toSec()) + "_";
                
                // transform rotation matrix to euler angles
                Eigen::Vector3d euler = observation_map_2_body.block<3,3>(0,0).eulerAngles(0, 1, 2);
                std::string maskNmae = 
                "/" + std::to_string(ros::Time::now().toSec()) + "_"
                + std::to_string(this->centerOfMass_(0)) + "_" 
                + std::to_string(this->centerOfMass_(1)) + "_" 
                + std::to_string(this->centerOfMass_(2)) + "_"
                + std::to_string(i) + "_" 
                // + std::to_string(observationPosition(0)) + "_" + std::to_string(observationPosition(1)) + "_" + std::to_string(observationPosition(2)) + "_" 
                // + std::to_string(euler(0)) + "_" + std::to_string(euler(1)) + "_" + std::to_string(euler(2)) + "_" 
                +std::to_string(this->pointCloud_.size()) + "_" 
                + std::to_string(this->observationList_[i].detected_num_) + " " + std::to_string(this->observationList_[i].observed_num_) + " "
                // + std::to_string(this->observationList_[i].mask_score_) 
                // + "_" + std::to_string(reprojectionOutlierFlag) +
                + std::to_string(this->observationList_[i].observation_map_2_body_(0,3)) + "_" + std::to_string(this->observationList_[i].observation_map_2_body_(1,3)) + "_" + std::to_string(this->observationList_[i].observation_map_2_body_(2,3))
                + ".png";

                struct stat info;
                if (stat(maskDir.c_str(), &info) != 0){
                    std::string cmd = "mkdir -p " + maskDir;
                    system(cmd.c_str());
                }
                cv::imwrite(maskDir + maskNmae, instanceMaskVisCopy);
            }
            
        }
    }

    void objectMap::updateMask(const sensor_msgs::Image& mask, const int label, const cv::Mat& lidarDpethImg, Eigen::Matrix4d observation_map_2_body){
        this->isMaskUpdated_ = true;
        // update view angles
        int view_idx = this->calcViewAgIdx(observation_map_2_body.block<3,1>(0,3));
        this->viewAngles_[view_idx] = 1;
        // std::cout << "object of center " << this->position_.transpose() << " has view angle idx: " << view_idx << std::endl;
        // update observations
        cv_bridge::CvImagePtr mask_cv_ptr = cv_bridge::toCvCopy(mask, sensor_msgs::image_encodings::MONO8);
        double score = calcMaskScore(mask_cv_ptr->image, lidarDpethImg);
        this->observationList_[view_idx].observed_num_++;
        if (score >= this->minMaskScoreThresh_){
            this->observationList_[view_idx].detected_num_++;
        }
        double prob = (double)this->observationList_[view_idx].detected_num_ / (double)this->observationList_[view_idx].observed_num_;
        if (score >= this->observationList_[view_idx].mask_score_ and prob >= this->obsValidProbThresh_){
            this->observationList_[view_idx].mask_score_ = score;
            this->observationList_[view_idx].instance_mask_ = mask_cv_ptr->image;
            this->observationList_[view_idx].observation_map_2_body_ = observation_map_2_body;
            this->observationList_[view_idx].stamp_ = mask.header.stamp;
            // this->viewAngles_[view_idx] = 1;
        }
        // fuse label id
        if (this->labels_probability_.find(label) == this->labels_probability_.end()){
            this->labels_probability_[label] = 1;
        }
        else{
            this->labels_probability_[label]++;
        }
    }

    void objectMap::updateEmptyMask(const cv::Mat& mask, Eigen::Matrix4d observation_map_2_body){
        this->isMaskUpdated_ = true;
        // update view angles
        int view_idx = this->calcViewAgIdx(observation_map_2_body.block<3,1>(0,3));
        this->viewAngles_[view_idx] = 1;
        // std::cout << "object of center " << this->position_.transpose() << " has view angle idx: " << view_idx << std::endl;
        // update observations
        this->observationList_[view_idx].observed_num_++;
        double prob = (double)this->observationList_[view_idx].detected_num_ / (double)this->observationList_[view_idx].observed_num_;
        if (prob < this->obsInvalidProbThresh_){
            this->observationList_[view_idx].mask_score_ = 0;
            this->observationList_[view_idx].instance_mask_ = mask;
            this->observationList_[view_idx].observation_map_2_body_ = observation_map_2_body;
            this->observationList_[view_idx].stamp_ = ros::Time::now();
            // this->viewAngles_[view_idx] = 1;
        }
    }

    
    // set dir of first glance as idx 0 view angle
    int objectMap::calcViewAgIdx(const Eigen::Vector3d& observationPosition){
        // update view angles
        Eigen::Vector3d viewDir = this->position_ - observationPosition;
        viewDir.normalize();
        double angle = atan2(viewDir(1), viewDir(0));
        // std::cout <<"angle: " << angle << std::endl;
        if (angle < 0){
            angle += 2*M_PI;
        }
        return this->viewAng2Idx(angle);
    }

    double objectMap::viewAngIdx2Ang(int idx){
        double angRes = 2*M_PI / this->viewAgIntervals_;
        return this->viewAngBase_ + idx * angRes - angRes/2;
    }

    int objectMap::viewAng2Idx(double angle){
        double relativeAngle = angle - this->viewAngBase_;
        double angRes = 2*M_PI / this->viewAgIntervals_;
        if (relativeAngle < 0){
            relativeAngle += 2*M_PI;
        }
        // std::cout << "viewangle base : " << this->viewAngBase_ << ", relative angle: " << relativeAngle << std::endl;
        int idx = floor((relativeAngle+angRes/2.0) / angRes);
        // if idx >= viewAgIntervals, set 0 because loop
        if (idx >= this->viewAgIntervals_){
            idx = 0;
        }
        return idx;
    }

    double objectMap::calcMaskScore(const cv::Mat& mask, const cv::Mat& lidarDepthImg){
        // calculate the quality score of the mask
        // the score is number of pixels in the mask that are not zero in original lidar depth map
        double CV_16UC1_MAX = 65535;
        double score = 0;
        cv::Mat result;
        // count how many pixels are less than CV_16UC1_MAX
        for (int i = 0; i < mask.rows; i++){
            for (int j = 0; j < mask.cols; j++){
                if (mask.at<uchar>(i, j) != 0 and lidarDepthImg.at<uint16_t>(i, j) < CV_16UC1_MAX){
                    score++;
                }
            }
        }
        // std::cout << "mask score: " << score << std::endl;
        return score;

    }

    double objectMap::get3dIOUwithOther(std::shared_ptr<objectMap> otherMap){
        // calculate 3d IOU with other object map
        // get bounding box of the other object map
        vision_msgs::Detection3D otherBoundingBox;
        otherMap->getBoundingBox(otherBoundingBox);
        // other box center and size
        Eigen::Vector3d otherCenter, otherSize;
        otherCenter << otherBoundingBox.bbox.center.position.x, otherBoundingBox.bbox.center.position.y, otherBoundingBox.bbox.center.position.z;
        // transform other center to this object frame
        otherMap->object2GlobalFrame(otherCenter, otherCenter);
        this->global2ObjectFrame(otherCenter, otherCenter);

        
        otherSize << otherBoundingBox.bbox.size.x, otherBoundingBox.bbox.size.y, otherBoundingBox.bbox.size.z;
        // current box center and size
        Eigen::Vector3d currentCenter, currentSize;
        currentCenter << this->boundingBox_.bbox.center.position.x, this->boundingBox_.bbox.center.position.y, this->boundingBox_.bbox.center.position.z;
        currentSize << this->boundingBox_.bbox.size.x, this->boundingBox_.bbox.size.y, this->boundingBox_.bbox.size.z;
       
        // calculate 3d IOU
        return calc3dIOO(currentCenter, currentSize, otherCenter, otherSize);
    }
    
} // namespace map_manager