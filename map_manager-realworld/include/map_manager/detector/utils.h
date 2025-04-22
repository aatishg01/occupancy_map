 /*
 	FILE: utils.h
 	--------------------------
 	function utils for detectors
 */

#ifndef MAPMANAGER_DETECTOR_UTILS_H
#define MAPMANAGER_DETECTOR_UTILS_H

#include <opencv2/opencv.hpp>

namespace mapManager{
    struct box3D
    {
        /* data */
        float x, y, z;
        float x_width, y_width, z_width;
        float id;
        float Vx, Vy;
        float Ax, Ay;
        bool is_human=false; // false: not detected by yolo as dynamic, true: detected by yolo
        bool is_dynamic=false; // false: not detected as dynamic(either yolo or classificationCB), true: detected as dynamic
        bool fix_size=false; // flag to force future boxes to fix size
        bool is_dynamic_candidate=false;
    };

    inline double calc2DIou(cv::Rect boxA, cv::Rect boxB){
        int xA = std::max(boxA.x, boxB.x);
        int yA = std::max(boxA.y, boxB.y);
        int xB = std::min(boxA.x + boxA.width, boxB.x + boxB.width);
        int yB = std::min(boxA.y + boxA.height, boxB.y + boxB.height);

        int interArea = std::max(0, xB - xA) * std::max(0, yB - yA);

        int boxAArea = boxA.width * boxA.height;
        int boxBArea = boxB.width * boxB.height;

        return interArea / (double)(boxAArea + boxBArea - interArea);
    }
}

#endif