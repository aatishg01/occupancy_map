#!/usr/bin/env python3
import rospy
from vision_msgs.msg import Detection2DArray

def bounding_box_callback(msg):
    for detection in msg.detections:
        center_x = detection.bbox.center.x
        center_y = detection.bbox.center.y
        width = detection.bbox.size_x
        height = detection.bbox.size_y
        class_name = detection.header.frame_id
        class_index = detection.results[0]
        
        rospy.loginfo(f"Class name: {class_name} Class ID: {class_index}")

def main():
    rospy.init_node('bounding_box_viewer', anonymous=True)
    rospy.Subscriber("/yolo_detector/detected_bounding_boxes", Detection2DArray, bounding_box_callback)
    rospy.spin()

if __name__ == '__main__':
    main()