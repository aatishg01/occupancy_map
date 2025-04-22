#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import std_msgs.msg

target_classes = ["person"]

img_topic = "/camera/color/image_raw"
weight = "yolov8l.pt"  # or path to your custom trained weight
conf_thresh = 0.85
iou_thresh = 0.45

class YOLOv8Detector:
    def __init__(self):
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available. This script requires GPU acceleration.")
        
        self.device = "cuda"
        print(f"YOLOv8 detector initializing on {self.device}")
        self.time = None
        self.img = None
        self.img_received = False
        self.img_detected = False

        # Initialize YOLO model
        self.model = YOLO(weight, verbose=False)
        self.model.to(self.device)
        
        # Subscribers and Publishers
        self.br = CvBridge()
        self.img_sub = rospy.Subscriber(img_topic, Image, self.image_callback)
        self.img_pub = rospy.Publisher("yolo_detector/detected_image", Image, queue_size=10)
        self.bbox_pub = rospy.Publisher("yolo_detector/detected_bounding_boxes", Detection2DArray, queue_size=10)
        self.time_pub = rospy.Publisher("yolo_detector/yolo_time", std_msgs.msg.Float64, queue_size=1)

        # Timers
        rospy.Timer(rospy.Duration(0.033), self.detect_callback)
        rospy.Timer(rospy.Duration(0.033), self.vis_callback)
        rospy.Timer(rospy.Duration(0.033), self.bbox_callback)
    
    def image_callback(self, msg):
        self.img = self.br.imgmsg_to_cv2(msg, "bgr8")
        self.time = msg.header.stamp
        self.img_received = True

    def detect_callback(self, event):
        start_time = rospy.Time.now()
        if self.img_received:
            results = self.inference(self.img)
            self.detected_img, self.detected_bboxes = self.postprocess(self.img.copy(), results)
            self.img_detected = True
        end_time = rospy.Time.now()
        self.time_pub.publish((end_time - start_time).to_sec())

    def vis_callback(self, event):
        if self.img_detected:
            self.img_pub.publish(self.br.cv2_to_imgmsg(self.detected_img, "bgr8"))

    def bbox_callback(self, event):
        if self.img_detected:
            bboxes_msg = Detection2DArray()
            for detected_box in self.detected_bboxes:
                class_name = detected_box[4]
                class_index = detected_box[5]
                conf = detected_box[6]
                
                bbox_msg = Detection2D()
                bbox_msg.bbox.center.x = (detected_box[0] + detected_box[2]) / 2
                bbox_msg.bbox.center.y = (detected_box[1] + detected_box[3]) / 2
                bbox_msg.bbox.size_x = detected_box[2] - detected_box[0]
                bbox_msg.bbox.size_y = detected_box[3] - detected_box[1]
                
                bbox_msg.header.frame_id = class_name
                bbox_msg.source_img = self.br.cv2_to_imgmsg(self.detected_img, "bgr8")
                bbox_msg.results.append(ObjectHypothesisWithPose(
                    id=class_index,
                    score=conf,
                ))
                
                bboxes_msg.detections.append(bbox_msg)
            bboxes_msg.header.stamp = self.time
            self.bbox_pub.publish(bboxes_msg)

    def inference(self, img):
        # Convert BGR to RGB and maintain original dimensions
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.model.predict(
            source=img_rgb,
            conf=conf_thresh,
            iou=iou_thresh,
            device=self.device,
            verbose=False
        )
        return results

    def postprocess(self, img, results):
        detected_boxes = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get coordinates in original image dimensions
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                conf = box.conf.item()
                cls = int(box.cls.item())
                class_name = self.model.names[cls]
                
                detected_boxes.append([x1, y1, x2, y2, class_name, cls, conf])
                
                # Draw bounding box and label
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(img, f'{class_name} {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
        return img, detected_boxes

if __name__ == '__main__':
    rospy.init_node('yolov8_detector', anonymous=True)
    try:
        detector = YOLOv8Detector()
        rospy.spin()
    except RuntimeError as e:
        rospy.logerr(str(e))
        rospy.signal_shutdown("CUDA not available")