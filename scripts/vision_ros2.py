#!/usr/bin/env python3

from utils import position2pose, project_point, img_to_cv2
from mask_rcnn import MaskRCNN
import time
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped
import rospy
from cv_bridge import CvBridge
import numpy as np
import argparse
import cv2
from torchvision.models.detection import maskrcnn_resnet50_fpn
from torchvision import transforms as T
import torch
from utils import position2pose
# from mask_rcnn import MaskRCNN, COCO_INSTANCE_CATEGORY_NAMES, COLORS
import time
from stream import Stream
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
import pyrealsense2 as rs
from cv_bridge import CvBridge


bridge = CvBridge()
stream = Stream()
stream.start()
if __name__ == '__main__':
    mask_rcnn = MaskRCNN()
    segment_pub = rospy.Publisher(
        '/mask_rcnn/segment_image', Image, queue_size=1)

    def callback(rgb_image):
        print("entered")
        rgb_image = img_to_cv2(rgb_image)

        masks, boxes, labels = mask_rcnn.forward(rgb_image)
        image = mask_rcnn.get_segmentation_image(
            rgb_image, masks, boxes, labels)

        target_pose = position2pose(target_centroid_xyz)
        segment_pub.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))

        print("x:", target_centroid_xyz[0], "y:",
              target_centroid_xyz[1], "z:", target_centroid_xyz[2])

    try:
        rospy.init_node('mask_rcnn_node', anonymous=True)
        rgb_image = message_filters.Subscriber(
            "/camera/color/image_raw", Image)
        ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_image], 10, 0.1, allow_headerless=True)
        ts.registerCallback(callback)
        time.sleep(3)
        rospy.spin()
    except rospy.ROSInterruptException:
        stream.stop()
        pass
    except KeyboardInterrupt:
        stream.stop()
        pass

    except Exception as e:
        print(e)

