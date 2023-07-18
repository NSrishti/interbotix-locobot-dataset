#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
import time
import subprocess
from rostopic import get_topic_type

from sensor_msgs.msg import Image, CompressedImage
from locobot_simulation.msg import BoundingBox, BoundingBoxes

from interbotix_xs_modules.locobot import InterbotixLocobotXS

# add yolov5 submodule to path
FILE = Path(__file__).resolve()
sys.path.insert(0, './yolov5')
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox

obj_class = []
exp_proc = None
search_flag = 0

def pan_tilt():
    
    cam_cmd = "(cd /home/srishti; ./pantilt.sh)"
    exp_proc = subprocess.Popen(cam_cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)
                            
def rotate_base():
    
    base_cmd = "(cd /home/srishti; ./rotatebase.sh)"
    exp_proc = subprocess.Popen(base_cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

def pick_place_near():
    
    base_cmd = "(cd /home/srishti; ./near.sh)"
    exp_proc = subprocess.Popen(base_cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

def pick_place_far():
    
    base_cmd = "(cd /home/srishti; ./far.sh)"
    exp_proc = subprocess.Popen(base_cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

def pick_place_left():
    
    base_cmd = "(cd /home/srishti; ./left.sh)"
    exp_proc = subprocess.Popen(base_cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

def pick_place_right():
    
    base_cmd = "(cd /home/srishti; ./right.sh)"
    exp_proc = subprocess.Popen(base_cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

def pick_place_above():
    
    base_cmd = "(cd /home/srishti; ./above.sh)"
    exp_proc = subprocess.Popen(base_cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

def pick_place_below():
    
    base_cmd = "(cd /home/srishti; ./below.sh)"
    exp_proc = subprocess.Popen(base_cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

def pick_place_front():
    
    base_cmd = "(cd /home/srishti; ./front.sh)"
    exp_proc = subprocess.Popen(base_cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

def pick_place_behind():
    
    base_cmd = "(cd /home/srishti; ./behind.sh)"
    exp_proc = subprocess.Popen(base_cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

onto = get_ontology("file:///Users/srishti/Downloads/ontology/urn_webprotege_ontology_eb1b07c2-68da-4f99-a0da-8ad9f4ad5a3d.owl").load()
onto.load()

def obj_action_in_ontology(action, obj):
    if(onto.search_one(label = action) and onto.search_one(label= obj)) :
        return 1
    else
        return 0

@torch.no_grad()    
class Yolov5Detector:
    
    def __init__(self, option):
        self.option = option
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        # Initialize weights 
        weights = rospy.get_param("~weights")
        # Initialize model
        #self.device = select_device(str(rospy.get_param("~device","")))
        
        device_num=''  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.device = select_device(device_num)
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup(imgsz=(1 if self.pt else bs, 3, *self.img_size), half=self.half)  # warmup        
        
        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.callback, queue_size=1
            )
        else:
            obj_class = ""	
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.callback, queue_size=1
            )
	
        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic"), BoundingBoxes, queue_size=10
        )
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=10
            )
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()
        
            
    
    def callback(self, data):
        """adapted from yolov5/detect.py"""
        #print("callback called")
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        im, im0 = self.preprocess(im)
        # print(im.shape)
        # print(img0.shape)
        # print(img.shape)

        # Run inference
        im = torch.from_numpy(im).to(self.device) 
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        )

        ### To-do move pred to CPU and fill BoundingBox messages
        
        # Process predictions 
        det = pred[0].cpu().numpy()

        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = data.header
        bounding_boxes.image_header = data.header
        
        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det):
                bounding_box = BoundingBox()
                c = int(cls)
                # Fill in bounding box message
                bounding_box.Class = self.names[c]
                bounding_box.probability = conf 
                bounding_box.xmin = int(xyxy[0])
                bounding_box.ymin = int(xyxy[1])
                bounding_box.xmax = int(xyxy[2])
                bounding_box.ymax = int(xyxy[3])

                bounding_boxes.bounding_boxes.append(bounding_box)
                obj_class.append(bounding_box.Class)

                # Annotate the image
                if self.publish_image or self.view_image:  # Add bbox to image
                      # integer class
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))       

            
                ### POPULATE THE DETECTION MESSAGE HERE

            # Stream results
            im0 = annotator.result()

        # Publish prediction
        self.pred_pub.publish(bounding_boxes)

        if(self.option == 1):
            object1 = "Cup"
            object2 = "Potted Plant"
            if(search_flag == 0):
                if(object1.lower() in obj_class and object2.lower() in obj_class):
                    if(obj_action_in_ontology("Picking", object1) and obj_action_in_ontology("KeepNear", object2)):
                        pick_place_near()
                        search_flag = 1

        if(self.option == 2):
            object1 = "Cup"
            object2 = "Potted Plant"
            if(search_flag == 0):
                if(object1.lower() in obj_class and object2.lower() in obj_class):
                    if(obj_action_in_ontology("Picking", object1) and obj_action_in_ontology("KeepFar", object2)):
                        pick_place_far()
                        search_flag = 1       

        if(self.option == 3):
            object1 = "Cup"
            object2 = "Bottle"
            if(search_flag == 0):
                if(object1.lower() in obj_class and object2.lower() in obj_class):
                    if(obj_action_in_ontology("Picking", object1) and obj_action_in_ontology("KeepLeft", object2)):
                        pick_place_left()
                        search_flag = 1    

        if(self.option == 4):
            object1 = "Cup"
            object2 = "Bottle"
            if(search_flag == 0):
                if(object1.lower() in obj_class and object2.lower() in obj_class):
                    if(obj_action_in_ontology("Picking", object1) and obj_action_in_ontology("KeepRight", object2)):
                        pick_place_right()
                        search_flag = 1    
    
        if(self.option == 5):
            object1 = "Cup"
            object2 = "Bench"
            if(search_flag == 0):
                if(object1.lower() in obj_class and object2.lower() in obj_class):
                    if(obj_action_in_ontology("Picking", object1) and obj_action_in_ontology("KeepAbove", object2)):
                        pick_place_above()
                        search_flag = 1    

        if(self.option == 6):
            object1 = "Cup"
            object2 = "Bench"
            if(search_flag == 0):
                if(object1.lower() in obj_class and object2.lower() in obj_class):
                    if(obj_action_in_ontology("Picking", object1) and obj_action_in_ontology("KeepBelow", object2)):
                        pick_place_below()
                        search_flag = 1    

        if(self.option == 7):
            object1 = "Cup"
            object2 = "Bottle"
            if(search_flag == 0):
                if(object1.lower() in obj_class and object2.lower() in obj_class):
                    if(obj_action_in_ontology("Picking", object1) and obj_action_in_ontology("KeepFront", object2)):
                        pick_place_front()
                        search_flag = 1    

        if(self.option == 8):
            object1 = "Cup"
            object2 = "Bottle"
            if(search_flag == 0):
                if(object1.lower() in obj_class and object2.lower() in obj_class):
                    if(obj_action_in_ontology("Picking", object1) and obj_action_in_ontology("KeepBehind", object2)):
                        pick_place_behind()
                        search_flag = 1      
                
	
        # Publish & visualize images
        if self.view_image:
            cv2.imshow(str(0), im0)
            cv2.waitKey(1)  # 1 millisecond
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))
            
        

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 


if __name__ == "__main__":

    check_requirements(exclude=("tensorboard", "thop"))
    rospy.init_node("yolov5", anonymous=True)
    print("1. KeepNear 2. KeepFar 3.KeepLeft 4. KeepRight 5. KeepAbove 6. KeepBelow 7. KeepFront 8. KeepBehind 0. Other")
    option = input("Enter your choice: ")
    detector = Yolov5Detector(option)
    rotate_base()
    time.sleep(60)
    pan_tilt()
    time.sleep(10)
    
    rospy.spin()
