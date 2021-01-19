"""
Based on the work of Waleed Abdulla (Matterport)
Modified by github.com/GustavZ
"""
# python 2 compability
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import sys
import random
import math
import re
import datetime
import numpy as np
import tensorflow as tf
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import skimage.io
import cv2
# Root directory of the project
ROOT_DIR = os.path.abspath("../")

# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
from mmrcnn import utils
from mmrcnn import visualize
from mmrcnn.visualize import display_images
import mmrcnn.model as modellib
from mmrcnn.model import log

def out_bbox_lines_totext(result, fid, gray_image):

    f_bbox = open( RESULT_DIR+'/'+ fid + "_mrcnn.txt" ,"w+")
    f_edge = open( RESULT_DIR+'/'+ fid + "_edge.txt" ,"w+")
        
    for i, (y1, x1, y2, x2) in enumerate(result['rois']):
            
        x_offset = np.asscalar(x1)
        y_offset = np.asscalar(y1)
        height = np.asscalar(y2 - y1)
        width = np.asscalar(x2 - x1)

        class_id = result['class_ids'][i]
        score = result['scores'][i]
        
        out_per_obj = str(class_id)+  " " + str(x_offset) + " " +str(y_offset) +  " " + str(width) + " " + str(height) + " " + str(score) 
        out_per_obj += "\n"
        f_bbox.write( out_per_obj)
        f_bbox.flush()
    
        lsd = cv2.createLineSegmentDetector(0)
        lines = lsd.detect(gray_image)[0]
    
        for line in lines:
            line_str = ""
            for l in line[0]:
                line_str = line_str + " " + str(l)
            f_edge.write(line_str+"\n")

# Directory of images to run detection on
IMAGE_DIR = os.path.join(ROOT_DIR, "data")

# Directory to save logs and trained model
MODEL_DIR = os.path.join(ROOT_DIR, "logs")

# Directory to save logs and trained model
RESULT_DIR = os.path.join(ROOT_DIR, "results")

# Local path to trained weights file
DEFAULT_WEIGHTS = os.path.join(ROOT_DIR, "mobile_mask_rcnn_coco.h5")

# Override the training configurations with a few
# changes for inferencing.
import coco
config = coco.CocoConfig()
class InferenceConfig(config.__class__):
    # Run detection on one image at a time
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    POST_NMS_ROIS_INFERENCE = 100

config = InferenceConfig()
config.display()

# Device to load the neural network on.
# Useful if you're training a model on the same 
# machine, in which case use CPU and leave the
# GPU for training.
#DEVICE = "/cpu:0"
DEVICE = "/gpu:0"

# Inspect the model in training or inference modes
# values: 'inference' or 'training'
# TODO: code for 'training' test mode not ready yet
TEST_MODE = "inference"
#TEST_MODE = "training"

# Create model in inference mode
with tf.device(DEVICE):
    model = modellib.MaskRCNN(mode=TEST_MODE, model_dir=MODEL_DIR,config=config)

# Set path to model weights
weights_path = DEFAULT_WEIGHTS
#weights_path = model.find_last()[1]

# Load weights
print("Loading weights ", weights_path)
model.load_weights(weights_path, by_name=True)

# COCO Class names
# Index of the class in the list is its ID. For example, to get ID of
# the teddy bear class, use: class_names.index('teddy bear')
class_names = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
               'bus', 'train', 'truck', 'boat', 'traffic light',
               'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
               'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
               'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
               'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
               'kite', 'baseball bat', 'baseball glove', 'skateboard',
               'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
               'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
               'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
               'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
               'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
               'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
               'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
               'teddy bear', 'hair drier', 'toothbrush']


# Load random images from the images folder
NUM_IMAGES=20
images = []
gray_images = []
file_names = next(os.walk(IMAGE_DIR))[2]
file_names.sort()
filtered_names=[]
for names in file_names:
    if 'DS' not in names:
        images.append(cv2.imread( os.path.join(IMAGE_DIR, names )))
        gray_images.append(cv2.imread( os.path.join(IMAGE_DIR, names ), 0) )
        filtered_names.append(names)

"""for i in range(NUM_IMAGES):
    # Read Image
    #images.append(skimage.io.imread(os.path.join(IMAGE_DIR, random.choice(file_names))))
    images.append(cv2.imread( os.path.join(IMAGE_DIR, random.choice(file_names) )))
"""
# Detection
times = []
for (i,image) in enumerate(images):
    # Run detection
    print("Run detection for {}".format( filtered_names[i]))

    start = datetime.datetime.now()
    
    if image is not None:
        results = model.detect([image], verbose=1)
        stop = datetime.datetime.now()
        t = (stop-start).total_seconds()
        times.append(t)
        # Visualize results
        r = results[0]
        contours_list = visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'], 
                                    class_names, r['scores'])
        #contours_list = visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'], 
        #                        class_names, r['scores'], show_bbox=False, show_mask=False)

        fid = str(i)
        while len(fid) < 4:
            fid = '0' + fid
        #fid = filtered_names[i].split('.')[0][-4:]
        f_plygon= open( RESULT_DIR+'/'+fid + "_obj_vertices.txt" ,"w+")

        for contours in contours_list:
            vert_y = ""
            vert_x = ""
            for v in contours[0]:
                vert_x += str(v[0]) + " "
                vert_y += str(v[1]) + " "
            f_plygon.write( vert_x + "\n" + vert_y + "\n")


        out_bbox_lines_totext(r, fid, gray_images[i])
        print("elapsed time for detection: {}s".format(t))
    
#print("median FPS: {}".format(1./np.median(times)))


print("median FPS: {}".format(1./np.median(times)))
print(np.median(times))

