#!  /usr/bin/env python
import sys
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'    # Suppress TensorFlow logging (1)
import pathlib
import tensorflow as tf
import warnings


from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils


import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import  CvBridge,CvBridgeError

import  time




warnings.filterwarnings("ignore")
tf.get_logger().setLevel('ERROR')   



flag=False
flag_dep = True
dictPos=None
dictDirect=None
save_path='/home/yum/EDisk/.keras/datasets/'

IMAGE_PATHS="/home/yum/image.png"
LABEL_FILENAME = 'mscoco_label_map.pbtxt'
PATH_TO_LABELS = save_path+LABEL_FILENAME
PATH_TO_SAVED_MODEL = "/home/yum/EDisk/.keras/datasets/centernet_hg104_1024x1024_coco17_tpu-32/saved_model"

np_img = None
Catlabel = None
category_index = None
detect_fn = None
StrD = None
voicePub  = rospy.Publisher('/voiceWords',String,queue_size=10)


def load_image_into_numpy_array(path):
    """Load an image from file into a numpy array.
    Puts image into numpy array to feed into tensorflow graph.
    Note that by convention we put it into a numpy array with shape
    (height, width, channels), where channels=3 for RGB.
    Args:
      path: the file path to the image
    Returns:
      uint8 numpy array with shape (img_height, img_width, 3)
    """
    return np.array(Image.open(path))
def get_coco_label(label_map_path):
    global category_index
    keys=np.arange(1,91)
    Categorylabel=  dict.fromkeys(keys,None)
    label_map = label_map_util.load_labelmap(label_map_path)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=90, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)
    

# 打印类别信息
    for idx, category in category_index.items():
        Categorylabel[idx]=category['name']

    return Categorylabel





def call_backtoMatch(data):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data,'bgr8')
        #cv2.imwrite('/home/bluestar/image/template_pencilbags.jpg',cv_image)
       # cv2.waitKey(1000)
        template=cv2.imread('/home/bluestar/image/template.bmp')
        # cv2.imshow('template',template)
        # cv2.waitKey(2000)


        h,w = template.shape[:2]

        #rospy.loginfo("h:%d",h)

        res = cv2.matchTemplate(cv_image,template,cv2.TM_SQDIFF)
        min_val ,max_val,min_loc,max_loc  = cv2.minMaxLoc(res)

        
        # h=100
        # w=100

        top_left = min_loc
        bottom_right = (top_left[0]+w,top_left[1]+h)
        rospy.loginfo("tp:%d,w:%d",top_left[0],w)
        cv2.rectangle(cv_image,top_left,bottom_right,(0,0,255),6)
        cv2.imshow('dst',cv_image)
        cv2.imwrite('/home/yum/imageAns.jpg',cv_image)
        cv2.waitKey(2000)
    except Exception as  exp:
        print(exp)

def loadKerasModel(pathLoad):
        global detect_fn
        gpus = tf.config.experimental.list_physical_devices('GPU')
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        #rospy.loginfo('Loading model...', end='')
        start_time = time.time()
 
        # Load saved model and build the detection function
        detect_fn = tf.saved_model.load(pathLoad)
 
        end_time = time.time()
        elapsed_time = end_time - start_time
        rospy.loginfo('Done! Took {} seconds'.format(elapsed_time))
def VoiceWakeupLocal(msg):
    global flag
    global flag_dep
    if msg.data == '你看到了什么？':
        flag=True
    elif msg.data == '感知一下周围。' :
        flag_dep = True 
        
    

def PredictImage(image_path=None):
        
        global Catlabel

        strfrom = ""
        strdirection =""
        centerPos = []
        rospy.loginfo('开始预测')
        if image_path is not None:
            print('Running inference for {}... '.format(image_path), end='')
 
            image_np = load_image_into_numpy_array(image_path)
        else:
            if np_img is None:
                rospy.loginfo('无效')
            image_np = np_img.copy()
 
    # Things to try:
    # Flip horizontally
    # image_np = np.fliplr(image_np).copy()
 
    # Convert image to grayscale
    # image_np = np.tile(
    #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)
 
    # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(image_np)
    # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis, ...]
 
    # input_tensor = np.expand_dims(image_np, 0)
        detections = detect_fn(input_tensor)
 
    # All outputs are batches tensors.
    # Convert to numpy arrays, and take index [0] to remove the batch dimension.
    # We're only interested in the first num_detections.
        num_detections = int(detections.pop('num_detections'))
        detections = {key: value[0, :num_detections].numpy()
                   for key, value in detections.items()}
        detections['num_detections'] = num_detections
 
    # detection_classes should be ints.
        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
 
        image_np_with_detections = image_np.copy()


     #image_sub = rospy.Subscriber(image_topic,Image,call_backtoMatch)

        boxes = detections['detection_boxes']
        classes = detections['detection_classes'].astype(np.int32)
        scores = detections['detection_scores']

# 绘制检测框
        #rospy.loginfo(classes)
        for i in range(num_detections):
            direction=None
            if scores[i] > 0.6:  # 设定阈值来选择置信度较高的检测结果
                ymin, xmin, ymax, xmax = boxes[i]
                xmin = int(xmin * image_np.shape[1])
                xmax = int(xmax * image_np.shape[1])
                ymin = int(ymin * image_np.shape[0])
                ymax = int(ymax * image_np.shape[0])
                #rospy.loginfo(str(classes[i]))
                label = Catlabel[classes[i]]
                strfrom+=label
                strfrom+=' and '
                #rospy.loginfo('label is {}'.format(label))
                if (xmin+xmax)< image_np.shape[1]:
                        direction="left"
                else:
                        direction="right"
                dictPos[label]=[(xmin+xmax)/2,(ymin+ymax)/2]
                dictDirect[label] = direction
                if label == "person":
                    centerPos.append((xmin+xmax)/2)
                    centerPos.append((ymin+ymax)/2)
                    if (xmin+xmax)< image_np.shape[1]:
                        strdirection="left"
                    else:
                        strdirection="right"

           

 
        viz_utils.visualize_boxes_and_labels_on_image_array(
            image_np_with_detections,
            detections['detection_boxes'],
            detections['detection_classes'],
            detections['detection_scores'],
            category_index,
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            min_score_thresh=.30,
            agnostic_mode=False)
 
        plt.figure()
        plt.imsave('/home/yum/imagefinal.png',image_np_with_detections)
        rospy.loginfo('Done') 
        return  strfrom,centerPos,strdirection
def call_back(data):
    global np_img
    global flag
    global StrD
    try:
        if flag==False:
            return
        flag=False
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data,'rgb8')
        #np_img = np_img[:,:,::-1]
        np_img = np.array(cv_image)

        strby,pos,strDirection= PredictImage()
        rospy.loginfo("get "+strby)
        msg = String()
        msg.data = "I can see ,"+strby+"!    "
        if len(pos)!=0:
            rospy.loginfo('I can see a person!')
            StrD = strDirection
        msg.data += ",which is nearby my "+strDirection+" hand"
        voicePub.publish(msg)
        # cv2.imshow('src',cv_image)
        # cv2.waitKey(1000)

    except Exception as  e: 
        print(e) 
def call_backfromRGBD(data):
    global np_img
    global flag_dep
    depth_value = None
    try:
        if flag_dep==False:
            return
        flag_dep=False
        bridge=CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
        #cv2.imshow('src',cv_image)
        np_img = np.array(cv_image)
        strby,pos,_=PredictImage()
        posx,posy=0.0,0.0
        msg=String()
        if len(pos)!=0:
            posx=pos[0]
            posy=pos[1]
            rospy.loginfo('x pos {}'.format(posx))
            depth_value = cv_image[posx,posy]
            rospy.loginfo("depth Image {}".format(depth_value))
            msg.data = "I can see, person away {} meter".format(int(depth_value))
        else:
            msg.data = strby
        
        voicePub.publish(msg)



    except Exception  as e:
        print(e)

if  __name__ == '__main__':

    rospy.init_node('test_photo',anonymous=False) 
    loadKerasModel(PATH_TO_SAVED_MODEL)
    Catlabel=get_coco_label(PATH_TO_LABELS)
    #rospy.loginfo(str(Catlabel))
    #rospy.loginfo(Catlabel[61])
    dictPos = dict.fromkeys(Catlabel,None)
    dictDirect=dict.fromkeys(Catlabel,None)

    image_topic = '/camera/rgb/image_raw'
    image_depth_topic ='/camera/depth_registered/image'
    image_point2_topic = '/camera/depth_registered/points'

    r= rospy.Rate(10)


    while not rospy.is_shutdown():


     #image_sub = rospy.Subscriber(image_topic,Image,call_backtoMatch)
        voice_sub = rospy.Subscriber('/voiceWords',String,VoiceWakeupLocal)
        image_sub = rospy.Subscriber(image_topic,Image,call_back)
        image_dep_sub  = rospy.Subscriber(image_depth_topic,Image,call_backfromRGBD)
        rospy.sleep(5)
        r.sleep()

    