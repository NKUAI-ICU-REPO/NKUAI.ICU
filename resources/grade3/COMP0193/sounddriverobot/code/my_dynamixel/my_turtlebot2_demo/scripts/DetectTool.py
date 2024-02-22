#! /usr/bin/env python


# import os
# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'    # Suppress TensorFlow logging (1)
# import pathlib
# import tensorflow as tf
# import warnings


# import rospy
# import cv2

# from std_msgs.msg import String
# from sensor_msgs.msg import Image



# warnings.filterwarnings("ignore")
# tf.get_logger().setLevel('ERROR')           # Suppress TensorFlow logging (2)
 
# # Enable GPU dynamic memory allocation
# gpus = tf.config.experimental.list_physical_devices('GPU')
# for gpu in gpus:
#     tf.config.experimental.set_memory_growth(gpu, True)
 
# def download_images():
#     base_url = 'https://raw.githubusercontent.com/tensorflow/models/master/research/object_detection/test_images/'
#     filenames = ['image1.jpg', 'image2.jpg']
#     image_paths = []
#     for filename in filenames:
#         image_path = tf.keras.utils.get_file(fname=filename,
#                                             origin=base_url + filename,
#                                             untar=False)
#         image_path = pathlib.Path(image_path)
#         image_paths.append(str(image_path))
#     return image_paths
 
# #IMAGE_PATHS = download_images()
# IMAGE_PATHS=[save_path+"image1.jpg",save_path+"image2.jpg"]


# save_path='/home/yum/EDisk/.keras/datasets/'
# # Download and extract model
# def download_model(model_name, model_date):
#     base_url = 'http://download.tensorflow.org/models/object_detection/tf2/'
#     model_file = model_name + '.tar.gz'
#     model_dir = tf.keras.utils.get_file(fname=model_name,
#                                         origin=base_url + model_date + '/' + model_file,
#                                         untar=True)
#     return str(model_dir)
 
# # MODEL_DATE = '20200711'
# # MODEL_NAME = 'centernet_hg104_1024x1024_coco17_tpu-32'
# # PATH_TO_MODEL_DIR = download_model(MODEL_NAMsave_pathE, MODEL_DATE)

# def download_labels(filename):
#      base_url = 'https://raw.githubusercontent.com/tensorflow/models/master/research/object_detection/data/'
#      label_dir = tf.keras.utils.get_file(fname=filename,
#                                          origin=base_url + filename,
#                                          untar=False)
#      label_dir = pathlib.Path(label_dir)
#      return str(label_dir)
 
# LABEL_FILENAME = 'mscoco_label_map.pbtxt'
# PATH_TO_LABELS = save_path+"mscoco_label_map.pbtxt"


# import time
# from object_detection.utils import label_map_util
# from object_detection.utils import visualization_utils as viz_utils
 
# PATH_TO_SAVED_MODEL = "/home/yum/EDisk/.keras/datasets/centernet_hg104_1024x1024_coco17_tpu-32/saved_model"
 
# print('Loading model...', end='')
# start_time = time.time()
 
# # Load saved model and build the detection function
# detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)
 
# end_time = time.time()
# elapsed_time = end_time - start_time
# print('Done! Took {} seconds'.format(elapsed_time))


# category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS,use_display_name=True)


# import numpy as np
# from PIL import Image
# import matplotlib.pyplot as plt
# import warnings
# warnings.filterwarnings('ignore')   # Suppress Matplotlib warnings
# def load_image_into_numpy_array(path):
#     """Load an image from file into a numpy array.
#     Puts image into numpy array to feed into tensorflow graph.
#     Note that by convention we put it into a numpy array with shape
#     (height, width, channels), where channels=3 for RGB.
#     Args:
#       path: the file path to the image
#     Returns:
#       uint8 numpy array with shape (img_height, img_width, 3)
#     """
#     return np.array(Image.open(path))
 


# cnt=0

 
# for image_path in IMAGE_PATHS:

#     cnt+=1
 
#     print('Running inference for {}... '.format(image_path), end='')
 
#     image_np = load_image_into_numpy_array(image_path)
 
#     # Things to try:
#     # Flip horizontally
#     # image_np = np.fliplr(image_np).copy()
 
#     # Convert image to grayscale
#     # image_np = np.tile(
#     #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)
 
#     # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
#     input_tensor = tf.convert_to_tensor(image_np)
#     # The model expects a batch of images, so add an axis with `tf.newaxis`.
#     input_tensor = input_tensor[tf.newaxis, ...]
 
#     # input_tensor = np.expand_dims(image_np, 0)
#     detections = detect_fn(input_tensor)
 
#     # All outputs are batches tensors.
#     # Convert to numpy arrays, and take index [0] to remove the batch dimension.
#     # We're only interested in the first num_detections.
#     num_detections = int(detections.pop('num_detections'))
#     detections = {key: value[0, :num_detections].numpy()
#                    for key, value in detections.items()}
#     detections['num_detections'] = num_detections
 
#     # detection_classes should be ints.
#     detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
 
#     image_np_with_detections = image_np.copy()
 
#     viz_utils.visualize_boxes_and_labels_on_image_array(
#           image_np_with_detections,
#           detections['detection_boxes'],
#           detections['detection_classes'],
#           detections['detection_scores'],
#           category_index,
#           use_normalized_coordinates=True,
#           max_boxes_to_draw=200,
#           min_score_thresh=.30,
#           agnostic_mode=False)
 
#     plt.figure()
#     plt.imsave('image{}.png'.format(cnt),image_np_with_detections)
#     print('Done')
# plt.show()

from object_detection.utils import label_map_util

# 路径到 mscoco_label_map.pbtxt 文件
label_map_path = '/home/yum/EDisk/.keras/datasets/mscoco_label_map.pbtxt'

# 加载标签映射文件
label_map = label_map_util.load_labelmap(label_map_path)

# 创建类别索引到名称的映射
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=90, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# 打印类别信息
for idx, category in category_index.items():
    print('类别ID:', idx)
    print('类别名称:', category['name'])
