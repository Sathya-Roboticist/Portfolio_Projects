import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'    # Suppress TensorFlow logging (1)
#os.environ["CUDA_VISIBLE_DEVICES"]="0,1"
import pathlib
import tensorflow as tf
import sys
import cv2
import time
program_start = time.time()
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')   # Suppress Matplotlib warnings
print("Importing Libraries Successfull.")
tf.get_logger().setLevel('ERROR')           # Suppress TensorFlow logging (2)

# Enable GPU dynamic memory allocation
os.environ['TF_GPU_ALLOCATOR'] = 'cuda_malloc_async'
gpus = tf.config.experimental.list_physical_devices('GPU')
#print(str(gpus))
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)

	
IMAGE_PATHS = []
count = 0
PATH_TO_IMG_DIR = 'C:/DL/TensorFlow/workspace/training_demo/test__images' #Need to change test images folder
for file in sorted(os.listdir(PATH_TO_IMG_DIR)):
	if file.endswith(".jpg"):
		IMAGE_PATHS.append('test__images/'+file)
		count += 1
print("Total Images = "+str(count))

PATH_TO_MODEL_DIR = 'C:/DL/TensorFlow/workspace/training_demo/exported-models/my_model/saved_model' #Need to change saved model folder exported model
PATH_TO_LABELS = 'C:/DL/TensorFlow/workspace/training_demo/annotations/label_map.pbtxt' #Need to change labelmap file
PATH_TO_SAVED_MODEL = "C:/DL/TensorFlow/workspace/training_demo/exported-models/my_model" + "/saved_model"

print('Loading model...', end='')
start_time = time.time()
# Load saved model and build the detection function
detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)
end_time = time.time()
elapsed_time = end_time - start_time
print('Done! Took {} seconds'.format(elapsed_time))

category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS,
                                                                    use_display_name=True)

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

image_no = 0
for image_path in IMAGE_PATHS:
    Inference_start_time = time.time()
    print('Running inference for {}, '.format(image_path), end='')
    image_np = load_image_into_numpy_array(image_path)

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

    viz_utils.visualize_boxes_and_labels_on_image_array(
          image_np_with_detections,
          detections['detection_boxes'],
          detections['detection_classes'],
          detections['detection_scores'],
          category_index,
          use_normalized_coordinates=True,
          max_boxes_to_draw=200,
          min_score_thresh=.50,
          agnostic_mode=False)
    image_rgb = cv2.cvtColor(image_np_with_detections, cv2.COLOR_BGR2RGB)
    # detections['detection_scores'] = detections['detection_scores'].astype(np.int64)
    image_name = image_path[-23:]
    final_score = detections['detection_scores']
    fruit_count = 0
    image_no += 1
    for i in range(50):
        if final_score[i] >= 0.5:
            fruit_count += 1
    Inference_stop_time = time.time()
    Inference_time = Inference_stop_time - Inference_start_time
    print("Image No: "+str(image_no)+":,"+str(fruit_count)+", Time:, "+str(Inference_time))
    cv2.imwrite("output/"+image_name, image_rgb)
    #cv2.imwrite('output/'+image_name, image_np_with_detections)

program_end =time.time()
program_runtime = program_end - program_start
print("Total program runtime: "+str(program_runtime))
print('Done')


