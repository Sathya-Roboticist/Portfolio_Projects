import cv2
import numpy as np
import tensorflow as tf
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

def load_model(model_path):
    return tf.saved_model.load(model_path)

def run_inference_for_single_image(image, model):
    input_tensor = tf.convert_to_tensor(np.expand_dims(image, 0), dtype=tf.uint8)
    detections = model(input_tensor)
    return detections

def estimate_distance(object_points, image_points, camera_matrix, dist_coeffs):
    _, rvecs, tvecs, _ = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    distance = np.linalg.norm(tvecs)
    return distance

# Define your camera matrix and distortion coefficients
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
dist_coeffs = np.zeros((5, 1))

# Define 3D coordinates of object points (replace with actual values)
object_points = np.array([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]], dtype=np.float32)

model_path = 'C:/DL/TensorFlow/workspace/training_smart_ship/exported-models/my_model/saved_model'
detection_model = load_model(model_path)

label_map_path = 'C:/DL/TensorFlow/workspace/training_smart_ship/annotations/label_map.pbtxt'
category_index = label_map_util.create_category_index_from_labelmap(label_map_path, use_display_name=True)

video_path = 'C:/Users/R.Sathya Narayanan/Downloads/sweep_smart.mp4'
cap = cv2.VideoCapture(video_path)

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
output_video_path = 'output_video_1.mp4'
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_video_path, fourcc, 20.0, (frame_width, frame_height))

cv2.namedWindow('Object Detection', cv2.WINDOW_NORMAL)

while cap.isOpened():
    ret, image_np = cap.read()
    if not ret:
        break

    output_dict = run_inference_for_single_image(image_np, detection_model)

    # Extract 2D image points from the bounding box of the detected object
    image_points = np.array([
        [output_dict['detection_boxes'][0][1], output_dict['detection_boxes'][0][0]],  # Top-left corner
        [output_dict['detection_boxes'][0][3], output_dict['detection_boxes'][0][0]],  # Top-right corner
        [output_dict['detection_boxes'][0][3], output_dict['detection_boxes'][0][2]],  # Bottom-right corner
        [output_dict['detection_boxes'][0][1], output_dict['detection_boxes'][0][2]]   # Bottom-left corner
    ], dtype=np.float32)

    # Estimate distance using cv::solvePnP
    distance = estimate_distance(object_points, image_points, camera_matrix, dist_coeffs)

    # Display the estimated distance
    cv2.putText(image_np, f'Distance: {distance:.2f} units', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    vis_util.visualize_boxes_and_labels_on_image_array(
        image_np,
        output_dict['detection_boxes'][0].numpy(),
        output_dict['detection_classes'][0].numpy().astype(np.int32),
        output_dict['detection_scores'][0].numpy(),
        category_index,
        instance_masks=output_dict.get('detection_masks'),
        use_normalized_coordinates=True,
        line_thickness=8
    )

    cv2.imshow('Object Detection', cv2.resize(image_np, (800, 600)))
    out.write(image_np)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
