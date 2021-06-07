import rospy
import sys
import message_filters
import threading
import math

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage

from perception.msg import Object
from perception.msg import ObjectArray

import os

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

import cv2
import numpy as np

import tensorflow as tf
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

physical_devices = tf.config.list_physical_devices("GPU")
tf.config.experimental.set_memory_growth(physical_devices[0], True)
tf.config.experimental.set_virtual_device_configuration(
    physical_devices[0],
    [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=800)],
)

K = [381.36246688113556, 0.0, 320.5, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 1.0]
Fx = K[0]
Fy = K[4]
Cx = K[2]
Cy = K[5]
baseline = 0.210
HEIGHT = 480
WIDTH = 640
UPDATE_HZ = 10.0
CAMERA_FRAME_LINK = "_left_camera_optical"
CLASS_SCORE_THRESHOLD = 0.6

g_bridge = CvBridge()
g_seq = 0
g_lock = threading.Lock()

g_class_individual_thresh = {
    "processingPlant": 0.8,
    "repairStation": 0.8,
    "hopper": 0.6,
    "excavator": 0.7,
    "scout": 0.7,
}
g_class_not_to_be_duplicated = {
    "processingPlant",
    "repairStation",
    "hopper",
    "furnace",
    "excavatorArm",
}


def preProcessObjectDetection(
    category_index, scores, classes, num_detections, final_list
) -> None:
    """
    Preprocesses the detected object i.e. it outputs the list of indices of only those objects which have a confidence of higher than a default threshold
    Also, if there is a custom threshold defined for the class in g_class_individual_thresh, that threshold is also check, if the confidence is higher than
    both the threshold, then we check if there is already an object of same label detected (done to remove certain duplicate object because some objects will occur only
    once in an image as there is no copy such as processingPlant, so if there are two copies of processingPlant, then the copy with higher confidence will be published),
    if the class does not belong to g_class_not_to_be_duplicated, it is simply added to the output list
    """

    # maintains class labels which are added to the result
    current_class = set()
    for i in range(num_detections):
        if scores[i] > CLASS_SCORE_THRESHOLD:
            # score greater than a default threshold
            class_label = (category_index.get(classes[i])).get("name")

            if (
                class_label in g_class_individual_thresh
                and scores[i] < g_class_individual_thresh[class_label]
            ):
                # if individual threshold is defined and the current confidence is not greater than individual threshold
                continue

            if (
                class_label in g_class_not_to_be_duplicated
                and class_label in current_class
            ):
                # if there is already an element added to the output of a specific class which should not be duplicated
                # input scores are sorted high to low, so if there is already a label added to output, there should not another label added
                continue
            else:
                # add index to the output and buffer list maintained to keep track of added labels to output
                current_class.add(class_label)
                final_list.append(i)
        else:
            break

def estimate3dLocation(category_index, i, box, cl, score, disp_img):
    """
    Estimates the 3D attributes of a detected object using the disparity image and bounding box
    """
    # calculating higher and lower limits of bounding box in pixels
    l_y = box[0] * HEIGHT
    l_x = box[1] * WIDTH
    h_y = box[2] * HEIGHT
    h_x = box[3] * WIDTH

    # center of bounding box in pixels
    center_x = int(l_x + (h_x - l_x) / 2)
    center_y = int(l_y + (h_y - l_y) / 2)

    low_x = int(l_x)
    high_x = int(h_x)

    # disparity of center pixel
    min_disp = disp_img[center_y, center_x]

    # calculating the disparity of the pixel which is farthest to the robot in a window of 10 pixels in height and width of full bounding box
    if center_y - 10 > 10:
        low_y = int(center_y - 10)
    else:
        low_y = int(center_y)

    if center_y + 10 < (HEIGHT - 1):
        high_y = int(center_y + 10)
    else:
        high_y = int(center_y)

    if ((category_index.get(cl))).get("name") == "robotAntenna":
        low_y = int(l_y) + 20
        high_y = int(l_y) + 80

    depth_min = 0.01
    disp_max = baseline * Fx / depth_min

    depth_max = 150
    disp_min = baseline * Fx / depth_max

    disp_final = 0.0001
    num_pixels = 0

    # calculates mean disparity in the middle rows of the detected object bounding boxes
    for w in range(low_x, high_x):
        for v in range(low_y, high_y):
            if disp_img[v, w] >= disp_min and disp_img[v, w] <= disp_max:
                disp_final += disp_img[v, w]
                num_pixels += 1

    if num_pixels != 0:
        disp_final /= num_pixels

    # calculating 3d location from the estimated disparity and camera's intrinsic parameters
    center_disparity = disp_final
    depth = baseline * Fx / center_disparity
    x = (center_x - Cx) * depth / Fx
    y = (center_y - Cy) * depth / Fy
    z = depth

    # # for visualization

    # label = (category_index.get(cl)).get('name')

    # if (label == "processingPlant") :
    #     br.sendTransform((x,y,z), tf2.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "processing_plant_position_estimate", g_robot_name + "_left_camera_optical")

    # if (label == "repairStation") :
    #     br.sendTransform((x,y,z), tf2.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "repair_station_position_estimate", g_robot_name + "_left_camera_optical")

    # if (label == "excavator") :
    #     br.sendTransform((x,y,z), tf2.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "small_excavator_1_position_estimate", g_robot_name + "_left_camera_optical")

    # if (label == "hauler") :
    #     br.sendTransform((x,y,z), tf2.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "small_hauler_1_position_estimate", g_robot_name + "_left_camera_optical")

    # calculating the width of the output object
    w_x_1 = (l_x - Cx) * depth / Fx
    w_y_1 = (center_y - Cy) * depth / Fy
    w_z_1 = depth

    w_x_2 = (h_x - Cx) * depth / Fx
    w_y_2 = (center_y - Cy) * depth / Fy
    w_z_2 = depth

    # width of the output object
    w = math.sqrt((w_x_1 - w_x_2) ** 2 + (w_y_1 - w_y_2) ** 2 + (w_z_1 - w_z_2) ** 2)

    result = {
        "score": score,
        "class": cl,
        "x": x,
        "y": y,
        "z": z,
        "w": w,
        "s_x": (h_x - l_x),
        "s_y": (h_y - l_y),
        "c_x": center_x,
        "c_y": center_y,
    }
    return result


def constructObjectMsg(category_index, result, robot_name):
    """
    Construct the geometry_msgs/PoseStamped message for single object
    """
    object_msg = Object()
    object_msg.score = result["score"]
    object_msg.label = (category_index.get(result["class"])).get("name")
    object_msg.point.header.seq = g_seq
    object_msg.point.header.frame_id = robot_name + CAMERA_FRAME_LINK
    object_msg.point.pose.position.x = result["x"]
    object_msg.point.pose.position.y = result["y"]
    object_msg.point.pose.position.z = result["z"]
    object_msg.width = result["w"]
    object_msg.center.x = result["c_x"]
    object_msg.center.y = result["c_y"]
    object_msg.size_x = result["s_x"]
    object_msg.size_y = result["s_y"]
    return object_msg

def overlayInfoImage(result, image_np):
    """
    Overlay the 3D location of the object on the image, the white colored text having x, y, z values
    """
    WHITE = (255, 255, 255)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_size = 0.28
    font_color = WHITE
    font_thickness = 0
    text = "x: {:.3f}, y: {:.3f}, z: {:.3f}".format(
        result["x"], result["y"], result["z"]
    )
    cv2.putText(
        image_np,
        text,
        (result["c_x"], result["c_y"]),
        font,
        font_size,
        font_color,
        font_thickness,
        cv2.LINE_AA,
    )


def detectionAlgorithm(
    category_index, model_fn, image, disparity, img_pub, disp_pub, robot_name
):
    rospy.loginfo_once("Object Detection Algorithm Working")

    # getting buffered global image and disparity from callback
    image_np = image.copy()
    disp_img = disparity.copy()

    # convert images to tensor
    input_tensor = tf.convert_to_tensor(image_np)
    input_tensor = input_tensor[tf.newaxis, ...]

    # run object detection on input image
    output_dict = model_fn(input_tensor)
    rospy.logwarn_once("Started Object Detection Finally")

    # processing object detected data
    num_detections = int(output_dict.pop("num_detections"))
    output_dict = {
        key: value[0, :num_detections].numpy() for key, value in output_dict.items()
    }
    output_dict["num_detections"] = num_detections

    output_dict["detection_classes"] = output_dict["detection_classes"].astype(np.int64)

    # adding visualization of boxes and score on the existing image, published to view on rviz
    vis_util.visualize_boxes_and_labels_on_image_array(
        image_np,
        output_dict["detection_boxes"],
        output_dict["detection_classes"],
        output_dict["detection_scores"],
        category_index,
        use_normalized_coordinates=True,
        line_thickness=2,
        min_score_thresh=CLASS_SCORE_THRESHOLD,
    )

    boxes = output_dict["detection_boxes"]
    scores = output_dict["detection_scores"]
    classes = output_dict["detection_classes"]
    num_detections = output_dict["num_detections"]

    # preprocess the objects
    final_list = []
    preProcessObjectDetection(
        category_index, scores, classes, num_detections, final_list
    )

    # create output perception/Objects message
    objects_msg = ObjectArray()

    global g_seq
    objects_msg.header.seq = g_seq
    g_seq += 1
    objects_msg.header.frame_id = robot_name + CAMERA_FRAME_LINK

    for i in final_list:
        # estimate 3D location of object
        result = estimate3dLocation(
            category_index, i, boxes[i], classes[i], scores[i], disp_img
        )
        overlayInfoImage(result, image_np)
        # add to output message
        objects_msg.obj.append(constructObjectMsg(category_index, result, robot_name))

    objects_msg.number_of_objects = len(objects_msg.obj)
    msg = g_bridge.cv2_to_imgmsg(image_np)
    img_pub.publish(msg)
    disp_pub.publish(objects_msg)
