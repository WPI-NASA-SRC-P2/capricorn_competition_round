# Usage of package

## Launch Files:

### object_detection.launch

Launches the object detection node as well as the stereo processing node for generating disparity. Keep in mind **RTABMAP also launches this node**, so just use **object_detection_cap** directly in that case

Argumemts:

robot_name: small_scout_1 or small_excavator_2, small_hauler_2, by default = **small_scout_1**

rviz: If you want to launch rviz to display the image topic published by object detection node, by default = **True**

path_to_model: If you want to use your own model (not need to pass, it will take by default path), by default = **$(find perception)/model**

path_to_labelmap: If you want to use your own label map (not need to pass, it will take by default path), by default = **$(find perception)/model/saved_model.pbtxt**

launch_stereo_proc: If you want launch the stereo processing node for disparity, by default = **True**

Example:

```
roslaunch perception object_detection.launch 
```

## Scripts

### Find Processing Plant and Repair Station (find_pp_rs):

Example:

`rosrun perception find_pp_rs small_scout_1`

The script will move the rover's camera 360 degrees in order to find the PP and RS, once anyone of them are find, it will orient the camera of the rover such that PP and RS are exactly at the center of the image, in condition when only once is available from PP and RS, it will orient the camera using that object as center. The script will exit after orienting the camera.

[Isage Video](https://youtu.be/7D91GapPlfg)

### Object Detection (object_detection_cap):

Example:

`rosrun perception object_detection_cap small_scout_1 model_path labelmap_path`

Uses the model from path specified to detect objects in the images received from image topics of the rover specified.

