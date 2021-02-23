# Usage of the package:

## Launch files:

### make_dataset_object_detection.launch

This file launches all the necessary backend services, clients needed for navigation, **teleop.launch is not needed to be launched**, it captures key. 

Argumemts:

robot_name: small_scout_1 or small_excavator_2, small_hauler_2
prefix: initial letters of output file name to identify the user, for example *mb1_* for 1st session of saving images by user Mahimana Bhatt

Normal teleop operation keys
For saving images, please press **SPACE** in the terminal

All the images captured are stored in "utils/dataset/images/". The folder is included in .gitignore, so no rosbag will be pushed to git repository.

Example:

```
roslaunch utils make_dataset_object_detection.launch robot_name:=small_scout_1 prefix:=mb1_
```

[Tutorial for making dataset](https://youtu.be/5SokPAdBn2c)

### teleop.launch

This file launches necessary backend services, client needed for navigation as well as launch "teleop_twist_keyboard.py" from "teleop_twist_keyboard" package.

Arguments:

robot_name: small_scout_1 or small_excavator_2, small_hauler_2

Example:

```
roslaunch utils teleop.launch robot_name:=small_excavator_2

or

if you want to teleop "small_scout_1:
roslaunch utils teleop.launch
```

### rosbag_record.launch

This file records rosbags containing topics needed for specific purpose such as rtabmap. The output file will be stored in "utils/bags/" and the folder is included in .gitignore, so no rosbag will be pushed to git repository

Arguments:

robot_name: string, eg. small_scout_1 or small_excavator_2, small_hauler_2
record_rtabmap: boolean, if you want to record topics related to rtabmap
filename: output_file_name, eg: "test_run_1"

```
roslaunch utils rosbag_record.launch robot_name:=small_scout_1 record_rtabmap:=true file_name:=test_run_1
```


[Tutorial for recording Rosbag | Teleop](https://youtu.be/uRyz2Q1Ur-Q)