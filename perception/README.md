# Usage of the scripts in the package

Find Processing Plant and Repair Station:

`rosrun perception find_pp_rs small_scout_1`

The script will move the rover's camera 360 degrees in order to find the PP and RS, once anyone of them are find, it will orient the camera of the rover such that PP 
and RS are exactly at the center of the image, in condition when only once is available from PP and RS, it will orient the camera using that object as center.
