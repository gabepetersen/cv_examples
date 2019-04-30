# cv_examples
Node demonstrating Sobel filtering

------------------------------------------------------------------------
# Implementation 
------------------------------------------------------------------------

### Instructions on how to run:

1. Make sure you have a webcam, a linux webcam driver, and opencv installed

2. Install the package "video_stream_opencv"

3. ```catkin build``` 

2. ```roslaunch cv_examples sobel_convert.launch```

3. You should then see a camera stream of your webcam and then another stream with a simple sobel filter applied

4. If you do not, your camera driver may not be working or you may have to edit the subscription inside of cv_examples/src/sobel_converter.cpp from "/camera/image_raw" to another topic that your video_stream_opencv node is publishing to. 

------------------------------------------------------------------------
# Changing Things
------------------------------------------------------------------------

### To change the sobel filter
Go to sobel_converter.cpp and alter the settings of the opencv functions under "Apply SOBEL filter, bring up window, and publish it"

### To not show the image stream
Go to sobel_converter.cpp and sobel_get.cpp and comment out the cv::imshow(...) functions near the end of the code
