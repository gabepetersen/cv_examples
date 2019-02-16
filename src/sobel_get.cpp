#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

void mi_cb(const std_msgs::Float32::ConstPtr& msg);
void img_cb(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char** argv) {

  	// initialize the node
	ros::init(argc, argv, "sobel_get");
  	ros::NodeHandle n;
  
  	// Create the CV Windows
  	cv::namedWindow("Sobel Image");
  	cv::startWindowThread();

	// initilize the subscribers for mean_intensity and Sobel Image
  	image_transport::ImageTransport it(n);
  	image_transport::Subscriber sobel_sub = it.subscribe("/sobel_converter/output_video", 1, img_cb);
  	ros::Subscriber mi = n.subscribe("/sobel_converter/mean_intensity", 100, mi_cb);
  
  	ros::spin();
  	cv::destroyWindow("Sobel Image");
  	return 0;
}

// Callback function for displaying the mean intensity of the Image
void mi_cb(const std_msgs::Float32::ConstPtr& msg) {
	ROS_INFO("Mean Intensity = %f", msg->data);
}

// Callback function for displaying Sobel image
void img_cb(const sensor_msgs::ImageConstPtr& msg) {
    try
    {
		// toCvShare is used since data doesn't need to be copied
		cv::imshow("Sobel Image", cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image);
		cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
    	ROS_ERROR("cv_bridge exception: %s", e.what());
    	return;
    }
}
