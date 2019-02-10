#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

class SobelGet
{
	ros::NodeHandle nh_;
	ros::Subscriber mi_;
 	image_transport::ImageTransport it_;
  	image_transport::Subscriber sobel_sub_;
  

	public:
		void img_cb(const sensor_msgs::ImageConstPtr& msg);

  		SobelGet() : it_(nh_) {

			// declare a subscription to topic: camera/image_raw
    		sobel_sub_ = it_.subscribe("/sobel_converter/output_video", 1, &SobelGet::img_cb, this);

			// create a new cv window
			cv::namedWindow("Sobel Image");
  		}
  
  		// destructor of window
  		~SobelGet() {
			cv::destroyWindow("Sobel Image");
  		}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sobel_get");
  SobelGet ic;
  ros::spin();
  return 0;
}


void SobelGet::img_cb(const sensor_msgs::ImageConstPtr& msg) {
	// create a CVImagePtr so data can get copied in
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
		// convert sensor_msgs/Image to a MONO8 greyscale CV img encoding
    	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
    	ROS_ERROR("cv_bridge exception: %s", e.what());
    	return;
    }
	cv::imshow("Sobel Image", cv_ptr->image);
}
