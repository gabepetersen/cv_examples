#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

class SobelConverter
{
	ros::NodeHandle nh_;
	ros::Publisher mi_;
 	image_transport::ImageTransport it_;
  	image_transport::Subscriber image_sub_;
  	image_transport::Publisher image_pub_;
  

	public:
		void imageCb(const sensor_msgs::ImageConstPtr& msg);

  		SobelConverter() : it_(nh_) {

			// declare a subscription to topic: camera/image_raw
    		image_sub_ = it_.subscribe("/camera/image_raw", 1, &SobelConverter::imageCb, this);

			// declare where node publishes the end message
    		image_pub_ = it_.advertise("/sobel_converter/output_video", 1);
			mi_ = nh_.advertise<std_msgs::Float32>("/sobel_converter/mean_intensity", 100);

			// create 2 new cv windows
			cv::namedWindow("Original Image");
			cv::namedWindow("Sobel Image");
  		}
  
  		// destructor of window
  		~SobelConverter() {
    		cv::destroyWindow("Original Image");
			cv::destroyWindow("Sobel Image");
  		}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sobel_converter");
  SobelConverter ic;
  ros::spin();
  return 0;
}


void SobelConverter::imageCb(const sensor_msgs::ImageConstPtr& msg) {

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

	/*****************************************************************************/
	/******* Calculate the Mean Intensity with cv::mean() and Publish It *********/
	/*****************************************************************************/

	cv::Scalar tempInt = mean(cv_ptr->image);
	double meanI = tempInt.val[0];
	std_msgs::Float32 meanIntensity;
	meanIntensity.data = meanI;

	/*****************************************************************************/
	/*********** Apply SOBEL filter, bring up window, and publish it *************/
	/*****************************************************************************/

	// declare all the CV image vars needed as well as ddepth of resultant sobel image
	cv::Mat sobel8U, sobel8U_x, sobel8U_y, gradx, grady, blurImg, srcImg = cv_ptr->image;

	// remove any noise of the image with a gaussian blur with kernel size 3
	cv::GaussianBlur(srcImg, blurImg, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);

	// ddepth = CV_16S(3), x_order derivative = 1, y_order derivative = 0, 
    // kernel size = 3, scale = 1, delta = 0, 
	cv::Sobel(blurImg, gradx, 3, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);

	// ddepth = CV_16S(3), x_order derivative = 0, y_order derivative = 1, 
	// kernel size = 3, scale = 1, delta = 0
	cv::Sobel(blurImg, grady, 3, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);

	// convert the image to CV_8U depth
	convertScaleAbs(gradx, sobel8U_x);
	convertScaleAbs(grady, sobel8U_y);

	// combine the gradients into one image
	addWeighted(sobel8U_x, 0.5, sobel8U_y, 0.5, 0, sobel8U);

	// convert to cv_bridge pointer
	cv_ptr->image = sobel8U;

	/*****************************************************************************/
	/******************** Update GUIs and Publish Messages **********************/
	/*****************************************************************************/

    // Update GUI Windows
    cv::imshow("Original Image", srcImg);
	cv::imshow("Sobel Image", cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream as a ROS sensor_msg/Image
    image_pub_.publish(cv_ptr->toImageMsg());
	mi_.publish(meanIntensity);
}
