#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	namespace enc = sensor_msgs::image_encodings;
        cv_bridge::CvImageConstPtr cv_ptr;
	try {
		if (enc::isColor(msg->encoding)) {
			cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
		} else {
			cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
		}
		cv::Mat(cv_ptr->image);
		float dist_val = cv_ptr->image.at<float>( 10, 10 );
		ROS_INFO("%f", dist_val);
        }
        catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
}

int main(int argc, char **argv) {
        ros::init(argc, argv, "simple_depth_image_listener");
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("camera/depth/image_rect", 1, imageCallback);
	//image_transport::Subscriber sub = it.subscribe("image_converter/output_video", 1, imageCallback);
	//image_transport::Subscriber sub = it.subscribe("camera/depth/image_rect", 1, imageCallback);
        ros::spin();
}
