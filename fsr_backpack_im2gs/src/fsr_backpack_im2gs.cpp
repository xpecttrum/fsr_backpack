 #include <ros/ros.h>
 #include <image_transport/image_transport.h>
 #include <opencv/cv.h>
 #include <opencv/highgui.h>
 
 
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
 
using namespace cv_bridge;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

Mat im_rgb;
//Mat im_gray;
Mat im_gray(Size(640, 480), CV_8UC1);
Mat im_gray2(Size(752, 480), CV_8UC1);

int i_sequence = 0;

image_transport::Publisher im_pub;

 void imageCallback(const sensor_msgs::ImageConstPtr& msg)
 {
	//  fprintf(stderr,"\nimageCallback...");
    
   i_sequence++;
   
   cv_bridge::CvImagePtr cv_ptr;
   //sensor_msgs::CvBridge bridge;
   try
   {
       cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
   }


	//convert to gray scale
	im_rgb = cv_ptr->image;
	cvtColor(im_rgb,im_gray,CV_RGB2GRAY);

	//resise to match svo requiremets 752x480
	cv::resize(im_gray, im_gray2, im_gray2.size());
   // Process cv_ptr->image using OpenCV
       cv::imshow("view", im_gray2);
       cv::waitKey(3);
	
    //fprintf(stderr,"%d",im_gray.type());

   //Prepare the message to be published "gray image"
    cv_bridge::CvImage out_msg;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;      //BGR8; 
   
    out_msg.image    = im_gray2;
    out_msg.header.seq = i_sequence;
    out_msg.header.frame_id = i_sequence;
    out_msg.header.stamp = ros::Time::now();
            
    
    im_pub.publish(out_msg.toImageMsg());
       
   
  
 }
 
 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "image_to_grayscale");
   ros::NodeHandle nh;
   
   
   cvNamedWindow("view");
   cvStartWindowThread();
   image_transport::ImageTransport it(nh);
   image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
   //image_transport::Subscriber sub = it.subscribe("camera/rgb/image_color", 1, imageCallback);
   
   im_pub = it.advertise("camera/image_raw", 1);
   
   ros::spin();
   cvDestroyWindow("view");
 }
