/*
 * visual_odometry.cpp
 *
 *  Created on: 2013-01-16
 *      Author: mathieu
 */

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include <std_srvs/Empty.h>

#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>

#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <utilite/UTimer.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "util.h"

class VisualOdometry
{
public:
	VisualOdometry() :
		frameId_("base_link"),
		image_mono_sub_(nh_, "image", 1),
		image_depth_sub_(nh_, "image_depth", 1),
		info_sub_(nh_, "camera_info", 1),
		sync_(MySyncPolicy(5), image_mono_sub_, image_depth_sub_, info_sub_)
	{
		pose_.setIdentity();

		sync_.registerCallback(boost::bind(&VisualOdometry::callback, this, _1, _2, _3));
		odomPub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);

		ros::NodeHandle pnh("~");
		pnh.param("frame_id", frameId_, frameId_);

		resetSrv_ = pnh.advertiseService("reset", &VisualOdometry::reset, this);
	}

	~VisualOdometry()
	{
	}

	void callback(const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& imageDepth,
			const sensor_msgs::CameraInfoConstPtr& camInfo)
	{
		if(!(image->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 image->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 image->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) &&
			 imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)!=0)
		{
			ROS_ERROR("Input type must be image=mono8,rgb8,bgr8 and image_depth=32FC1");
			return;
		}

		cv_bridge::CvImageConstPtr imgPtr = cv_bridge::toCvShare(image);
		cv::Mat imageMono2;
		// convert to grayscale
		if(image->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0)
		{
			cv::cvtColor(imgPtr->image, imageMono2, cv::COLOR_BGR2GRAY);
		}
		else if(image->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0)
		{
			cv::cvtColor(imgPtr->image, imageMono2, cv::COLOR_RGB2GRAY);
		}
		else
		{
			imageMono2 = imgPtr->image.clone();
		}
		cv::Mat imageDepth2 = cv_bridge::toCvShare(imageDepth)->image.clone();
		if(!lastImages_.first.empty())
		{
			cv::Mat imageMono1 = lastImages_.first;
			cv::Mat imageDepth1 = lastImages_.second;

			if( imageMono1.cols != imageMono2.cols || imageMono1.rows != imageMono2.rows ||
				imageDepth1.cols != imageDepth2.cols || imageDepth1.rows != imageDepth2.rows)
			{
				lastImages_.first = imageMono2;
				lastImages_.second = imageDepth2;
				ROS_ERROR("clouds must be the same size!\n");
				return;
			}

			ros::WallTime time = ros::WallTime::now();

			// Find 2d correspondences
			std::list<std::pair<cv::Point2f, cv::Point2f> > correspondences = findCorrespondences(imageMono1, imageMono2);

			// Extract XYZ point clouds from correspondences
			pcl::PointCloud<pcl::PointXYZ>::Ptr inliers1(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr inliers2(new pcl::PointCloud<pcl::PointXYZ>);
			extractXYZCorrespondences(
			    correspondences,
				imageDepth1,
				imageDepth2,
				*camInfo,
				*inliers1,
				*inliers2);

			if(correspondences.size() < 15)
			{
				ROS_WARN("Low correspondences -> %d", correspondences.size());
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr inliersTransformed1;
			pcl::PointCloud<pcl::PointXYZ>::Ptr inliersTransformed2;
			if(frameId_.size())
			{
				inliers1->header.frame_id = image->header.frame_id;
				inliers2->header.frame_id = image->header.frame_id;
				inliersTransformed1.reset(new pcl::PointCloud<pcl::PointXYZ>);
				inliersTransformed2.reset(new pcl::PointCloud<pcl::PointXYZ>);
				pcl_ros::transformPointCloud(frameId_, *inliers1, *inliersTransformed1, tfListener_);
				pcl_ros::transformPointCloud(frameId_, *inliers2, *inliersTransformed2, tfListener_);
			}
			else
			{
				inliersTransformed1 = inliers1;
				inliersTransformed2 = inliers2;
			}

			// Compute transform
			tf::Transform transform = transformFromXYZCorrespondences(inliersTransformed1, inliersTransformed2);

			if(std::isfinite(transform.getOrigin().x()))
			{
				// from optical frame to world frame
				tf::Transform motion = transform;

				//Update pose
				pose_ *= motion;

				double roll, pitch, yaw;
				pose_.getBasis().getEulerYPR(yaw,pitch,roll);
				ROS_INFO("Odom xyz(%f,%f,%f) rpy(%f,%f,%f) update time(%f s)",
						pose_.getOrigin().x(),
						pose_.getOrigin().y(),
						pose_.getOrigin().z(),
						roll,
						pitch,
						yaw,
						(ros::WallTime::now()-time).toSec());
			}
			else
			{
				ROS_WARN("transform nan");
			}
		}
		lastImages_.first = imageMono2;
		lastImages_.second = imageDepth2;

		//*********************
		// Publish odometry
		//*********************
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped trans;
		trans.header.stamp = image->header.stamp;
		trans.header.frame_id = "odom";
		trans.child_frame_id = frameId_;
		tf::transformTFToMsg(pose_, trans.transform);

		//send the transform
		tfBroadcaster_.sendTransform(trans);
		if(odomPub_.getNumSubscribers())
		{
			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = image->header.stamp; // use corresponding time stamp to cloud
			odom.header.frame_id = "odom";

			//set the position
			odom.pose.pose.position.x = pose_.getOrigin().x();
			odom.pose.pose.position.y = pose_.getOrigin().y();
			odom.pose.pose.position.z = pose_.getOrigin().z();
			tf::quaternionTFToMsg(pose_.getRotation(), odom.pose.pose.orientation);

			//publish the message
			odomPub_.publish(odom);
		}
	}

	bool reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		//TODO reset odom
		pose_.setIdentity();
		return true;
	}

private:
	ros::NodeHandle nh_;
	std::string frameId_;

	ros::Publisher odomPub_;
	ros::ServiceServer resetSrv_;
	tf::TransformBroadcaster tfBroadcaster_;
	tf::TransformListener tfListener_;


	message_filters::Subscriber<sensor_msgs::Image> image_mono_sub_;
	message_filters::Subscriber<sensor_msgs::Image> image_depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync_;

	std::pair<cv::Mat, cv::Mat> lastImages_; // <image_mono, image_depth>
	tf::Pose pose_;
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "visual_odometry");
	VisualOdometry vOdom;
	ros::spin();
	return 0;
}
