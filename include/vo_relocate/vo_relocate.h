/*
 * brief: come on
 * author: Created by bpeer on 18-4-16.
 */

#ifndef VO_RELOCATE_VO_RELOCATE_H
#define VO_RELOCATE_VO_RELOCATE_H

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <map>
#include <sys/time.h>
#include <chrono>
#include <boost/timer.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <DBoW3/DBoW3.h>

#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

using namespace cv;

struct pose2D
{
	float x;
	float y;
	float th;
};

struct result_reloc
{
	pose2D odo_locate;
	std::map<int, pose2D> map_result;
};

class VoRelocate
{
public:
	VoRelocate();
	VoRelocate(const int i){std::cout<< "get out here.. " << std::endl;};
	void load_file( const std::string &filename );
	void locateCb( Mat& image, pose2D odom );
	pose2D baselink2odom_;
	void spinCb( const ros::TimerEvent& e);

	std::string voCb(nav_msgs::OdometryConstPtr& odom);

	void reset();

	std::string result_;
	DBoW3::Database db;

	~VoRelocate();

private:
	bool relocate();

	std::map<int, pose2D> map_pose;  //robot pose in img for reloc

	int index;
	pose2D locate;
	pose2D ref_locate;

	float common_threa;
	float motion_thre;
	bool new_flag;
	bool relocate_success;
	float err_linear;
	float err_angular;

	Mat img;
	Mat ref_img;

	result_reloc result;
	std::vector< result_reloc > results;

	pose2D imageLocate;  //cur pos2D 对应的重定位中筛选图像所带的位姿

	int test_num;
};

#endif //VO_RELOCATE_VO_RELOCATE_H
