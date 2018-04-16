#include "vo_relocate/vo_relocate.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "vo_relocate_node" );

	// 重定位 直接声明， 不停的调用voCb即可；
	VoRelocate voRelocate;
	nav_msgs::OdometryConstPtr odom;

	std::string vo_res = voRelocate.voCb( odom );  //这里面除了odom的结构获取x y th外，没有用到ros相关。。

	ros::spin();
	return 0;
}