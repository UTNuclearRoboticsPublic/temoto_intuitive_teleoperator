#include "temoto/preplanned_sequences/robot_please_scan.h"

robot_please_scan::robot_please_scan()
{
	/*
	ros::NodeHandle nh;
	ros::ServiceClient scanning_client = nh.serviceClient<laser_stitcher::stationary_scan>("laser_stitcher/stationary_scan");
	ros::ServiceClient client = nh.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_service");
	ros::Publisher final_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_stitcher/final_cloud", 1);

	std::string postprocessing_file_name;
	nh.param<std::string>("laser_stitcher/postprocessing_file_name", postprocessing_file_name, "handle_turning_demo_postprocess");

	laser_stitcher::stationary_scan scan_srv;
	float temp_angle;
	nh.param<float>("lidar_ur5_manager/min_angle", temp_angle, -1.57);
	scan_srv.request.min_angle = temp_angle;
	nh.param<float>("lidar_ur5_manager/max_angle", temp_angle, 1.57);
	scan_srv.request.max_angle = temp_angle;
	scan_srv.request.external_angle_sensing = false;

	ros::Duration(2.0).sleep();

	ROS_INFO_STREAM("[LaserStitcherClient] Attempting first scan.");
	while( ros::ok() )
	{
		
		if( ! scanning_client.call(scan_srv) )
			ROS_ERROR_STREAM("[LaserStitcherClient] Scanning service call failed - prob not up yet");
		else
			ROS_ERROR_STREAM("[LaserStitcherClient] Successfully called scanning service");
		ros::Duration(0.2).sleep();

		break;
	}

	pointcloud_processing_server::pointcloud_process postprocess;
	postprocess.request.pointcloud = scan_srv.response.output_cloud;
	PointcloudTaskCreation::processFromYAML(&postprocess, postprocessing_file_name, "pointcloud_process");

	while(!client.call(postprocess))
	{
		ROS_ERROR("[LaserStitcherClient] Postprocessing call failed - trying again...");
		ros::Duration(1.0).sleep();
	}

	int postprocess_length = postprocess.request.tasks.size();
	sensor_msgs::PointCloud2 final_cloud = postprocess.response.task_results[postprocess_length-1].task_pointcloud;
	ROS_INFO_STREAM("[LaserStitcherClient] Publishing final cloud! Size: " << final_cloud.height*final_cloud.width);
	final_cloud_pub.publish(final_cloud);

	ros::Duration(1.0).sleep();
*/
}
