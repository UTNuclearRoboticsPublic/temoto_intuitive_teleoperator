#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ~~~~~~~ PARAMETERS ~~~~~~~~~~~~
double loop_rate = 20; //hz
std::string ee_fixed_frame_name = "right_gripper_toolpoint";
std::string ee_world_up_frame_name = "ee_world_up";
std::string base_link_name = "base_link";
std::string ur_ee_link_name = "right_ur5_2_finger_140_gripper_link";
double ur5_gripper_offset = 0.2216; // meters

// ~~~~~~~~ MAIN ~~~~~~~~~~~~~

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_frame_broadcaster");
    ros::NodeHandle n;
    ros::Rate node_rate(loop_rate);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster tfBroadcaster;

    geometry_msgs::TransformStamped base_gripper_tf_msg; // From the tf listener
    geometry_msgs::TransformStamped gripper_toolpoint_tf_msg, toolpoint_up_tf_msg; // We will publish these

    // Set up the frame organization for our published frames
    gripper_toolpoint_tf_msg.header.frame_id = ur_ee_link_name;
    gripper_toolpoint_tf_msg.child_frame_id = ee_fixed_frame_name;
    gripper_toolpoint_tf_msg.transform.translation.x = ur5_gripper_offset;
    gripper_toolpoint_tf_msg.transform.rotation.w = 1;

    toolpoint_up_tf_msg.header.frame_id = base_link_name;
    toolpoint_up_tf_msg.child_frame_id = ee_world_up_frame_name;

    // Setup some variables used in the loop
    double yaw;
    tf2::Quaternion rot_quat_tf;
    geometry_msgs::Vector3Stamped ee_x_axis, ee_x_axis_in_base_frame;
    ee_x_axis.header.frame_id = ee_fixed_frame_name;
    ee_x_axis.vector.x = 1;

    while(ros::ok())
    {
        // Publish the gripper -> toolpoint transform
        gripper_toolpoint_tf_msg.header.stamp = ros::Time::now();
        tfBroadcaster.sendTransform(gripper_toolpoint_tf_msg);

        // Get the base -> gripper transform
        try
        {
        base_gripper_tf_msg = tfBuffer.lookupTransform(base_link_name, ee_fixed_frame_name, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
        ROS_WARN_THROTTLE(1, "%s",ex.what());
        continue;
        }

        // Calculate the base -> ee_world_up yaw rotation
        tf2::doTransform(ee_x_axis, ee_x_axis_in_base_frame, base_gripper_tf_msg);
        yaw = atan2(ee_x_axis_in_base_frame.vector.y, ee_x_axis_in_base_frame.vector.x);
        rot_quat_tf.setRPY( 0, 0, yaw );

        // Populate the transform msg and publish
        toolpoint_up_tf_msg.transform.translation = base_gripper_tf_msg.transform.translation;
        toolpoint_up_tf_msg.transform.rotation.x = rot_quat_tf.x();
        toolpoint_up_tf_msg.transform.rotation.y = rot_quat_tf.y();
        toolpoint_up_tf_msg.transform.rotation.z = rot_quat_tf.z();
        toolpoint_up_tf_msg.transform.rotation.w = rot_quat_tf.w();

        toolpoint_up_tf_msg.header.stamp = ros::Time::now();
        tfBroadcaster.sendTransform(toolpoint_up_tf_msg);

        node_rate.sleep();
    }

  return 0;
}