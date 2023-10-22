// #pragma once
// #include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
// #include <octomap/octomap.h>
// #include <octomap_msgs/Octomap.h>
// #include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include "Eigen/Dense"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include "Astar.h"

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/octree/octree_search.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/geometry/distance.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

// #include <pcl/kdtree/kdtree_flann.h>
// #include "utility.h"
// #include <mutex>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <caric_mission/CreatePPComTopic.h>
#include <std_msgs/String.h>

// #include "general_task_init.h"
// #include "Astar.h"
#include <map>
#include "baseline_planner.h"
// #include "rotors_control"


visualization_msgs::Marker fov_limits;
sensor_msgs::PointCloud bbox_vertices;
bool new_bbox_traverse = false;
Eigen::Vector3d drone_pos_global;
Eigen::Matrix3d drone_rotation_matrix;
Eigen::Vector3d bbox_target_angle_rpy = 0;

void update_position(Eigen::Vector3d point, Eigen::Matrix3d rotation)
{
    // if (!odom_get)
    // {
    //     initial_position = point;
    //     initial_position = initial_position + Eigen::Vector3d(0, 0, 3);
    // }
    drone_rotation_matrix = rotation;
    drone_pos_global = point;
    // if (!finish_init)
    // {
    //     return;
    // }

    // if (map_set.size() > now_id)
    // {
    //     global_map.update_position(point);
    //     map_set[now_id].update_position(point);
    // }
    // else
    // {
    // update_position(point);
    // }
    // odom_get = true;
}

// void update_position(Eigen::Vector3d point)
// {
//     Eigen::Vector3d point_local = rotation_matrix * (point - map_global_center);
//     if (out_of_range(point_local, false))
//     {
//         in_my_range = false;
//         return;
//     }
//     Eigen::Vector3i index = get_index(point);
//     if (now_position_index != index && visited_map[index.x()][index.y()][index.z()] == 0 && search_direction.empty())
//     {
//         search_direction = get_search_target(index);
//         time_start=ros::Time::now().toSec();
//     }
//     if (search_direction.empty())
//     {
//         visited_map[index.x()][index.y()][index.z()] = 1;
//     }
//     if(fabs(ros::Time::now().toSec()-time_start)>3)
//     {
//         visited_map[index.x()][index.y()][index.z()] = 1;
//     }
//     now_position_global = point;
//     now_position_index = index;
//     now_position_local = point_local;
//     in_my_range = true;
// }



visualization_msgs::Marker fovLimitsCallback(visualization_msgs::Marker& fov_lim)
{
    fov_limits = fov_lim;
}

sensor_msgs::PointCloud bboxVerticesCallback(sensor_msgs::PointCloud& bbox_vert)
{
    bbox_vertices = bbox_vert;
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    Eigen::Vector3d my_position = Eigen::Vector3d(msg->pose.pose.position.X, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Matrix3d R = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
    update_position(my_position, R);
}

//position message building taken from caric_baseline
trajectory_msgs::MultiDOFJointTrajectory position_msg_build(Eigen::Vector3d position, Eigen::Vector3d target, double target_yaw)
{
    if (fabs(target_yaw) < M_PI / 2)
        {
            target_yaw = 0;
        }
        trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
        trajset_msg.header.frame_id = "world";
        geometry_msgs::Transform transform_msg;
        geometry_msgs::Twist accel_msg, vel_msg;

        Eigen::Vector3d difference = (target - position);
        if (difference.norm() < 2)
        {
            transform_msg.translation.x = target.x();
            transform_msg.translation.y = target.y();
            transform_msg.translation.z = target.z();
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;
        }
        else
        {
            Eigen::Vector3d target_pos = 2 * difference / difference.norm();
            transform_msg.translation.x = 0;
            transform_msg.translation.y = 0;
            transform_msg.translation.z = 0;
            vel_msg.linear.x = target_pos.x();
            vel_msg.linear.y = target_pos.y();
            vel_msg.linear.z = target_pos.z();
        }
        transform_msg.rotation.x = 0;
        transform_msg.rotation.y = 0;
        transform_msg.rotation.z = sinf(target_yaw * 0.5);
        transform_msg.rotation.w = cosf(target_yaw * 0.5);

        trajpt_msg.transforms.push_back(transform_msg);

        accel_msg.linear.x = 0;
        accel_msg.linear.y = 0;
        accel_msg.linear.z = 0;

        trajpt_msg.velocities.push_back(vel_msg);
        trajpt_msg.accelerations.push_back(accel_msg);
        trajset_msg.points.push_back(trajpt_msg);

        trajset_msg.header.frame_id = "world";
        return trajset_msg;    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "photo_drone_planner");
    ros::NodeHandle nh;
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);
    ros::Subscriber fov_sub = nh_ptr->subscribe("/sentosa/visualize", 1000, fovLimitsCallback);
    ros::Subscriber bbox_sub = nh_ptr->subscribe("/gcs/bounding_box_vertices", 1000, bboxVerticesCallback);
    ros::Subscriber odom_sub = nh_ptr->subscribe("/sentosa/ground_truth/odometry", 1000, odomCallback);
    ros::Publisher motion_pub = nh_ptr->advertise<trajectory_msgs::MultiDOFJointTrajectory>("/sentosa/command/trajectory", 1);
    
    std::vector<Vector3d> fov_endpoints;
    trajectory_msgs::MultiDOFJointTrajectory position_msg;
    geometry_msgs::Twist gimbal_msg;

    fov_endpoints.push_back(fov_limits.points[1]);
    fov_endpoints.push_back(fov_limits.points[2]);
    fov_endpoints.push_back(fov_limits.points[4]);
    fov_endpoints.push_back(fov_limits.points[5]);
    std::vector<Vector3d> cam_pos_world;
    cam_pos_world.push_back(fov_limits.points[0]);

    if(new_bbox_traverse)
    {
        //align drone with the top and leftmost coordinate of the bounding box
        ROS_INFO("%s, %s", bbox_vertices.points[0],fov_limits.pose.position.z[1]);
        if(abs(bbox_vertices.points[0] - fov_limits.pose.position.z[1]) >= 1)
        {
            //create position message to send drone to coordinates of bbox's top leftmost vertex
            position_msg = position_msg_build(drone_pos_global, bbox_vertices.points[0], bbox_target_angle_rpy.z());
            //if time elapsed = 2 seconds at the bbox vertex, move on downwards
        }
    }
    else
    {
        //go downwards
        position_msg = position_msg_build(drone_pos_global, bbox_vertices.points[1], bbox_target_angle_rpy.z());

    }

    position_msg.header.stamp = ros::Time::now();
    motion_pub.publish(position_msg);
    // drone_cmd_raf.publish(trajectory_msg_raf);
    ros::spin();
    // ros::MultiThreadedSpinner spinner(0);
    // spinner.spin();
    return 0;
}




