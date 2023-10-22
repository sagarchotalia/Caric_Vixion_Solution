// #pragma once
// #include <iostream>
// #include <ros/ros.h>
// #include <visualization_msgs/Marker.h>
// #include <octomap/octomap.h>
// #include <octomap_msgs/Octomap.h>
// #include <octomap_msgs/conversions.h>
// #include <visualization_msgs/MarkerArray.h>

// #include "Eigen/Dense"

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include "Astar.h"

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/octree/octree_search.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/geometry/distance.h>
// #include <trajectory_msgs/MultiDOFJointTrajectory.h>

// #include <pcl/kdtree/kdtree_flann.h>
// #include "utility.h"
// #include <mutex>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <caric_mission/CreatePPComTopic.h>
// #include <std_msgs/String.h>

// #include "general_task_init.h"
// #include "Astar.h"
// #include <map>
// #include "baseline_planner.h"

// struct photo_drone_nav
// {
//     bool is_photo_drone = true;
//     Eigen::Vector3i drone_pos;
//     double priority = 0;
// };

// sensor_msgs::PointCloud bounding_box_scan(sensor_msgs::PointCloud& bbox_vertices)
// {
//     return bbox_vertices;
// }

// void sliding_window(const visualization_msgs::Marker& msg)
// {
//     std::vector<Vector3d> fov_endpoints;
//     fov_endpoints.push_back(msg.points[1]);
//     fov_endpoints.push_back(msg.points[2]);
//     fov_endpoints.push_back(msg.points[4]);
//     fov_endpoints.push_back(msg.points[5]);
//     std::vector<Vector3d> cam_pos_world;
//     cam_pos_world.push_back(msg.points[0]);

//     sensor_msgs::PointCloud bbox_vertices = bounding_box_scan();
//     bool not_aligned = true;
//     bool new_bbox_inspect = false;
//     trajectory_msgs::MultiDOFJointTrajectory position_cmd;
//     // align the top left bounding box vertex with the drone FOV top left vertex
//     // then perform a lawnmower-type manoeuver to scan all interest points
    
//     //put below code in loop
//     if(new_bbox_inspect)
//     {
//         while(abs(bbox_vertices[0] - msg.points[1]) >= 1)
//         {
//             //create position message to send drone to coordinates of bbox's top leftmost vertex
//             //for now, we will assume that the drone has to always be upright
//             position_cmd = mainbrain::position_msg_build(drone_global_position, bbox_target_vertex, bbox_target_angle_rpy.z());
//             motion_pub.publish(position_cmd);
            
//             //if time elapsed = 2 seconds at the bbox vertex, move on downwards
//         }

//         //create position message to move drone downwards
        
//         mainbrain::update_position();
//         downwards_position_command = mainbrain::position_msg_build(drone_global_position, bbox)



//     }

// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "photo_drone_planner");
//     ros::NodeHandle nh;
//     ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

//     ros::Subscriber sub1 = nh.subscribe("/sentosa/visualize", 1000, sliding_window);
//     ros::Subscriber sub2 = nh.subscribe("/gcs/bounding_box_vertices", 1000, bounding_box_scan);
//     ros::Publisher motion_pub = nh_ptr->advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);

//     // Agent a(nh_ptr);
//     ros::MultiThreadedSpinner spinner(0);

//     spinner.spin();
//     return 0;
// }     