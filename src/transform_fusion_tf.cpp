// #include <ros/ros.h>
// #include <iostream>
// #include <nav_msgs/Odometry.h>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <Eigen/Dense>


#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <thread>
#include <chrono>

nav_msgs::Odometry cur_odom_to_baselink;
nav_msgs::Odometry cur_map_to_odom;


void pose_to_mat(const geometry_msgs::Pose& pose_msg, tf2::Transform& transform) {
    tf2::Transform translation, rotation;
    tf2::fromMsg(pose_msg.position, translation.getOrigin());
    tf2::fromMsg(pose_msg.orientation, rotation);
    transform = translation * rotation;
}


void transform_fusion(ros::Publisher& pub_localization, double FREQ_PUB_LOCALIZATION) {
    static tf2_ros::TransformBroadcaster br;

    ros::Rate rate(FREQ_PUB_LOCALIZATION);
    while (ros::ok()) {
        rate.sleep();

        nav_msgs::Odometry cur_odom = cur_odom_to_baselink;
        tf2::Transform T_map_to_odom;
        pose_to_mat(cur_map_to_odom.pose.pose, T_map_to_odom);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "camera_init";
        transformStamped.transform = tf2::toMsg(T_map_to_odom);
        br.sendTransform(transformStamped);

        if (cur_odom.header.stamp != ros::Time(0)) {
            // 发布全局定位的odometry
            nav_msgs::Odometry localization;
            tf2::Transform T_odom_to_base_link;
            pose_to_mat(cur_odom.pose.pose, T_odom_to_base_link);
            tf2::Transform T_map_to_base_link = T_map_to_odom * T_odom_to_base_link;

            localization.pose.pose.position = tf2::toMsg(T_map_to_base_link.getOrigin());
            localization.pose.pose.orientation = tf2::toMsg(T_map_to_base_link.getRotation());
            // localization.pose.pose = tf2::toMsg(T_map_to_base_link);
            localization.twist = cur_odom.twist;

            localization.header.stamp = cur_odom.header.stamp;
            localization.header.frame_id = "map";
            localization.child_frame_id = "body";
            pub_localization.publish(localization);
        }
    }
}

void cb_save_cur_odom(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    cur_odom_to_baselink = *odom_msg;
}

void cb_save_map_to_odom(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    cur_map_to_odom = *odom_msg;
}

int main(int argc, char** argv) {
    // tf and localization publishing frequency (HZ)
    double FREQ_PUB_LOCALIZATION = 200;

    ros::init(argc, argv, "transform_fusion");
    ros::NodeHandle nh;
    ROS_INFO("Transform Fusion Node Inited...");

    ros::Subscriber sub_cur_odom = nh.subscribe("/Odometry_imu", 1000, cb_save_cur_odom);
    ros::Subscriber sub_map_to_odom = nh.subscribe("/map_to_odom", 1000, cb_save_map_to_odom);

    ros::Publisher pub_localization = nh.advertise<nav_msgs::Odometry>("/localization", 1000);

    // 发布定位消息
    std::thread fusion_thread(transform_fusion, std::ref(pub_localization), FREQ_PUB_LOCALIZATION);

    ros::spin();
    fusion_thread.join();

    return 0;
}
