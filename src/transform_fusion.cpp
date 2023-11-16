#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <thread>
#include <iostream>

nav_msgs::Odometry cur_odom_to_baselink;
nav_msgs::Odometry cur_map_to_odom;
bool has_transform = false;

ros::Publisher pub_localization;

void convertOdometryToEigenMatrix(const nav_msgs::Odometry &odometry, Eigen::Matrix4d &matrix)
{
  ROS_INFO("enter odom to eigen");

  // 获取 Odometry 消息中的位置和方向
  const auto &position = odometry.pose.pose.position;
  const auto &orientation = odometry.pose.pose.orientation;

  // 填充矩阵
  matrix << 1, 0, 0, position.x,
      0, 1, 0, position.y,
      0, 0, 1, position.z,
      0, 0, 0, 1;

  // 获取方向的四元数
  Eigen::Quaterniond quat(orientation.w, orientation.x, orientation.y, orientation.z);

  // 将四元数转换为旋转矩阵
  Eigen::Matrix3d rotationMatrix = quat.normalized().toRotationMatrix();

  // 将旋转矩阵插入矩阵的左上角
  matrix.block<3, 3>(0, 0) = rotationMatrix;

  std::cout << "mat1:\n"
            << matrix << std::endl;
}

void convertMatrixToOdometry(const Eigen::Matrix4d &matrix, nav_msgs::Odometry &odometry)
{
  ROS_INFO("enter eigen to odom");

  // 提取矩阵中的平移向量
  Eigen::Vector3d translation = matrix.block<3, 1>(0, 3);

  // 提取矩阵中的旋转矩阵
  Eigen::Matrix3d rotationMatrix = matrix.block<3, 3>(0, 0);

  // 将旋转矩阵转换为四元数
  Eigen::Quaterniond quaternion(rotationMatrix);

  // 将位置和方向分别填充到 Odometry 消息的 pose 中
  odometry.pose.pose.position.x = translation.x();
  odometry.pose.pose.position.y = translation.y();
  odometry.pose.pose.position.z = translation.z();
  odometry.pose.pose.orientation.w = quaternion.w();
  odometry.pose.pose.orientation.x = quaternion.x();
  odometry.pose.pose.orientation.y = quaternion.y();
  odometry.pose.pose.orientation.z = quaternion.z();
  ROS_INFO("convertMatrixToOdometry done");
}

void cb_save_cur_odom(const nav_msgs::Odometry &odom_msg)
{
  if (has_transform == false)
  {
    return;
  }
  cur_odom_to_baselink = odom_msg;
  // if (cur_odom_to_baselink.header.stamp != ros::Time(0))
  // {
  //   return;
  // }
  // mat1 stands for cur_odom_to_baselink, mat2 stands for map_to_odom, mat3 stands for map_to_base_link
  Eigen::Matrix4d mat1, mat2, mat3;
  convertOdometryToEigenMatrix(cur_odom_to_baselink, mat1);
  convertOdometryToEigenMatrix(cur_map_to_odom, mat2);

  if (mat1.isZero() || mat2.isZero())
  {
    ROS_WARN("cur_odom_to_baselink or map_to_odom is empty!!!");
    return;
  }
  else
  {
    mat3 = mat2 * mat1;
    ROS_INFO("mat3 got");
    std::cout << "mat3:\n"
              << mat3 << std::endl;
  }

  nav_msgs::Odometry localization;
  convertMatrixToOdometry(mat3, localization);

  localization.twist = cur_odom_to_baselink.twist;
  localization.header.stamp = cur_odom_to_baselink.header.stamp;
  localization.header.frame_id = "map";
  localization.child_frame_id = "body";
  pub_localization.publish(localization);
  // std::cout << localization << std::endl;
}

void cb_save_map_to_odom(const nav_msgs::Odometry &odom_msg)
{
  has_transform = true;
  cur_map_to_odom = odom_msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_fusion");
  ros::NodeHandle nh;
  ROS_INFO("Transform Fusion Node Inited...");

  ros::Subscriber sub_cur_odom = nh.subscribe("/Odometry_imu", 100, cb_save_cur_odom, ros::TransportHints().tcpNoDelay());
  // ros::Subscriber sub_cur_odom = nh.subscribe("/Odometry_imu", 100, cb_save_cur_odom);
  ros::Subscriber sub_map_to_odom = nh.subscribe("/map_to_odom", 100, cb_save_map_to_odom, ros::TransportHints().tcpNoDelay());
  pub_localization = nh.advertise<nav_msgs::Odometry>("/localization", 100);

  ros::spin();
  return 0;
}
