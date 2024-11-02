#include "object_trajectory_estimator/sphere_fitting.h"

namespace object_trajectory_estimator
{

SphereFitting::SphereFitting() {}

void SphereFitting::onInit()
{
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();
    point_cloud_sub_ = pnh_.subscribe("pcl_input", 1, &SphereFitting::pointCloudCallback, this);
    center_pub_ = pnh_.advertise<geometry_msgs::PointStamped>("center_output", 1);

    // 既知の半径を設定
    radius_ = 0.1225; // 必要に応じて変更
}

void SphereFitting::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // ROS点群メッセージをPCL点群形式に変換
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  
  // 最小二乗法を用いて球をフィッティング
  Eigen::Vector3f center;
  this->fitSphere(cloud, center, radius_);
  
  // 球の中心の座標を取得
  geometry_msgs::PointStamped center_msg;
  center_msg.header = cloud_msg->header;
  center_msg.point.x = center(0);
  center_msg.point.y = center(1);
  center_msg.point.z = center(2);
  
  ROS_INFO("Fitted sphere center: x=%f, y=%f, z=%f, radius=%f", center(0), center(1), center(2), radius_);
  
  // 球の中心をパブリッシュ
  center_pub_.publish(center_msg);
}

void SphereFitting::fitSphere(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Vector3f& center, float& radius)
{
  Eigen::MatrixXf A(cloud->points.size(), 3);
  Eigen::VectorXf b(cloud->points.size());
  
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const auto& point = cloud->points[i];
    A(i, 0) = 2 * point.x;
    A(i, 1) = 2 * point.y;
    A(i, 2) = 2 * point.z;
    std::cerr << "xyz:" << point.x << "," << point.y << "," << point.z << std::endl;
    b(i) = point.x * point.x + point.y * point.y + point.z * point.z - radius * radius;
  }
  
  // 正規方程式を解いて球の中心を計算
  center = A.colPivHouseholderQr().solve(b);
}
  
} // namespace object_trajectory_estimator

// Register the nodelet
#ifdef USE_PLUGINLIB_CLASS_LIST_MACROS_H
#include <pluginlib/class_list_macros.h>
#else
#include <pluginlib/class_list_macros.hpp>
#endif
PLUGINLIB_EXPORT_CLASS(object_trajectory_estimator::SphereFitting, nodelet::Nodelet);
