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
}

void SphereFitting::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // ROS点群メッセージをPCL点群形式に変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // RANSACを使用して球をフィッティング
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_sphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_sphere);
    
    double radius = 0.1225;
    double thr = 0.010;
    ransac.setDistanceThreshold(thr);
    
    std::vector<int> inliers;
    
    if (ransac.computeModel())
    {
        Eigen::VectorXf coeff;
        ransac.getModelCoefficients(coeff);

        // 球の中心の座標を取得
        geometry_msgs::PointStamped center;
        center.header = cloud_msg->header;
        center.point.x = coeff[0];
        center.point.y = coeff[1];
        center.point.z = coeff[2];

        ROS_INFO("Sphere center: x=%f, y=%f, z=%f", center.point.x, center.point.y, center.point.z);

        model_sphere->selectWithinDistance(coeff, radius + thr, inliers);

        // 球の中心をパブリッシュ
        center_pub_.publish(center);
    }
    else
    {
        ROS_WARN("Failed to fit a sphere to the point cloud.");
    }
}

} // namespace object_trajectory_estimator

// Register the nodelet
#ifdef USE_PLUGINLIB_CLASS_LIST_MACROS_H
#include <pluginlib/class_list_macros.h>
#else
#include <pluginlib/class_list_macros.hpp>
#endif
PLUGINLIB_EXPORT_CLASS(object_trajectory_estimator::SphereFitting, nodelet::Nodelet);