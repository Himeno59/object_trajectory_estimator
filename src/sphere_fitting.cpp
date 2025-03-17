#include "bouncing_ball_estimator/sphere_fitting.h"

namespace bouncing_ball_estimator
{
  
SphereFitting::SphereFitting() {}

void SphereFitting::onInit()
{
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();
    point_cloud_sub_ = pnh_.subscribe("pcl_input", 1, &SphereFitting::pointCloudCallback, this);
    center_pub_ = pnh_.advertise<geometry_msgs::PointStamped>("center_output", 1);

    pnh_.param("radius", radius_, 0.1225);
    pnh_.param("thr", thr_, 0.01);
}

void SphereFitting::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    // ROS点群メッセージをPCL点群形式に変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // SACSegmentationの設定
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true); // 最適化
    seg.setModelType(pcl::SACMODEL_SPHERE); // 球の推定
    seg.setMethodType(pcl::SAC_RANSAC); // 手法の選択 -> SAC
    seg.setDistanceThreshold(thr_); // モデルとポイントの許容距離
    seg.setRadiusLimits(radius_ - thr_, radius_ + thr_);  // 半径範囲を設定    
    
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    // モデルの適合
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.empty())
      {
        // ROS_WARN("No sphere found in the point cloud.");
        return;
      }
    
    // 重心の取得
    geometry_msgs::PointStamped sphere_center;
    sphere_center.header = cloud_msg->header;
    sphere_center.point.x = coefficients->values[0];
    sphere_center.point.y = coefficients->values[1];
    sphere_center.point.z = coefficients->values[2];
    
    // 重心のパブリッシュ
    center_pub_.publish(sphere_center);
    // ROS_INFO("Sphere center found at: (%f, %f, %f)", sphere_center.point.x, sphere_center.point.y, sphere_center.point.z);
  

    // // RANSACを使用して球をフィッティング
    // pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_sphere(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    // pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_sphere);
    
    // double radius = 0.1225;
    // double thr = 0.010;
    // ransac.setDistanceThreshold(thr);
    
    // std::vector<int> inliers;
    
    // if (ransac.computeModel())
    // {
    //     Eigen::VectorXf coeff;
    //     ransac.getModelCoefficients(coeff);

    //     // 球の中心の座標を取得
    //     geometry_msgs::PointStamped center;
    //     center.header = cloud_msg->header;
    //     center.point.x = coeff[0];
    //     center.point.y = coeff[1];
    //     center.point.z = coeff[2];

    //     ROS_INFO("Sphere center: x=%f, y=%f, z=%f", center.point.x, center.point.y, center.point.z);

    //     model_sphere->selectWithinDistance(coeff, radius + thr, inliers);

    //     // 球の中心をパブリッシュ
    //     center_pub_.publish(center);
    // }
    // else
    // {
    //     ROS_WARN("Failed to fit a sphere to the point cloud.");
    // }
}

} // namespace bouncing_ball_estimator

// Register the nodelet
#ifdef USE_PLUGINLIB_CLASS_LIST_MACROS_H
#include <pluginlib/class_list_macros.h>
#else
#include <pluginlib/class_list_macros.hpp>
#endif
PLUGINLIB_EXPORT_CLASS(bouncing_ball_estimator::SphereFitting, nodelet::Nodelet);
