#ifndef SPHERE_FITTING_H
#define SPHERE_FITTING_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

namespace object_trajectory_estimator
{

class SphereFitting : public nodelet::Nodelet
{
public:
  SphereFitting();
  ~SphereFitting() {};

  virtual void onInit();
    
private:
  
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void fitSphere(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Vector3f& center, float& radius);
  
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher center_pub_;
  
  float radius_; // 既知の半径
};
  
} // namespace object_trajectory_estimator

#endif // SPHERE_FITTING_H
