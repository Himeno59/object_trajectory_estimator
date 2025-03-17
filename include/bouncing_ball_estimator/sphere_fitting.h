#ifndef SPHERE_FITTING_H
#define SPHERE_FITTING_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

namespace bouncing_ball_estimator
{
  class SphereFitting : public nodelet::Nodelet {
  public:
    SphereFitting();
    ~SphereFitting(){};
    
    virtual void onInit();
    
  private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher center_pub_;

    double radius_;
    double thr_;
  };
  
} // namespace bouncing_ball_estimator

#endif // SPHERE_FITTING_H
