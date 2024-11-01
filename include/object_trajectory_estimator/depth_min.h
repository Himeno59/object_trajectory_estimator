#ifndef DEPTH_MIN_H
#define DEPTH_MIN_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace object_trajectory_estimator {

  class DepthMinMaskNode : public nodelet::Nodelet {
  public:
    DepthMinMaskNode();
    ~DepthMinMaskNode(){};

    virtual void onInit();
    
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber depth_image_sub_;
    ros::Publisher depth_mask_image_pub_;
  
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
  };
  
} // namespace object_trajectory_estimator

#endif // DEPTH_MIN_H
