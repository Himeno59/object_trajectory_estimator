#include "object_trajectory_estimator/depth_min.h"

namespace object_trajectory_estimator {

DepthMinMaskNode::DepthMinMaskNode() {}

void DepthMinMaskNode::onInit() {
  // node handler
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  // sub
  depth_image_sub_ = pnh_.subscribe("depth_image_input", 1, &DepthMinMaskNode::depthImageCallback, this);
  // pub
  depth_mask_image_pub_ = pnh_.advertise<sensor_msgs::Image>("depth_mask_image_output", 1);
}
 
void DepthMinMaskNode::depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 画像メッセージをOpenCV形式に変換
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat tmp_image = cv_ptr->image;
    
    // minMaxLocで最大値取得
    double maxVal;
    cv::Point maxLoc;
    cv::minMaxLoc(tmp_image, nullptr, &maxVal, nullptr, &maxLoc);
      
    // 深度が存在しない0のピクセルを全てmaxValにする <- 次のminMaxLocで0部分を検出しないようにするため
    tmp_image.setTo(static_cast<uint16_t>(maxVal), tmp_image == 0);
    
    // minMaxLocで最小値取得
    double minVal;
    cv::Point minLoc;
    cv::minMaxLoc(tmp_image, &minVal, nullptr, &minLoc, nullptr);
    
    std::cerr << "maxVal: " << maxVal << std::endl;
    std::cerr << "maxLoc: " << maxLoc << std::endl;
    std::cerr << "minVal: " << minVal << std::endl;
    std::cerr << "minLoc: " << minLoc << std::endl;
    
    double depth_thr;
    double thr_offset = 5.0;
    depth_thr = minVal + thr_offset;
    std::cerr << "depth_thr: " << depth_thr << std::endl;

    // points_xyzrgbに流す深度情報付きのマスク画像
    cv::Mat depthMaskImage = cv::Mat::zeros(cv_ptr->image.size(), CV_16UC1);
    tmp_image.copyTo(depthMaskImage, tmp_image < depth_thr);
    // depthMaskImage.copyTo(tmp_image, tmp_image < depth_thr);
    // depthMaskImage.setTo(static_cast<uint16_t>(minVal), cv_ptr->image == minVal);
    
    // publish
    cv_bridge::CvImage depth_mask_msg;
    depth_mask_msg.header = msg->header;
    depth_mask_msg.encoding = msg->encoding;
    depth_mask_msg.image = depthMaskImage;
    depth_mask_image_pub_.publish(depth_mask_msg.toImageMsg());
}
 
} // namespace object_trajectory_estimator

// Register the nodelet
#ifdef USE_PLUGINLIB_CLASS_LIST_MACROS_H
#include <pluginlib/class_list_macros.h>
#else
#include <pluginlib/class_list_macros.hpp>
#endif
PLUGINLIB_EXPORT_CLASS(object_trajectory_estimator::DepthMinMaskNode, nodelet::Nodelet);
