#ifndef OBJECT_TRAJECTORY_ESTIMATOR_NODELET_HPP
#define OBJECT_TRAJECTORY_ESTIMATOR_NODELET_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <object_trajectory_estimator/BallStateStamped.h>
#include <object_trajectory_estimator/FbCheck.h>

#include <object_trajectory_estimator/SetRLSParameters.h>
#include <object_trajectory_estimator/GetRLSParameters.h>

#include <object_trajectory_estimator/recursive_least_square.hpp>
#include <object_trajectory_estimator/least_square.hpp>

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

#ifndef GRAVITY
#define GRAVITY 9.80665
#endif

namespace object_trajectory_estimator {

  class ObjectTrajectoryEstimator : public nodelet::Nodelet {
  public:
    ObjectTrajectoryEstimator();
    ~ObjectTrajectoryEstimator(){};

    virtual void onInit();

  private:
    void callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void publish();
    void updateStamp(const geometry_msgs::PointStamped::ConstPtr &msg);
    void stateManager(const geometry_msgs::PointStamped::ConstPtr &point);
    
    void calcCurrentState(const geometry_msgs::PointStamped::ConstPtr &msg);
    void calcPredState();
    
    void calcInitState();
    geometry_msgs::PointStamped applyFilter(const geometry_msgs::PointStamped &msg);
    geometry_msgs::PointStamped transformPoint(const tf2_ros::Buffer &tfBuffer, const geometry_msgs::PointStamped &msg);

    bool setRLSParameters(object_trajectory_estimator::SetRLSParameters::Request &req, object_trajectory_estimator::SetRLSParameters::Response &res);
    bool getRLSParameters(object_trajectory_estimator::GetRLSParameters::Request &req, object_trajectory_estimator::GetRLSParameters::Response &res);

    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::ServiceServer setService;
    ros::ServiceServer getService;
    ros::Subscriber point_sub;
    ros::Publisher current_state_pub;
    ros::Publisher pred_state_pub;
    ros::Publisher check_pub;
    ros::Publisher current_state_pos_pub;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    object_trajectory_estimator::BallStateStamped current_state;
    object_trajectory_estimator::BallStateStamped pred_state;
    object_trajectory_estimator::BallStateStamped prev_state;
    object_trajectory_estimator::FbCheck fb_check;
    geometry_msgs::PointStamped current_state_pos; // visualize用

    int window_size;
    std::vector<Eigen::Vector3d> window;
    int rls_degree;
    RLS3D rls;

    double dt;
    double current_time;
    double target_time;
    Eigen::VectorXd timeVector;

    // flag
    bool ready_flag;
    bool ballSet_flag;
    bool theta_initialize;
    bool predict_flag;
    bool loop_init;

    std::vector<double> init_vel;

    // rosparam
    std::string frame_id;
    std::string camera_frame;
    std::string base_frame;
    double init_thr;
    double start_thr;
    double bound_thr;
    double pred_time;
    double wait_fb;
    double centroid_offset;
    std::vector<double> x_init_theta;
    std::vector<double> y_init_theta;
    std::vector<double> z_init_theta;
  };

} // namespace object_trajectory_estimator

#endif // OBJECT_TRAJECTORY_ESTIMATOR_NODELET_HPP
