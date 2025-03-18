#ifndef BOUNCING_BALL_ESTIMATOR_NODELET_HPP
#define BOUNCING_BALL_ESTIMATOR_NODELET_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <bouncing_ball_estimator/BallStateStamped.h>
#include <bouncing_ball_estimator/FbCheck.h>

#include "bouncing_ball_estimator/SetRLSParameters.h"
#include "bouncing_ball_estimator/SetRLSMatrix.h"
#include "bouncing_ball_estimator/GetRLSParameters.h"

#include "bouncing_ball_estimator/recursive_least_square.hpp"
#include "bouncing_ball_estimator/least_square.hpp"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

#ifndef GRAVITY
#define GRAVITY 9.80665
#endif

namespace bouncing_ball_estimator {

  class BouncingBallEstimator : public nodelet::Nodelet {
  public:
    BouncingBallEstimator();
    ~BouncingBallEstimator(){};

    virtual void onInit();

  private:
    void callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void publish();
    void updateStamp(const geometry_msgs::PointStamped::ConstPtr &msg);
    void stateManager(const geometry_msgs::PointStamped::ConstPtr &point);
    void calcCurrentState(const geometry_msgs::PointStamped::ConstPtr &msg);
    void calcPredState();
    
    geometry_msgs::PointStamped applyFilter(const geometry_msgs::PointStamped &msg);
    geometry_msgs::PointStamped transformPoint(const tf2_ros::Buffer &tfBuffer, const geometry_msgs::PointStamped &msg);

    bool setRLSParameters(bouncing_ball_estimator::SetRLSParameters::Request &req, bouncing_ball_estimator::SetRLSParameters::Response &res);
    bool setRLSMatrix(bouncing_ball_estimator::SetRLSMatrix::Request &req, bouncing_ball_estimator::SetRLSMatrix::Response &res);
    bool getRLSParameters(bouncing_ball_estimator::GetRLSParameters::Request &req, bouncing_ball_estimator::GetRLSParameters::Response &res);

    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::ServiceServer setService;
    ros::ServiceServer getService;
    ros::ServiceServer matrixService;
    ros::Subscriber point_sub;
    ros::Publisher current_state_pub;
    ros::Publisher pred_state_pub;
    ros::Publisher check_pub;
    ros::Publisher current_state_pos_pub;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    bouncing_ball_estimator::BallStateStamped current_state;
    bouncing_ball_estimator::BallStateStamped pred_state;
    bouncing_ball_estimator::BallStateStamped prev_state;
    bouncing_ball_estimator::FbCheck fb_check;
    geometry_msgs::PointStamped current_state_pos; // visualize用

    int window_size;
    std::vector<Eigen::Vector3d> window;

    RLS3D rls;

    double dt;
    double current_time;

    // flag
    bool ready_flag;
    bool ballSet_flag;
    bool predict_flag;
    bool loop_init;

    // rosparam
    std::string frame_id;
    std::string camera_frame;
    std::string base_frame;
    double init_thr;
    double start_thr;
    double bound_thr;
    double wait_fb;
    std::vector<double> x_init_theta;
    std::vector<double> y_init_theta;
    std::vector<double> z_init_theta;
    double lambda;
  };

} // namespace bouncing_ball_estimator

#endif // BOUNCING_BALL_ESTIMATOR_NODELET_HPP
