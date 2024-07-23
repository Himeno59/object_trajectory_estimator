// ros msg
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "object_trajectory_estimator/BallStateStamped.h"

// srv
#include "object_trajectory_estimator/SetRLSParameters.h"
#include "object_trajectory_estimator/GetRLSParameters.h"

#include "object_trajectory_estimator/recursive_least_square.hpp"
#include "object_trajectory_estimator/least_square.hpp"

#include "iostream"
#include "vector"

#ifndef GRAVITY
#define GRAVITY 9.80665
#endif

class ObjectTrajectoryEstimator
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::ServiceServer setService;
  ros::ServiceServer getService;

  // input: カメラ相対の色つき3次元点群の重心
  ros::Subscriber point_sub;
  // output: 現在のBODY相対の物体位置、速度
  //         予測したBODY相対の物体の最高到達点、速度
  ros::Publisher current_state_pub, pred_state_pub;          // to BasketballMotionController
  ros::Publisher current_state_pos_pub, pred_state_pos_pub;  // to ros-visualization
  // ros::Publisher real_traj_pub, pred_traj_pub;
  
  object_trajectory_estimator::BallStateStamped current_state, pred_state;
  object_trajectory_estimator::BallStateStamped prev_state; // 値保持用
  geometry_msgs::PointStamped current_state_pos, pred_state_pos;
  // tf2_msgs::TFMessage real_traj, pred_traj;
  
  // デバッグ用: current_state_posをpred_time秒左に平行移動させたものをpublish
  ros::Publisher dummy_state_pos_pub;
  geometry_msgs::PointStamped dummy_state_pos;

  void callback(const geometry_msgs::PointStamped::ConstPtr &msg);
  void publish();
  void updateStamp();
  void stateManager();
  void calcCurrentState(const geometry_msgs::PointStamped::ConstPtr &msg);
  void calcPredState();
  void calcInitState(); // rlsの初期値更新用

  // 平滑化
  int window_size;
  std::vector<Eigen::Vector3d> window;
  geometry_msgs::PointStamped applyFilter(const geometry_msgs::PointStamped::ConstPtr &msg);

  // tf変換
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  geometry_msgs::PointStamped transformPoint(const tf2_ros::Buffer &tfBuffer, const geometry_msgs::PointStamped &filteredPoint);

  // rls
  int rls_degree;
  RLS3D rls;

private:
  double dt;
  double current_time;
  double target_time;
  Eigen::VectorXd timeVector;

  bool ready_flag;
  bool ballSet_flag;
  bool theta_initialize;
  bool predict_flag; // true:予測する, false:予測しない、paramをリセット
  bool loop_init;

  std::vector<double> init_vel;

  // rosparam
  std::string frame_id, camera_frame, base_frame;
  double init_thr;
  double start_thr;
  double bound_thr; // 今のjaxonの姿勢から出せるようにしたい
  double pred_time;
  std::vector<double> x_init_theta;
  std::vector<double> y_init_theta;
  std::vector<double> z_init_theta;
  
public:
  ObjectTrajectoryEstimator(int k_x, int k_y, int k_z);
  ~ObjectTrajectoryEstimator(){};

  // srv
  bool setRLSParameters(object_trajectory_estimator::SetRLSParameters::Request  &req,
			object_trajectory_estimator::SetRLSParameters::Response &res);
  bool getRLSParameters(object_trajectory_estimator::GetRLSParameters::Request  &req,
			object_trajectory_estimator::GetRLSParameters::Response &res);
};
