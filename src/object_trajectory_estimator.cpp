#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "object_trajectory_estimator/BallStateStamped.h"

#include "basketball_motion/recursive_least_square.hpp"
#include "basketball_motion/least_square.hpp"

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

  // 入力: カメラ相対の色つき3次元点群の重心
  // 出力: 現在のBODY相対の物体位置、速度
  //       予測したBODY相対の物体の最高到達点、速度
  ros::Subscriber point_sub;
  ros::Publisher now_state_pub, pred_state_pub; // to BasketballMotionController
  ros::Publisher now_state_pos_pub, pred_state_pos_pub; // for ros-visualization
  // ros::Publisher real_traj_pub, pred_traj_pub;
  
  object_trajectory_estimator::BallStateStamped now_state, pred_state;
  object_trajectory_estimator::BallStateStamped prev_state; // 値保持用
  geometry_msgs::PointStamped now_state_pos, pred_state_pos;
  // tf2_msgs::TFMessage real_traj, pred_traj;

  // デバッグ用: now_state_posをpred_time秒左に平行移動させたものをpublish
  ros::Publisher dummy_state_pos_pub;
  geometry_msgs::PointStamped dummy_state_pos;

  void Callback(const geometry_msgs::PointStamped::ConstPtr &msg);
  void Publish();
  void updateStamp();
  void stateManager();
  void calcInitTheta();
  void calcNowState(const geometry_msgs::PointStamped::ConstPtr &msg);

  // tf変換
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  geometry_msgs::PointStamped transformPoint(const tf2_ros::Buffer &tfBuffer, const geometry_msgs::PointStamped::ConstPtr &msg);

  // rls: パラメタ更新
  // todo: rlsの実装をx,y,z同時に扱えるようにする
  RecursiveLS rls_x;
  RecursiveLS rls_y;
  RecursiveLS rls_z;

  // ls: 各サイクルの初期パラメタの計算
  LeastSquare ls_x;
  LeastSquare ls_y;
  LeastSquare ls_z;
  // 各サイクルの初期係数決定パラメタ
  std::vector<double> init_time_list;
  std::vector<double> init_x_list;
  std::vector<double> init_y_list;
  std::vector<double> init_z_list;

  double dt;
  double target_time;
  double current_time;
  Eigen::VectorXd time_x;
  Eigen::VectorXd time_y;
  Eigen::VectorXd time_z;

  bool ready_flag;
  bool ballSet_flag;
  bool thetaInitialize;

  bool predictStart_flag; // true:予測する, false:予測しない、paramをリセット
  bool loop_init;

  // rosparam
  std::string frame_id, camera_frame, base_frame;
  double bound_thr; // 今のjaxonの姿勢から出せるようにする
  double pred_time;
  
public:
  ObjectTrajectoryEstimator(int k_x, int k_y, int k_z);
  ~ObjectTrajectoryEstimator(){};
  
};

ObjectTrajectoryEstimator::ObjectTrajectoryEstimator(int k_x, int k_y, int k_z)
  : nh(""), pnh("~"), rls_x(k_x), rls_y(k_y), rls_z(k_z), ls_x(k_x), ls_y(k_y), ls_z(k_z), tfListener(tfBuffer)
{
  // subscriber
  point_sub = pnh.subscribe("point_input", 1, &ObjectTrajectoryEstimator::Callback, this);
   
  // publisher
  now_state_pub = pnh.advertise<object_trajectory_estimator::BallStateStamped>("now_state_output", 1);
  pred_state_pub = pnh.advertise<object_trajectory_estimator::BallStateStamped>("pred_state_output", 1);
  now_state_pos_pub = pnh.advertise<geometry_msgs::PointStamped>("now_state_pos_output", 1);
  pred_state_pos_pub = pnh.advertise<geometry_msgs::PointStamped>("pred_state_pos_output", 1);
  dummy_state_pos_pub = pnh.advertise<geometry_msgs::PointStamped>("dummy_state_pos_output", 1);
  // real_traj_pub = nh.advertise<tf2_msgs::TFMessage>(real_traj_output, 1);
  // pred_traj_pub = nh.advertise<tf2_msgs::TFMessage>(pred_traj_output, 1);
  
  // get rosparam
  pnh.getParam("frame_id", frame_id);
  pnh.getParam("camera_frame", camera_frame);
  pnh.getParam("base_frame", base_frame);
  pnh.getParam("bound_thr", bound_thr);
  pnh.getParam("pred_time", pred_time);
  
  // header setup
  // frame_id
  now_state.header.frame_id = base_frame;
  pred_state.header.frame_id = base_frame;
  now_state_pos.header.frame_id = base_frame;
  pred_state_pos.header.frame_id = base_frame;
  // stamp
  ros::Time tmp_time = ros::Time::now();
  now_state.header.stamp = tmp_time;
  pred_state.header.stamp = tmp_time;
  now_state_pos.header.stamp = tmp_time;
  pred_state_pos.header.stamp = tmp_time;

  current_time = 0.0;
  time_x = Eigen::VectorXd::Zero(k_x+1);
  time_y = Eigen::VectorXd::Zero(k_y+1);
  time_z = Eigen::VectorXd::Zero(k_z+1);

  ready_flag = true;
  ballSet_flag = false;
  loop_init = true;
  predictStart_flag = false;
  thetaInitialize = true;
}

void ObjectTrajectoryEstimator::updateStamp(){
  ros::Time now_time = ros::Time::now();
  now_state.header.stamp = now_time; 
  pred_state.header.stamp = now_time;
  now_state_pos.header.stamp = now_time; 
  pred_state_pos.header.stamp = now_time;
  dummy_state_pos.header.stamp = now_time - ros::Duration(pred_time);
  dt = (now_state.header.stamp - prev_state.header.stamp).toSec();
}

// 各サイクルの初期係数の決定
// hzの高いカメラで最初に十分点を取得できるときは使えそう
void ObjectTrajectoryEstimator::calcInitTheta() {
  
  init_time_list.push_back(current_time);
  init_x_list.push_back(now_state.pos.x);
  init_y_list.push_back(now_state.pos.y);
  init_z_list.push_back(now_state.pos.z);
  
  // 最小二乗法で計算
  if (init_time_list.size() == 15) { // n点集まってからスタート   
    // lsでの計算
    ls_x.calcTheta(init_time_list, init_x_list);
    ls_y.calcTheta(init_time_list, init_y_list);
    ls_z.calcTheta(init_time_list, init_z_list);
    
    // rlsの初期係数として設定
    rls_x.setInitialTheta(ls_x.getParameters());
    rls_y.setInitialTheta(ls_y.getParameters());
    rls_z.setInitialTheta(ls_z.getParameters());
    
    // reset
    thetaInitialize = false;
    init_time_list.clear();
    init_x_list.clear();
    init_y_list.clear();
    init_z_list.clear();
  }
}

// カメラ->BODYの座標変換
geometry_msgs::PointStamped ObjectTrajectoryEstimator::transformPoint
(const tf2_ros::Buffer &tfBuffer, const geometry_msgs::PointStamped::ConstPtr &msg) {
  // 半球殻の重心->球の重心
  Eigen::Vector3d pos(msg->point.x, msg->point.y, msg->point.z);
  Eigen::Vector3d fixed_pos;
  geometry_msgs::PointStamped tmp_point;
  double D = pos.norm();
  double k = 0.5 * 0.1225; // r=0.1225[m]
  fixed_pos = pos + k * pos.normalized();
  tmp_point.point.x = fixed_pos.x();
  tmp_point.point.y = fixed_pos.y();
  tmp_point.point.z = fixed_pos.z();

  // 座標変換
  geometry_msgs::TransformStamped trans;
  trans = tfBuffer.lookupTransform(base_frame, camera_frame, ros::Time(0)); // (出力, 入力)の順番
  geometry_msgs::PointStamped transformedPoint;
  tf2::doTransform(tmp_point, transformedPoint, trans);
  transformedPoint.header.stamp = ros::Time::now();
  
  return transformedPoint;
}

// BODY相対の物体の位置、速度の計算
void ObjectTrajectoryEstimator::calcNowState(const geometry_msgs::PointStamped::ConstPtr &msg) {
  // 位置
  geometry_msgs::PointStamped transformedPoint = transformPoint(tfBuffer, msg);
  now_state.pos.x = transformedPoint.point.x;
  now_state.pos.y = transformedPoint.point.y;
  now_state.pos.z = transformedPoint.point.z;
  // 速度
  now_state.vel.x = (now_state.pos.x - prev_state.pos.x) / dt;
  now_state.vel.y = (now_state.pos.y - prev_state.pos.y) / dt;
  now_state.vel.z = (now_state.pos.z - prev_state.pos.z) / dt;

  // 確認用
  dummy_state_pos.point.x = transformedPoint.point.x;
  dummy_state_pos.point.y = transformedPoint.point.y;
  dummy_state_pos.point.z = transformedPoint.point.z;
}

void ObjectTrajectoryEstimator::stateManager() {
  // 動作開始の判定
  if (ready_flag) {
    if (now_state.pos.z >= -0.5) {
      // ボールをセットしたと判定
      ballSet_flag = true;
    } else if (ballSet_flag && now_state.pos.z < -0.4) {
      // セットした後にある程度ボール位置が落ちてきたら動作が始まったと判定
      ballSet_flag = false;
      ready_flag = false;
    }
  } else {
    if (now_state.vel.z > 0) {
      // ボールの速度が正のときは常に予測する
      predictStart_flag = true;
    } else if (now_state.vel.z <= 0) {
      // ボールの速度が0以下のときは予測しない
      predictStart_flag = false;

      // rslのパラメタ初期化
      // 一旦やらないでおく
      // rls_x.reset();
      // rls_y.reset();
      // rls_z.reset();
      // thetaInitialize = true;

      // 時間のリセット
      current_time = 0.0;
    }
  }
}

void ObjectTrajectoryEstimator::Callback(const geometry_msgs::PointStamped::ConstPtr &msg){
  // stampの更新 & dtの計算
  updateStamp();
  
  // 現在の(BODY相対の)objの状態 = now_state_pos/vel を計算
  calcNowState(msg);
  
  // 状態の管理 + パラメタのリセット
  stateManager();

  // rlsの初期値を利用するver
  if (predictStart_flag) {

    if (!loop_init) current_time += dt; // 最初は current_time = 0.0 とする
    loop_init = false;
    
    for (int i = 0; i < time_x.size(); ++i) {
      time_x(i) = std::pow(current_time, i);
    }
    for (int i = 0; i < time_y.size(); ++i) {
      time_y(i) = std::pow(current_time, i); 
    }
    for (int i = 0; i < time_z.size(); ++i) {
      time_z(i) = std::pow(current_time, i); 
    }
    // 係数の更新
    rls_x.update(time_x, now_state.pos.x);
    rls_y.update(time_y, now_state.pos.y);
    rls_z.update(time_z, now_state.pos.z);
    
    // 予測値の計算
    // 位置
    // // 時間指定ver
    // target_time = current_time + pred_time;
    // pred_state.pos.x = rls_x.predict(target_time);
    // pred_state.pos.y = rls_y.predict(target_time);
    // pred_state.pos.z = rls_z.predict(target_time);
    // 最高到達点ver
    pred_state.pos.x = rls_x.predictHighestPoint();
    pred_state.pos.y = rls_y.predictHighestPoint(); // x,yは一旦1を返すようにしている
    pred_state.pos.z = rls_z.predictHighestPoint();

    // todo: 速度
  }

  // // 各サイクルの最初に最小二乗法で推定するフェーズを挟むver
  // if (predictStart_flag) {
    
  //   if (!loop_init) current_time += dt;
  //   loop_init = false;
    
  //   if (thetaInitialize) {
  //     calcInitTheta();
  //   } else {
  //     for (int i = 0; i < time_x.size(); ++i) {
  // 	time_x(i) = std::pow(current_time, i);
  //     }
  //     for (int i = 0; i < time_y.size(); ++i) {
  // 	time_y(i) = std::pow(current_time, i); 
  //     }
  //     for (int i = 0; i < time_z.size(); ++i) {
  // 	time_z(i) = std::pow(current_time, i); 
  //     }
  //     // 係数の更新
  //     rls_x.update(time_x, now_state.pos.x);
  //     rls_y.update(time_y, now_state.pos.y);
  //     rls_z.update(time_z, now_state.pos.z);
      
  //     // 予測値の計算
  //     // 時間指定ver
  //     target_time = current_time + pred_time;
  //     pred_state.pos.x = rls_x.predict(target_time);
  //     pred_state.pos.y = rls_y.predict(target_time);
  //     pred_state.pos.z = rls_z.predict(target_time);
  //     // // 最高到達点ver
  //     // pred_state.pos.x = rls_x.predictContactPoint();
  //     // pred_state.pos.y = rls_y.predictContactPoint();
  //     // pred_state.pos.z = rls_z.predictContactPoint(); 
  //   }
  // }
  
  Publish();
  
  // 値の保持
  prev_state = now_state;
}

void ObjectTrajectoryEstimator::Publish(){
  now_state_pub.publish(now_state);
  pred_state_pub.publish(pred_state);
  now_state_pos_pub.publish(now_state_pos);
  pred_state_pos_pub.publish(pred_state_pos);
  // real_traj_pub.publish(real_traj);
  // pred_trag_pub.publish(pred_traj);

  dummy_state_pos_pub.publish(dummy_state_pos);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_trajectory_estimate");
  ObjectTrajectoryEstimator object_trajectory_estimator(1, 1, 2);
  
  while(ros::ok()){
    ros::spinOnce();
  }
}
