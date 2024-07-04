#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "object_trajectory_estimator/BallStateStamped.h"

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

  // 入力: カメラ相対の色つき3次元点群の重心
  ros::Subscriber point_sub;
  // 出力: 現在のBODY相対の物体位置、速度
  //       予測したBODY相対の物体の最高到達点、速度
  ros::Publisher now_state_pub, pred_state_pub;          // to BasketballMotionController
  ros::Publisher now_state_pos_pub, pred_state_pos_pub;  // to ros-visualization
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
  void calcNowState(const geometry_msgs::PointStamped::ConstPtr &msg);
  void calcPredState();

  // tf変換
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  geometry_msgs::PointStamped transformPoint(const tf2_ros::Buffer &tfBuffer, const geometry_msgs::PointStamped::ConstPtr &msg);

  // rls: パラメタ更新
  RLS3D rls;

  double dt;
  double current_time;
  double target_time;
  Eigen::VectorXd timeVector;

  bool ready_flag;
  bool ballSet_flag;

  // bool thetaInitialize;

  bool predict_flag; // true:予測する, false:予測しない、paramをリセット
  bool loop_init;

  // rosparam
  std::string frame_id, camera_frame, base_frame;
  double bound_thr; // 今のjaxonの姿勢から出せるようにしたい
  double pred_time;
  
public:
  ObjectTrajectoryEstimator(int k_x, int k_y, int k_z);
  ~ObjectTrajectoryEstimator(){};
  
};

ObjectTrajectoryEstimator::ObjectTrajectoryEstimator(int k_x, int k_y, int k_z)
  : nh(""), pnh("~"), rls(k_x, k_y, k_z), tfListener(tfBuffer)
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

  // fb_flag
  pred_state.fb_flag.data = false;

  current_time = 0.0;
  timeVector = Eigen::VectorXd::Zero(3);

  ready_flag = true;  
  ballSet_flag = false;
  loop_init = false;
  predict_flag = false;
  // thetaInitialize = false;
}

void ObjectTrajectoryEstimator::Callback(const geometry_msgs::PointStamped::ConstPtr &msg){
  // stampの更新 & dtの計算
  updateStamp();
  
  // 現在の(BODY相対の)objの状態 = now_state_pos/vel を計算
  calcNowState(msg);
  
  // 状態の管理 + パラメタのリセット
  stateManager();

  // 予測フラグが立っている場合、予測する
  if (predict_flag) calcPredState();

  // 値をpublish
  Publish();
  
  // 値の保持
  prev_state = now_state;
}

/* -- stampの更新 & dtの計算-- */
void ObjectTrajectoryEstimator::updateStamp(){
  ros::Time now_time = ros::Time::now();
  now_state.header.stamp = now_time; 
  pred_state.header.stamp = now_time;
  now_state_pos.header.stamp = now_time; 
  pred_state_pos.header.stamp = now_time;
  dummy_state_pos.header.stamp = now_time - ros::Duration(pred_time);
  dt = (now_state.header.stamp - prev_state.header.stamp).toSec();
}

/* -- 現在の(BODY相対の)objの状態 = now_state_pos/vel を計算 -- */
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
  now_state_pos.point.x = transformedPoint.point.x;
  now_state_pos.point.y = transformedPoint.point.y;
  now_state_pos.point.z = transformedPoint.point.z;
  dummy_state_pos.point.x = transformedPoint.point.x;
  dummy_state_pos.point.y = transformedPoint.point.y;
  dummy_state_pos.point.z = transformedPoint.point.z;
}

/* -- カメラ->BODYの座標変換 -- */
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

/* -- 状態の管理 + パラメタのリセット -- */
void ObjectTrajectoryEstimator::stateManager() {
  // 動作開始の判定
  if (ready_flag) {
    if (now_state.pos.z >= -0.60) {
      // ボールをセットしたと判定
      ballSet_flag = true;
    } else if (ballSet_flag && now_state.pos.z < -0.30) {
      // セットした後にある程度ボール位置が落ちてきたら動作が始まったと判定
      ballSet_flag = false;
      ready_flag = false;
    }
  } else {
    if (now_state.vel.z > 0) {
      // ボールの速度が正のときは常に予測する
      predict_flag = true;
      pred_state.fb_flag.data = true;
    } else if (now_state.vel.z <= 0) {
      // ボールの速度が0以下のときは予測しない
      predict_flag = false;
      pred_state.fb_flag.data = false;

      // rlsの共分散行列初期化 <- 次も同じくらいだと信じて行わないことにする
      
      // 時間のリセット
      current_time = 0.0;
    }
  }
}

void ObjectTrajectoryEstimator::calcPredState() {
  if (loop_init) current_time += dt;
  loop_init = true;

  for (int i=0;i<timeVector.size();i++) {
    timeVector(i) = pow(current_time, i);
  }

  // 係数の計算
  std::vector<double> tmp_now_state = {now_state.pos.x, now_state.pos.y, now_state.pos.z};
  rls.update(timeVector, tmp_now_state);
  
  // 予測値の計算(最高到達点ver)
  rls.calcVertex();
  pred_state.target_tm = rls.vertexTime - current_time;
  pred_state.pos.x = rls.getVertex()[0];
  pred_state.pos.y = rls.getVertex()[1];
  pred_state.pos.z = rls.getVertex()[2];
  
  // todo: 速度
  
  // 確認用
  pred_state_pos.point.x = rls.getVertex()[0];
  pred_state_pos.point.y = rls.getVertex()[1];
  pred_state_pos.point.z = rls.getVertex()[2];
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
