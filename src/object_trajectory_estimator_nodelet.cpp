#include "object_trajectory_estimator/object_trajectory_estimator_nodelet.h"

#include "numeric" 

namespace object_trajectory_estimator {

ObjectTrajectoryEstimator::ObjectTrajectoryEstimator()
  : tfListener(tfBuffer) {}

void ObjectTrajectoryEstimator::onInit() {
  // node handler
  nh = getNodeHandle();
  pnh = getPrivateNodeHandle();  
  // srv
  setService = nh.advertiseService("/ObjectTrajectoryEstimator/set_rls_parameters", &ObjectTrajectoryEstimator::setRLSParameters, this);
  getService = nh.advertiseService("/ObjectTrajectoryEstimator/get_rls_parameters", &ObjectTrajectoryEstimator::getRLSParameters, this);
  // subscriber
  point_sub = pnh.subscribe("point_input", 1, &ObjectTrajectoryEstimator::callback, this);
  // publisher
  current_state_pub = pnh.advertise<object_trajectory_estimator::BallStateStamped>("current_state_output", 1);
  pred_state_pub = pnh.advertise<object_trajectory_estimator::BallStateStamped>("pred_state_output", 1);
  // rosparam
  pnh.getParam("frame_id", frame_id);
  pnh.getParam("camera_frame", camera_frame);
  pnh.getParam("base_frame", base_frame);
  pnh.getParam("init_thr", init_thr);
  pnh.getParam("start_thr", start_thr);
  pnh.getParam("bound_thr", bound_thr);
  pnh.getParam("pred_time", pred_time);
  pnh.getParam("wait_fb", wait_fb);
  pnh.getParam("x_init_theta", x_init_theta);
  pnh.getParam("y_init_theta", y_init_theta);
  pnh.getParam("z_init_theta", z_init_theta);  
  // header
  current_state.header.frame_id = base_frame;
  pred_state.header.frame_id = base_frame;

  // rls
  rls = RLS3D(1, 1, 1, x_init_theta, y_init_theta, z_init_theta);
  
  // flag
  pred_state.fb_flag.data = false;
  ready_flag = true;
  ballSet_flag = false;
  loop_init = false;
  predict_flag = false;
  theta_initialize = true;
  
  
  current_time = 0.0;
  rls_degree = 1; 
  timeVector = Eigen::VectorXd::Zero(rls_degree+1); // [1, t]
  window_size = 3;
}

void ObjectTrajectoryEstimator::callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  // stampの更新 & dtの計算
  updateStamp(msg);
  
  // 現在の(BODY相対の)objの状態 = current_state_pos/vel を計算
  calcCurrentState(msg);
  
  // 状態の管理 + パラメタのリセット
  stateManager();

  // 予測フラグが立っている場合、予測する
  if (predict_flag) calcPredState();

  // 値をpublish
  publish();
  
  // 値の保持
  prev_state = current_state;
}

void ObjectTrajectoryEstimator::publish() {
  current_state_pub.publish(current_state);
  pred_state_pub.publish(pred_state);
}

void ObjectTrajectoryEstimator::updateStamp(const geometry_msgs::PointStamped::ConstPtr &msg) {
  current_state.header.stamp = msg->header.stamp;
  pred_state.header.stamp = msg->header.stamp;
  dt = (current_state.header.stamp - prev_state.header.stamp).toSec();
}

void ObjectTrajectoryEstimator::stateManager() {
  // 動作開始の判定
  if (ready_flag) {
    if (current_state.pos.z >= init_thr) {
      // ボールをセットしたと判定
      ballSet_flag = true;
    } else if (ballSet_flag && current_state.pos.z < start_thr) {
      // セットした後にある程度ボール位置が落ちてきたら動作が始まったと判定
      ballSet_flag = false;
      ready_flag = false;
    }
  } else {
    // 速度ver1
    if (current_state.vel.z > 0) {
      predict_flag = true;
    } else if (current_state.vel.z <= 0) {
      predict_flag = false;
      current_time = 0.0;
      // rlsの共分散行列初期化
      rls.reset();
    }

    // 速度ver2
    if (current_state.vel.z > 0 && prev_state.vel.z > 0) {
      predict_flag = true;
    } else if (current_state.vel.z > 0 && prev_state.vel.z <= 0) {
      predict_flag = false;
      current_time += dt; // リセットしない
    } else if (current_state.vel.z <= 0 && prev_state.vel.z > 0) {
      predict_flag = false;
      current_time += dt; // リセットしない
    } else if (current_state.vel.z <= 0 && prev_state.vel.z <= 0) {
      predict_flag = false;
      current_time = 0.0; // リセットしてok
      // rlsの共分散行列初期化
      // rls.reset();
      rls.rls3d[2].reset();

      // 次の目標軌道
      // setParamters()~~
    } 

    // // 高さ+速度ver
    // if (current_state.pos.z > bound_thr) {
    //   if (current_state.vel.z > 0 && prev_state.vel.z > 0) {
    // 	predict_flag = true;
    //   } else if (current_state.vel.z > 0 && prev_state.vel.z <= 0) {
    // 	predict_flag = false;
    // 	current_time += dt; // リセットしない
    //   } else if (current_state.vel.z <= 0 && prev_state.vel.z > 0) {
    // 	predict_flag = false;
    // 	current_time += dt; // リセットしない
    //   } else if (current_state.vel.z <= 0 && prev_state.vel.z <= 0) {
    // 	predict_flag = false;
    // 	current_time = 0.0; // リセットしてok
    //   }
    // } else if (current_state.vel.z <= bound_thr) {
    //   predict_flag = false;
    //   current_time = 0.0;

    //   // rlsの共分散行列初期化
    //   // rls.reset();
    //   rls.rls3d[2].reset();
    // }

    pred_state.fb_flag.data = predict_flag;
  }
}

void ObjectTrajectoryEstimator::calcCurrentState(const geometry_msgs::PointStamped::ConstPtr &msg) {
  // 平滑化
  geometry_msgs::PointStamped filteredPoint = applyFilter(msg);
  // 位置
  geometry_msgs::PointStamped transformedPoint = transformPoint(tfBuffer, filteredPoint);
  current_state.pos.x = transformedPoint.point.x;
  current_state.pos.y = transformedPoint.point.y;
  current_state.pos.z = transformedPoint.point.z;
  // 速度
  current_state.vel.x = (current_state.pos.x - prev_state.pos.x) / dt;
  current_state.vel.y = (current_state.pos.y - prev_state.pos.y) / dt;
  current_state.vel.z = (current_state.pos.z - prev_state.pos.z) / dt;
}

void ObjectTrajectoryEstimator::calcPredState() {
  if (loop_init) current_time += dt;
  loop_init = true;

  for (int i=0;i<timeVector.size();i++) {
    timeVector(i) = pow(current_time, i);
  }

  // 以下の式の右辺の係数を推定する
  // x             = vx_0*t + x_0
  // y             = vy_0*t + y_0
  // z + 0.5*g*t^2 = vz_0*t + z_0

  std::vector<double> tmp_current_state = {current_state.pos.x, current_state.pos.y, current_state.pos.z - (-0.5*GRAVITY*pow(current_time, 2))};
  rls.update(timeVector, tmp_current_state);
  
  // 予測値の計算(最高到達点ver)
  rls.calcVertex();
  pred_state.target_tm = rls.vertexTime - current_time;
  pred_state.pos.x = rls.getVertex()[0];
  pred_state.pos.y = rls.getVertex()[1];
  pred_state.pos.z = std::min(rls.getVertex()[2] - 0.5*GRAVITY*pow(rls.vertexTime, 2), 1.0); // 1.0以下に抑える
  if (current_time > wait_fb) {
    pred_state.pos.x = rls.getVertex()[0];
    pred_state.pos.y = rls.getVertex()[1];
    pred_state.pos.z = std::min(rls.getVertex()[2] - 0.5*GRAVITY*pow(rls.vertexTime, 2), 1.0); // 1.0以下に抑える
  }

  // pred_state.fb_flagを上書く
  if (current_time > wait_fb) {
    pred_state.fb_flag.data = true;
  } else {
    pred_state.fb_flag.data = false;
  }
  
  // todo: 速度
}

void ObjectTrajectoryEstimator::calcInitState() {
  init_vel.push_back(current_state.vel.z);
  if (init_vel.size() == 3) {
    std::vector<double> new_theta = {current_state.pos.z, std::accumulate(init_vel.begin(), init_vel.end(), 0.0) / 3.0};
    rls.rls3d[2].setParameters(new_theta);
    theta_initialize = false;
  }
  // x,yもここで計算するようにする
}

geometry_msgs::PointStamped ObjectTrajectoryEstimator::applyFilter(const geometry_msgs::PointStamped::ConstPtr &msg) {
  // windowへの追加
  Eigen::Vector3d pos(msg->point.x, msg->point.y, msg->point.z);
  window.push_back(pos);
  if (window.size() > window_size) window.erase(window.begin());

  // median filter
  std::vector<double> median_value = std::vector<double>(3); // xyz
  for (int i=0;i<3;i++) {
    std::vector<double> tmp_data;
    for (int j=0;j<window.size();j++) {
      tmp_data.push_back(window[j](i));
    }
    std::sort(tmp_data.begin(), tmp_data.end());
    size_t median_index = window.size() / 2;
    median_value[i] = tmp_data[median_index];
  }
  geometry_msgs::PointStamped filteredPoint;
  filteredPoint.point.x = median_value[0];
  filteredPoint.point.y = median_value[1];
  filteredPoint.point.z = median_value[2];
  return filteredPoint;
}

geometry_msgs::PointStamped ObjectTrajectoryEstimator::transformPoint(const tf2_ros::Buffer &tfBuffer, const geometry_msgs::PointStamped &filteredPoint) {
  // 半球殻の重心->球の重心
  Eigen::Vector3d pos(filteredPoint.point.x, filteredPoint.point.y, filteredPoint.point.z);
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

bool ObjectTrajectoryEstimator::setRLSParameters(object_trajectory_estimator::SetRLSParameters::Request &req,
						 object_trajectory_estimator::SetRLSParameters::Response &res) {
  std::vector<double> params(req.params.begin(), req.params.end());
  bool success = rls.rls3d[2].setParameters(params);
  res.success = success;
  return true;
}

bool ObjectTrajectoryEstimator::getRLSParameters(object_trajectory_estimator::GetRLSParameters::Request &req,
						 object_trajectory_estimator::GetRLSParameters::Response &res) {
  res.params[0] = rls.rls3d[2].getParameters()[0];
  res.params[1] = rls.rls3d[2].getParameters()[1];
  return true;
}

} // namespace object_trajectory_estimator

// Register the nodelet
#ifdef USE_PLUGINLIB_CLASS_LIST_MACROS_H
#include <pluginlib/class_list_macros.h>
#else
#include <pluginlib/class_list_macros.hpp>
#endif
PLUGINLIB_EXPORT_CLASS(object_trajectory_estimator::ObjectTrajectoryEstimator, nodelet::Nodelet);
