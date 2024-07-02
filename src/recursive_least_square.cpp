#include "object_trajectory_estimatorrecursive_least_square.hpp"

#include "limits"

#ifndef GRAVITY
#define GRAVITY 9.80665 
#endif

RecursiveLS::RecursiveLS(int k) {
  // k次の多項式モデルでフィッティング
  degree = k;
  // パラメータベクトルの初期化
  if (degree == 2) {
    theta = Eigen::VectorXd(3);
    theta << -0.855, 4.3, -0.5*GRAVITY;
  } else {
    theta = Eigen::VectorXd(2);
    theta << 0.0, 0.0;
  }
  
  // 共分散行列の初期化
  P = Eigen::MatrixXd::Identity(degree+1, degree+1) * 2; // 最初は少し早く反応させる
  // 忘却係数の設定
  lambda = 0.99;
  // 予測値
  predict_value = 0.0;
  // 予測値計算用の時間のリスト
  time_vector = Eigen::VectorXd::Zero(degree+1);

  // 予想最高到達点
  if (degree == 2) {
    predict_contact_value = theta[0] - std::pow(theta[1], 2) / (4 * theta[2]);
  } else if (degree == 1) {
    predict_contact_value = 1.0;
  } 
    
}

void RecursiveLS::setInitialTheta(const Eigen::VectorXd& initial_theta) {
  theta = initial_theta;
}

void RecursiveLS::update(const Eigen::VectorXd& x, double y) {
  // 観測値と予測値の誤差
  double e = y - x.dot(theta); // x.transpose()*thetaより速い
  // ゲインのv計算
  Eigen::VectorXd K = P * x / (lambda + x.transpose() * P * x);

  Eigen::VectorXd prev_theta = theta;
  // パラメータの更新
  theta += K * e;

  // thetaにnanなどが含まれていた場合、直前の値を入れる
  if (theta.hasNaN() || (theta.array() == std::numeric_limits<double>::infinity()).any() || (theta.array() == -std::numeric_limits<double>::infinity()).any()) {
    theta = prev_theta;
  } else {
    // 含まれていなければ更新する
    // 共分散行列の更新
    P = (Eigen::MatrixXd::Identity(theta.size(), theta.size()) - K * x.transpose()) * P / lambda;
  }
}

Eigen::VectorXd RecursiveLS::getParameters() const {
  return theta;
}

double RecursiveLS::predict(double target_time) {
  for (int i = 0; i <= degree; ++i) {
    time_vector(i) = std::pow(target_time, i); 
  }
  predict_value = theta.dot(time_vector);
  return predict_value;
}

std::vector<double> RecursiveLS::calcVertex() {
  if (theta.size() == 3) {
    
    predict_contact_value = theta[0] - std::pow(theta[1], 2) / (4 * theta[2]);
  } else if (theta.size() == 2) {
    predict_contact_value = 1.0;
  }
  return predict_contact_value;
}

void RecursiveLS::reset() {
  // 共分散行列の初期化
  P = P*1.1;
}
