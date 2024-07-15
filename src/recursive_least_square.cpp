#include "object_trajectory_estimator/recursive_least_square.hpp"

#include "limits"
#include "iostream"

#ifndef GRAVITY
#define GRAVITY 9.80665 
#endif

/* ---------- RLS ---------- */
RLS::RLS(int k) : degree(k), lambda(0.99) 
{
  if (degree == 2) {
    theta = Eigen::VectorXd(3);
    theta << -0.855, 4.3, -0.5*GRAVITY;
  } else if (degree == 1) {
    theta = Eigen::VectorXd(2);
    theta << 0.0, 0.0;
  }
  P = Eigen::MatrixXd::Identity(degree+1, degree+1) * 2; // 最初は少し早く反応させる
}

void RLS::update(const Eigen::VectorXd& x, double y) {
  // 観測値と予測値の誤差
  double e = y - x.dot(theta);
  // ゲインのv計算
  Eigen::VectorXd K = P * x / (lambda + x.transpose() * P * x);
  Eigen::VectorXd prev_theta = theta;
  // パラメータの更新
  theta += K * e;
  // thetaにnanなどが含まれていた場合、直前の値を入れる
  if (theta.hasNaN() || (theta.array() == std::numeric_limits<double>::infinity()).any() || (theta.array() == -std::numeric_limits<double>::infinity()).any()) {
    theta = prev_theta;
  } else {
    // 含まれていなければ共分散行列を更新する
    P = (Eigen::MatrixXd::Identity(theta.size(), theta.size()) - K * x.transpose()) * P / lambda;
  }
}

Eigen::VectorXd RLS::getParameters() const {
  return theta;
}

void RLS::setParameters(double theta0, double theta1, double theta2) {
  if (theta.size() == 3) {
    theta[0] = theta0;
    theta[1] = theta1;
    theta[2] = theta2;
  }
}

double RLS::predict(double target_time) {
  Eigen::VectorXd time_vector(degree+1);
  for (int i=0;i<degree+1;i++) {
    time_vector(i) = std::pow(target_time, i); 
  }
  predictValue = theta.dot(time_vector);
  return predictValue;
}

void RLS::reset() {
  // 共分散行列の初期化
  P = P*1.1;
}

void RLS::setInitialTheta(const Eigen::VectorXd& initial_theta) {
  theta = initial_theta;
}

/* ---------- RLS3D ---------- */
RLS3D::RLS3D(int k_x, int k_y, int k_z) : rls3d{RLS(k_x), RLS(k_y), RLS(k_z)} {
  vertexPoint.resize(3);
}

void RLS3D::update(const Eigen::VectorXd& x, std::vector<double> y) {
  for (int i=0;i<rls3d.size();i++) {
    Eigen::VectorXd tmp_x = x;
    tmp_x.conservativeResize(rls3d[i].degree+1);
    rls3d[i].update(tmp_x, y[i]);
  }
}

void RLS3D::calcVertex() {
  vertexTime = - rls3d[2].theta(1)/(2*rls3d[2].theta(2));
  for (int i=0;i<rls3d.size();i++) {
    vertexPoint[i] = rls3d[i].predict(vertexTime);
  }
}

std::vector<double> RLS3D::getVertex() {
  return vertexPoint;
}
