#include "object_trajectory_estimator/recursive_least_square.hpp"

#include "limits"
#include "iostream"

#ifndef GRAVITY
#define GRAVITY 9.80665 
#endif

/* ---------- RLS ---------- */
RLS::RLS() {}

RLS::RLS(int k, std::vector<double> new_theta) : degree(k), lambda(1.0) 
{
  theta.resize(k+1);
  P = Eigen::MatrixXd::Identity(degree+1, degree+1) * 1e2;

  // // P = (A^t*A)^-1
  // // n=1は逆行列なしでn=2はある(それ以降もあるはず)
  // // n=2のときから計算を始めることにする
  // // A << 1 t1
  // //      1 t2
  // // t1 = 0, t2 = 1/90 が期待される
  // P = Eigen::MatrixXd::Zero(degree+1, degree+1);
  // P << 1.0,    -90.0,
  //      -90.0, 2.0*(90.0*90.0);
  
  init_theta = new_theta;
  setParameters(init_theta);
}

void RLS::update(const Eigen::VectorXd& x, double y) {
  // 観測値と予測値の誤差
  double e = y - x.dot(theta);
  
  // ゲインのv計算
  Eigen::VectorXd K = P * x / (lambda + x.transpose() * P * x);
  Eigen::VectorXd tmp_theta = theta;
  
  // パラメータの更新
  theta += K * e;
  
  // thetaにnanなどが含まれていた場合の処理 -> 直前の値を入れる
  if (theta.hasNaN() || (theta.array() == std::numeric_limits<double>::infinity()).any() || (theta.array() == -std::numeric_limits<double>::infinity()).any()) {
    theta = tmp_theta;
  } else {
    // 含まれていなければ共分散行列を更新する
    P = (Eigen::MatrixXd::Identity(theta.size(), theta.size()) - K * x.transpose()) * P / lambda;
  }
}

Eigen::VectorXd RLS::getParameters() const {
  return theta;
}

bool RLS::setParameters(std::vector<double>& new_theta) {
  if (theta.size() == new_theta.size()) {
    for (int i=0;i<theta.size();i++) {
      theta[i] = new_theta[i];
    }
    return true;
  } else {
    std::cout << "not match degree" << std::endl;
    return false;
  }
}

double RLS::predict(double target_time) {
  Eigen::VectorXd time_vector(degree+1);
  for (int i=0;i<degree+1;i++) {
    time_vector(i) = std::pow(target_time, i); 
  }
  predictValue = time_vector.dot(theta);
  return predictValue;
}

void RLS::reset() {
  // 共分散行列の初期化
  // 係数も毎回初期化?
  // std::cerr << "reset" << std::endl;
  // P << 1.0,    -90.0,
  //      -90.0, 2.0*(90.0*90.0);
  P = Eigen::MatrixXd::Identity(degree+1, degree+1) * 1e2;
  
  // setParameters(init_theta);
}

/* ---------- RLS3D ---------- */
RLS3D::RLS3D() {}

RLS3D::RLS3D(int k_x, int k_y, int k_z,
	     std::vector<double> x_new_theta, std::vector<double> y_new_theta, std::vector<double> z_new_theta)
  : rls3d{RLS(k_x, x_new_theta), RLS(k_y, y_new_theta), RLS(k_z, z_new_theta)}
{
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
  vertexTime = rls3d[2].theta(1) / GRAVITY;
  for (int i=0;i<rls3d.size();i++) {
    vertexPoint[i] = rls3d[i].predict(vertexTime);
  }
}

std::vector<double> RLS3D::getVertex() const {
  return vertexPoint;
}

void RLS3D::reset() {
  for (int i=0;i<rls3d.size();i++) {
    rls3d[i].reset();
  }
}
