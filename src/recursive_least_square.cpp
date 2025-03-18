#include "bouncing_ball_estimator/recursive_least_square.hpp"

#include <limits>
#include <iostream>

RLS::RLS(int k, const std::vector<double>& new_theta, double lambda) : degree(k), lambda(lambda) 
{
  theta.resize(k+1);
  setParameters(new_theta);  
  P = Eigen::MatrixXd::Identity(degree+1, degree+1)*1e2; 
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
    std::cerr << "can't update" << std::endl;
  } else {
    // 含まれていなければ共分散行列を更新する
    P = (Eigen::MatrixXd::Identity(theta.size(), theta.size()) - K * x.transpose()) * P / lambda;
  }
}

Eigen::VectorXd RLS::getParameters() const {
  return theta;
}

bool RLS::setParameters(const std::vector<double>& new_theta) {
  for (int i=0;i<new_theta.size();i++) theta(i) = new_theta[i];
}

bool RLS::setMatrix(Eigen::MatrixXd& matrix) {
  if (matrix.rows() == P.rows() && matrix.cols() == P.cols()) {
    setP = matrix;
    P = matrix;
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
  return time_vector.dot(theta);
}

void RLS::reset() {
  setMatrix(setP); // 共分散行列の初期化
}

RLS3D::RLS3D(int k_x, int k_y, int k_z,
             const std::vector<double>& x_new_theta, const std::vector<double>& y_new_theta, const std::vector<double>& z_new_theta,
             double lambda)
  : rls3d{RLS(k_x, x_new_theta, lambda), RLS(k_y, y_new_theta, lambda), RLS(k_z, z_new_theta, lambda)}
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

bool RLS3D::setMatrix(Eigen::MatrixXd& matrix) {
  std::vector<bool> success_check = std::vector<bool>(3);
  for (int i=0;i<rls3d.size();i++) {
    success_check[i] = rls3d[i].setMatrix(matrix);
  }
  if (success_check[0] == true && success_check[1] == true && success_check[2] == true) {
    return true;
  } else {
    return false;
  }
}
