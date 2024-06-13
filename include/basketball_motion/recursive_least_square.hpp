#ifndef RECURSIVE_LEAST_SQUARE_HPP
#define RECURSIVE_LEAST_SQUARE_HPP

#include "eigen3/Eigen/Dense"
#include "iostream"
#include "vector"

class RecursiveLS
{
private:
  Eigen::MatrixXd P; // 共分散行列
  Eigen::VectorXd theta; // パラメータベクトル(求める値)
  double lambda; // 忘却係数
  double predict_value; // 予測値
  double predict_contact_value; // 予測したボールの最高到達点位置 
  int degree; // k次の多項式モデルでフィッティング
  Eigen::VectorXd time_vector; // 予測値計算用の時間のリスト
  
public:
  RecursiveLS(int k);
  // void setInitialTheta(const std::vector<double>& initial_theta);
  void setInitialTheta(const Eigen::VectorXd& initial_theta) ;
  void update(const Eigen::VectorXd& x, double y); // x = [1, x_n, ~ , x_n^k].T, y = y_n
  Eigen::VectorXd getParameters() const;
  double predict(double target_time);
  double predictContactPoint();
  void reset(); // 跳ね返ったタイミングでリセットする

  
};

#endif
