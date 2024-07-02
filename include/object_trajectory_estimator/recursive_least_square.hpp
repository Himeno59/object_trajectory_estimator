#ifndef RECURSIVE_LEAST_SQUARE_HPP
#define RECURSIVE_LEAST_SQUARE_HPP

#include "eigen3/Eigen/Dense"
#include "iostream"
#include "vector"

class RecursiveLS
{
private:
  int degree;                       // k次の多項式モデルでフィッティング
  Eigen::VectorXd theta;            // パラメータベクトル(求める値)
  Eigen::MatrixXd P;                // 共分散行列
  double lambda;                    // 忘却係数
  double predict_value;             // 予測値
  std::vector<double> vertexPoint;  // 頂点の座標 (time, value) 
  Eigen::VectorXd time_vector;      // 予測値計算用の時間のリスト
  
public:
  RecursiveLS(int k);
  // void setInitialTheta(const std::vector<double>& initial_theta);
  void setInitialTheta(const Eigen::VectorXd& initial_theta) ;
  void update(const Eigen::VectorXd& x, double y); // x = [1, x_n, ~ , x_n^k].T, y = y_n
  Eigen::VectorXd getParameters() const;
  double predict(double target_time);
  std::vector<double> calcVertex();
  void reset();
  
};

#endif
