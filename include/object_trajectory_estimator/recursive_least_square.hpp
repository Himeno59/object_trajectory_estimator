#ifndef RECURSIVE_LEAST_SQUARE_HPP
#define RECURSIVE_LEAST_SQUARE_HPP

#include "eigen3/Eigen/Dense"
#include "vector"

class RLS
{
public:
  RLS(int k);
  void setInitialTheta(const Eigen::VectorXd& initial_theta) ;
  void update(const Eigen::VectorXd& x, double y); // x = [1, x_n, ~ , x_n^k].T, y = y_n
  Eigen::VectorXd getParameters() const;
  double predict(double target_time);
  void reset();

  void setParameters(double theta0, double theta1, double theta2);  // srvでsetする用

public:
  int degree;                       // k次の多項式モデルでフィッティング
  double lambda;                    // 忘却係数
  Eigen::VectorXd theta;            // パラメータベクトル(求める値)
  double predictValue;              // 予測値
  
private:
  Eigen::MatrixXd P;                // 共分散行列
};

class RLS3D
{
public:
  RLS3D(int k_x, int k_y, int k_z);
  void update(const Eigen::VectorXd& x, std::vector<double> y);
  void calcVertex();
  std::vector<double> getVertex();

public:
  double vertexTime;
  std::vector<double> vertexPoint;
  std::vector<RLS> rls3d;
};

#endif
