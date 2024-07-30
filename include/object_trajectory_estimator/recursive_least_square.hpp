#ifndef RECURSIVE_LEAST_SQUARE_HPP
#define RECURSIVE_LEAST_SQUARE_HPP

#include "eigen3/Eigen/Dense"
#include "vector"


class RLS
{
public:
  RLS(int k, std::vector<double> new_theta);
  void update(const Eigen::VectorXd& x, double y); // x = [1, x_n, ~ , x_n^k].T, y = y_n 
  double predict(double target_time);
  void reset();
  
public:
  Eigen::VectorXd getParameters() const;
  bool setParameters(std::vector<double>& new_theta);
  
public:
  int degree;                      // k次の多項式モデルでフィッティング
  double lambda;                   // 忘却係数
  std::vector<double> init_theta;  // 理想的な軌道のパラメタ
  Eigen::VectorXd theta;           // パラメータベクトル(求める値) todo: eigenかstd::vectorどっちかに揃える
  double predictValue;             // 予測値
  
private:
  Eigen::MatrixXd P;               // 共分散行列
};

/* -------------------------------------------------------------------------------------------- */

class RLS3D
{
public:
  RLS3D(int k_x, int k_y, int k_z,
	std::vector<double> x_new_theta, std::vector<double> y_new_theta, std::vector<double> z_new_theta);
  void update(const Eigen::VectorXd& x, std::vector<double> y);
  void calcVertex();
  std::vector<double> getVertex() const;
  void reset();

public:
  std::vector<RLS> rls3d;
  double vertexTime;
  std::vector<double> vertexPoint;
  
};

#endif
