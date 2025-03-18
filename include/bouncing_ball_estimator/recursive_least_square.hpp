#ifndef RECURSIVE_LEAST_SQUARE_HPP
#define RECURSIVE_LEAST_SQUARE_HPP

#include <eigen3/Eigen/Dense>
#include <vector>

#ifndef GRAVITY
#define GRAVITY 9.80665 
#endif

class RLS
{
public:
  RLS(int k, const std::vector<double>& new_theta, double lambda = 1.0);
  
  void update(const Eigen::VectorXd& x, double y); // x = [1, x_n, ~ , x_n^k].T, y = y_n 
  double predict(double target_time);
  void reset();
  
public:
  Eigen::VectorXd getParameters() const;
  bool setParameters(const std::vector<double>& new_theta);
  bool setMatrix(Eigen::MatrixXd& matrix);
  
public:
  int degree;                      // k次の多項式モデルでフィッティング
  double lambda;                   // 忘却係数
  Eigen::VectorXd theta;           // パラメータベクトル(求める値)
  Eigen::MatrixXd setP;            // Pの初期値保存用
  Eigen::MatrixXd P;               // 共分散行列
};

////////////////////////////////////////////////////////////////////////////////////

class RLS3D
{
public:
  RLS3D() {};
  RLS3D(int k_x, int k_y, int k_z,
        const std::vector<double>& x_new_theta, const std::vector<double>& y_new_theta, const std::vector<double>& z_new_theta,
        double lambda);
  
  void update(const Eigen::VectorXd& x, std::vector<double> y);
  void calcVertex();
  std::vector<double> getVertex() const;
  void reset();
  bool setMatrix(Eigen::MatrixXd& matrix);

public:
  std::vector<RLS> rls3d;
  double vertexTime;
  std::vector<double> vertexPoint;  
};

#endif
