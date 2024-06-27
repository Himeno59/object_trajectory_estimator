#ifndef LEAST_SQUARE_HPP
#define LEAST_SQUARE_HPP

#include "eigen3/Eigen/Dense"
#include "iostream"
#include "vector"

class LeastSquare
{
private:
  Eigen::VectorXd theta; // パラメータベクトル(求める値)
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  int degree; // k次の多項式モデルでフィッティング
        
public:
  LeastSquare(int k);
  void calcTheta(const std::vector<double>& x, std::vector<double>& y);
  Eigen::VectorXd getParameters() const;
};

#endif
