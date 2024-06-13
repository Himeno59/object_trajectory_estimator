#include "basketball_motion/least_square.hpp"

LeastSquare::LeastSquare(int k) {
  // k次の多項式モデルでフィッティング
  degree = k;
  theta = Eigen::VectorXd::Zero(degree+1);
  A = Eigen::MatrixXd::Zero(3, degree+1);
  b = Eigen::VectorXd::Zero(3);
}

void LeastSquare::calcTheta(const std::vector<double>& x, std::vector<double>& y) {
  int data_size = x.size();
  
  A.resize(data_size, degree+1);
  b.resize(data_size);
  
  for (int i = 0; i < data_size; i++) {
    b(i) = y[i];
    for (int j = 0; j < degree+1; j++){
      A(i, j) = std::pow(x[i], j);
    }
  }
  
  theta = A.colPivHouseholderQr().solve(b);
  std::cout << "theta: " << theta << std::endl;
}

Eigen::VectorXd LeastSquare::getParameters () const {
  return theta;
}
