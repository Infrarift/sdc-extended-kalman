#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:

  Tools();
  virtual ~Tools();

	static bool InputIsValid(const std::vector<Eigen::VectorXd>& estimations, const std::vector<Eigen::VectorXd>& ground_truth);
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

};

#endif /* TOOLS_H_ */
