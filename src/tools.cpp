#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}
Tools::~Tools() {}

bool Tools::InputIsValid(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth)
{
	return estimations.size() == ground_truth.size()
		&& estimations.size() != 0;
}

void CalculateResidual(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth, VectorXd& rmse)
{
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array()*residual.array();
		rmse += residual;
	}
}

void CalculateMean(const vector<VectorXd>& estimations, VectorXd& rmse)
{
	rmse = rmse / estimations.size();
}

void CalculateSqrt(VectorXd& rmse)
{
	rmse = rmse.array().sqrt();
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if (!InputIsValid(estimations, ground_truth)) {
		std::cout << "Invalid estimation or ground_truth data." << std::endl;
		return rmse;
	}

	CalculateResidual(estimations, ground_truth, rmse);
	CalculateMean(estimations, rmse);
	CalculateSqrt(rmse);
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
	MatrixXd Hj(3, 4);

	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	auto c1 = px*px + py*py;
	auto c2 = sqrt(c1);
	auto c3 = (c1*c2);

	if (fabs(c1) < 0.0001) {
		std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
		return Hj;
	}

	Hj << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

	return Hj;
}
