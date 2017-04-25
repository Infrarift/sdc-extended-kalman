#include "kalman_filter.h"
#include <valarray>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init(VectorXd& x_in, MatrixXd& P_in, MatrixXd& F_in,
                        MatrixXd& H_in, MatrixXd& R_in, MatrixXd& Q_in)
{
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::UpdateStateTransitionMatrix(float delta_time)
{
	F_(0, 2) = delta_time;
	F_(1, 3) = delta_time;
}

void KalmanFilter::UpdateNoiseCovariance(float deltaTime, float noise_ax, float noise_ay)
{
	MatrixXd G = MatrixXd(4, 2);
	double dt2 = deltaTime * deltaTime;
	G << dt2 / 2 , 0 ,
		0 , dt2 / 2 ,
		deltaTime , 0 ,
		0 , deltaTime;

	MatrixXd Qn = MatrixXd(2, 2);
	Qn << noise_ax , 0 ,
		0 , noise_ay;

	Q_ = G * Qn * G.transpose();
}

void KalmanFilter::Predict(float deltaTime)
{
	UpdateStateTransitionMatrix(deltaTime);
	UpdateNoiseCovariance(deltaTime, 9, 9);
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd& z)
{
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	InternalUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd& z)
{
	VectorXd y = z - ConvertToPolar(x_);
	y(1) = NormalizeAngle(y(1));
	InternalUpdate(y);
}

double KalmanFilter::NormalizeAngle(double theta)
{
	while (theta > M_PI || theta < -M_PI)
	{
		if (theta > M_PI)
			theta -= M_PI * 2;

		if (theta < -M_PI)
			theta += M_PI * 2;
	}

	return theta;
}

VectorXd KalmanFilter::ConvertToPolar(VectorXd& x)
{
	VectorXd x_asPolar = VectorXd(3);
	double px = x_(0);
	double py = x_(1);
	double vx = x_(2);
	double vy = x_(3);

	x_asPolar(0) = std::sqrt(px * px + py * py);
	x_asPolar(1) = std::atan2(py, px);
	x_asPolar(2) = (px * vx + py * vy) / x_asPolar(0);

	return x_asPolar;
}

void KalmanFilter::InternalUpdate(VectorXd y)
{
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	x_ = x_ + K * y;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
