#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF()
{
	is_initialized_ = false;

	previous_timestamp_ = 0;
	delta_time_ = 0;

	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	R_laser_ << 0.0225 , 0 ,
				0 , 0.0225;

	R_radar_ << 0.09 , 0 , 0 ,
				0 , 0.0009 , 0 ,
				0 , 0 , 0.09;

	H_laser_ << 1, 0, 0, 0,
			    0, 1, 0, 0;

	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ <<  1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;
}

FusionEKF::~FusionEKF()
{
}

void FusionEKF::ProcessTime(const MeasurementPackage& measurement_pack)
{
	delta_time_ = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
}

void FusionEKF::UpdateRadar(const MeasurementPackage& measurement_pack)
{
	Hj_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.H_ = Hj_;
	ekf_.R_ = R_radar_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
}

void FusionEKF::UpdateLidar(const MeasurementPackage& measurement_pack)
{
	ekf_.H_ = H_laser_;
	ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
}

void FusionEKF::InitializeRadar(const MeasurementPackage& measurement_pack)
{
	auto rho = measurement_pack.raw_measurements_(0);
	auto phi = measurement_pack.raw_measurements_(1);
	auto px = rho * cos(phi);
	auto py = rho * sin(phi);
	ekf_.x_ << px, py, 0, 0;
}

void FusionEKF::InitializeLidar(const MeasurementPackage& measurement_pack)
{
	ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
}

void FusionEKF::InitializeSensor(const MeasurementPackage& measurement_pack)
{
	cout << "EKF: " << endl;
	ekf_.x_ = VectorXd(4);

	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
		InitializeRadar(measurement_pack);
	else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
		InitializeLidar(measurement_pack);

	is_initialized_ = true;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
	ProcessTime(measurement_pack);

	if (!is_initialized_)
	{
		InitializeSensor(measurement_pack);
		return;
	}

	ekf_.Predict(delta_time_);

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
		UpdateRadar(measurement_pack);
	else
		UpdateLidar(measurement_pack);

	cout << "x_ = " << endl << ekf_.x_ << endl;
	cout << "P_ = " << endl << ekf_.P_ << endl;
}
