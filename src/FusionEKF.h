#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF
{
public:

	FusionEKF();
	virtual ~FusionEKF();
	void ProcessMeasurement(const MeasurementPackage& measurement_pack);
	KalmanFilter ekf_;

private:

	bool is_initialized_;
	long long previous_timestamp_;
	float delta_time_;

	void ProcessTime(const MeasurementPackage& measurement_pack);
	void UpdateRadar(const MeasurementPackage& measurement_pack);
	void UpdateLidar(const MeasurementPackage& measurement_pack);
	void InitializeRadar(const MeasurementPackage& measurement_pack);
	void InitializeLidar(const MeasurementPackage& measurement_pack);
	void InitializeSensor(const MeasurementPackage& measurement_pack);

	Tools tools;
	Eigen::MatrixXd R_laser_;
	Eigen::MatrixXd R_radar_;
	Eigen::MatrixXd H_laser_;
	Eigen::MatrixXd Hj_;
};

#endif /* FusionEKF_H_ */
