#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter
{
public:

	// state vector
	Eigen::VectorXd x_;

	// state covariance matrix
	Eigen::MatrixXd P_;

	// state transistion matrix
	Eigen::MatrixXd F_;

	// process covariance matrix
	Eigen::MatrixXd Q_;

	// measurement matrix
	Eigen::MatrixXd H_;

	// measurement covariance matrix
	Eigen::MatrixXd R_;

	KalmanFilter();
	virtual ~KalmanFilter();

	/**
	 * Init Initializes Kalman filter
	 * @param x_in Initial state
	 * @param P_in Initial state covariance
	 * @param F_in Transition matrix
	 * @param H_in Measurement matrix
	 * @param R_in Measurement covariance matrix
	 * @param Q_in Process covariance matrix
	 */
	void Init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in, Eigen::MatrixXd& F_in,
	          Eigen::MatrixXd& H_in, Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in);
	void Predict(float deltaTime);
	void Update(const Eigen::VectorXd& z);
	void UpdateEKF(const Eigen::VectorXd& z);
	

private:
	void UpdateStateTransitionMatrix(float delta_time);
	void UpdateNoiseCovariance(float deltaTime, float noise_ax, float noise_ay);
	Eigen::VectorXd ConvertToPolar(Eigen::VectorXd& x);
	void InternalUpdate(Eigen::VectorXd y);
	static double NormalizeAngle(double theta);
};

#endif /* KALMAN_FILTER_H_ */
