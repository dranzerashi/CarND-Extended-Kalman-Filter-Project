#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
	// Predict the new estimate
  x_=F_*x_;
	MatrixXd Ft=F_.transpose();
	//Predict the new uncertainity covariance matrix
	P_=F_*P_*Ft+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Update the state for Laser by using the given Kalman Filter Equations
	VectorXd y=z-(H_*x_);
	MatrixXd Ht=H_.transpose();
	MatrixXd S=H_*P_*Ht+R_;
	MatrixXd Si=S.inverse();
	MatrixXd PHt=P_*Ht;
	MatrixXd K=PHt*Si;

	//new estimate
	x_=x_+(K*y);
	long x_size = x_.size();
	MatrixXd I=MatrixXd::Identity(x_size, x_size);
	P_=(I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Update the state for RADAR by using the given Kalman Filter Equations
	
	float px=x_(0);
	float py=x_(1);
	float vx=x_(2);
	float vy=x_(3);
	
	
	float rho=sqrt(px*px+py*py);
	// If rho is too low increment it to avoid division by zero
	rho = (rho < 0.00001) ? 0.00001 : rho;
	
	
	float phi=atan2(py, px);
	float radial_velocity=(vx*px+vy*py)/rho;
	
	VectorXd H_of_x(3);
	H_of_x<< rho,phi,radial_velocity;
	
	VectorXd y=z-H_of_x;
	
	//normalize phi so that its angle is between -pi and pi
	if(y(1) < -M_PI){
		y(1)+= (2*M_PI);
	}
	else if(y(1) > M_PI){
		y(1)-= (2*M_PI);
	}
	MatrixXd Ht=H_.transpose();
	MatrixXd S=H_*P_*Ht+R_;
	MatrixXd Si=S.inverse();
	MatrixXd PHt=P_*Ht;
	MatrixXd K=PHt*Si;

	//new estimate
	x_=x_+(K*y);
	long x_size = x_.size();
	MatrixXd I=MatrixXd::Identity(x_size, x_size);
	P_=(I-K*H_)*P_;
}
