#include <stdlib.h>
#include <iostream>
#include <math.h>
#include "ekf.h"



using namespace ssr;


EKF4MobileRobot::EKF4MobileRobot(const Matrix33 &odomCovMat, const Matrix33 &imuCovMat) :
  estimatedPose(0,0,0), predictedPose(0,0,0), 
  estimatedCovarianceMatrix(1, 0, 0,
			    0, 1, 0,
			    0, 0, 1),
  predictedCovarianceMatrix(1, 0, 0,
			    0, 1, 0,
			    0, 0, 1),
  odometryCovarianceMatrix(odomCovMat), 
  imuCovarianceMatrix(imuCovMat) {
}


EKF4MobileRobot::~EKF4MobileRobot() {
}

static Matrix33 rotationMatrix(const double theta) {
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);
  return Matrix33(cosTheta, -sinTheta, 0,
	     sinTheta, cosTheta, 0,
	     0, 0, 1);
}

Pose2D EKF4MobileRobot::operator()(const Velocity2D& v_odom, const Velocity2D& v_acc, const double dt) {
  static Matrix33 eye(1, 0, 0, 0, 1, 0, 0, 0, 1);
  RowVector3 dox = (RowVector3)v_odom * dt;
  double theta_odom = estimatedPose(2) + dox(2)/2;
  Matrix33 b = rotationMatrix(theta_odom);

  RowVector3 dax = (RowVector3)v_acc * dt;
  double theta_acc = estimatedPose(2) + dax(2)/2;
  Matrix33 d = rotationMatrix(theta_acc);

  predictedPose = estimatedPose + b * dox;
  predictedCovarianceMatrix = estimatedCovarianceMatrix + odometryCovarianceMatrix * b * transpose(b);

  Matrix33 gain = predictedCovarianceMatrix * inverse(predictedCovarianceMatrix + imuCovarianceMatrix);
  estimatedPose = predictedPose  + gain * (estimatedPose + d * dax - predictedPose);
  estimatedCovarianceMatrix = (eye - gain) * predictedCovarianceMatrix;
  return estimatedPose;
}
