
#include <iostream>
#include <fstream>
#include <math.h>

#include <stdlib.h> // for rand

#include "ekf.h"
#define FILENAME "logout.csv"

using namespace ssr;
///RowVector3 operator(const Matrix33 &m, const RowVector3 &v) {


Pose2D controlSystem(const Pose2D& currentPose, 
		     const Velocity2D& currentVelocity,
		     const double dt) {

  RowVector3 dx = (RowVector3)currentVelocity * dt;
  double theta = currentPose.th + dx(2);
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);
  Matrix33 A(cosTheta, -sinTheta, 0,
	     sinTheta, cosTheta, 0,
	     0, 0, 1);
  return Pose2D(A * dx + (RowVector3)currentPose);
}

RowVector3 vrand(const double _min, const double _max) { 
  return RowVector3(
		 ((double)rand())/RAND_MAX * (_max-_min) + _min,
		 ((double)rand())/RAND_MAX * (_max-_min) + _min,
		 ((double)rand())/RAND_MAX * (_max-_min) + _min);
}

int main() {
  std::cout << " ---- EKF Demo ----" << std::endl;
  std::ofstream fout(FILENAME);

  std::cout << "--test" << std::endl;
  Matrix33 i(101.1, 0, 0, 0, 101.1, 0, 0, 0, 101.1);
  std::cout << inverse(i).str() << std::endl;

  double dt = 0.005;
  double T = 10;
  int count = T / dt;
  double* time = new double[count];
  double so = 0.1;
  double sa = 3.0;

  Pose2D x0(0, 0, 0);
  Velocity2D v_real(1.0, 0, 0.5);  

  Pose2D x(0, 0, 0);
  Pose2D ox(0, 0, 0);
  Pose2D ax(0, 0, 0);
  Pose2D kx(0, 0, 0);

  Matrix33 odomCovMat(0.1, 0, 0,
		      0, 0.1, 0,
		      0, 0, 100);

  Matrix33 imuCovMat(100, 0, 0,
		      0, 100, 0,
		      0, 0, 0.1);
  EKF4MobileRobot ekf(odomCovMat, imuCovMat);
  for(int i = 0;i < count;i++) {
    x = controlSystem(x, v_real, dt);
    fout << x.x << "," << x.y <<"," << x.th << ",";

    RowVector3 odom_noise = vrand(-so, so);
    odom_noise.set(2, odom_noise(2) * 30);
    Velocity2D v_odom((RowVector3)v_real + odom_noise);
    v_odom.vy = 0;
    ox = controlSystem(ox, v_odom, dt);
    fout << ox.x << "," << ox.y <<"," << ox.th << ",";

    RowVector3 acc_noise = vrand(-sa, sa);
    acc_noise.set(2, acc_noise(2) / 30);
    Velocity2D v_acc((RowVector3)v_real + acc_noise);
    ax = controlSystem(ax, v_acc, dt);
    fout << ax.x << "," << ax.y <<"," << ax.th << ",";

    kx = ekf(v_odom, v_acc, dt);
    fout << kx.x << "," << kx.y <<"," << kx.th << std::endl;    
  }
  fout.close();


  delete time;
  return 0;
}
