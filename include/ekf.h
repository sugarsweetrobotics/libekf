#pragma once

#include <string>
#include <sstream>

namespace ssr {

  class Velocity2D;
  class Pose2D;

  class RowVector3 {
  public:
    double a[3];
  public:
    RowVector3(const double a1, const double a2, const double a3)  {
      a[0] = a1; a[1] = a2; a[2] = a3;
    }
    ~RowVector3() {}
    
    RowVector3(const RowVector3 &v) {
      a[0] = v.a[0]; a[1] = v.a[1]; a[2] = v.a[2];
    }

    double operator()(const int i) const {
      return a[i];
    }


    std::string str() const {
      std::ostringstream oss;
      oss << "[" << a[0] << " " << a[1] << " " << a[2] << "]";
      return oss.str();
    }

    RowVector3(const Velocity2D& v);
    RowVector3(const Pose2D& p);

    void set(const int i, const double v) {
      a[i] = v;
    }
  };


  class ColumnVector3 {
  public:
    double a[3];
  public:
    ColumnVector3(const double a1, const double a2, const double a3)  {
      a[0] = a1; a[1] = a2; a[2] = a3;
    }
    ~ColumnVector3() {}
    
    ColumnVector3(const ColumnVector3 &v) {
      a[0] = v.a[0]; a[1] = v.a[1]; a[2] = v.a[2];
    }

    double operator()(const int i) const {
      return a[i];
    }


    std::string str() const {
      std::ostringstream oss;
      oss << "[" << a[0] << " " << a[1] << " " << a[2] << "]";
      return oss.str();
    }

    ColumnVector3(const Velocity2D& v);
    ColumnVector3(const Pose2D& p);

    void set(const int i, const double v) {
      a[i] = v;
    }
  };

  inline ColumnVector3 transpose(const RowVector3& v) {
    return ColumnVector3(v(0), v(1), v(2));
  }

  inline double operator*(const RowVector3& v0, const ColumnVector3& v1) {
    return v0(0) * v1(0) + v0(1) * v1(1) + v0(2) * v1(2);
  }

  inline RowVector3 operator*(const RowVector3& v, const double t) {
    return RowVector3(v(0)*t, v(1)*t, v(2)*t);
  }

  inline RowVector3 operator*(const double t, const RowVector3& v) {
    return RowVector3(v(0)*t, v(1)*t, v(2)*t);
  }



  class Matrix33 {

  public:
    double a[3][3];
  public:
    Matrix33(const double a11, const double a12, const double a13,
	     const double a21, const double a22, const double a23,
	     const double a31, const double a32, const double a33) {
      a[0][0] = a11; a[0][1] = a12; a[0][2] = a13;
      a[1][0] = a21; a[1][1] = a22; a[1][2] = a23;
      a[2][0] = a31; a[2][1] = a32; a[2][2] = a33;
    }

    Matrix33(const Matrix33& m) {
      for(int i = 0;i < 9;i++) {
	a[i/3][i%3] = m.a[i/3][i%3];
      }
    }

    ~Matrix33() {}
    double operator()(const int i, const int j) const {
      return a[i][j];
    }


    std::string str() const {
      std::ostringstream oss;
      oss << "[" << a[0][0] << " " << a[0][1] << " " << a[0][2] << ";\n" 
	  << a[1][0] << " " << a[1][1] << " " << a[1][2] << ";\n"
	  << a[2][0] << " " << a[2][1] << " " << a[2][2] << "]";
      return oss.str();
    }
  };
  
  inline Matrix33 operator+(const Matrix33 &m0, const Matrix33 &m1) {
    return Matrix33(m0(0,0)+m1(0,0), m0(0,1)+m1(0,1), m0(0,2)+m1(0,2),
		    m0(1,0)+m1(1,0), m0(1,1)+m1(1,1), m0(1,2)+m1(1,2),
		    m0(2,0)+m1(2,0), m0(2,1)+m1(2,1), m0(2,2)+m1(2,2));
  }

  inline Matrix33 operator-(const Matrix33 &m0, const Matrix33 &m1) {
    return Matrix33(m0(0,0)-m1(0,0), m0(0,1)-m1(0,1), m0(0,2)-m1(0,2),
		    m0(1,0)-m1(1,0), m0(1,1)-m1(1,1), m0(0,2)-m1(1,2),
		    m0(2,0)-m1(2,0), m0(2,1)-m1(2,1), m0(0,2)-m1(2,2));
  }

  inline Matrix33 operator*(const Matrix33 &m0, const Matrix33 &m1) {
    return Matrix33(m0(0,0)*m1(0,0) + m0(0,1)*m1(1,0) + m0(0,2)*m1(2,0),
		    m0(0,0)*m1(0,1) + m0(0,1)*m1(1,1) + m0(0,2)*m1(2,1),
		    m0(0,0)*m1(0,2) + m0(0,1)*m1(1,2) + m0(0,2)*m1(2,2),
		    m0(1,0)*m1(0,0) + m0(1,1)*m1(1,0) + m0(1,2)*m1(2,0),
		    m0(1,0)*m1(0,1) + m0(1,1)*m1(1,1) + m0(1,2)*m1(2,1),
		    m0(1,0)*m1(0,2) + m0(1,1)*m1(1,2) + m0(1,2)*m1(2,2),
		    m0(2,0)*m1(0,0) + m0(2,1)*m1(1,0) + m0(2,2)*m1(2,0),
		    m0(2,0)*m1(0,1) + m0(2,1)*m1(1,1) + m0(2,2)*m1(2,1),
		    m0(2,0)*m1(0,2) + m0(2,1)*m1(1,2) + m0(2,2)*m1(2,2));
  }

  inline Matrix33 transpose(const Matrix33 &m) {
    return Matrix33(m(0,0), m(1,0), m(2,0),
		    m(0,1), m(1,1), m(2,1),
		    m(0,2), m(1,2), m(2,2));
  }

  inline double determinant(const Matrix33 &m) {
    double det=0.0;
    det=m.a[0][0]*m.a[1][1]*m.a[2][2];
    det+=m.a[1][0]*m.a[2][1]*m.a[0][2];
    det+=m.a[2][0]*m.a[0][1]*m.a[1][2];
    det-=m.a[2][0]*m.a[1][1]*m.a[0][2];
    det-=m.a[1][0]*m.a[0][1]*m.a[2][2];
    det-=m.a[0][0]*m.a[2][1]*m.a[1][2];
    return det;
  }

  inline Matrix33 inverse(const Matrix33 &m) {
    double buf_a[3][3] = {{m.a[0][0], m.a[0][1], m.a[0][2]},
		      {m.a[1][0], m.a[1][1], m.a[1][2]},
		      {m.a[2][0], m.a[2][1], m.a[2][2]}};
		      
    double inv_a[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double buf = 0;
    //掃き出し法
    for(int i = 0;i < 3;i++){
      buf=1/buf_a[i][i];
      for(int j = 0;j < 3;j++){
	buf_a[i][j] *= buf;
	inv_a[i][j] *= buf;
      }
      for(int j = 0;j < 3;j++){
	if(i != j){
	  buf=buf_a[j][i];
	  for(int k = 0;k < 3;k++){
	    buf_a[j][k] -= buf_a[i][k] * buf;
	    inv_a[j][k] -= inv_a[i][k] * buf;
	  }
	}
      }
    }
    return Matrix33(inv_a[0][0], inv_a[0][1], inv_a[0][2],
		    inv_a[1][0], inv_a[1][1], inv_a[1][2],
		    inv_a[2][0], inv_a[2][1], inv_a[2][2]);
  }

  inline RowVector3 operator*(const Matrix33 &m, const RowVector3 &v) {
    return RowVector3(m(0,0) * v(0) + m(0,1)*v(1) + m(0,2)*v(2),
		   m(1,0)*v(0) + m(1,1)*v(1) + m(1,2)*v(2),
		   m(2,0)*v(0) + m(2,1)*v(1) + m(2,2)*v(2));
  };
  
  inline RowVector3 operator+(const RowVector3 &v0, const RowVector3 &v1) {
    return RowVector3(v0.a[0] + v1.a[0],
		   v0.a[1] + v1.a[1],
		   v0.a[2] + v1.a[2]);
  }

  inline RowVector3 operator-(const RowVector3 &v0, const RowVector3 &v1) {
    return RowVector3(v0.a[0] - v1.a[0],
		   v0.a[1] - v1.a[1],
		   v0.a[2] - v1.a[2]);
  }



  class Pose2D {
  public:
    double x;
    double y;
    double th;
  public:
    Pose2D(const double x, const double y, const double th) : x(x), y(y), th(th) {}
    Pose2D(const Pose2D &p) : x(p.x), y(p.y), th(p.th) {}
  Pose2D(const RowVector3 &v) : x(v(0)), y(v(1)), th(v(2)) {}
    ~Pose2D() {}

    void operator+=(const Pose2D &p) {
      this->x += p.x;
      this->y += p.y;
      this->th += p.th;
    }

  };

  class Velocity2D { 
  public:
    double vx;
    double vy;
    double va;

  public:
    Velocity2D(const double vx, const double vy, const double va) : vx(vx), vy(vy), va(va) {}
    ~Velocity2D() {}
  Velocity2D(const RowVector3& v) : vx(v(0)), vy(v(1)), va(v(2)) {}
      
  };

  inline RowVector3::RowVector3(const Velocity2D& v) {
      a[0] = v.vx, a[1] = v.vy, a[2] = v.va;
    }

  inline RowVector3::RowVector3(const Pose2D& p) {
    a[0] = p.x, a[1] = p.y, a[2] = p.th;
    }


  class EKF4MobileRobot {
  public:
    RowVector3 estimatedPose;
    RowVector3 predictedPose;
    Matrix33 estimatedCovarianceMatrix;
    Matrix33 predictedCovarianceMatrix;

    Matrix33 odometryCovarianceMatrix;
    Matrix33 imuCovarianceMatrix;
  public:
    EKF4MobileRobot(const Matrix33 &odomCovMat, const Matrix33 &imuCovMat);

    ~EKF4MobileRobot();


    Pose2D operator()(const Velocity2D& v_odom, const Velocity2D& v_acc, const double dt);
  };


}
