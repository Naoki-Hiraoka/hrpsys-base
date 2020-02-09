//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//---------------------------------------------------------------------------------------------------
// Header files

//#include "MadgwickAHRS.h"
#include <math.h>

class MKFilter {
public:
  MKFilter()
      : samplefreq(512),
        beta(0.1),
        q0(1.0),
        q1(0.0),
        q2(0.0),
        q3(0.0)
  {
  }

  void setdt (const double _dt) { samplefreq = 1.0 / _dt;};

  void setParam (const double _dt, const double _beta, const std::string print_str = "")
    {
      setdt(_dt);
      beta = _beta;

      std::cerr << "[" << print_str << "]   beta=" << _beta << std::endl;
    };

  void resetKalmanFilterState() {
      q0 = 1.0; q1 = 0.0; q2 = 0.0; q3 = 0.0;
  };

  void main_one (hrp::Vector3& rpy, hrp::Vector3& rpyRaw, const hrp::Vector3& acc, const hrp::Vector3& gyro)
  {
      MadgwickAHRSupdateIMU(acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
      Eigen::Quaternion<double> q(q0, q1, q2, q3);
      rpy = hrp::rpyFromRot(q.toRotationMatrix());
  };

  double getbeta(){
      return beta;
  }

private:
  // IMU algorithm update

  void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
      float recipNorm;
      float s0, s1, s2, s3;
      float qDot1, qDot2, qDot3, qDot4;
      float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

      // Rate of change of quaternion from gyroscope
      qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
      qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
      qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
      qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

      // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
      if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

          // Normalise accelerometer measurement
          recipNorm = invSqrt(ax * ax + ay * ay + az * az);
          ax *= recipNorm;
          ay *= recipNorm;
          az *= recipNorm;   

          // Auxiliary variables to avoid repeated arithmetic
          _2q0 = 2.0f * q0;
          _2q1 = 2.0f * q1;
          _2q2 = 2.0f * q2;
          _2q3 = 2.0f * q3;
          _4q0 = 4.0f * q0;
          _4q1 = 4.0f * q1;
          _4q2 = 4.0f * q2;
          _8q1 = 8.0f * q1;
          _8q2 = 8.0f * q2;
          q0q0 = q0 * q0;
          q1q1 = q1 * q1;
          q2q2 = q2 * q2;
          q3q3 = q3 * q3;

          // Gradient decent algorithm corrective step
          s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
          s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
          s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
          s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
          recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
          s0 *= recipNorm;
          s1 *= recipNorm;
          s2 *= recipNorm;
          s3 *= recipNorm;

          // Apply feedback step
          qDot1 -= beta * s0;
          qDot2 -= beta * s1;
          qDot3 -= beta * s2;
          qDot4 -= beta * s3;
      }

      // Integrate rate of change of quaternion to yield quaternion
      q0 += qDot1 * (1.0f / samplefreq);
      q1 += qDot2 * (1.0f / samplefreq);
      q2 += qDot3 * (1.0f / samplefreq);
      q3 += qDot4 * (1.0f / samplefreq);

      // Normalise quaternion
      recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
      q0 *= recipNorm;
      q1 *= recipNorm;
      q2 *= recipNorm;
      q3 *= recipNorm;
  }

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

  float invSqrt(float x) {
      float halfx = 0.5f * x;
      float y = x;
      long i = *(long*)&y;
      i = 0x5f3759df - (i>>1);
      y = *(float*)&i;
      y = y * (1.5f - (halfx * y * y));
      return y;
  }

    float samplefreq;
    float beta;								// 2 * proportional gain (Kp)
    float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
};



#endif
