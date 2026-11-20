#pragma once
#include <math.h>

// Two-state Kalman filter: pitch angle (rad) + gyroscope bias (rad/s).
// Optimized for the BALA2 operating range (+/-45 deg).
class KalmanPitch {
public:
  float angle=0.0f, bias=0.0f, rate=0.0f;
  float qAngle=0.001f;
  float qBias=0.00002f; // Slow adaptation; bias already calibrated at startup
  float rAngle=0.0012f;
  float biasAdaptLimit=0.26f; // +/-15 deg

  void reset(float accelAngle, float initialBias) {
    angle=accelAngle; bias=initialBias; rate=0.0f;
    p00=0.002f; p01=0.0f; p10=0.0f; p11=0.00005f;
  }
  static float wrapPi(float a) {
    return atan2f(sinf(a),cosf(a));
  }
  void update(float gyroRadS, float accelRad, float dt, bool accelValid) {
    if (!isfinite(gyroRadS) || !isfinite(accelRad) ||
        !isfinite(dt) || dt<=0.0f || dt>0.03f) return;
    rate=gyroRadS-bias;
    angle=wrapPi(angle + dt*rate);
    const float n00=p00-dt*(p01+p10)+dt*dt*p11+qAngle*dt;
    const float n01=p01-dt*p11, n10=p10-dt*p11;
    const bool adaptBias=fabsf(angle)<biasAdaptLimit &&
                         fabsf(accelRad)<biasAdaptLimit;
    const float n11=p11+(adaptBias ? qBias*dt : 0.0f);
    p00=n00; p01=n01; p10=n10; p11=n11;
    if (accelValid) {
      const float innovation=wrapPi(accelRad-angle);
      // Prevents an inconsistent accelerometer reading from causing a sudden Kalman estimate jump.
      if (fabsf(innovation)<0.35f) {
        const float s=p00+rAngle;
        if (s>1e-12f && isfinite(s)) {
          const float k0=p00/s, k1=adaptBias ? p10/s : 0.0f;
          angle=wrapPi(angle+k0*innovation);
          bias+=k1*innovation;
          const float oldP00=p00, oldP01=p01;
          p00-=k0*oldP00;
          p01-=k0*oldP01;
          p10-=k1*oldP00;
          p11-=k1*oldP01;
          const float sym=0.5f*(p01+p10);
          p01=p10=sym;
          if (p00<1e-9f) p00=1e-9f;
          if (p11<1e-10f) p11=1e-10f;
        }
      }
    }
    rate=gyroRadS-bias;
  }
private:
  float p00=0.002f, p01=0, p10=0, p11=0.00005f;
};
