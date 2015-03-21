#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <cstddef>

class PIDController
{
  double d_kp;
  double d_ki;
  double d_kd;
  double d_kb;
  double d_smooth;
  double d_input;
  double d_output;
  double d_error;
  double d_setpoint;
  double d_integral;
  double d_derivative;
  double d_min;
  double d_max;
  unsigned int d_period;
  double d_ready;

  public:
    PIDController(double kp, double ki, double kd, double setpoint, double kb = 1.0, double smooth = 0.25);
    void setParameters(double kp, double ki, double kd, double kb = 1.0, double smooth = 0.25);
    void setConstraints(double min, double max);
    void setSetpoint(double setpoint);
    void setSamplePeriod(unsigned int period);
    double control(double input);

  private:
    double constrain(double val);
    double calculateOutput();
};

inline double PIDController::constrain(double val)
{
  if (val > d_max)
    return d_max;
  else if (val < d_min)
    return d_min;

  return val;
}

inline double PIDController::calculateOutput()
{
  return d_kp * d_error + d_integral + d_derivative;
}
#endif
