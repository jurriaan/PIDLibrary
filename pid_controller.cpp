#include "pid_controller.h"

PIDController::PIDController(double kp, double ki, double kd, double setpoint, double kb, double smooth)
  : d_period(1000.0), d_input(0.0), d_output(0.0), d_integral(0.0), d_derivative(0.0), d_ready(false)
{
  setParameters(kp, ki, kd, kb, smooth);
  setSetpoint(setpoint);
  setConstraints(0.0, 500.0);
}

void PIDController::setSetpoint(double setpoint)
{
  d_setpoint = setpoint;
}

void PIDController::setParameters(double kp, double ki, double kd, double kb, double smooth)
{
  double period_in_sec = static_cast<double>(d_period) / 1000.0;

  d_kp = kp;
  d_ki = ki * period_in_sec;
  d_kd = kd / period_in_sec;
  d_kb = kb;
  d_smooth = smooth;
}

void PIDController::setSamplePeriod(unsigned int period)
{
  double ratio = static_cast<double>(period) / static_cast<double>(d_period);

  d_ki *= ratio;
  d_kd /= ratio;
  d_period = period;
}

void PIDController::setConstraints(double min, double max)
{
  d_min = min;
  d_max = max;
}

double PIDController::control(double input)
{
  d_error = d_setpoint - input;
  if(d_ready)
    d_derivative = (1 - d_smooth) * d_derivative + d_kd * d_smooth * (input - d_input); // Low pass filtered derivative
  else
  {
    d_derivative = 0; d_ready = true;
  }
  d_integral += d_ki * d_error;

  double output = calculateOutput();

  d_integral += d_kb * (constrain(output) - output);
  d_input = input;

  return d_output = constrain(calculateOutput());
}
