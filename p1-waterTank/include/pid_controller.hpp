#pragma once 

class PIDController
{
private:
  double K_p, K_i, K_d, u, t;
  public:
  PIDController(/* args */);
  ~PIDController();
  double compute(double setpoint, double current_value, double dt);
};

