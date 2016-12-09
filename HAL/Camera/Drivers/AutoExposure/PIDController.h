#pragma once

#include <functional>

namespace hal
{

class PIDController
{
  public:

    typedef std::function<void(double)> ControlFunc;

  public:

    PIDController(ControlFunc function) :
      setpoint(0),
      Kp(0), Ki(0), Kd(0),
      ApplyControlUpdate(function),
      lastError(0),
      integral(0)
    {
    }

    ~PIDController()
    {
    }

    inline void Update(double feedback)
    {
      const double error = setpoint - feedback;
      const double update = GetControlUpdate(error);
      ApplyControlUpdate(update);
    }

    inline void Reset()
    {
      lastError = 0;
      integral = 0;
    }

  protected:

    inline double GetControlUpdate(double error)
    {
      double update = 0;
      update += GetProportional(error);
      update += GetIntegral(error);
      update += GetDerivative(error);
      return update;
    }

    inline double GetProportional(double error)
    {
      return Kp * error;
    }

    inline double GetIntegral(double error)
    {
      integral += error;
      return Ki * integral;
    }

    inline double GetDerivative(double error)
    {
      const double result = Kd * (error - lastError);
      lastError = error;
      return result;
    }

  public:

    double setpoint;

    double Kp;

    double Ki;

    double Kd;

  protected:

    ControlFunc ApplyControlUpdate;

    double lastError;

    double integral;
};

} // namespace hal