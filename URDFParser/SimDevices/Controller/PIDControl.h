#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include <iostream>

using namespace std;

class PIDControl
{
public:
    // init PID control parameters
    void init(int PIDtype,float setpoint, float actural, float previous_error, float integral_error, float kp, float ki, float kd, float bear_rate)
    {
        m_PIDtype               = PIDtype;
        m_setpoint              = setpoint;
        m_actural               = actural;
        m_previous_error        = previous_error;
        m_integral_error        = integral_error;
        m_kp                    = kp;
        m_ki                    = ki;
        m_kd                    = kd;
        m_bear_error_rate       = bear_rate* m_setpoint;
        m_reach_setpoint_flag   = 0;
    }

    // Run PID control. here we use PD control for the motor.
    float do_pid(float iter)
    {
        float p_error;	// The difference between the desired velocity and current velocity.
        float i_error;	// The sum of errors over time.
        float d_error;	// The difference between the previous proportional error and the current proportional error.

        float p_output;	// The proportional component of the output.
        float i_output;	// The integral component of the output. This term dampens the output to prevent overshoot and oscillation.
        float d_output;	// The derivative component of the output. This term is responsible for accelerating the output if the error is large.

        float output;	// The sum of the proportional, integral and derivative terms.

        // Calculate the three errors.
        p_error = m_setpoint - m_actural; // error of proportional term
        i_error = m_integral_error; //
        d_error = p_error - m_previous_error;

        // Calculate the three components of the PID output.
        p_output = m_kp * p_error;
        i_output = m_ki * i_error;
        d_output = m_kd * d_error;

        // Sum the three components of the PID output.
        if(m_PIDtype == PD)
        {
            output = p_output + d_output; // for PD controller
        }
        else if(m_PIDtype == PID)
        {
            output = p_output + i_output + d_output; // for PID controller
        }

        // Update the previous error and the integral error.
        m_previous_error = p_error;
        m_integral_error += p_error;

        // Record the first time reach error rate.
        if (m_reach_setpoint_flag==0 && output< m_bear_error_rate)
        {
           m_reach_setpoint_iter = iter;
           m_reach_setpoint_flag = 1;
        }

        return output;
    }

private:
    // select between PD or PID control
    enum PIDtype{
         PD = 1,
         PID = 2
    };

    int   m_PIDtype;               // select between PID control or PD control
    float m_setpoint;              // desire velocity
    float m_actural;               // currenty velocity
    float m_previous_error;        // previous error
    float m_integral_error;        // integral error, for I
    float m_kp;                    // P gain
    float m_ki;                    // I gain
    float m_kd;                    // D gian

    float m_bear_error_rate;       // max error that we can bear
    int   m_reach_setpoint_flag;   // flag if we reach desire velocity
    float m_reach_setpoint_iter;   // when we reach desire velocity
};



//int main()
//{
//    PIDcontrol myPIDControl;
//    myPIDControl.init(PIDcontrol::PD, 5, 0, 0, 0, 0.01, 0.01, 0.01, 0.001);

//    Motor  myMotor;
//    myMotor.init(50, -2, 0.01, 10);

//    // run pid control
//    float desire_delta_velocity;  // this is the velocity we should reach base on the requirement of PID control.
//    for (int iter=0;iter!=1000;iter++)
//    {
//       // compute desire change of velocity
//       desire_delta_velocity= myPIDControl.do_pid(iter);

//       // updaye velocity
//       myPIDControl.m_actural = myPIDControl.m_actural + myMotor.UpdateVelocityChange(desire_delta_velocity);

//       // out put information
//       cout<<"current velocity is "<<myPIDControl.m_actural<<endl;
//    }

//    cout<<"Time for archive error rate is "<<myPIDControl.m_reach_setpoint_iter * myMotor.m_StepTime<<" second "<<endl;
//    return 0;
//}


#endif // PIDCONTROLLER_H
