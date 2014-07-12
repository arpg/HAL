#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <SimDevices/Controller/PIDControl.h>

class PIDController
{
public:
    PIDControl      m_PIDController;
    float           m_MaxEngineForce;           // max force the engine can reach. The engine can only run in one direction.
    float           m_Brake;
    float           m_Friction;                 // friction of the system. including all friction. In real work, this Friction will increase with the speed on robot
    float           m_StepTime;                 // time that the force of engine apply.
    float           m_CarMass;                     // mass of the object that Engine apply.
    float           m_MaxStepForwardVelocity;   // max step velocity that the engine can reach.
    float           m_MaxStepBrakeVelocity;     //
    float           m_CurEngineForce;                    // actural force of the engine.
    float           m_MaxSteeringleft;          //
    float           m_MaxSteeringRight;         //


    // The following parameters should be init form xml file
    void Init(float MaxEngineForce, float MaxSteeringleft, float MaxSteeringRight, float Brake, float Friction, float StepTime, float CarMass)
    {
        m_MaxEngineForce          = MaxEngineForce;
        m_Friction                = Friction;
        m_Brake                   = Brake;
        m_StepTime                = StepTime;
        m_CarMass                 = CarMass;
        m_MaxStepForwardVelocity  = (m_MaxEngineForce - m_Friction) * m_StepTime / m_CarMass;
        m_MaxStepBrakeVelocity    = (m_Brake + m_Friction) * m_StepTime / m_CarMass;
        m_MaxSteeringleft         = MaxSteeringleft; // degree
        m_MaxSteeringRight        = MaxSteeringRight; // degree
    }


    // Update Steering by setting angel
    // update velocity of the robot base on desire change of velocity. Move forward when MovingDirection = 1, backward when MovingDirection = -1;
    float SetDesireVelocity(float DesireDeltaVelocity)
    {
        // get current velocity
        float CurVelocity = GetCurrentVelocity();

        // the robot is moving forward, it can brake but cannot move backward
        if (CurVelocity > 0)
        {
            // full forward force
            if (DesireDeltaVelocity >= m_MaxStepForwardVelocity )
            {
                m_CurEngineForce = m_MaxEngineForce - m_Friction;
            }
            // not full forward force
            else if(DesireDeltaVelocity >0 && DesireDeltaVelocity < m_MaxStepForwardVelocity)
            {
                m_CurEngineForce = DesireDeltaVelocity * m_CarMass / m_StepTime;
            }
            // full brake
            else if(DesireDeltaVelocity < -m_MaxStepBrakeVelocity)
            {
                m_CurEngineForce =-(m_Friction + m_Brake);
            }
            // not full brake
            else if(DesireDeltaVelocity <=0 && DesireDeltaVelocity >= -m_MaxStepBrakeVelocity)
            {
                m_CurEngineForce = DesireDeltaVelocity * m_CarMass / m_StepTime;
            }
        }

        // the robot is still. It can move both forward and backward
        else if(CurVelocity == 0)
        {
            // full forward force
            if (DesireDeltaVelocity >= m_MaxStepForwardVelocity )
            {
                m_CurEngineForce = m_MaxEngineForce - m_Friction;
            }
            // not full forward force
            else if(DesireDeltaVelocity >0 && DesireDeltaVelocity < m_MaxStepForwardVelocity)
            {
                m_CurEngineForce = DesireDeltaVelocity * m_CarMass / m_StepTime;
            }
            // full backward force
            else if(DesireDeltaVelocity< -m_MaxStepForwardVelocity)
            {
                m_CurEngineForce =-m_MaxEngineForce +m_Friction;
            }
            // not full backward force
            else if(DesireDeltaVelocity <0 && DesireDeltaVelocity>= -m_MaxStepForwardVelocity)
            {
                m_CurEngineForce = DesireDeltaVelocity * m_CarMass / m_StepTime;
            }
        }

        // the robot is moving backward, it can brake but cannot move forward
        else if(CurVelocity < 0)
        {
            // full backward force
            if (DesireDeltaVelocity < -m_MaxStepForwardVelocity )
            {
                m_CurEngineForce = -m_MaxEngineForce + m_Friction;
            }
            // not full backward force
            else if(DesireDeltaVelocity <0 && DesireDeltaVelocity > -m_MaxStepForwardVelocity)
            {
                m_CurEngineForce = DesireDeltaVelocity * m_CarMass / m_StepTime;
            }
            // full brake
            else if(DesireDeltaVelocity >0 && DesireDeltaVelocity > m_MaxStepBrakeVelocity)
            {
                m_CurEngineForce = m_Friction+m_Brake;
            }
            // not full brake
            else if(DesireDeltaVelocity >0 && DesireDeltaVelocity <= m_MaxStepBrakeVelocity)
            {
                m_CurEngineForce = DesireDeltaVelocity * m_CarMass / m_StepTime;
            }
        }

        // return change of velocity
        return m_CurEngineForce * m_StepTime/m_CarMass;
    }


    // get velocity from bullet
    float GetCurrentVelocity()
    {
         float Velocity;
         // get velocity from bullet

         return Velocity;
    }


};

#endif // SIMENGINE_H
