#pragma once
#include "Common.h"
#include "Pid.h"

//-----------------------------------------------------------------------------
// initialises the PID variables to their default values
//-----------------------------------------------------------------------------
Pid::Pid(double pGain, double iGain, double dGain, double maxOut, double minOut)
{
        m_bHeading = false;
        m_bIncremental = false;
        m_TotalError = 0;
        m_LastError = 0;
        m_SetPoint = 0;
        m_Input = 0;
        m_InputGain = 1;
        m_Output = 0;
        m_OutputPID = 0;
        m_OutGain = 1;
        m_IGain = iGain;//0.5;
        m_PGain = pGain;//2;
        m_DGain= dGain;//0.5;

        m_MaxOutput = maxOut;
        m_MinOutput = minOut;

        m_MaxIntegralError = 1000;
        m_MinIntegralError = -1000;
}

//-----------------------------------------------------------------------------
// sets the target position of the PID. can reset the integral hold error if required
//-----------------------------------------------------------------------------
void Pid::SetPoint(double sp, bool bReset /*= true*/)
{
        m_SetPoint = sp;
        if( bReset)
        {
                m_TotalError = 0;
        }
}

//-----------------------------------------------------------------------------
// calculates the PID output values based on the PID input values.
//-----------------------------------------------------------------------------
void Pid::Iterate(double input, double dt)
{
    m_Input = input;

        double e = m_SetPoint - (m_Input*m_InputGain);
        //check if we need to correct for heading
        if( m_bHeading )
        {
                //account for discontinuity at 180 degrees
                if( e > double(M_PI) )
                        e = (double(2*M_PI)-e)*-1;
                else if( e < double(-M_PI) )
                        e = double(2*M_PI) + e;
        }

        double dE;
        if( dt!= 0 )
                dE = (e- m_LastError)/dt;//_dErrorSmooth.AddValue((e- m_LastError)/dt);
        else
                dE = 0;

        //TODO: this can overflow for large heading angles
        if( m_IGain != 0)
                m_TotalError+= e*dt;

        if( m_TotalError > m_MaxIntegralError )
                m_TotalError = m_MaxIntegralError;
        else if( m_TotalError < m_MinIntegralError )
                m_TotalError = m_MinIntegralError;

        //this is scaled as value / (1000*1000*1000) * RESOLUTION = value/1000

        m_InternalOutput = (e*m_PGain + dE *m_DGain + m_TotalError*m_IGain)     * m_OutGain;

        if( m_bIncremental )
        {
                if( m_Output != 0 )
                        m_InternalOutput += m_Output;
                else if ( m_OutputPID != 0 )
                        m_InternalOutput += m_OutputPID->m_SetPoint;
        }

        if( m_InternalOutput > m_MaxOutput )
                m_InternalOutput = m_MaxOutput;
        else if (m_InternalOutput < m_MinOutput)
                m_InternalOutput = m_MinOutput;

        if( m_Output != 0 )
                m_Output = m_InternalOutput;
        else if ( m_OutputPID != 0 )
        {
                m_OutputPID->m_SetPoint = m_InternalOutput;
                m_OutputPID->Iterate(m_Input,dt);
        }
        m_LastError = e;
}