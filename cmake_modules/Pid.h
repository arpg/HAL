#pragma once
//#include "utilities.h"
//#include "smooth.h"
//#include "double.h"
#include "math.h"

//-----------------------------------------------------------------------------
// Class that handles the PID math
//-----------------------------------------------------------------------------
class Pid
{
private:
        bool m_bHeading;
        double m_IGain;
        double m_PGain;
        double m_DGain;
        double m_OutGain;
        double m_Input;
        double m_Output;
        double m_InternalOutput;
        Pid *m_OutputPID;
        double m_InputGain;


        //PID values
        bool m_bIncremental;
        //float m_LastTime;
        double m_SetPoint;
        double m_TotalError;
        double m_LastError;

        double  m_MaxIntegralError;
        double m_MinIntegralError;
        double m_MaxOutput;
        double m_MinOutput;
        //Smooth _dErrorSmooth;

public:
        Pid(double pGain, double iGain, double dGain, double maxOut, double minOut);
        void SetPoint(double sp, bool bReset = false);
        double GetOutput(){ return m_Output; }
        void Iterate(double input, double dt);
};