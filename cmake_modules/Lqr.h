#ifndef Lqr_H
#define Lqr_H
#include <Eigen/Eigen>
#include "MochaException.h"
#include <boost/thread.hpp>
#include <boost/signals2/mutex.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
 #include <iostream>

class Lqr : public boost::mutex
{
private:
    Eigen::MatrixXd m_State;
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;
    Eigen::MatrixXd m_C;

    //cost function matrices
    Eigen::MatrixXd m_S;
    Eigen::MatrixXd m_R;
    Eigen::MatrixXd m_Q;

    //gain matrices
    Eigen::MatrixXd *m_pK;
    Eigen::MatrixXd m_Ki;
    int m_nSteps;
    int m_nCurrentStep;

    Eigen::MatrixXd m_U;
    Eigen::MatrixXd m_Y;

    Eigen::MatrixXd m_setPoint;
    bool m_bIsActive;

    boost::thread *m_pThread;
    bool m_bIsStarted;
    double m_dTimeStep;

public:
    Lqr(Eigen::MatrixXd a, Eigen::MatrixXd b ,Eigen::MatrixXd c, double timeStepMs);

    Eigen::MatrixXd GetInput();
    Eigen::MatrixXd GetState();
    void SetState(Eigen::VectorXd state);
    void ScheduleGains(double durationMs);
    void Control(Eigen::VectorXd setPoint);
    void SetCostGains(Eigen::MatrixXd s, Eigen::MatrixXd r,Eigen::MatrixXd q);

    void Start();
    void Stop();
    bool IsStarted(){ return m_bIsStarted; }

    static void ThreadHandler(Lqr *arg);
    void ThreadFunction();
};

#endif // Lqr_H
