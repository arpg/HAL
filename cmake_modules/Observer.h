#ifndef OBSERVER_H
#define OBSERVER_H
#include <Eigen/Eigen>
#include "MochaException.h"
#include <boost/thread.hpp>
#include <boost/signals2/mutex.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
 #include <iostream>

class Observer : public boost::mutex
{

private:
    Eigen::MatrixXd m_State;
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;
    Eigen::MatrixXd m_C;
    Eigen::MatrixXd m_K;

    Eigen::MatrixXd m_U;
    Eigen::MatrixXd m_Y;

    boost::thread *m_pThread;
    bool m_bIsStarted;
    double m_dTimeStep;

public:
    Observer(Eigen::MatrixXd state,Eigen::MatrixXd a, Eigen::MatrixXd b ,Eigen::MatrixXd c ,Eigen::MatrixXd K, double timeStepMs);

    Eigen::MatrixXd GetState();
    void SetInputs(Eigen::MatrixXd u, Eigen::MatrixXd y);
    void SetState(Eigen::MatrixXd state){ m_State = state;}

    void Start();
    void Stop();
    bool IsStarted(){ return m_bIsStarted; }

    static void ThreadHandler(Observer *arg);
    void ThreadFunction();

};

#endif // OBSERVER_H
