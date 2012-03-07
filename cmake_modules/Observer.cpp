#include "Observer.h"

using namespace Mochaccino;

Observer::Observer(Eigen::MatrixXd state,Eigen::MatrixXd a, Eigen::MatrixXd b ,Eigen::MatrixXd c ,Eigen::MatrixXd k, double timeStepMs)
{
    m_State = state;
    m_A = a;
    m_B = b;
    m_C = c;
    m_K = k;

    m_dTimeStep = timeStepMs;

    m_bIsStarted = false;
}

void Observer::Start()
{
    //do not start again
    if( m_bIsStarted == true )
        throw MochaException("The PPM thread has already started.");


    //now call the thread function
    m_pThread = new boost::thread(Observer::ThreadHandler, this);
    m_bIsStarted = true;
}

void Observer::Stop()
{
    if( m_bIsStarted == false )
        throw MochaException("No PPM thread is running.");

    //stop the audio thread
    m_pThread->interrupt();
    m_pThread->join();

    m_bIsStarted = false;
}

void Observer::ThreadHandler(Observer *arg)
{
    arg->ThreadFunction();
}

void Observer::ThreadFunction()
{
    time_t start, end;
    while (1)
    {
        boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
        double ms = 0;
        while(ms < (m_dTimeStep))
        {
            boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration dif = end - start;
            ms = dif.total_milliseconds();

            boost::this_thread::interruption_point();
        }

        Eigen::MatrixXd u,y,state;
        //read the input values
        lock();
        u = m_U;
        y = m_Y;
        state = m_State;


        Eigen::MatrixXd stateT = state.transpose();
        state = (m_A-m_K*m_C)*stateT + (m_B - m_K*0)*u + m_K*y;

        //set the state variable

        m_State = state.transpose();
        unlock();
    }
}

Eigen::MatrixXd Observer::GetState()
{
    Eigen::MatrixXd state;
    lock();
    state = m_State;
    unlock();

    return state;
}

void Observer::SetInputs(Eigen::MatrixXd u, Eigen::MatrixXd y)
{
    lock();
    m_Y = y;
    m_U = u;
    unlock();
}



