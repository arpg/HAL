#include "Lqr.h"


using namespace Mochaccino;

Lqr::Lqr(Eigen::MatrixXd a, Eigen::MatrixXd b ,Eigen::MatrixXd c,  double timeStepMs)
{
    //m_State = state;
    m_A = a;
    m_B = b;
    m_C = c;

    m_U = Eigen::MatrixXd::Zero(1,1);


    m_nSteps = 0;
    m_nCurrentStep = 0;
    m_pK = 0;
    m_bIsActive = false;

    m_dTimeStep = timeStepMs;

    m_bIsStarted = false;
}

void Lqr::Start()
{
    //do not start again
    if( m_bIsStarted == true )
        throw MochaException("The PPM thread has already started.");


    //now call the thread function
    m_pThread = new boost::thread(Lqr::ThreadHandler, this);
    m_bIsStarted = true;
}

void Lqr::Stop()
{
    if( m_bIsStarted == false )
        throw MochaException("No PPM thread is running.");

    //stop the audio thread
    m_pThread->interrupt();
    m_pThread->join();

    m_bIsStarted = false;
}

void Lqr::ThreadHandler(Lqr *arg)
{
    arg->ThreadFunction();
}

void Lqr::ThreadFunction()
{
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

        //if control is active, then update the input value
        lock();
        if( m_bIsActive == true )
        {
            if(m_nCurrentStep < m_nSteps)
            {
                m_U = m_pK[m_nCurrentStep]*(m_State.transpose());
            }else
                m_U = m_Ki*(m_State.transpose());

            m_nCurrentStep++;
        }
        unlock();

//        Eigen::MatrixXd u,y,state;
//        //read the input values
//        lock();
//        u = m_U;
//        y = m_Y;
//        state = m_State;


//        Eigen::MatrixXd stateT = state.transpose();
//        state = (m_A-m_K*m_C)*stateT + (m_B - m_K*0)*u + m_K*y;

//        //set the state variable

//        m_State = state.transpose();
//        unlock();
    }
}

Eigen::MatrixXd Lqr::GetInput()
{
    Eigen::MatrixXd u;
    lock();
    u = m_U;
    unlock();

    return u;
}

Eigen::MatrixXd Lqr::GetState()
{
    Eigen::MatrixXd state;
    lock();
    state = m_State;
    unlock();

    return state;
}

void Lqr::SetState(Eigen::VectorXd state)
{
    lock();
    m_State = state-m_setPoint;
    unlock();
}

void Lqr::Control(Eigen::VectorXd setPoint)
{
    lock();
    m_setPoint = setPoint;
    m_bIsActive = true;
    m_nCurrentStep = 0;
    unlock();
}

void Lqr::SetCostGains(Eigen::MatrixXd s, Eigen::MatrixXd r,Eigen::MatrixXd q)
{
    m_S = s;
    m_R = r;
    m_Q = q;

}

void Lqr::ScheduleGains(double duration)
{
    //calculat ethe number of steps needed for the op
    m_nSteps = duration / m_dTimeStep;

    //it is critical to gain full access to the Lqr during this
    lock();

    //delete previous gains
    if( m_pK != 0 )
        delete m_pK;

    m_pK = new Eigen::MatrixXd[m_nSteps];

    Eigen::MatrixXd S = m_S;

    //now schedule a gain for each step
    for( int i = 0 ; i< m_nSteps ; i++)
    {
        Eigen::MatrixXd temp = (m_R+ (m_B.transpose())*S*m_B);
        m_pK[i] = -(temp.inverse());
        m_pK[i] *= (m_B.transpose())*S*m_A;

        //and now calculate the new S
        temp = (m_A+m_B*m_pK[i]).transpose();
        temp *= S*(m_A+m_B*m_pK[i]);
        temp += m_Q;
        temp += ((m_pK[i].transpose())*m_R*m_pK[i]);
        S = temp;
    }

    //now solve for infinit horizon
    Eigen::MatrixXd temp = m_R + (m_B.transpose())*m_S*m_B;
    m_Ki = -(temp.inverse());
    m_Ki *= (m_B.transpose())*m_S*m_A;

    //disable all control
    m_bIsActive = false;
    m_nCurrentStep = 0;

    unlock();
}

