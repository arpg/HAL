#include <ao/ao.h>
#include <Eigen/Eigen>

#include "Common.h"
#include "MochaException.h"
#include "AudioBuffer.h"
#include "Ppm.h"
#include "Observer.h"
#include "Controller.h"
//#include "ms3dmgx3driver.h"

#include "Node.h"
#include "Messages.pb.h"

#include <iostream>
//#include <pangolin/pangolin.h>
//#include <pangolin/display_internal.h>

//#include "GLConsole.h"
#include "AirController.h"
#include <CarPlanner.h>
#include <RpgUtils.h>
#include "TrajectoryTracker.h"
#include "Vicon.h"

#include "Pid.h"

//using namespace pangolin;
using namespace Mochaccino;
using namespace rpg;

MochaConfig g_MochaConfig;
std::vector<Eigen::Vector5d> g_vWaypoints;
std::vector<unsigned int> g_vPath;
bool g_bIsStarted = false;

void WaypointsRpcHandler(Waypoints &wp, RetVal &rv)
{
    g_vWaypoints.resize(wp.wpoints_size());
    //go through the waypoints and assign the new ones
    for(int i = 0 ; i < wp.wpoints_size() ; i++ )
    {
        Waypoint * wpoint = wp.mutable_wpoints(i);
        g_vWaypoints[i] << wpoint->x() , wpoint->y(), wpoint->theta(), wpoint->velocity(), wpoint->w();
    }

    rv.set_val(true);

}

void PathRpcHandler(Path &wp, RetVal &rv)
{
    g_vPath.resize(wp.indices_size());
    for( int i = 0 ; i <  wp.indices_size() ; i++)
    {
        g_vPath[i] = wp.indices(i);
    }

    rv.set_val(true);
}

void StartRpcHandler(RetVal &wp, RetVal &rv)
{
    g_bIsStarted = true;
    rv.set_val(true);
}

void StopRpcHandler(RetVal &wp, RetVal &rv)
{
    g_bIsStarted = false;
    rv.set_val(true);
}

void PopulateEntity(Entity *pEnt, std::string name, Eigen::Vector6d pose)
{
    pEnt->set_name(name);
    pEnt->mutable_pose()->set_x(pose(0));
    pEnt->mutable_pose()->set_y(pose(1));
    pEnt->mutable_pose()->set_z(pose(2));
    pEnt->mutable_pose()->set_roll(pose(3));
    pEnt->mutable_pose()->set_pitch(pose(4));
    pEnt->mutable_pose()->set_yaw(pose(5));
}

void PopulateEntity(Entity *pEnt, std::string name, Eigen::Vector5d pose)
{
    pEnt->set_name(name);
    pEnt->mutable_pose()->set_x(pose(0));
    pEnt->mutable_pose()->set_y(pose(1));
    pEnt->mutable_pose()->set_z(0);
    pEnt->mutable_pose()->set_roll(0);
    pEnt->mutable_pose()->set_pitch(0);
    pEnt->mutable_pose()->set_yaw(pose(2));
}

int main(int argc, char *argv[])
{
    //initialize matrices
    Eigen::Vector6d vtPose,carPose,rampPose,lastRawPose,lastPose,derivatives;
    rampPose << 0, 0, 0, 0, 0, 0;
    carPose = rampPose;
    Eigen::Vector3d orientation, orientationOffset;
    orientationOffset << 0,0,0;
    orientation = orientationOffset;
    Eigen::Vector5d curPose,desiredPose;
    desiredPose << 0,0,0,0,0;
    Eigen::Vector2d control;
    control << 0,0;

    //index of the current path segment
    int pathIndex = -1;

    //Initiate the tracker and start tracking objects
    Vicon Tracker;
    Tracker.TrackObject( "CAR", "192.168.10.1" );
    Tracker.TrackObject( "RECRAMP", "192.168.10.1" );
    Tracker.Start();

    //register our interfaces
    Node mochaNode(6666);
    mochaNode.Register("Waypoints",&WaypointsRpcHandler);
    mochaNode.Register("Path",&PathRpcHandler);
    mochaNode.Register("Start",&StartRpcHandler);
    mochaNode.Register("Stop",&StopRpcHandler);
    mochaNode.Publish("WorldState",6667);

    //start the control system and ppm
    Ppm ppm;
    CarPlanner planner;
    TrajectoryTracker tracker(10,10,5);
    AirController airController;
    ppm.Start();

    while(1)
    {
        while( g_bIsStarted == false )
        {

        }

        //store the last raw and actual pose
        lastPose = vtPose;
        lastRawPose = vtPose;

        //set the first waypoint goal
        pathIndex = 0;
        double updateTime = 0;
        double t = Tic();
        planner.SetGoal(t,g_vWaypoints[pathIndex],g_vWaypoints[pathIndex+1]);

        while(g_bIsStarted == true )
        {
            //update the pose from the tracker
            vtPose = Tracker.GetPose("CAR");

            //get the position type
            eLocType type = Tracker.WhereAmI(vtPose);

            //for all three angles p,q and r
//            for(int i = 3 ; i < 6 ; i++)
//            {
//                if(abs(vtPose[i] - lastRawPose[i]) > PI)
//                {
//                    if(vtPose[i]>0)
//                        orientationOffset[i-3] -= 2*PI;
//                    else
//                        orientationOffset[i-3] += 2*PI;

//                }
//            }


            //set the actual car pose with the offsets
            carPose = vtPose;
            carPose(3) += orientationOffset(0);
            carPose(4) += orientationOffset(1);
            carPose(5) += orientationOffset(2);

            //calculate body derivatives
            double dT = Toc(t);
            derivatives = (vtPose - lastPose)/dT;
            t = Tic();


            //move the raw and offset last pose values forward
            lastRawPose = vtPose;
            lastPose = vtPose;


            if(type == VT_AIR)
            {
//                Eigen::Vector3d orientation;
//                orientation << carPose(3), carPose(4), carPose(5);
//                airController.SetCurrentOrientation(orientation);
//                Eigen::Vector3d controlInputs = airController.GetControlInputs();
            }else if( type == VT_GROUND)
            {
                double v = sqrt(pow(derivatives(0),2)+pow(derivatives(1),2));
                curPose << vtPose(0), vtPose(1), vtPose(5),v,derivatives(5);
                if(planner.IsCurveDone(t) == true )
                {
                    pathIndex++;

                    if(pathIndex == g_vPath.size()-1)
                    {
                        vector<unsigned int>::iterator it = std::find(g_vPath.begin(), g_vPath.end(),g_vPath[pathIndex]);
                        if( it != g_vPath.end())
                        {
                            int index = *it;
                            planner.SetGoal(t,g_vWaypoints[g_vPath[index]],g_vWaypoints[index+1]);
                            pathIndex = index;
                        }else
                        {
                            break;
                        }
                    }else
                    {
                        planner.SetGoal(t,g_vWaypoints[g_vPath[pathIndex]],g_vWaypoints[g_vPath[pathIndex+1]]);
                    }
                }
                desiredPose = planner.GetCurvePose(t);
                control = tracker.UpdateControl(curPose,desiredPose);
            }else
            {
            }

            //progress the vehicle state
            //vtPose(0) += cos(vtPose(5))*control(0)*dT;
            //vtPose(1) += sin(vtPose(5))*control(0)*dT;
            //vtPose(5) += control(1) * dT;

            //now set the control signals
            double vPpm = control(0)*1.5;
            if(vPpm > 10)
                vPpm = 10;
            else if (vPpm < -10)
                vPpm = -10;
            ppm.SetChannel(0,vPpm+55);
            ppm.SetChannel(1,control(1)*-10 + 55);

            //create WorldState
            PublishMsg msg;
            Entity *pEnt = msg.add_name();
            PopulateEntity(pEnt, "CAR", vtPose);

            pEnt = msg.add_name();
            PopulateEntity(pEnt,"Desired CAR",desiredPose);

            rampPose = Tracker.GetPose("RECRAMP");
            PopulateEntity(pEnt,"Ramp",rampPose);

            switch(type)
            {
                case VT_AIR:
                    msg.set_loctype("Air");
                    break;

                case VT_GROUND:
                    msg.set_loctype("Ground");
                    break;

                case VT_CIR_RAMP:
                    msg.set_loctype("Circular Ramp");
                    break;

                case VT_REC_RAMP:
                    msg.set_loctype("Rectangular Ramp");
                    break;
            }

            updateTime += dT;
            if( updateTime > 0.05)
            {
                try
                {
                    mochaNode.Write("WorldState",msg);
                }catch(...){}
                updateTime = 0;
            }

            std::cout << "Test" << std::endl;

            usleep(1000);
        }

        g_bIsStarted = false;
    }

	/*


	//start the observer
	Eigen::MatrixXd oState = Eigen::MatrixXd::Zero(1,3);
	Eigen::MatrixXd a = Eigen::MatrixXd::Zero(3,3);
	a << 1, 0.01, 0.0001,
		 0,    1,   0.01,
		 0,    0,      1;

	Eigen::MatrixXd b = Eigen::MatrixXd::Zero(3,1);
	b   << 0,
		   0,
		   0;

	Eigen::MatrixXd c = Eigen::MatrixXd::Zero(1,3);
	c << 1,
		 0,
		 0;

	Eigen::MatrixXd k = Eigen::MatrixXd::Zero(3,1);
	k << 0.6,
		 10.7,
		 60;

	Observer obs(oState,a,b,c,k,10);

	a = Eigen::MatrixXd::Zero(2,2);
	a << 1, 0.01,
		 0,    1;


      //YAW CONTROL
    Eigen::MatrixXd s = Eigen::MatrixXd::Zero(2,2);
    s << 8, 0,
         0,  0.07;

    Eigen::MatrixXd r = Eigen::MatrixXd::Zero(1,1);
    r << 0.001;


    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(2,2);
    q << 0.02, 0,
         0,    0.0001;



    //ROW CONTROL
  Eigen::MatrixXd s = Eigen::MatrixXd::Zero(2,2);
  s << 8, 0,
       0,  0.02;

  Eigen::MatrixXd r = Eigen::MatrixXd::Zero(1,1);
  r << 0.004;

  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(2,2);
  q << 0.02, 0,
       0,    0.0001;

    Controller con(a,b,c,0.01);
    con.ScheduleGains(s,r,q,5);


	Eigen::MatrixXd q = Eigen::MatrixXd::Zero(2,2);
	q << 0.02, 0,
		 0,    0.01;

	Controller con(a,b,c,0.01);
	con.ScheduleGains(s,r,q,1);



	//start the PPM generator
	Ppm *ppm = new Ppm();
	ppm->Start();
	ppm->SetChannel(1,50);
	ppm->SetChannel(0,55);


	//setup the plotter
	//Create OpenGL window in single line thanks to GLUT
	pangolin::CreateGlutWindowAndBind("Main",640,480);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Issue specific OpenGl we might need
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	DataLog log;

	View& d_graph = pangolin::CreatePlotter("x",&log);
	d_graph.SetBounds(1.0, 0.0, 0, 1.0);


    labels.push_back(std::string("yaw"));
    labels.push_back(std::string("target"));
    labels.push_back(std::string("LQR input"));


	vector<std::string> labels;

	labels.push_back(std::string("yaw"));
	labels.push_back(std::string("observed yaw"));
	labels.push_back(std::string("observed yaw dot"));
	labels.push_back(std::string("observed torque"));
	labels.push_back(std::string("pwm"));
	labels.push_back(std::string("LQR input"));
    double target;

	log.SetLabels(labels);

	double yaw = 0;
	double yawOffset = 0;
	double time = 0;


    sleep(1);


//    while(1)
//       {
//        int s;
//        cout << "70";
//        cin >> s;
//         struct timeval tv;
//         struct timezone tz;
//         struct tm *tm;
//         gettimeofday(&tv, &tz);
//         tm=localtime(&tv.tv_sec);
//         printf ("KeyPress %02d:%02d:%02d:%03ld \n",tm->tm_hour, tm->tm_min, tm->tm_sec, (tv.tv_usec/1000) );

//        ppm->SetChannel(1,70);

//        cout << "30";
//        cin >> s;
//        gettimeofday(&tv, &tz);
//        tm=localtime(&tv.tv_sec);
//        printf ("KeyPress %02d:%02d:%02d:%03ld \n",tm->tm_hour, tm->tm_min, tm->tm_sec, (tv.tv_usec/1000) );
//        ppm->SetChannel(1,30);
//    }

    Ms3dmGx3Driver driver;
    if(driver.Connect() == true )
    {
        while(1)
        {
            driver.SendCommand(MS_3DMGX3_EULER_RATES);
            MS_3DMGX3_EULER_RATES_STRUCT cmdStruct;
            memset((unsigned char *)&cmdStruct,0,sizeof(MS_3DMGX3_EULER_RATES_STRUCT));
            if(driver.ReceiveCommand((unsigned char*)&cmdStruct,sizeof(MS_3DMGX3_EULER_RATES_STRUCT)) == true)
            {
                if(cmdStruct.m_cCommand == MS_3DMGX3_EULER_RATES)
                {

//                    short responseChecksum = convert2ushort(&response[RESPONSE_SIZE-2]);
//                    short calculatedChecksum = i3dmgx2_Checksum(&response[0], RESPONSE_SIZE-2);

//                    if(calculatedChecksum != responseChecksum)
//                    {
//                            cerr << "calculatedChecksum" << calculatedChecksum << endl;
//                            cerr << "responseChecksum"  <<  responseChecksum << endl;
//                            cerr << "Invalid Checksum"  << endl;
//                            return false;
//                    }

					cmdStruct.m_fPitch = driver.FloatFromBytes((unsigned char *)&cmdStruct.m_fPitch);
					cmdStruct.m_fRoll = driver.FloatFromBytes((unsigned char *)&cmdStruct.m_fRoll);
					cmdStruct.m_fYaw = driver.FloatFromBytes((unsigned char *)&cmdStruct.m_fYaw);


                    if(abs(cmdStruct.m_fYaw - yaw) > 3.1415)
                    {
                        if(cmdStruct.m_fYaw > 0)
                            yawOffset -= 3.1515*2;
                        else
                            yawOffset += 3.1515*2;

                    }
                    yaw = cmdStruct.m_fYaw;


					Eigen::MatrixXd u = Eigen::MatrixXd::Zero(1,1);
					Eigen::MatrixXd y = Eigen::MatrixXd::Zero(1,1);

					y(0,0) = cmdStruct.m_fYaw+yawOffset;

					obs.SetInputs(u,y);

					if(obs.IsStarted() == false)
					{
						Eigen::MatrixXd oState = Eigen::MatrixXd::Zero(1,3);
						oState << cmdStruct.m_fYaw, 0 ,0;
						obs.SetState(oState);
						obs.Start();
						yaw = cmdStruct.m_fYaw;
					}

                    if(con.IsStarted() == false)
                    {
                        Eigen::MatrixXd setPoint = Eigen::MatrixXd::Zero(1,2);
                        target = -1;
                        setPoint << target,0;

                        con.Control(setPoint);


						Eigen::MatrixXd state = Eigen::MatrixXd::Zero(1,2);
						state <<  0,0;
						con.SetState(state);

						con.Start();
					}

					Eigen::MatrixXd ostate = obs.GetState();

					Eigen::MatrixXd state = Eigen::MatrixXd::Zero(1,2);
					state <<  cmdStruct.m_fYaw + yawOffset, ostate(0,1);
					con.SetState(state);


                    double pwm = sin(time);
                    time += 0.03;

//                    float pitch = cmdStruct.m_fPitch;
//                    pitch += 3.1415;
//                    float servo = pitch/ 3.1415 * 100;


                    //get the input
                    double input = con.GetInput()(0,0);
                    ppm->SetChannel(1,pwm*30+50);


                    //yaw control
                    //ppm->SetChannel(0,input*-20+55);

                    //row control
                    ppm->SetChannel(0,input*40+55);

                    if(time > 15)
                    {
                        Eigen::MatrixXd setPoint = Eigen::MatrixXd::Zero(1,2);
                        target = 1;
                        setPoint << target,0;
                        con.Control(setPoint);
                    }
                    if(time > 30)
                    {
                        Eigen::MatrixXd setPoint = Eigen::MatrixXd::Zero(1,2);
                        target = -1;
                        setPoint << target,0;
                        con.Control(setPoint);
                        time = 0;
                    }

//\




                    //LOGGING
                    log.Log(cmdStruct.m_fYaw+yawOffset, target,input);
                    d_graph.Render();


					// Swap frames and Process Events
					glutSwapBuffers();
					glutMainLoopEvent();

					printf("pitch:%0.4f roll:%0.4f yaw:%0.4f\n", cmdStruct.m_fPitch,cmdStruct.m_fRoll, cmdStruct.m_fYaw);
				}
			}

		}

	}

	 ppm->Stop();
	 */
	return 0;
}
