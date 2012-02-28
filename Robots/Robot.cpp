/*
 * File:   Robot.cpp
 * Author: jmf
 *
 * Created on February 28, 2012, 1:58 PM
 */

#include "Robot.h"

using namespace rpg;

Robot::Robot()
{
}

Robot::Robot(const Robot& orig)
{
}

Robot::~Robot()
{
}

void Robot::Init( std::string sFile )
{
	/*
	// open file and parse it
	// fill devices into the vector arrays
	// for each camera in urdf
		CameraDevice* pCam = new CameraDevice;
		m_vCameras[NameFromURDF] = pCam;
		if(TypeFromURDF == SIM) {
			// get host property to construct
			// instantiate SimClient (if not already instantiated)
		}
		// set properties (if any)
		// loop
			pCam->SetProperty(PropertyFromURDF, ValueFromURDF );
		// end loop
		// if type Sim, add SimClient pointer as a property
		// init driver
		m_vCameras[NameFromURDF].Init(DevFromURDF);
	// close loop camera
	*/
}