/*
 * File:   Robot.h
 * Author: jmf
 *
 * Created on February 28, 2012, 1:58 PM
 */

#ifndef ROBOT_H
#define	ROBOT_H

#include <RPG/Devices.h>

namespace rpg {

class Robot {
public:
	Robot();
	virtual ~Robot();
	bool Init( const std::string& sFile );
private:
//	bool _Parse( const std::string& sFile );
private:
	std::string					m_sName;
	std::map < std::string, CameraDevice* >		m_vCameras;
	// ... add more things

} ;

} // end namespace

#endif	/* ROBOT_H */
