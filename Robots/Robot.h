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
	void Init( const std::string& sFile );
private:
	bool _Parse( const std::string& sFile );
private:
	std::string									m_sName;

//	SimClient									m_pSimClient;
	std::map < std::string, CameraDevice* >		m_vCameras;
	// ... add more things

} ;

} // end namespace

#endif	/* ROBOT_H */

/*

<robot name="pr2">
	<sensors>
		<camera name="Bumblebee1" dev="Bumblebee" type="real">
			<lcmod> lcmod.xml </lcmod>
			<rcmod> rcmod.xml </rcmod>
		</camera>
		<camera name="Bumblebee2" dev="SimStereo" type="sim">
			<lcmod> lcmod.xml </lcmod>
			<rcmod> rcmod.xml </rcmod>
			<host> w.x.y.z:post </host>
		</camera>
		<camera name="LeftArm" dev="FileReader" type="real">
			<lcmod> lcmod.xml </lcmod>
			<rcmod> rcmod.xml </rcmod>
			<lfile> left*.pgm </lfile>
			<rfile> right*.pgm </rfile>
		</camera>

		<rangers>
		</rangers>
		<imu>
		</imu>
	</sensor>

	<link> ... </link>
	<link> ... </link>
	<link> ... </link>

	<joint>  ....  </joint>
	<joint>  ....  </joint>
	<joint>  ....  </joint>
</robot>



 */