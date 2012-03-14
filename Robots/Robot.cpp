/*
 * File:   Robot.cpp
 * Author: jmf
 *
 * Created on February 28, 2012, 1:58 PM
 */

#include "Robot.h"

#include "XML/rg_tinyxml.h"

using namespace rpg;

Robot::Robot()
{
}

Robot::~Robot()
{
}

bool Robot::Init( const std::string& sFile )
{
	TiXmlDocument doc( sFile.c_str() );
	if (!doc.LoadFile()) {
		std::cout << "Error loading config file." << std::endl;
		return false;
	}

	TiXmlHandle hDoc(&doc);
	TiXmlHandle hRoot(0);
	TiXmlNode* pNode;
	TiXmlElement* pElem;

    pElem = hDoc.FirstChildElement().Element();

	// should always have a valid root but handle gracefully with failure to load via XML if it does not
    if (!pElem) {
		std::cout << "File empty." << std::endl;
		return false;
	}

    // since pElem is good then set the Root from it
    hRoot = TiXmlHandle(pElem);

	pElem->QueryValueAttribute( "name", &m_sName );

    // Path Tree is a child of the Root
    pNode = hRoot.FirstChild( "sensors" ).FirstChild( "camera" ).Node();

//	SimClient *pSimClient = NULL;

    for( pNode; pNode; pNode=pNode->NextSibling())
    {
		pElem = pNode->ToElement();
		std::string sName, sDev, sType;
		pElem->QueryValueAttribute( "name", &sName );
		pElem->QueryValueAttribute( "dev", &sDev );
		pElem->QueryValueAttribute( "type", &sType );

		CameraDevice *pCam = new CameraDevice;

		if( sType == "sim" ) {
//			if(pSimClient == NULL) {
//				pSimClient = new SimClient();
//			}
//			pCam->GetProperty("SimClientPtr",pSimClient);
		}

		pElem = pNode->FirstChildElement();
		for( pElem; pElem; pElem=pElem->NextSiblingElement())
		{
			const char* pKey = pElem->Value();
			const char* pText = pElem->GetText();

			pCam->SetProperty(pKey, pText);
		}

		if( !pCam->InitDriver( sDev ) ) {
			std::cout << "'" << sDev << "' is an invalid input device." << std::endl;
			exit(0);
		}

		// add new camera to map list
		m_vCameras[sName] = pCam;
    }


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

/*
bool Robot::_Parse( const std::string& sFile,	//< Input:
					SimClient& pClient			//< Output:
				  )
{
	TiXmlDocument doc( sFile.c_str() );
	if (!doc.LoadFile()) {
		std::cout << "Error loading config file." << std::endl;
		return false;
	}

	TiXmlHandle hDoc(&doc);
	TiXmlHandle hRoot(0);
	TiXmlNode* pNode;
	TiXmlElement* pElem;

    pElem = hDoc.FirstChildElement().Element();

	// should always have a valid root but handle gracefully with failure to load via XML if it does not
    if (!pElem) {
		std::cout << "File empty." << std::endl;
		return false;
	}

    // since pElem is good then set the Root from it
    hRoot = TiXmlHandle(pElem);

	pElem->QueryValueAttribute( "name", &m_sName );

    // Path Tree is a child of the Root
    pNode = hRoot.FirstChild( "sensors" ).FirstChild( "camera" ).Node();
    for( pNode; pNode; pNode=pNode->NextSibling())
    {
		pElem = pNode->FirstChildElement();
		for( pElem; pElem; pElem=pElem->NextSiblingElement())
		{
			const char* pKey = pElem->Value();
			const char* pText = pElem->GetText();
			std::cout << pKey << " has " << pText << std::endl;
			// init cameras with those properties
			// insert in camera vector
		}
    }

	return true;
}
*/