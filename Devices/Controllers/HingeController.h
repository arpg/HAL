/*
   \file HingeController.h

   Node aware joint controller.

 */

#pragma once

#include <RPG/Utils/PropertyMap.h>
#include <RPG/Utils/SimLauncher.h>
#include <Node/Node.h>


namespace rpg
{

    ///////////////////////////////////////////////////////////////////////////////
    class HingeJointController : public PropertyMap
    {
        public:
            ///////////////////////////////////////////////////////////////
            void ApplyTorque( double dTorque ) 
            {
                m_Node.publish( sRemoteNodeName+"-DesiredTorque", dTorque );
            }

            ///////////////////////////////////////////////////////////////
            void SetDesiredAnlge( double dAngle ) 
            {
                m_Node.publish( sRemoteNodeName+"-DesiredAngle", dAngle );
            }

            ///////////////////////////////////////////////////////////////
            double GetAnlge() 
            {
                m_Node.receive( sRemoteNodeName );
            }

            ///////////////////////////////////////////////////////////////
            void Init( 
                    const std::string& sRemoteNodeName,
                    const std::string& sRobotConfigFile
                    )

            {
                rpg::LaunchSimulationIfNeeded( sRobotConfigFile );

                m_sRemoteNodeName = sRemoteNodeName; 
                m_Node.Init( sRemoteNodeName+"-Controller" );
                m_Node.advertise( sRemoteNodeName+"-DesiredAngle" );
                m_Node.advertise( sRemoteNodeName+"-DesiredTorque" );
                m_Node.subscribe( sRemoteNodeName+"-SensedAngle" );
            }

       private:
            std::string     m_sRemoteNodeName;
            rpg::node       m_Node;
    };

}


