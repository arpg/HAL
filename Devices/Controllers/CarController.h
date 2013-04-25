/*
   \file CarController.h

 */

#pragma once

#include <RPG/Utils/PropertyMap.h>
#include <RPG/Utils/SimLauncher.h>
#include <Node/Node.h>


namespace rpg
{

    ///////////////////////////////////////////////////////////////////////////////
    class CarController : public PropertyMap
    {
        public:
            ///////////////////////////////////////////////////////////////
            void SetVelocity( Eigen::Vector2d dV ) 
            {
                m_Node.publish( sRemoteNodeName+"-DesiredVelocity", dV );
            }

            ///////////////////////////////////////////////////////////////
            Eigen::Vector2d GetVelocity() 
            {
            }

            ///////////////////////////////////////////////////////////////
            void Init( 
                    const std::string& sRemoteNodeName,
                    const std::string& sRobotConfigFile
                    )

            {
                rpg::LaunchSimulationIfNeeded( sRobotConfigFile );

                m_sRemoteNodeName = sRemoteNodeName; 
                m_Node.Init( sRemoteNodeName+"-CarController" );
                m_Node.advertise( sRemoteNodeName+"-DesiredVelocity" );
                m_Node.subscribe( sRemoteNodeName+"-SensedVelocity" );
            }

        private:
            std::string     m_sRemoteNodeName;
            rpg::node       m_Node;
    };
}

