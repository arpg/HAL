/*
   \file JointController.h

   Node aware joint controller.

 */

#ifndef _JOINT_DEVICE_H_
#define _JOINT_DEVICE_H_

#include <RPG/Utils/PropertyMap.h>

namespace rpg
{

    ///////////////////////////////////////////////////////////////////////////////
    class Hinge2JointController : public PropertyMap
    {
        public:
            ///////////////////////////////////////////////////////////////
            Hinge2JointController()
            {
            }

            ///////////////////////////////////////////////////////////////
            ~Hinge2JointController()
            {
            }

            ///////////////////////////////////////////////////////////////
            void ApplyTorque( double dTorque ) 
            {

            }

            ///////////////////////////////////////////////////////////////
            void SetDesiredAnlge( double dAngle ) 
            {

            }

            ///////////////////////////////////////////////////////////////
            double GetAnlge() 
            {

            }

       private:
            rpg::node  m_pNode;
    };


    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    class HingeJointController : public PropertyMap
    {
        public:
            ///////////////////////////////////////////////////////////////
            HingeJointController()
            {

            }

            ///////////////////////////////////////////////////////////////
            ~HingeJointController()
            {
                if( m_pDriver ) {
                    delete m_pDriver;
                }
            }

            ///////////////////////////////////////////////////////////////
            void ApplyTorque( double dTorque ) 
            {

            }

            ///////////////////////////////////////////////////////////////
            void SetDesiredAnlge( double dAngle ) 
            {

            }

            ///////////////////////////////////////////////////////////////
            double GetAnlge() 
            {

            }

       private:
            rpg::node  m_pNode;
    };

}
#endif

