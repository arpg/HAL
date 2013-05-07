/* 
 
   A Model is the base class used to define thigns like Robots

 */

#ifndef _MODEL_H_
#define _MODEL_H_

#include <Eigen/Eigen>

#include <string>
#include <vector>
#include <map>
#include <stdio.h>

<<<<<<< HEAD
#include <RPG/ModelGraph/ModelNode.h>
#include <RPG/ModelGraph/Body.h>
#include <RPG/ModelGraph/Joint.h>
=======
#include <ModelGraph/ModelNode.h>
#include <ModelGraph/Body.h>
#include <ModelGraph/Joint.h>
>>>>>>> ffb7b6953a74ed2eb5a15f34dd2302855651e229


class Model : public ModelNode
{    
    public:
        void SetBase( Body* pBase )
        {
            pBase->m_pParent = this;
            this->AddChild( pBase );
        }


//   
//    {
//        Eigen::Vector6d ChildPose;

//        fprintf(stderr, "Number of children for node '%s': %d\r\n", this->Name(), this->NumChildren());

//        for (int count=0; count < this->NumChildren(); count++) {
//            ChildPose = mvl::T2Cart(mvl::Cart2T(pose)*(this->m_Tpo));
//            this[count].AddToScene( glGraph, ChildPose );
//        }
//    }
};

#endif
