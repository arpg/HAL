/*

   Joint between links.

 */


#ifndef _JOINT_H_
#define _JOINT_H_

#include <RobotGames/ModelNode.h>
#include <RobotGames/Body.h>

class Joint : public ModelNode
{
//        Joint();

//        Joint(Body *A, Body *B, Eigen::Vector3d pose)
//        {
//            m_dAnchor = pose;
//            //        m_BodyA = A;
//            //        m_BodyB = B;
//            m_pParent = A;
//            this->AddChild(B);
//        }

        //    void AddToScene(SceneGraph::GLSceneGraph &glGraph, Eigen::Vector6d pose)
        //    {
        //        Eigen::Vector6d ChildPose;

        //            fprintf(stderr, "Number of children for node '%s': %d\r\n", this->Name(), this->NumChildren());

        //        for (int count=0; count < this->NumChildren(); count++) {
        //            ChildPose = mvl::T2Cart(mvl::Cart2T(pose)*(this->m_Tpo));
        //            m_vChildren[count]->AddToScene( glGraph, ChildPose );
        //        }

        //    }
public:

        Eigen::Vector3d m_dAnchor;
        double          m_dStiffness;
        double          m_dDamping;
        double          m_dLowerLimit;
        double          m_dUpperLimit;
        Body*           m_pParentBody;
        Body*           m_pChildBody;
};

class HingeJoint : public Joint
{
    public:
        HingeJoint( 
                const std::string& sName,
                Body* pParentBody, 
                Body* pChildBody, 

                const double dPivotX,  //< Input: axis x direction
                const double dPivotY,  //< Input: axis y direction
                const double dPivotZ,   //< Input: axis z direction

                const double dAxisX,  //< Input: axis x direction
                const double dAxisY,  //< Input: axis y direction 
                const double dAxisZ   //< Input: axis z direction
                )
        // Creates a Bullet HingeJoint.  Axis is the axis about which Child Body rotates and should be the
        // same in both Parent and Child body.  The Pivot Point is the point about which child B pivots and is expressed
        // in the Parent Reference Frame.
        // Hinges themselves carry no relative pose and are expressed in the Parent Reference frame
        {
            SetName( sName );
            Eigen::Matrix4d Tpo = pChildBody->GetPoseMatrix();
            Eigen::Vector4d xcv; // vector in child "o" frame
            Eigen::Vector4d pcv; // vector in child "o" frame
            xcv << dAxisX, dAxisY, dAxisZ, 1;
            pcv << dPivotX, dPivotY, dPivotZ, 1;
            m_dAxis1 << dAxisX, dAxisY, dAxisZ;
            m_dPivot1 << dPivotX, dPivotY, dPivotZ;

            Eigen::Matrix4d pose;
            pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
            SetPose(pose);

            // link the model graph up
            pParentBody->AddChild( this );
            this->AddChild( pChildBody );

            this->m_pParent = pParentBody;
            pChildBody->m_pParent = this;

            // some default parameters TODO: provide accessors!
            m_dStiffness = 1;
            m_dDamping = 1;
            m_dLowerLimit = 0;
            m_dUpperLimit = M_PI;

            m_pParentBody = pParentBody;
            m_pChildBody = pChildBody;
        }
        HingeJoint(
                const std::string& sName,
                Body* pParentBody,
                Body* pChildBody,

                const double dPivotX,  //< Input: axis x direction
                const double dPivotY,  //< Input: axis y direction
                const double dPivotZ,   //< Input: axis z direction

                const double dAxisX,  //< Input: axis x direction
                const double dAxisY,  //< Input: axis y direction
                const double dAxisZ,   //< Input: axis z direction

                const double dAxis2X,  //< Input: axis x direction
                const double dAxis2Y,  //< Input: axis y direction
                const double dAxis2Z   //< Input: axis z direction
                )
        // Creates a Bullet HingeJoint.  Axis is the axis about which Child Body rotates and should be the
        // same in both Parent and Child body.  The Pivot Point is the point about which child B pivots and is expressed
        // in the Parent Reference Frame.
        // Hinges themselves carry no relative pose and are expressed in the Parent Reference frame
        {
            SetName( sName );
            Eigen::Matrix4d Tpo = pChildBody->GetPoseMatrix();
            Eigen::Vector4d xcv; // vector in child "o" frame
            Eigen::Vector4d pcv; // vector in child "o" frame
            xcv << dAxisX, dAxisY, dAxisZ, 1;
            pcv << dPivotX, dPivotY, dPivotZ, 1;

            m_dAxis1 << dAxisX, dAxisY, dAxisZ;
            m_dAxis2 << dAxis2X, dAxis2Y, dAxis2Z;
            m_dPivot1 << dPivotX, dPivotY, dPivotZ;

            Eigen::Matrix4d pose;
            pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
            SetPose(pose);

            // link the model graph up
            pParentBody->AddChild( this );
            this->AddChild( pChildBody );

            this->m_pParent = pParentBody;
            pChildBody->m_pParent = this;

            // some default parameters TODO: provide accessors!
            m_dStiffness = 1;
            m_dDamping = 1;
            m_dLowerLimit = 0;
            m_dUpperLimit = M_PI;

            m_pParentBody = pParentBody;
            m_pChildBody = pChildBody;
        }

        Eigen::Vector3d m_dAxis1;
        Eigen::Vector3d m_dAxis2;
        Eigen::Vector3d m_dPivot1;

};

class Hinge2Joint : public Joint
{
    public:
        Hinge2Joint(
                std::string sName,
                Body* pParentBody,
                Body* pChildBody,
                const Eigen::Vector3d& dAxis1,
                const Eigen::Vector3d& dAxis2,
                const Eigen::Vector3d& dAnchor,
                const double& dStiffness,
                const double& dDamping,
                const Eigen::Vector3d& dLowerLimit,
                const Eigen::Vector3d& dUpperLimit,
                const Eigen::Vector3d& dLowerALimit,
                const Eigen::Vector3d& dUpperALimit):
            m_dAxis1(dAxis1),
            m_dAxis2(dAxis2),
            m_dAnchor(dAnchor),
            m_dStiffness(dStiffness),
            m_dDamping(dDamping),
            m_dLowerLimit(dLowerLimit),
            m_dUpperLimit(dUpperLimit),
            m_dLowerALimit(dLowerALimit),
            m_dUpperALimit(dUpperALimit)


          // Creates a Bullet Physics Hinge2Joint.  Axis1 is the axis in Parent, Axis2 the axis in Child
          // and Anchor the point about which the 2 bodies connect.  Axis1, Axis2, and Anchor are all
          // initialized in the WORLD COORDINATE SYSTEM!
    {
            SetName( sName );

            Eigen::Matrix4d pose;
            pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
            SetPose(pose);

            // link the model graph up
            pParentBody->AddChild( this );
            this->AddChild( pChildBody );

            this->m_pParent = pParentBody;
            pChildBody->m_pParent = this;

            // some default parameters TODO: provide accessors!
            m_dStiffness = 1;
            m_dDamping = 1;
            m_dLowerLimit << 0, 0, 0;
            m_dUpperLimit << 0, 0, 1;

            m_pParentBody = pParentBody;
            m_pChildBody = pChildBody;
    }

        ////////////////////////////////////////////////////////////////////
        void Attach(Body* pParent, Body*pChild)
        {
            this->SetParent(pParent);
            pParent->AddChild(this);
            this->AddChild(pChild);
        }

        Eigen::Vector3d m_dAxis1;
        Eigen::Vector3d m_dAxis2;
        Eigen::Vector3d m_dAnchor;
        double          m_dStiffness;
        double          m_dDamping;
        Eigen::Vector3d m_dLowerLimit;
        Eigen::Vector3d m_dUpperLimit;
        Eigen::Vector3d m_dLowerALimit;
        Eigen::Vector3d m_dUpperALimit;

};

#endif // JOINT_H

