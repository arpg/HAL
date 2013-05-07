/*

   Joint between links (bodies).

*/


#ifndef _JOINT_H_
#define _JOINT_H_

#include <RPG/ModelGraph/ModelNode.h>
#include <RPG/ModelGraph/Body.h>

class Joint : public ModelNode
{

public:

        Eigen::Vector3d m_dAnchor;
        double          m_dStiffness;
        double          m_dDamping;
        double          m_dLowerLimit;
        double          m_dUpperLimit;
        Body*           m_pParentBody;
        Body*           m_pChildBody;
};

class GenericJoint : public Joint
{
public:
    GenericJoint(
            const std::string& sName,
            Body* pParentBody,
            Body* pChildBody,
            Eigen::Vector6d TransformA,
            Eigen::Vector6d TransformB,
            bool useLinear
            )
    {
        m_sName = sName;
        m_pParentBody = pParentBody;
        m_pChildBody = pChildBody;
        m_bUseLinear = useLinear;
        m_TransformA = TransformA;
        m_TransformB = TransformB;
    }

private:
    bool m_bUseLinear;
    Eigen::Vector6d m_TransformA;
    Eigen::Vector6d m_TransformB;


};

class Point2Point : public Joint
{


public:
    Eigen::Vector3d m_Axis1;
    Eigen::Vector3d m_Axis2;

    Point2Point(
            const std::string& sName,
            Body* pParentBody,
            Body* pChildBody,
            Eigen::Vector3d Axis
            )
    {
        m_sName = sName;
        m_pParentBody = pParentBody;
        m_pChildBody = pChildBody;
        m_Axis1 = Axis;
    }

    Point2Point(
            const std::string& sName,
            Body* pParentBody,
            Body* pChildBody,
            Eigen::Vector3d AxisA,
            Eigen::Vector3d AxisB
            )
    {
        m_sName = sName;
        m_pParentBody = pParentBody;
        m_pChildBody = pChildBody;
        m_Axis1 = AxisA;
        m_Axis2 = AxisB;
    }

private:


};

class Slider : public Joint
{

public:
    Eigen::Vector6d m_TransformA;
    Eigen::Vector6d m_TransformB;


    Slider(
            const std::string& sName,
            Body* pParentBody,
            Body* pChildBody,
            Eigen::Vector6d TransformA,
            Eigen::Vector6d TransformB,
            bool useLinear
            )
    {
        m_sName = sName;
        m_pParentBody = pParentBody;
        m_pChildBody = pChildBody;
        m_bUseLinear = useLinear;
        m_TransformA = TransformA;
        m_TransformB = TransformB;
    }

    bool UseLinear( void )
    {
        return m_bUseLinear;
    }

private:
    bool m_bUseLinear;
};


class ConeTwist : public Joint
{
public:
    ConeTwist(
            const std::string& sName,
            Body* pParentBody,
            Body* pChildBody,
            Eigen::Vector6d TransformA,
            Eigen::Vector6d TransformB,
            bool useLinear
            )
    {
        m_sName = sName;
        m_pParentBody = pParentBody;
        m_pChildBody = pChildBody;
        m_bUseLinear = useLinear;
        m_TransformA = TransformA;
        m_TransformB = TransformB;
    }

private:
    bool m_bUseLinear;
    Eigen::Vector6d m_TransformA;
    Eigen::Vector6d m_TransformB;
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

