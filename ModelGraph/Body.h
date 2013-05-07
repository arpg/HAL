/*

   An Body is a physical object with mass and shape.

 */

#ifndef _BODY_H_
#define _BODY_H_

#include <Eigen/Eigen>

#include <string>
#include <vector>
#include <map>
#include <stdio.h>

#include <RPG/ModelGraph/ModelNode.h>
#include <RPG/ModelGraph/Shape.h>

class Body : public ModelNode
{
    public:    

        // Body Constructors
//    Body();

    Body(
            std::string sName,
            Shape& rRenderAndCollisionShape,
            double dMass = 0
            )
        {
            SetName( sName );
            if (dynamic_cast<BoxShape*>(&rRenderAndCollisionShape) != NULL)
            {
                BoxShape* pBoxShape = (BoxShape*) (&rRenderAndCollisionShape);
                m_RenderShape = new BoxShape(pBoxShape->m_dDims);
                m_CollisionShape = new BoxShape(pBoxShape->m_dDims);
            }
            else if (dynamic_cast<CylinderShape*>(&rRenderAndCollisionShape) != NULL)
            {
                CylinderShape* pCylinderShape = (CylinderShape*) (&rRenderAndCollisionShape);
                m_RenderShape = new CylinderShape(pCylinderShape->m_dThickness, pCylinderShape->m_dRadius);
                m_CollisionShape = new CylinderShape(pCylinderShape->m_dThickness, pCylinderShape->m_dRadius);
            }
            m_dMass = dMass;
        }

    Body(
            std::string sName,
            Shape& rRenderShape,
            Shape& rCollisionShape,
            double dMass
            ) 
        {
            SetName( sName );
            if (dynamic_cast<BoxShape*>(&rRenderShape) != NULL)
            {
                BoxShape* pRenderBoxShape = (BoxShape*) (&rRenderShape);
                m_RenderShape = new BoxShape(pRenderBoxShape->m_dDims);
            }
            else if (dynamic_cast<CylinderShape*>(&rRenderShape) != NULL)
            {
                CylinderShape* pCylinderShape = (CylinderShape*) (&rRenderShape);
                m_RenderShape = new CylinderShape(pCylinderShape->m_dThickness, pCylinderShape->m_dRadius);
            }
            if (dynamic_cast<BoxShape*>(&rCollisionShape) != NULL)
            {
                BoxShape* pCollisionBoxShape = (BoxShape*) (&rCollisionShape);
                m_CollisionShape = new BoxShape(pCollisionBoxShape->m_dDims);
            }
            else if (dynamic_cast<CylinderShape*>(&rCollisionShape) != NULL)
            {
                CylinderShape* pCCylinderShape = (CylinderShape*) (&rCollisionShape);
                m_CollisionShape = new CylinderShape(pCCylinderShape->m_dThickness, pCCylinderShape->m_dRadius);
            }

            m_dMass = dMass;
        }


/*
    void Attach( 
            Body*        pChildBody,
            HingeJoint*  pJoint,
            const double dX,      //< Input: x base position of axis
            const double dY,      //< Input: y base position of axis
            const double dZ,      //< Input: z base position of axis
            const double dAxisX,  //< Input: axis x direction
            const double dAxisY,  //< Input: axis y direction 
            const double dAxisZ   //< Input: axis z direction
            )
    {
        this->AddChild( pJoint );
        pJoint->AddChild( pChildBody );
    }

        Body( const std::string& name)
        {
            // TODO make this independent of SceneGraph
            // Have SceneGraph "attach" the appropriate shape, just as Sim
            // attaches the appropriate bullet rigidBody.
            m_sType = "Body";
            if (name == "Chassis") {
                m_pRenderShape = new SceneGraph::GLBox;
                ((SceneGraph::GLBox *)m_pRenderShape)->SetExtent(2, 3, 1);
                m_dMass = 1;
                m_pShape = new BoxShape(m_pRenderShape->ObjectBounds().HalfSizeFromOrigin()) ;
            }
//            else if (name == "Wheel"){
//                SceneGraph::GLCube wheel;
//                m_pShape = new CylinderShape(1, 0.1);
//                m_pRenderShape = &wheel;
//            }
        }
*/

        ////////////////////////////////////////////////////////////////////
/*
        /// TODO move me elsewhere
        void AddToScene( SceneGraph::GLSceneGraph &glGraph, Eigen::Vector6d pose )
        {
            Eigen::Vector6d ChildPose;

            m_pRenderShape->SetPose(pose);
            glGraph.AddChild(m_pRenderShape);

//            fprintf(stderr, "Number of children for box node '%s': %d\r\n", this->Name(), this->NumChildren());

            for (int count=0; count < this->NumChildren(); count++) {
                ChildPose = mvl::T2Cart(mvl::Cart2T(pose)*(this->m_Tpo));
                m_vChildren[count]->AddToScene( glGraph, ChildPose );
            }
        }
 */

        /// Set bodies scale
        void SetScale( double s );
        void SetScale( const Eigen::Vector3d& s );
        Eigen::Vector3d GetScale();

        Eigen::Vector3d                m_dScale;
        double                         m_dMass;
        Shape*                         m_RenderShape;
        Shape*                         m_CollisionShape;
};

#endif
