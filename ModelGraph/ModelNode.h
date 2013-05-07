#ifndef _MODEL_NODE_H_
#define _MODEL_NODE_H_

#include <Eigen/Eigen>
//#include <SceneGraph/SceneGraph.h>
//#include <Mvlpp/Mvl.h>
#include <string>
#include <vector>
#include <map>
#include <stdio.h>
#include <ModelGraph/SE3.h> // TODO -- use Eigen equiv.

/////////////////////////////////////////////////////////////////////////////////
namespace Eigen
{
    typedef Matrix<double, 6, 1> Vector6d;
    typedef std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > Vector6dAlignedVec;
}


/////////////////////////////////////////////////////////////////////////////////
class ModelNode
{
    public:

        ModelNode()
        {
            m_nId = -1;
            m_pParent = NULL;
            m_vChildren.clear();
            m_sName = "ModelNode";
            m_Tpo = Eigen::Matrix4d::Identity();
        }

        ////////////////////////////////////////////////////////////////////
        // ModelNode Properties
        virtual const char* Name() const
        {
            return m_sName.c_str();
        }

        ////////////////////////////////////////////////////////////////////
        void SetName( const std::string& sName )
        {
            m_sName = sName;
        }

        ////////////////////////////////////////////////////////////////////
        std::string GetName( void )
        {
            return m_sName;
        }

        ////////////////////////////////////////////////////////////////////
        // Pose (in parent frame)
        Eigen::Matrix4d GetPoseMatrix() const
        {
            return m_Tpo;
        }

        ////////////////////////////////////////////////////////////////////
        /// Return pose as (x,y,z,roll,pitch,yaw) vector
        Eigen::Vector6d GetPose() const
        {
            return _T2Cart( m_Tpo );
        }

        ////////////////////////////////////////////////////////////////////
        /// Return pose as (x,y,z,roll,pitch,yaw) vector
        Eigen::Vector6d GetWPose() const
        {
            return _T2Cart( m_Two );
        }

        ////////////////////////////////////////////////////////////////////
        /// Set the parent of this node
        void SetParent( ModelNode* pParent )
        {
            m_pParent = pParent;
        }

        ////////////////////////////////////////////////////////////////////
        /// Set position only (x,y,z)
        void SetPosition( Eigen::Vector3d v )
        {
            SetPosition(v(0),v(1),v(2));
        }


        ////////////////////////////////////////////////////////////////////
        /// Set position only (x,y,z)
        void SetPosition(double x, double y, double z = 0)
        {
            SetPose(x,y,z,0,0,0);
        }

        ////////////////////////////////////////////////////////////////////
        /// Set pose as (x,y,z,roll,pitch,yaw) vector
        void SetPose(const Eigen::Vector6d& v)
        {
            SetPose(v(0), v(1), v(2), v(3), v(4), v(5));
        }

        ////////////////////////////////////////////////////////////////////
        /// Set pose as 4x4 matrix
        void SetPose( const Eigen::Matrix4d& Tpo )
        {
            m_Tpo = Tpo;
        }


        ////////////////////////////////////////////////////////////////////
        /// Set pose as 4x4 matrix
        void SetWPose( const Eigen::Matrix4d& Two )
        {
            m_Two = Two;
        }

        ////////////////////////////////////////////////////////////////////
        /// Set pose using x,y,z,roll,pitch,yaw parametrisation
        void SetPose(double x, double y, double z, double p, double q, double r)
        {
            m_Tpo = _Cart2T(x,y,z,r,p,q);
        }

        ////////////////////////////////////////////////////////////////////
        // Children
        void AddChild( ModelNode* pChild )
        {
            m_vChildren.push_back( pChild );
        }

        ////////////////////////////////////////////////////////////////////
        bool RemoveChild( ModelNode* pChild )
        {
            for(size_t ii = 0 ; ii < m_vChildren.size() ; ii++) {
                if(m_vChildren[ii] == pChild ){
                    m_vChildren.erase(m_vChildren.begin()+ii);
                    return true;
                }
            }
            return false;
        }

        ////////////////////////////////////////////////////////////////////
        size_t NumChildren() const
        {
            return m_vChildren.size();
        }

        ////////////////////////////////////////////////////////////////////
        /// Access child bodies
        ModelNode& operator[](int i)
        {
            return *m_vChildren[i];
        }

        ////////////////////////////////////////////////////////////////////
        const ModelNode& operator[](int i) const
        {
            return *m_vChildren[i];
        }

        /*
        ////////////////////////////////////////////////////////////////////
        virtual void AddToScene( SceneGraph::GLSceneGraph &glGraph, Eigen::Vector6d pose )
        {
            Eigen::Vector6d ChildPose;

//            fprintf(stderr, "Number of children for node '%s': %d\r\n", this->Name(), this->NumChildren());

            for (int count=0; count < this->NumChildren(); count++) {
                ChildPose = mvl::T2Cart(mvl::Cart2T(pose)*(this->m_Tpo));
                m_vChildren[count]->AddToScene( glGraph, ChildPose );
            }
        }
        */

        ////////////////////////////////////////////////////////////////////
        // static map of id to node
        static std::map<int,ModelNode*>  g_mNodes;
        static int                       g_nHandleCounter;
        int                              m_nId;

//    private:

        ModelNode*                     m_pParent;
        std::vector<ModelNode*>        m_vChildren;
        std::string                    m_sName;
        Eigen::Matrix4d                m_Tpo;  // object in parent's frame
        Eigen::Matrix4d                m_Two;  // object in parent's frame
        std::string                    m_sType;


};

#endif
