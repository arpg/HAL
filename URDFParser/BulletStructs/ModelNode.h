#ifndef _MODEL_NODE_H_
#define _MODEL_NODE_H_

#include <string>
#include <vector>
#include <map>
#include <stdio.h>
#include <URDFParser/EigenHelpers.h>

/*********************************************
  *
  * MODEL NODE
  * ModelNode can be thought of as our class of parts. It is further broken down
  * into subclasses of Shapes, Constraints, and RaycastVehicles (a unique body).
  * ModelNodes can have child nodes connected, and each ModelNode has a unique
  * name.
  *
  *********************************************/

class ModelNode
{
public:

  ModelNode()
  {
    m_nId = -1;
    m_pParent = NULL;
    m_vChildren.clear();
    m_sName = "ModelNode";
    m_dPose = Eigen::Vector6d::Identity();
  }

  ////////////////////////////////////////////////////////////////////
  // ModelNode Properties
  virtual const char* Name() const{
    return m_sName.c_str();
  }

  ////////////////////////////////////////////////////////////////////
  /// SETTERS

  void SetBase( ModelNode* pBase )
  {
    pBase->m_pParent = this;
    this->AddChild( pBase );
  }

  void SetName( const std::string& sName ){
    m_sName = sName;
  }

  /// Set the parent of this node
  void SetParent( ModelNode* pParent ){
    m_pParent = pParent;
  }

  void SetPose(double x, double y, double z, double p, double q, double r){
    m_dPose<<x,y,z,r,p,q;
  }

  void SetPose ( std::vector<double> dPose ){
    SetPose(dPose[0], dPose[1], dPose[2],
            dPose[3], dPose[4], dPose[5]);
  }

  /// Set pose as (x,y,z,roll,pitch,yaw) vector
  void SetPose(const Eigen::Vector6d& v){
    SetPose(v(0), v(1), v(2), v(3), v(4), v(5));
  }

  /// Set pose as 4x4 matrix
  void SetPose( const Eigen::Matrix4d& Two ){
    m_dPose = _T2Cart(Two);
  }

  /// Set position only (x,y,z)
  void SetPosition(double x, double y, double z){
    SetPose(x,y,z,m_dPose(3), m_dPose(4), m_dPose(5));
  }

  void SetPosition( std::vector<double> dPosition ){
    SetPosition(dPosition[0], dPosition[1], dPosition[2]);
  }

  /// Set position only (x,y,z)
  void SetPosition( Eigen::Vector3d v ){
    SetPosition(v(0),v(1),v(2));
  }



  ////////////////////////////////////////////////////////////////////
  /// GETTERS

  std::string GetName( void ){
    return m_sName;
  }

  /// Return pose as (x,y,z,roll,pitch,yaw) vector
  Eigen::Vector6d GetPose() const{
    return m_dPose;
  }

  // Pose (in parent frame)
  Eigen::Matrix4d GetPoseMatrix() const{
    return _Cart2T(m_dPose);
  }

  Eigen::Vector3d GetPositon(){
    return Eigen::Vector3d(m_dPose(0), m_dPose(1), m_dPose(2));
  }

  Eigen::Matrix3d GetRotationMatrix(){
    Eigen::Vector3d rpq;
    rpq<<m_dPose(3), m_dPose(4), m_dPose(5);
    return _Cart2R(rpq(0), rpq(1), rpq(2));
  }

  ////////////////////////////////////////////////////////////////////
  // CHILDREN FUNCTIONS

  void AddChild( ModelNode* pChild ){
    m_vChildren.push_back( pChild );
  }

  bool RemoveChild( ModelNode* pChild ){
    for(size_t ii = 0 ; ii < m_vChildren.size() ; ii++) {
      if(m_vChildren[ii] == pChild ){
        m_vChildren.erase(m_vChildren.begin()+ii);
        return true;
      }
    }
    return false;
  }

  size_t NumChildren() const{
    return m_vChildren.size();
  }

  /// Access child bodies
  ModelNode& operator[](int i){
    return *m_vChildren[i];
  }

  const ModelNode& operator[](int i) const{
    return *m_vChildren[i];
  }

  ////////////////////////////////////////////////////////////////////
  /// MEMBER VARIABLES

  static std::map<int,ModelNode*>  g_mNodes; // static map of id to node
  static int                       g_nHandleCounter;
  int                              m_nId;
  ModelNode*                       m_pParent;
  std::vector<ModelNode*>          m_vChildren;
  std::string                      m_sName;
  Eigen::Vector6d                  m_dPose;  // object in World frame
  std::string                      m_sType;

};








#endif
