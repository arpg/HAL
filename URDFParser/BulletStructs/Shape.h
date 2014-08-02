// Copyright (c) bminortx

#ifndef URDFPARSER_BULLETSTRUCTS_SHAPE_H_
#define URDFPARSER_BULLETSTRUCTS_SHAPE_H_

#include <string>
#include <vector>

#include "BulletStructs/ModelNode.h"

/////////////////////////////////////////
/// The Shape class
/// A shell for the ModelGraph-Shape subtype
/// It is a superclass holding all of the essential functions and members of our
/// Physics/ModelGraph Shapes.
/////////////////////////////////////////

//// Bullet instantiations reside in PhysicsEngine
//// SceneGraph instantiations are in RenderEngine
//// Both are called in ModelGraphBuilder

class Shape : public ModelNode {
 public:
  // SETTERS
  void SetScale(double s) {
    Eigen::Vector3d scale(s, s, s);
    m_dScale = scale;
  }

  void SetScale(const Eigen::Vector3d& s) {
    m_dScale = s;
  }

  void SetMass(double dMass) {
    m_dMass = dMass;
  }

  void SetRestitution(double dRestitution) {
    m_dRestitution = dRestitution;
  }

  void SetPosition(double x, double y, double z) {
    model_pose_[0] = x;
    model_pose_[1] = y;
    model_pose_[2] = z;
  }

  // GETTERS
  Eigen::Vector3d GetScale() {
    return m_dScale;
  }

  double GetMass() {
    return m_dMass;
  }

  double GetRestitution() {
    return m_dRestitution;
  }

  // Member variables
  Eigen::Vector3d m_dScale;
  double          m_dMass;
  double          m_dRestitution;
};

///////////////////////////////////////////////////////////////////////
///
/// ALL OF OUR PHYSICAL SHAPES
///
///////////////////////////////////////////////////////////////////////

class BoxShape : public Shape {
 public:
  BoxShape(std::string sName, double x_length, double y_length, double z_length,
           double dMass, double dRestitution, std::vector<double> dPose) {
    SetName(sName);
    SetMass(dMass);
    SetRestitution(dRestitution);
    SetPose(dPose);
    SetScale(1);
    m_dBounds << x_length, y_length, z_length;
  }

  Eigen::Vector3d m_dBounds;
};

///////////////////
class CylinderShape : public Shape {
 public:
  CylinderShape(std::string sName, double dRadius, double dHeight,
                double dMass, double dRestitution,
                std::vector<double> dPose):
      m_dRadius(dRadius), m_dHeight(dHeight) {
    SetName(sName);
    SetMass(dMass);
    SetRestitution(dRestitution);
    SetPose(dPose);
    SetScale(1);
  }

  double m_dRadius;
  double m_dHeight;
};

////////////////////
class SphereShape : public Shape {
 public:
  SphereShape(std::string sName, double dRadius,
              double dMass, double dRestitution,
              std::vector<double> dPose):
      m_dRadius(dRadius) {
    SetName(sName);
    SetMass(dMass);
    SetRestitution(dRestitution);
    SetPose(dPose);
    SetScale(1);
  }

  double m_dRadius;
};

/////////////////////
class PlaneShape : public Shape {
 public:
  PlaneShape(std::string sName, std::vector<double> normal,
             std::vector<double> dPose) {
    m_dNormal = normal;
    SetName(sName);
    SetMass(0);
    SetRestitution(0);
    SetPose(dPose);
    SetScale(1);
  }

  std::vector<double> m_dNormal;
};

/////////////////////
/// HeightmapShape is unique; it's really not meant to be used unless
/// we're getting heightmap vertices from MATLAB. Data describing X, Y, and
/// Z are double* because that's the format data comes in when passed through
/// a MEX function.

class HeightmapShape : public Shape {
 public:
  HeightmapShape(std::string sName, int row_count, int col_count,
                 std::vector<double> X, std::vector<double> Y,
                 std::vector<double> Z) {
    SetName(sName);
    SetMass(0);
    SetRestitution(0);
    std::vector<double> dPose;
    dPose.push_back(0.0);
    dPose.push_back(0.0);
    dPose.push_back(0.0);
    dPose.push_back(0.0);
    dPose.push_back(0.0);
    dPose.push_back(0.0);
    SetPose(dPose);
    SetScale(1);
    col_count_ = col_count;
    row_count_ = row_count;
    x_data_ = X;
    y_data_ = Y;
    z_data_ = Z;
  }

  //  Because of the way the constructor is set in bullet_heightmap,
  //  there are a lot of strange member variables here.
  std::vector<double> x_data_;
  std::vector<double> y_data_;
  std::vector<double> z_data_;
  int col_count_;
  int row_count_;
};

//////////////////////

class MeshShape : public Shape {
 public:
  MeshShape(std::string sName, std::string file_dir,
            std::vector<double> dPose) {
    File_Dir = file_dir;
    SetName(sName);
    SetMass(0);
    SetRestitution(0);
    SetPose(dPose);
    SetScale(1);
  }

  std::string GetFileDir() {
    return File_Dir;
  }

  std::string File_Dir;
};


///////////////////////////////////////////////////////////////////////
///
/// ALL OF OUR NON-COLLIDING SHAPES
/// These do not get implemented into Bullet Physics, but are still
/// essential for rendering. Lights fall under this category.
///
///////////////////////////////////////////////////////////////////////

class LightShape : public Shape {
 public:
  LightShape(std::string sName, std::vector<double> position) {
    SetName(sName);
    SetPosition(position[0], position[1], position[2]);
  }
};

// Waypoint is used for plotting trajectories
class WaypointShape : public Shape {
 public:
  WaypointShape(std::string sName, std::vector<double> pose,
                double velocity) {
    SetName(sName);
    SetPose(pose);
    velocity_ = velocity;
  }

  double GetVelocity() {
    return velocity_;
  }

  //  Scales the waypoint
  double velocity_;
};

#endif  // URDFPARSER_BULLETSTRUCTS_SHAPE_H_
