// Copyright (c) bminortx

#ifndef URDFPARSER_BULLETSTRUCTS_CONSTRAINT_H_
#define URDFPARSER_BULLETSTRUCTS_CONSTRAINT_H_

#include <string>

#include "BulletStructs/ModelNode.h"
#include "BulletStructs/Shape.h"

class Constraint : public ModelNode {
  //  I'm not sure what to put here; it's just useful for categorization.
};

//////////////////////////////////////////
/// TYPES OF CONSTRAINTS
/// The number after the typename in the class refers to the number of shapes
/// that the constraint connects; i.e. PToPOne is a constraint that
/// fixes the postion of one shape, while PToPTwo ties two shapes together.
//////////////////////////////////////////

//////////////
/// POINT-TO-POINT
//////////////

class PToPOne : public Constraint {
 public:
  PToPOne(std::string sName, Shape* Shape_A, Eigen::Vector3d pivot_in_A) {
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_pivot_in_A = pivot_in_A;

    //  Connect the shapes
    Shape_A->AddChild(this);
    this->model_parent_.reset(Shape_A);
  }
  std::string m_Shape_A;
  Eigen::Vector3d m_pivot_in_A;
};

class PToPTwo : public Constraint {
 public:
  PToPTwo(std::string sName, Shape* Shape_A, Shape* Shape_B,
          Eigen::Vector3d pivot_in_A, Eigen::Vector3d pivot_in_B) {
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_Shape_B = Shape_B->GetName();
    m_pivot_in_A = pivot_in_A;
    m_pivot_in_B = pivot_in_B;

    //  Connect the shapes
    Shape_A->AddChild(this);
    this->AddChild(Shape_B);
    this->model_parent_.reset(Shape_A);
    Shape_B->model_parent_.reset(this);
  }
  std::string m_Shape_A;
  std::string m_Shape_B;
  Eigen::Vector3d m_pivot_in_A;
  Eigen::Vector3d m_pivot_in_B;
};

//////////////
/// HINGE
//////////////
class HingeOnePivot : public Constraint {
 public:
  HingeOnePivot(std::string sName, Shape* Shape_A,
                Eigen::Vector3d pivot_in_A,
                Eigen::Vector3d Axis_in_A) {
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_axis_in_A = Axis_in_A;
    m_pivot_in_A = pivot_in_A;
    m_low_limit = -1;
    m_high_limit = 1;
    m_softness = .9;
    m_bias =  .3;
    m_relaxation = 1;

    //  Connect the shapes
    Shape_A->AddChild(this);
    this->model_parent_.reset(Shape_A);
  }

  void SetLimits(double low_limit, double high_limit, double softness,
                 double bias, double relaxation) {
    m_low_limit = low_limit;
    m_high_limit = high_limit;
    m_softness = softness;
    m_bias =  bias;
    m_relaxation = relaxation;
  }

  std::string m_Shape_A;
  Eigen::Vector3d m_axis_in_A;
  Eigen::Vector3d m_pivot_in_A;
  double m_low_limit;
  double m_high_limit;
  double m_softness;
  double m_bias;
  double m_relaxation;
};

class HingeTwoPivot : public Constraint {
 public:
  HingeTwoPivot(std::string sName, Shape* Shape_A, Shape* Shape_B,
                Eigen::Vector3d pivot_in_A, Eigen::Vector3d pivot_in_B,
                Eigen::Vector3d Axis_in_A, Eigen::Vector3d Axis_in_B) {
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_Shape_B = Shape_B->GetName();
    m_axis_in_A = Axis_in_A;
    m_axis_in_B = Axis_in_B;
    m_pivot_in_A = pivot_in_A;
    m_pivot_in_B = pivot_in_B;
    m_low_limit = -1;
    m_high_limit = 1;
    m_softness = .9;
    m_bias =  .3;
    m_relaxation = 1;

    //  Connect the shapes
    Shape_A->AddChild(this);
    this->AddChild(Shape_B);
    this->model_parent_.reset(Shape_A);
    Shape_B->model_parent_.reset(this);
  }

  void SetLimits(double low_limit, double high_limit, double softness,
                 double bias, double relaxation) {
    m_low_limit = low_limit;
    m_high_limit = high_limit;
    m_softness = softness;
    m_bias =  bias;
    m_relaxation = relaxation;
  }

  std::string m_Shape_A;
  std::string m_Shape_B;
  Eigen::Vector3d m_axis_in_A;
  Eigen::Vector3d m_axis_in_B;
  Eigen::Vector3d m_pivot_in_A;
  Eigen::Vector3d m_pivot_in_B;
  double m_low_limit;
  double m_high_limit;
  double m_softness;
  double m_bias;
  double m_relaxation;
};

////////
/// HINGE 2
////////

class Hinge2 : public Constraint {
 public:
  Hinge2(std::string sName, Shape* Shape_A, Shape* Shape_B,
         Eigen::Vector3d Anchor,
         Eigen::Vector3d Axis_1, Eigen::Vector3d Axis_2) {
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_Shape_B = Shape_B->GetName();
    m_Anchor = Anchor;
    m_Axis_1 = Axis_1;
    m_Axis_2 = Axis_2;
    m_damping = 50;
    m_stiffness = 1;
    m_LowerLinLimit = Eigen::Vector3d::Identity();

    //  Connect the shapes
    Shape_A->AddChild(this);
    this->AddChild(Shape_B);
    this->model_parent_.reset(Shape_A);
    Shape_B->model_parent_.reset(this);
  }


  void SetLimits(double damping, double stiffness,
                 Eigen::Vector3d LinLowLimit, Eigen::Vector3d LinUppLimit,
                 Eigen::Vector3d AngLowLimit, Eigen::Vector3d AngUppLimit) {
    m_damping = damping;
    m_stiffness = stiffness;
    m_LowerLinLimit = LinLowLimit;
    m_UpperLinLimit = LinUppLimit;
    m_LowerAngLimit = AngLowLimit;
    m_UpperAngLimit = AngUppLimit;
  }

  std::string m_Shape_A;
  std::string m_Shape_B;
  Eigen::Vector3d m_Anchor;
  Eigen::Vector3d m_Axis_1;
  Eigen::Vector3d m_Axis_2;
  double m_damping;
  double m_stiffness;
  Eigen::Vector3d m_LowerLinLimit;
  Eigen::Vector3d m_UpperLinLimit;
  Eigen::Vector3d m_LowerAngLimit;
  Eigen::Vector3d m_UpperAngLimit;
};


/////////////
///// Six DOF
/////////////

// NB: These transforms in the constructors for 6DOF constraints
// are based off of the reference frame of the parent shape.

class SixDOFOne : public Constraint {
 public:
  SixDOFOne(std::string sName, Shape* Shape_A, Eigen::Vector6d Transform_A) {
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_Transform_A = Transform_A;

    //  Connect the shapes
    Shape_A->AddChild(this);
    this->model_parent_.reset(Shape_A);
  }

  void SetLimits(Eigen::Vector3d LinLowLimit, Eigen::Vector3d LinUppLimit,
                 Eigen::Vector3d AngLowLimit, Eigen::Vector3d AngUppLimit) {
    m_LowerLinLimit = LinLowLimit;
    m_UpperLinLimit = LinUppLimit;
    m_LowerAngLimit = AngLowLimit;
    m_UpperAngLimit = AngUppLimit;
  }

  std::string m_Shape_A;
  Eigen::Vector6d m_Transform_A;
  Eigen::Vector3d m_LowerLinLimit;
  Eigen::Vector3d m_UpperLinLimit;
  Eigen::Vector3d m_LowerAngLimit;
  Eigen::Vector3d m_UpperAngLimit;
};

class SixDOFTwo : public Constraint {
 public:
  SixDOFTwo(std::string sName, Shape* Shape_A, Shape* Shape_B,
            Eigen::Vector6d Transform_A, Eigen::Vector6d Transform_B) {
    SetName(sName);
    m_Shape_A = Shape_A->GetName();
    m_Shape_B = Shape_B->GetName();
    m_Transform_A = Transform_A;
    m_Transform_B = Transform_B;

    //  Connect the shapes
    Shape_A->AddChild(this);
    this->AddChild(Shape_B);
    this->model_parent_.reset(Shape_A);
    Shape_B->model_parent_.reset(this);
  }

  void SetLimits(Eigen::Vector3d LinLowLimit, Eigen::Vector3d LinUppLimit,
                 Eigen::Vector3d AngLowLimit, Eigen::Vector3d AngUppLimit) {
    m_LowerLinLimit = LinLowLimit;
    m_UpperLinLimit = LinUppLimit;
    m_LowerAngLimit = AngLowLimit;
    m_UpperAngLimit = AngUppLimit;
  }

  std::string m_Shape_A;
  std::string m_Shape_B;
  Eigen::Vector6d m_Transform_A;
  Eigen::Vector6d m_Transform_B;
  Eigen::Vector3d m_LowerLinLimit;
  Eigen::Vector3d m_UpperLinLimit;
  Eigen::Vector3d m_LowerAngLimit;
  Eigen::Vector3d m_UpperAngLimit;
};

#endif  // URDFPARSER_BULLETSTRUCTS_CONSTRAINT_H_
