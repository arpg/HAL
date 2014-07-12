#ifndef VEHICLEENUMS_H
#define VEHICLEENUMS_H

// This enum list gives us a way to pluck out parameters from the vector.

enum{
  Width = 0,                    //< Width
  WheelBase = 1,                //< Wheel Base of the car
  Height = 2,                   //< Height of the CG above the vehicle datum
  DynamicFrictionCoef = 3,      //< Friction coefficient, which slows the car
  /// down.
  StaticSideFrictionCoef = 4,   //< Static side friction coefficient,
  /// which enforces nonholonomity
  SlipCoefficient = 5,
  ControlDelay = 6,             //< Control delay of the vehicle (in seconds)
  Mass = 7,
  //WHEEL OPTIONS
  WheelRadius = 8,              //< Radius of the wheels
  WheelWidth = 9,               //< Thickness of the wheels
  TractionFriction = 10,        //< Friction value of the wheels to the ground
  //SUSPENSION OPTIONS
  SuspConnectionHeight = 11,    //< Height of the suspension connection point
  /// above the vehicle datum
  Stiffness = 12,               //< Suspension spring stiffness
  MaxSuspForce = 13,            //< Maximum allowable susp. force, in Newtons
  MaxSuspTravel = 14,           //< Maximum allowable travel distance for the
  /// suspension from the rest length
  SuspRestLength = 15,          //< Rest length of the suspension. This should
  /// be the point where the suspension lies when
  /// there is no contact
  CompDamping = 16,             //< The damping coefficient for when the
  /// suspension is compressed. Set to k * 2.0 *
  /// btSqrt(m_suspensionStiffness)
  /// so k is proportional to critical damping.
  /// k = 0.0 undamped & bouncy, k = 1.0 critical
  /// damping. k = 0.1 to 0.3 are good values.
  ExpDamping = 17,              //< The damping coefficient for when the
  /// suspension is expanding. See the comments
  /// for m_wheelsDampingCompression for how to
  /// set k.
  RollInfluence = 18,
  SteeringCoef = 19,            //< Multiplier coefficient for steering
  MaxSteering = 20,
  MaxSteeringRate = 21,
  AccelOffset = 22,             //< Offset from the center for acceleration
  SteeringOffset = 23,          //< Offset from the center for steering
  ///DC MOTOR COEFFICIENTS
  StallTorqueCoef = 24,
  TorqueSpeedSlope = 25,
  //MAGIC FORMULA PARAMS
  MagicFormula_B  = 26,
  MagicFormula_C  = 27,
  MagicFormula_E = 28
};



#endif // VEHICLEENUMS_H
