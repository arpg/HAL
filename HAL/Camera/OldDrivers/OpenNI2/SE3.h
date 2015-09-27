/// Sigh

#ifndef _RG_SE3_H_
#define _RG_SE3_H_

#include <Eigen/Eigen>

inline Eigen::Matrix4d _Cart2T(
    double x,
    double y,
    double z,
    double r,
    double p,
    double q
    )
{
  Eigen::Matrix4d T;
  // psi = roll, th = pitch, phi = yaw
  double cq, cp, cr, sq, sp, sr;
  cr = std::cos( r );
  cp = std::cos( p );
  cq = std::cos( q );

  sr = std::sin( r );
  sp = std::sin( p );
  sq = std::sin( q );

  T(0,0) = cp*cq;
  T(0,1) = -cr*sq+sr*sp*cq;
  T(0,2) = sr*sq+cr*sp*cq;

  T(1,0) = cp*sq;
  T(1,1) = cr*cq+sr*sp*sq;
  T(1,2) = -sr*cq+cr*sp*sq;

  T(2,0) = -sp;
  T(2,1) = sr*cp;
  T(2,2) = cr*cp;

  T(0,3) = x;
  T(1,3) = y;
  T(2,3) = z;
  T.row(3) = Eigen::Vector4d( 0.0, 0.0, 0.0, 1.0 );
  return T;
}

inline Eigen::Matrix4d Cart2T(
    double x, double y, double z,
    double r, double p, double q
    )
{
  Eigen::Matrix4d T;
  double cq, cp, cr, sq, sp, sr;
  cr = std::cos( r );
  cp = std::cos( p );
  cq = std::cos( q );

  sr = std::sin( r );
  sp = std::sin( p );
  sq = std::sin( q );

  T(0,0) = cp*cq;
  T(0,1) = -cr*sq+sr*sp*cq;
  T(0,2) = sr*sq+cr*sp*cq;

  T(1,0) = cp*sq;
  T(1,1) = cr*cq+sr*sp*sq;
  T(1,2) = -sr*cq+cr*sp*sq;

  T(2,0) = -sp;
  T(2,1) = sr*cp;
  T(2,2) = cr*cp;

  T(0,3) = x;
  T(1,3) = y;
  T(2,3) = z;
  T.row(3) = Eigen::Vector4d( 0.0, 0.0, 0.0, 1.0 );
  return T;
}

inline Eigen::Matrix4d _Cart2T( Eigen::Matrix<double,6,1> x)
{
  return _Cart2T(x(0),x(1),x(2),x(3),x(4),x(5));
}

inline Eigen::Vector3d _R2Cart(
    const Eigen::Matrix3d& R
    )
{
  Eigen::Vector3d rpq;
  // roll
  rpq[0] = atan2( R(2,1), R(2,2) );

  // pitch
  double det = -R(2,0) * R(2,0) + 1.0;
  if (det <= 0) {
    if (R(2,0) > 0){
      rpq[1] = -M_PI / 2.0;
    }
    else{
      rpq[1] = M_PI / 2.0;
    }
  }
  else{
    rpq[1] = -asin(R(2,0));
  }

  // yaw
  rpq[2] = atan2(R(1,0), R(0,0));

  return rpq;
}

inline Eigen::Vector3d R2pqr(
    const Eigen::Matrix3d& R
    )
{
  Eigen::Vector3d rpq;
  // roll
  rpq[0] = atan2( R(2,1), R(2,2) );

  // pitch
  double det = -R(2,0) * R(2,0) + 1.0;
  if (det <= 0) {
    if (R(2,0) > 0){
      rpq[1] = -M_PI / 2.0;
    }
    else{
      rpq[1] = M_PI / 2.0;
    }
  }
  else{
    rpq[1] = -asin(R(2,0));
  }

  // yaw
  rpq[2] = atan2(R(1,0), R(0,0));

  return rpq;
}

inline Eigen::Matrix<double,6,1> _T2Cart(
    const Eigen::Matrix4d& T
    )
{
  Eigen::Matrix<double,6,1> Cart;
  Eigen::Vector3d rpq = _R2Cart( T.block<3,3>(0,0) );
  Cart[0] = T(0,3);
  Cart[1] = T(1,3);
  Cart[2] = T(2,3);
  Cart[3] = rpq[0];
  Cart[4] = rpq[1];
  Cart[5] = rpq[2];

  return Cart;
}

inline Eigen::Matrix<double,6,1> T2Cart(
    const Eigen::Matrix4d& T
    )
{
  Eigen::Matrix<double,6,1> Cart;
  Eigen::Vector3d rpq = _R2Cart( T.block<3,3>(0,0) );
  Cart[0] = T(0,3);
  Cart[1] = T(1,3);
  Cart[2] = T(2,3);
  Cart[3] = rpq[0];
  Cart[4] = rpq[1];
  Cart[5] = rpq[2];

  return Cart;
}


inline Eigen::Matrix3d _Cart2R(
    const double& r,
    const double& p,
    const double& q
    )
{
  Eigen::Matrix3d R;
  // psi = roll, th = pitch, phi = yaw
  double cq, cp, cr, sq, sp, sr;
  cr = std::cos( r );
  cp = std::cos( p );
  cq = std::cos( q );

  sr = std::sin( r );
  sp = std::sin( p );
  sq = std::sin( q );

  R(0,0) = cp*cq;
  R(0,1) = -cr*sq+sr*sp*cq;
  R(0,2) = sr*sq+cr*sp*cq;

  R(1,0) = cp*sq;
  R(1,1) = cr*cq+sr*sp*sq;
  R(1,2) = -sr*cq+cr*sp*sq;

  R(2,0) = -sp;
  R(2,1) = sr*cp;
  R(2,2) = cr*cp;
  return R;
}


inline Eigen::Matrix3d pqr2R(
    const double& r,
    const double& p,
    const double& q
    )
{
  Eigen::Matrix3d R;
  // psi = roll, th = pitch, phi = yaw
  double cq, cp, cr, sq, sp, sr;
  cr = std::cos( r );
  cp = std::cos( p );
  cq = std::cos( q );

  sr = std::sin( r );
  sp = std::sin( p );
  sq = std::sin( q );

  R(0,0) = cp*cq;
  R(0,1) = -cr*sq+sr*sp*cq;
  R(0,2) = sr*sq+cr*sp*cq;

  R(1,0) = cp*sq;
  R(1,1) = cr*cq+sr*sp*sq;
  R(1,2) = -sr*cq+cr*sp*sq;

  R(2,0) = -sp;
  R(2,1) = sr*cp;
  R(2,2) = cr*cp;
  return R;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix4d TInv( Eigen::Matrix4d T )
{
  // calc Hji = [ Hij(1:3,1:3).' -Hij(1:3,1:3).'*Hij(1:3,4); 0 0 0 1 ];
  Eigen::Matrix4d invT;
  invT.block<3,3>(0,0) =  T.block<3,3>(0,0).transpose();
  invT.block<3,1>(0,3) = -T.block<3,3>(0,0).transpose() * T.block<3,1>(0,3);
  invT.row(3) = Eigen::Vector4d( 0, 0, 0, 1 );
  return invT;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector3d TComp( Eigen::Matrix4d T, Eigen::Vector3d p  )
{
  Eigen::Vector4d tmp( p[0], p[1], p[2], 1 );
  return T.block<3,4>(0,0)*tmp;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix4d TComp( Eigen::Matrix4d Tab, Eigen::Matrix4d Tbc )
{
  return Tab*Tbc;
}


#endif

