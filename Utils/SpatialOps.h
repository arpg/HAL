/*
 *  \file SpatialOps.h
 *
 *  Helper routines for dealing with poses, coordinate frames, spatial operations, etc.
 *
 *  $Id$
 */

#ifndef _SPATIAL_OPS_H_
#define _SPATIAL_OPS_H_

#include <Eigen/Core>

namespace Eigen
{
    typedef Matrix<double,6,1> Vector6d;
}

namespace rpg
{

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void FillRotationMatrixFixedFrameXYZ( 
            Eigen::Matrix3d& R, 
            const double& r, 
            const double& p,
            const double& q 
            )
    {
        // psi = roll, th = pitch, phi = yaw 
        double cq, cp, cr, sq, sp, sr;
        cr = cos( r );
        cp = cos( p );
        cq = cos( q );

        sr = sin( r );
        sp = sin( p );
        sq = sin( q );

        R(0,0) = cp*cq;
        R(0,1) = -cr*sq+sr*sp*cq;
        R(0,2) = sr*sq+cr*sp*cq;

        R(1,0) = cp*sq;
        R(1,1) = cr*cq+sr*sp*sq;
        R(1,2) = -sr*cq+cr*sp*sq;

        R(2,0) = -sp;
        R(2,1) = sr*cp;
        R(2,2) = cr*cp;
    } 

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void FillRotationMatrixFixedFrameXYZ(
            Eigen::Matrix4d& T, 
            const double& r,
            const double& p, 
            const double& q 
            )
    {
        // psi = roll, th = pitch, phi = yaw 
        double cq, cp, cr, sq, sp, sr;
        cr = cos( r );
        cp = cos( p );
        cq = cos( q );

        sr = sin( r );
        sp = sin( p );
        sq = sin( q );

        T(0,0) = cp*cq;
        T(0,1) = -cr*sq+sr*sp*cq;
        T(0,2) = sr*sq+cr*sp*cq;

        T(1,0) = cp*sq;
        T(1,1) = cr*cq+sr*sp*sq;
        T(1,2) = -sr*cq+cr*sp*sq;

        T(2,0) = -sp;
        T(2,1) = sr*cp;
        T(2,2) = cr*cp;
    } 


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline Eigen::Vector3d R2Cart(
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


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline Eigen::Vector6d T2Cart( 
            double* dT 
            )
    {
        Eigen::Vector6d Cart;
        Eigen::Matrix4d T(dT);
        Eigen::Vector3d rpq = R2Cart( T.block<3,3>(0,0) );
        Cart[0] = T(0,3);
        Cart[1] = T(1,3);
        Cart[2] = T(2,3);
        Cart[3] = rpq[0]; 
        Cart[4] = rpq[1]; 
        Cart[5] = rpq[2]; 

        return Cart;
    }

    inline Eigen::Vector6d T2Cart( 
            const Eigen::Matrix4d& T 
            )
    {
        Eigen::Vector6d Cart;
        Eigen::Vector3d rpq = R2Cart( T.block<3,3>(0,0) );
        Cart[0] = T(0,3);
        Cart[1] = T(1,3);
        Cart[2] = T(2,3);
        Cart[3] = rpq[0]; 
        Cart[4] = rpq[1]; 
        Cart[5] = rpq[2]; 

        return Cart;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline Eigen::Matrix3d Cart2R(
            const double roll,  //< Input:
            const double pitch, //< Input:
            const double yaw    //< Input:
            )
    {
        Eigen::Matrix3d R;
        FillRotationMatrixFixedFrameXYZ( R, roll, pitch, yaw );
        return R;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline Eigen::Matrix3d Cart2R(
            const Eigen::Vector3d& rpy //< Input:
            )
    {
        return Cart2R( rpy[0], rpy[1], rpy[2] );
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline Eigen::Matrix4d Cart2T(
            const Eigen::Vector6d& Cart 
            )
    {
        Eigen::Matrix4d T;
        FillRotationMatrixFixedFrameXYZ( T, Cart[3], Cart[4], Cart[5] );
        T(0,3) = Cart[0];
        T(1,3) = Cart[1];
        T(2,3) = Cart[2];
        T.row(3) = Eigen::Vector4d( 0.0, 0.0, 0.0, 1.0 );
        return T;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline Eigen::Matrix4d Cart2T(
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
        cr = cos( r );
        cp = cos( p );
        cq = cos( q );

        sr = sin( r );
        sp = sin( p );
        sq = sin( q );

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
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Make sure top-left 3x3 matrix is orthonormal.
    inline void MakeOrthonormal( Eigen::Matrix4d& T )
    {
        Eigen::Vector3d rpy = R2Cart( T.topLeftCorner<3,3>() );
        T.topLeftCorner<3,3>() = Cart2R( rpy );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Make sure top-left 3x3 matrix is orthonormal.
    inline void MakeOrthonormal( Eigen::Matrix3d& R )
    {
        Eigen::Vector3d rpy = R2Cart( R );
        R = Cart2R( rpy );
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Make sure top-left 3x3 matrix is orthonormal.
    inline bool IsValidTransform( Eigen::Matrix4d& T )
    {
        if( T(3,0) != 0 || T(3,1) != 0 || T(3,2) != 0 || T(3,3) != 1 ){
            return false;
        }

        // Check the determinant.
        double det = T.topLeftCorner<3,3>().determinant();
        if( fabs( det - 1.0 ) < 1e-12 ){ // TODO: find better check
            return true; // good, it's an orthonormal right handed bases
        } else if( fabs( det + 1.0 ) < 1e-12 ){
            fprintf(stderr, "WARNING: basis is left handed!\n");
            return true;
        }
        fprintf(stderr, "WARNING: matrix determinant is%e \n", det );
        return false;
    }

}

#
