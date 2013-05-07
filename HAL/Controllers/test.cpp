

#include "HingeController.h"


int main( int argc, char** argv )
{

    rpg::HingeJointController joint;
    joint.Init( "HeadTiltJoint", argv[1] );

    rpg::CarController car;

    car.SetDesiredVelocity( v ); // compute v from keyboard or mouse ...

    for( double th = 0; th < 10*M_PI; th+=0.01 ){
        joint.SetDesiredAngle( cos(th) );
    }

    return 0;
}
