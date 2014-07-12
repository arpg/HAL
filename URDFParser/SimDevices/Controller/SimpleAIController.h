// provide a simple AI that control Robot in LocalSim

#ifndef SIMPLEAICONTROLLER_H
#define SIMPLEAICONTROLLER_H

#include <ModelGraph/PhysicsEngine.h>.h>

class SimpleAIController
{
public:
    void init(string DeviceName, PhysicsEngine& rPhysWrapper)
    {
        m_sDeviceName = DeviceName;
        m_rPhysWrapper = rPhysWrapper;
    }

    // define some AI behaviour here.



private:
    string                         m_sDeviceName;
    PhysicsEngine                 m_rPhysWrapper;
};



#endif // SIMPLEAICONTROLLER_H
