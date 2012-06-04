/*
    \file Defines the interface all IMU drivers must honor.
*/


#ifndef _IMU_DRIVER_H_
#define _IMU_DRIVER_H_

#include <Eigen/Eigen>
#include <RPG/Utils/PropertyMap.h>
#include <boost/function.hpp>

struct IMUData {
    Eigen::Quaterniond rotation;
};

typedef boost::function<void (const IMUData&)> IMUDriverDataCallback;

///////////////////////////////////////////////////////////////////////////////
// Generic IMU driver interface
class IMUDriver
{
    public:
        // Pure virtual functions driver writers must implement:
        virtual void RegisterDataCallback(IMUDriverDataCallback callback) = 0;
        virtual bool Init() = 0;

        IMUDriver()
        {
            m_pPropertyMap = NULL;
        }

        // Called by IMUDevice::InitDriver
        void SetPropertyMap( PropertyMap* pMap )
        {
            m_pPropertyMap = pMap;
        }
    protected:
        PropertyMap* m_pPropertyMap; // from parent device that instantiates this driver.
};
#endif
