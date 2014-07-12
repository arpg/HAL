#ifndef SIMDEVICES_H
#define SIMDEVICES_H

// Superclass for devices
#include <SimDevices/SimDeviceInfo.h>

// All of our Controllers
#include <SimDevices/Controller/CarController.h>
#include <SimDevices/Controller/SimpleController.h>

// All of our Sensors
#include <SimDevices/Sensor/SimCamera.h>
#include <SimDevices/Sensor/SimGPS.h>
#include <SimDevices/Sensor/SimLaser2D.h>
#include <SimDevices/Sensor/SimLaser3D.h>
#include <SimDevices/Sensor/SimVicon.h>

using namespace std;

class SimDevices
{

public:

  // Constructor
  SimDevices();

  // Initializers
  void AddDevice(SimDeviceInfo* devInfo);

  /// Update devices
  void UpdateSensors();

  // GETTERS
  vector<SimDeviceInfo*> GetAllRelatedDevices(string sDeviceBodyName);
  vector<SimDeviceInfo*> GetOnDevices();
  vector<SimDeviceInfo*> GetOnRelatedDevices(string sDeviceBodyName);

  // MEMBER VARIABLES
  map<string, SimDeviceInfo*>  m_vSimDevices;

};

#endif // SIMDEVICES_H
