// Copyright (c) bminortx

#include <SimDevices/SimDevices.h>
#include <map>
#include <string>
#include <vector>

////////////////////////////////////////////////////////////////////////
/// CONSTRUCTOR
////////////////////////////////////////////////////////////////////////

SimDevices::SimDevices() {
}

////////////////////////////////////////////////////////////////////////
/// INITIALIZERS
////////////////////////////////////////////////////////////////////////

void SimDevices::AddDevice(SimDeviceInfo* devInfo) {
  sim_device_map_[devInfo->GetDeviceName()] = devInfo;
}

////////////////////////////////////////////////////////////////////////
/// UPDATE ALL DEVICES
////////////////////////////////////////////////////////////////////////

void SimDevices::UpdateSensors() {
  for (std::map<std::string, SimDeviceInfo*>::iterator it =
           sim_device_map_.begin();
       it != sim_device_map_.end();
       it++) {
    //  SimDeviceInfo* Device = it->second;
    //  Cameras are taken care of in RenderEngine, so don't worry about
    //  them here.
    // TODO(anyone): GPS and Vicon
    //    if (Device->m_sDeviceType == "GPS") {
    //      SimGPS* pGPS = (SimGPS*) Device;
    //      pGPS->Update();
    //    }
    //    else if (Device->m_sDeviceType == "Vicon") {
    //      SimVicon* pVicon = (SimVicon*) Device;
    //      pVicon->Update();
    //    }
  }
}

/******************************************************************************
 * GETTERS
 *****************************************************************************/

std::vector<SimDeviceInfo*> SimDevices::GetAllRelatedDevices(
    std::string sDeviceBodyName) {
  std::vector<SimDeviceInfo*> related_devices;
  for (std::map<std::string, SimDeviceInfo*>::iterator it =
           sim_device_map_.begin();
       it != sim_device_map_.end();
       it++) {
    SimDeviceInfo* pDevice = it->second;
    if (pDevice->m_sDeviceType == "CarController") {
      if (pDevice->GetDeviceName() == sDeviceBodyName) {
        related_devices.push_back(pDevice);
      }
    } else if (pDevice->GetBodyName() == sDeviceBodyName) {
      related_devices.push_back(pDevice);
    }
  }
  return related_devices;
}

//////////////////////////

// Only returns devices if they're on. Really handy for NetworkManager.
std::vector<SimDeviceInfo*> SimDevices::GetOnDevices() {
  std::vector<SimDeviceInfo*> on_devices;
  for (std::map<std::string, SimDeviceInfo*>::iterator it =
           sim_device_map_.begin();
       it != sim_device_map_.end();
       it++) {
    SimDeviceInfo* pDevice = it->second;
    if (pDevice->m_bDeviceOn) {
      on_devices.push_back(pDevice);
    }
  }
  return on_devices;
}

////////////////////////

// Only returns devices if they're on. Really handy for NetworkManager.
std::vector<SimDeviceInfo*> SimDevices::GetOnRelatedDevices(
    std::string sDeviceBodyName) {
  std::vector<SimDeviceInfo*> related_devices;
  for (std::map<std::string, SimDeviceInfo*>::iterator it =
           sim_device_map_.begin();
       it != sim_device_map_.end();
       it++) {
    SimDeviceInfo* pDevice = it->second;
    if (pDevice->GetBodyName() == sDeviceBodyName && pDevice->m_bDeviceOn) {
      related_devices.push_back(pDevice);
    }
  }
  return related_devices;
}
