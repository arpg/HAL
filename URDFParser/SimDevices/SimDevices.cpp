#include <SimDevices/SimDevices.h>

////////////////////////////////////////////////////////////////////////
/// CONSTRUCTOR
////////////////////////////////////////////////////////////////////////

SimDevices::SimDevices(){
}

////////////////////////////////////////////////////////////////////////
/// INITIALIZERS
////////////////////////////////////////////////////////////////////////

void SimDevices::AddDevice(SimDeviceInfo* devInfo){
  m_vSimDevices[devInfo->GetDeviceName()] = devInfo;
}

////////////////////////////////////////////////////////////////////////
/// UPDATE ALL DEVICES
////////////////////////////////////////////////////////////////////////

void SimDevices::UpdateSensors(){
  for(map<string, SimDeviceInfo*>::iterator it = m_vSimDevices.begin();
      it != m_vSimDevices.end();
      it++){
    SimDeviceInfo* Device = it->second;
    // Cameras are taken care of in RenderEngine, so don't worry about
    // them here.
    // TODO:GPS and Vicon
//    if(Device->m_sDeviceType == "GPS"){
//      SimGPS* pGPS = (SimGPS*) Device;
//      pGPS->Update();
//    }
//    else if(Device->m_sDeviceType == "Vicon"){
//      SimVicon* pVicon = (SimVicon*) Device;
//      pVicon->Update();
//    }
  }
}

/******************************************************************************
  * GETTERS
  *****************************************************************************/

vector<SimDeviceInfo*> SimDevices::GetAllRelatedDevices(string sDeviceBodyName){
  vector<SimDeviceInfo*> related_devices;
  for(map<string, SimDeviceInfo*>::iterator it = m_vSimDevices.begin();
      it != m_vSimDevices.end();
      it++){
    SimDeviceInfo* pDevice = it->second;
    if(pDevice->m_sDeviceType=="CarController"){
      // cout<<"SimDevices::GetAllRelatedDevices: "<<
      //     pDevice->GetDeviceName()<<endl;
      // cout<<"SimDevices::GetAllRelatedDevices: "<<
      //     pDevice->GetBodyName()<<endl;
      // cout<<"SimDevices::GetAllRelatedDevices: "<<
      //     sDeviceBodyName<<endl;
      if (pDevice->GetDeviceName()==sDeviceBodyName){
        related_devices.push_back(pDevice);
      }
    }
    else if (pDevice->GetBodyName()==sDeviceBodyName){
      related_devices.push_back(pDevice);
    }
  }
  return related_devices;
}

//////////////////////////

// Only returns devices if they're on. Really handy for NetworkManager.
vector<SimDeviceInfo*> SimDevices::GetOnDevices(){
  vector<SimDeviceInfo*> on_devices;
  for(map<string, SimDeviceInfo*>::iterator it = m_vSimDevices.begin();
      it != m_vSimDevices.end();
      it++){
    SimDeviceInfo* pDevice = it->second;
    if (pDevice->m_bDeviceOn){
      on_devices.push_back(pDevice);
    }
  }
  return on_devices;
}

////////////////////////

// Only returns devices if they're on. Really handy for NetworkManager.
vector<SimDeviceInfo*> SimDevices::GetOnRelatedDevices(string sDeviceBodyName){
  vector<SimDeviceInfo*> related_devices;
  for(map<string, SimDeviceInfo*>::iterator it = m_vSimDevices.begin();
      it != m_vSimDevices.end();
      it++){
    SimDeviceInfo* pDevice = it->second;
    if (pDevice->GetBodyName()==sDeviceBodyName && pDevice->m_bDeviceOn){
      related_devices.push_back(pDevice);
    }
  }
  return related_devices;
}
