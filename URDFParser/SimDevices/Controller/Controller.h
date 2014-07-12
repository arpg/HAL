#ifndef CONTROLLER_H
#define CONTROLLER_H


/*********************************
  * CONTROLLER
  * This is a superclass for all controllers
  ********************************/



class Controller
{
public:

  /// SETTERS

  void SetControllerName(string sControllerName){
    m_sControllerName = sControllerName;
  }

  void SetRobotName(string sRobotName){
    m_sRobotName = sRobotName;
  }

  // Not sure what this does... keeping for now.
  void SetProxyName(string sProxyName){
    m_sProxyName = sProxyName;
  }

  /// GETTERS

  string GetControllerName(){
    return m_sControllerName;
  }

  string GetRobotName(){
    return m_sRobotName;
  }

  string GetProxyName(){
    return m_sProxyName;
  }


  /// MEMBER VARIABLES

  string m_sControllerName;
  string m_sProxyName;
  string m_sRobotName;

};

#endif // CONTROLLER_H
