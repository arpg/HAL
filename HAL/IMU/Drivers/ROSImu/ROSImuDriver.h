#pragma once

#include <memory>
#include <HAL/IMU/IMUDriverInterface.h>
#include <HAL/Devices/DeviceException.h>
#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include <unistd.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <functional>
#include <chrono>

//Protobuf includes
#include <HAL/Image.pb.h>
#include <HAL/Messages/ImageArray.h>

//ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/message.h>

#include <sensor_msgs/Imu.h>


using std::cout;
using std::string;
using std::endl;
using std::vector;
using std::mutex;
using std::condition_variable;

namespace hal
{

  class ROSImuDriver : public IMUDriverInterface
  {
  public:
    ROSImuDriver(string m_topic);
    ~ROSImuDriver();
    volatile bool           m_bShouldRun;
    void RegisterIMUDataCallback(IMUDriverDataCallback callback);
    bool IsRunning() const override {
      return m_bShouldRun;
    }

    //Callback function for ROS
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    string topic;
 
    //ROS variables
    ros::NodeHandle *nh;
    ros::Subscriber subscriber;
    ros::AsyncSpinner *spinner;
    sensor_msgs::ImuConstPtr lastMsg;
    /* Principle of operation:
       Only supports one subscriber
       Decouple ROS callback from subscriber so that end users can't block the data
       --Signal via condvar, ring the bell on new data present       
     */

    
    mutex dataLock;
    condition_variable dataReady;
    std::thread             m_DeviceThread;
    void service();
    IMUDriverDataCallback   m_IMUCallback;

  };

}
