#pragma once

#include <memory>
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Devices/DeviceException.h>
#include <boost/bind.hpp>
#include <iostream>
#include <vector>

#include <unistd.h>
#include <pthread.h>

//Protobuf includes
#include <PbMsgs/Image.pb.h>
#include <PbMsgs/ImageArray.h>

//ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/message.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using std::cout;
using std::string;
using std::endl;
using std::vector;


namespace hal
{

  class ROSDriver : public CameraDriverInterface
  {
  public:
    ROSDriver(string m_topics, int width, int height );
    ~ROSDriver();
    
    bool Capture( pb::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }
 
    void Start();
    void Stop();
    
    size_t NumChannels() const;
    size_t Width( size_t /*idx*/ = 0 ) const;
    size_t Height( size_t /*idx*/ = 0 ) const;
    
    string findROSFormat(uint32_t format);
    pb::Type findPbType(string format);
    pb::Format findPbFormat(string format);

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg, int topicIndex);
    template<typename T> vector<T> split(const T & str, const T & delimiters);
      
    string topics;
    vector<string> topicList;
    int topicCount;


    //ROS variables
    ros::NodeHandle *nh;
    image_transport::ImageTransport *it;
    image_transport::Subscriber *subscribers;

    ros::AsyncSpinner *spinner;

    
    int pbtype_rgb;
    int pbtype_d;
 
    int pbformat_rgb;
    int pbformat_d;

    int width_;
    int height_;
    int fps_;

    //Have an array of mutexes, one for each topic that we're subscribed to
    //The plan is that each topic spins, but is synced to the first one via condvar_wait
    //Capture() waits for the condvar to be signalled, then pulls the latest and greatest
    //image from each subscribed topic besides the first

    sensor_msgs::ImageConstPtr *freshImages;

    pthread_mutex_t *topicLocks;
    pthread_mutex_t topicBell;
    pthread_cond_t newTopic;

    int initComplete;
  };

}
