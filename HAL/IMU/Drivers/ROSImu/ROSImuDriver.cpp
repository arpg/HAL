#include "ROSImuDriver.h"



namespace hal {

  ROSImuDriver::ROSImuDriver(string m_topic )
    : topic(m_topic)
  {
    int argc = 0;
    ros::init(argc, NULL, "ros_hal_imu", ros::init_options::AnonymousName );
    spinner = new ros::AsyncSpinner(2);
    spinner->start();
    nh = new ros::NodeHandle("~");

    cout << "ROSImu: Subscribing to: " << topic << endl;
    subscriber = nh->subscribe(topic, 1000, &ROSImuDriver::imuCallback, this);

  }
  
  ROSImuDriver::~ROSImuDriver()
  {
    std::cout << "ROSImu: Stopping ROS driver" << std::endl;
    m_bShouldRun = false;
    subscriber.shutdown();
    m_DeviceThread.join();
    delete nh;
  }

  void ROSImuDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback)
  {
    m_IMUCallback = callback;
    if( !m_DeviceThread.joinable() ) {
      // start capture thread
      m_bShouldRun = true;
      m_DeviceThread = std::thread( &ROSImuDriver::service, this );
    }
  }
  
  void ROSImuDriver::service()
  {
    printf("ROSImu: Started service thread\n");
    while( m_bShouldRun ) {
      
        //---------------------------------------------------------
        // get data and pack

      //Wait for the ROS callback to signal us:
      //printf("ROSImu: Waiting for signal...\n");
      std::unique_lock<std::mutex> lk(dataLock);
      if (dataReady.wait_for(lk, std::chrono::milliseconds(1000)) == std::cv_status::timeout)
	{
	  printf("ROSImu: Timed out waiting for data or cancelled\n");
	  return;
	}
      //printf("ROSImu: Got signal of new data!\n");
      //Populate a new protobuf and call the callback on our service thread

      string     sValue;

      hal::ImuMsg dataIMU;
      hal::VectorMsg* pbAccel = dataIMU.mutable_accel();
      pbAccel->add_data(lastMsg->linear_acceleration.x);
      pbAccel->add_data(lastMsg->linear_acceleration.y);
      pbAccel->add_data(lastMsg->linear_acceleration.z);

      hal::VectorMsg* pbGyro = dataIMU.mutable_gyro();
      pbGyro->add_data(lastMsg->angular_velocity.x);
      pbGyro->add_data(lastMsg->angular_velocity.y);
      pbGyro->add_data(lastMsg->angular_velocity.z);
	
      double halStamp = lastMsg->header.stamp.sec + ((double)lastMsg->header.stamp.nsec)/1e9;
      lk.unlock(); 

      if( m_IMUCallback )
	{
	  //Pass through the timestamp
	  
	  dataIMU.set_device_time(halStamp);
	  dataIMU.set_system_time(halStamp);
	  m_IMUCallback( dataIMU );
	}
      
    }
    m_bShouldRun = false;
  }


  void ROSImuDriver::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    //printf("ROSImu: Got new data from ROS!\n");
    //Save the message, ring the bell, go back to listening
    std::unique_lock<std::mutex> lk(dataLock);
    lastMsg = msg;
    lk.unlock();
    dataReady.notify_one();
  }

 
	   
  
} //namespace
