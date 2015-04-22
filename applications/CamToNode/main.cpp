// Sends camera images trhough a Node connection
//

#include <iostream>
#include <memory>
#include <chrono>

#include <gflags/gflags.h>

#include <HAL/Camera/CameraDevice.h>
#include <Node/Node.h>
#include <HAL/Messages/ImageArray.h>
#include <miniglog/logging.h>

DEFINE_string(node, "camtonode", "Node name");
DEFINE_string(topic, "images", "Topic name");
DEFINE_int32(verbosity, 2, "Verbosity level of Node "
                           "(the lower the less verbose)");
DEFINE_string(cam, "", "Camera driver");
DEFINE_int32(fps, 30, "Frames per second");

int main(int argc, char *argv[])
{
  // get params
  google::SetUsageMessage("Read images from a HAL driver and send them over "
                          "the network through Node.");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if(FLAGS_cam.empty())
    LOG(FATAL) << "Camera driver not provided. Use parameter -cam";

  // initiate camera
  std::unique_ptr<hal::Camera> cam;
  try {
    cam.reset(new hal::Camera(hal::Uri(FLAGS_cam)));
  } catch (...) {
    LOG(FATAL) << "Could not create camera from URI: " << FLAGS_cam;
  }

  // initiate node
  node::node n;
  n.set_verbosity(FLAGS_verbosity);
  n.init(FLAGS_node);
  n.advertise(FLAGS_topic);

  LOG(INFO) << "Publishing images from node \"" << FLAGS_node
            << "\" through topic \"" << FLAGS_topic << "\" at "
            << FLAGS_fps << " fps";

  // publish images
  const long long int period_ms = static_cast<long long int>
      (1000. / static_cast<double>(FLAGS_fps));
  for(std::shared_ptr<hal::ImageArray> vImages = hal::ImageArray::Create();;)
  {
    // current time
    auto start_ms = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();

    // publish
    cam->Capture(*vImages);
    n.publish(FLAGS_topic, vImages->Ref());

    // calculate time to sleep
    auto end_ms = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();
    if(end_ms - start_ms < period_ms)
      std::this_thread::sleep_for(std::chrono::milliseconds
                                  (period_ms - (end_ms - start_ms)));
  }

  return 0;
}
