#include <math.h>
#include <chrono>
#include <thread>
#include <HAL/Posys/PosysDevice.h>
#include <gflags/gflags.h>
#include <miniglog/logging.h>

#include <geocon/geodetic2local.h>
#include <Accuracy.h>

DEFINE_string(posys, "",
              "Path to Posys device that is the source of GPS coordinates.");

DEFINE_string(output, "cartesian.csv", "Path to output CSV.");

static std::shared_ptr<geocon::geodetic2local> converter;

template <typename Scalar>
Scalar radians(const Scalar& deg) {
  return deg * M_PI / 180.;
}

double pose_std(const hal::PoseMsg& msg) {
  if (msg.has_covariance()) {
    // mean of stds
    double sum = 0.;
    for (int i = 0; i < msg.covariance().data_size();
         i += msg.covariance().rows() + 1) {
      sum += sqrt(msg.covariance().data(i));
    }
    return sum / msg.covariance().rows();
  } else
    return 0;
}

void gps_to_file(std::ofstream* fout, hal::PoseMsg& msg) {
  CHECK_NOTNULL(fout);
  if (msg.type() != hal::PoseMsg::LatLongAlt) {
    LOG(INFO) << "Skipping non-WGS84 PoseMsg";
    return;
  }

  const hal::VectorMsg& pose = msg.pose();
  if (!converter) {
    converter.reset(geocon::geodetic2local::Create(radians(pose.data(0)),
                                                   radians(pose.data(1)),
                                                   pose.data(2)));
    LOG_IF(FATAL, !converter) << "Failed to initialize geocon converter.";
  }

  MSP::CCS::CartesianCoordinates cart_coords;
  MSP::CCS::Accuracy cart_acc;
  converter->to_local(
      radians(pose.data(1)), radians(pose.data(0)), pose.data(2), pose_std(msg),
      &cart_coords, &cart_acc);

  *fout << msg.device_time() << ","
        << cart_coords.x() << ","
        << cart_coords.y() << ","
        << cart_coords.z() << ","
        << cart_acc.sphericalError90() / 1.64 << "\n";
}

int main(int argc, char *argv[]) {
  static const std::string header =
      "timestamp,x,y,z,std\n";

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_posys.empty()) {
    LOG(FATAL) << "posys argument required.";
  }

  std::ofstream fout(FLAGS_output);
  if (!fout.is_open()) {
    LOG(FATAL) << "Output file " << FLAGS_output << " could not be opened";
  }

  hal::Posys posys;
  try {
    posys = hal::Posys(FLAGS_posys);
  } catch (const hal::DeviceException& e) {
    LOG(FATAL) << "Could not open Posys device: " << e.what();
  }

  fout << header;

  posys.RegisterPosysDataCallback(
      std::bind(gps_to_file, &fout, std::placeholders::_1));

  while (posys.IsRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  fout.close();
  LOG(INFO) << "Finished converting";
  return 0;
}
