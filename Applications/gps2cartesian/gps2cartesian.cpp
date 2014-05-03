#include <math.h>
#include <chrono>
#include <thread>
#include <HAL/Posys/PosysDevice.h>
#include <gflags/gflags.h>
#include <miniglog/logging.h>

#include <Accuracy.h>
#include <CartesianCoordinates.h>
#include <CoordinateConversionException.h>
#include <CoordinateConversionService.h>
#include <GeodeticCoordinates.h>
#include <GeodeticParameters.h>
#include <HeightType.h>
#include <LocalCartesianParameters.h>

DEFINE_string(posys, "",
              "Path to Posys device that is the source of GPS coordinates.");

DEFINE_string(output, "cartesian.csv", "Path to output CSV.");

static std::shared_ptr<MSP::CCS::CoordinateConversionService> converter;

template <typename Scalar>
Scalar radians(const Scalar& deg) {
  return deg * M_PI / 180.;
}

/** Create a Geodetic -> LocalCartesian CCS centered around the given message */
std::shared_ptr<MSP::CCS::CoordinateConversionService>
create_local_converter(pb::PoseMsg& msg) {
  CHECK_EQ(msg.type(), pb::PoseMsg::LatLongAlt);

  const pb::VectorMsg& pose = msg.pose();

  MSP::CCS::GeodeticParameters ellipsoidParameters(
      MSP::CCS::CoordinateType::geodetic,
      MSP::CCS::HeightType::ellipsoidHeight);

  MSP::CCS::LocalCartesianParameters localParameters(
      MSP::CCS::CoordinateType::localCartesian,
      // Longitude, latitude, altitude, orientation.
      radians(pose.data(1)), radians(pose.data(0)), pose.data(2), 0);

  LOG(INFO) << "Initializing local cartesian space at (Lat, Long, Alt): "
            << pose.data(0) << ", " << pose.data(1) << ", " << pose.data(2);

  try {
    return std::make_shared<MSP::CCS::CoordinateConversionService>(
        "WGE", &ellipsoidParameters,
        "WGE", &localParameters);
  } catch (MSP::CCS::CoordinateConversionException& e) {
    LOG(FATAL) << "Failed to create converter: " << e.getMessage();
  }
  return nullptr;
}

void gps_to_file(std::ofstream* fout, pb::PoseMsg& msg) {
  CHECK_NOTNULL(fout);
  if (msg.type() != pb::PoseMsg::LatLongAlt) {
    LOG(INFO) << "Skipping non-WGS84 PoseMsg";
    return;
  }
  if (!converter) {
    converter = create_local_converter(msg);
  }

  const pb::VectorMsg& pose = msg.pose();
  MSP::CCS::GeodeticCoordinates geo_coords(
      MSP::CCS::CoordinateType::geodetic,
      radians(pose.data(1)), radians(pose.data(0)), pose.data(2));
  MSP::CCS::Accuracy geo_acc;

  // 90% confidence interval is 1.64 STD
  geo_acc.setSphericalError90(msg.std() * 1.64);

  MSP::CCS::Accuracy cart_acc;
  MSP::CCS::CartesianCoordinates cart_coords;

  try {
    converter->convertSourceToTarget(&geo_coords, &geo_acc,
                                     cart_coords, cart_acc);
  } catch (MSP::CCS::CoordinateConversionException& e) {
    LOG(ERROR) << "Failed to convert coordinates: " << e.getMessage();
    return;
  }

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
