#include <iomanip>
#include <unistd.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif
#include <opencv.hpp>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <HAL/Messages/Image.h>
#include <HAL/Messages/Logger.h>
#include <HAL/Messages/Reader.h>

/**
 * logtool functionality
 *
 * - [X] Extract a given set of frames
 * - [X] Extract a log into individual images
 * - [X] Extract imu from a log and save it as csv
 * - [X] Extract posys from a log and save it as csv
 * - [X] Concatenate multiple logs
 * - [ ] Concatenate multiple single images into a log
 * - [ ] Reorder a log by timestamp
 * - [ ] Add an index
 * - [ ] Remove an index
 * - [ ] Output index, header to human-readable format
 */

DEFINE_string(in, "", "Input log file or input directory.");
DEFINE_string(out, "", "Output log file or output directory.");

DEFINE_bool(extract_log, false, "Enable log subset extraction.");
DEFINE_bool(extract_images, false, "Enable image extraction to individual files.");
DEFINE_bool(extract_imu, false, "Enable IMU extraction to individual files.");
DEFINE_bool(extract_posys, false, "Enable Posys extraction to individual files.");
DEFINE_string(extract_types, "",
              "Comma-separated list of types to extract from log. "
              "Options include \"cam\", \"imu\", and \"posys\".");
DEFINE_string(extract_frame_range, "",
              "Range (inclusive) of image frames to extract from the log. "
              "Should be a comma-separated pair, e.g \"0,200\"");

DEFINE_string(cat_logs, "",
              "Comma-separated list of logs to concatenate together. ");

typedef std::function<bool(char)> TrimPred;

inline std::string& LTrim(std::string& s, const TrimPred& pred) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), pred));
  return s;
}

inline std::string& RTrim(std::string& s, const TrimPred& pred) {
  s.erase(std::find_if(s.rbegin(), s.rend(), pred).base(), s.end());
  return s;
}

inline std::string& Trim(std::string& s, const TrimPred& pred) {
  return LTrim(RTrim(s, pred), pred);
}

inline std::string& TrimWhitespace(std::string& s) {
  TrimPred wspred = std::not1(std::ptr_fun<int, int>(std::isspace));
  return LTrim(RTrim(s, wspred), wspred);
}

inline std::string& TrimQuotes(std::string& s) {
  TrimPred qpred = [](char c) { return c != '\'' && c != '\"'; };
  return LTrim(RTrim(s, qpred), qpred);
}

/**
 * Given a delim-separated string, split it into component elements.
 *
 * Use a stringstream to convert to the destination type.
 */
template <typename T>
inline void Split(const std::string& s, char delim,
                  std::vector<T>* elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    T val;
    std::stringstream val_ss(item);
    val_ss >> val;
    elems->push_back(val);
  }
}

/**
 * Special case for string splitting, since it doesn't need an extra
 * re-parsing through a stringstream.
 */
template<>
inline void Split<std::string>(const std::string& s, char delim,
                               std::vector<std::string>* elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems->push_back(item);
  }
}

inline hal::MessageType MsgTypeForString(const std::string& str) {
  static std::map<std::string, hal::MessageType> kTypeStrings = {
    {"cam", hal::Msg_Type_Camera},
    {"imu", hal::Msg_Type_IMU},
    {"encoder", hal::Msg_Type_Encoder},
    {"lidar", hal::Msg_Type_LIDAR},
    {"posys", hal::Msg_Type_Posys},
  };
  auto it = kTypeStrings.find(str);
  if(it != kTypeStrings.end()) {
    LOG(INFO) << "Type string '" << str
              << "' does not match a known data type";
  }

  /*
  // Note: no overloaded operator available.

  CHECK_NE(it, kTypeStrings.end()) << "Type string '" << str
                                   << "' does not match a known data type";
  */
  return it->second;
}

/** Save individual file based on pb:Image type. */
inline void SaveImage(const std::string& out_dir,
                      int channel_index,
                      unsigned int frame_number,
                      double timestamp,
                      const hal::ImageMsg& image) {

  // Convert index to string.
  std::string index;
  std::ostringstream convert;
  convert << channel_index;
  index = convert.str();

  std::string file_prefix = out_dir + "/";
  file_prefix = file_prefix + "channel" + index;

  convert.str("");
  convert.clear();
  if (timestamp == 0.)
    convert << std::fixed << std::setfill('0') << std::setw(5) << frame_number;
  else
    convert << std::fixed << std::setfill('0') << std::setw(5) << frame_number
            << "_" << std::setprecision(9) << timestamp;
  index = convert.str();

  std::string filename;

  // Use OpenCV to handle saving the file for us.
  cv::Mat cv_image = hal::WriteCvMat(image);

  if (image.type() == hal::Type::PB_FLOAT) {
    // Save floats to our own "portable depth map" format.
    filename = file_prefix + "_" + index + ".pdm";

    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    file << "P7" << std::endl;
    file << cv_image.cols << " " << cv_image.rows << std::endl;
    const size_t size = cv_image.elemSize1() * cv_image.rows * cv_image.cols;
    file << 4294967295 << std::endl;
    file.write((const char*)cv_image.data, size);
    file.close();
  } else if (image.type() == hal::Type::PB_BYTE
             || image.type() == hal::Type::PB_UNSIGNED_BYTE
             || image.type() == hal::Type::PB_SHORT
             || image.type() == hal::Type::PB_UNSIGNED_SHORT) {
    // OpenCV only supports byte/short data types with 1/3 channel images.
    filename = file_prefix + "_" + index + ".pgm";
    cv::imwrite(filename, cv_image);
  } else {
    LOG(FATAL) << "Input image type not supported for extraction.";
  }
}

/** Extracts imu out of a log file. */
void ExtractImu() {
  std::ofstream accel_file(FLAGS_out + "/accel.txt", std::ios_base::trunc);
  std::ofstream gyro_file(FLAGS_out + "/gyro.txt", std::ios_base::trunc);
  std::ofstream mag_file(FLAGS_out + "/mag.txt", std::ios_base::trunc);
  std::ofstream timestamp_file(FLAGS_out + "/timestamp.txt",
                               std::ios_base::trunc);

  static const int kNoRange = -1;

  int frame_min = kNoRange, frame_max = kNoRange;
  std::vector<int> frames;
  Split(TrimQuotes(FLAGS_extract_frame_range), ',', &frames);
  if (!frames.empty()) {
    CHECK_EQ(2, frames.size()) << "extract_frame_range must be frame PAIR";
    frame_min = frames[0];
    frame_max = frames[1];
    CHECK_LE(frame_min, frame_max)
        << "Minimum frame index must be <= than max frame index.";
  }

  hal::Reader reader(FLAGS_in);
  reader.Enable(hal::Msg_Type_IMU);

  int idx = 0;
  std::unique_ptr<hal::Msg> msg;
  while (frame_min != kNoRange && idx < frame_min) {
    if ((msg = reader.ReadMessage()) && msg->has_camera()) {
      ++idx;
    }
  }

  while ((frame_max == kNoRange ||
          idx <= frame_max) &&
         (msg = reader.ReadMessage())) {
    if (msg->has_imu()) {
      const hal::ImuMsg& imu_msg = msg->imu();
      if (imu_msg.has_accel()) {
        // Write the accel to the accel csv
        accel_file  << imu_msg.accel().data(0) << ", " <<
                       imu_msg.accel().data(1) << ", " <<
                       imu_msg.accel().data(2) << std::endl;
      } else {
        accel_file << "0, 0, 0" << std::endl;
      }

      if (imu_msg.has_gyro()) {
        // Write the accel to the accel csv
        gyro_file  << imu_msg.gyro().data(0) << ", " <<
                      imu_msg.gyro().data(1) << ", " <<
                      imu_msg.gyro().data(2) << std::endl;
      } else {
        gyro_file << "0, 0, 0" << std::endl;
      }

      if (imu_msg.has_mag()) {
        // Write the accel to the accel csv
        mag_file  << imu_msg.mag().data(0) << ", " <<
                     imu_msg.mag().data(1) << ", " <<
                     imu_msg.mag().data(2) << std::endl;
      } else {
        mag_file << "0, 0, 0" << std::endl;
      }

      timestamp_file << std::fixed << std::setprecision(9) << imu_msg.system_time() << ", "
                     << std::setprecision(9) << imu_msg.device_time()
                     << std::endl;
      // WRITE THE IMU
      ++idx;
    }
  }
}

/** Extracts posys out of a log file. */
void ExtractPosys() {
  static const int kNoRange = -1;
  int frame_min = kNoRange, frame_max = kNoRange;
  std::vector<int> frames;
  Split(TrimQuotes(FLAGS_extract_frame_range), ',', &frames);
  if (!frames.empty()) {
    CHECK_EQ(2, frames.size()) << "extract_frame_range must be frame PAIR";
    frame_min = frames[0];
    frame_max = frames[1];
    CHECK_LE(frame_min, frame_max)
        << "Minimum frame index must be <= than max frame index.";
  }

  hal::Reader reader(FLAGS_in);
  reader.Enable(hal::Msg_Type_Posys);

  int idx = 0;
  std::unique_ptr<hal::Msg> msg;
  while (frame_min != kNoRange && idx < frame_min) {
    if ((msg = reader.ReadMessage()) && msg->has_camera()) {
      ++idx;
    }
  }

  // A csv file is created per object with names object00.csv, object01.csv, etc
  // Each csv file contains these columns (depending on data):
  // 1. System timestamp
  // 2. Device timestamp
  // 3. Object id
  // 4. Pose type
  // 5. Size of pose data (this must be consistent with Pose type)
  // 6. Pose data (as many columns as previous field says)
  // 7. Size of covariance data (>= 0)
  // 8. Covariance data (as many columns as previous field says)
  // The redundancy in items 4 and 5 is due to the RAW pose type that can
  // contain any number of elements.

  // object id, csv file
  std::map<int, std::shared_ptr<std::ofstream>> files;
  while ((frame_max == kNoRange ||
          idx <= frame_max) &&
         (msg = reader.ReadMessage())) {
    if (msg->has_pose()) {
      const hal::PoseMsg& pose_msg = msg->pose();
      std::shared_ptr<std::ofstream>& file = files[pose_msg.id()];

      if (!file) {
        std::stringstream filename;
        filename << FLAGS_out << "/object" << std::setw(2) << std::setfill('0')
                 << pose_msg.id() << ".csv";
        file.reset(new std::ofstream(filename.str().c_str()));
      }

      *file << std::fixed << std::setprecision(9) << pose_msg.system_time()
            << ", " << std::setprecision(9) << pose_msg.device_time()
            << ", " << pose_msg.id() << ", " << pose_msg.type()
            << ", " << pose_msg.pose().data_size() << ",";
      for (int i = 0; i < pose_msg.pose().data_size(); ++i)
        *file << pose_msg.pose().data(i) << ",";
      *file << " " << pose_msg.covariance().data_size();
      if (pose_msg.covariance().data_size() > 0) {
        for (int i = 0; i < pose_msg.covariance().data_size(); ++i)
          *file << "," << pose_msg.covariance().data(i);
      }
      *file << std::endl;
    }
    ++idx;
  }
}

/** Extracts single images out of a log file. */
void ExtractImages() {
  static const int kNoRange = -1;

  int frame_min = kNoRange, frame_max = kNoRange;
  std::vector<int> frames;
  Split(TrimQuotes(FLAGS_extract_frame_range), ',', &frames);
  if (!frames.empty()) {
    CHECK_EQ(2, frames.size()) << "extract_frame_range must be frame PAIR";
    frame_min = frames[0];
    frame_max = frames[1];
    CHECK_LE(frame_min, frame_max)
        << "Minimum frame index must be <= than max frame index.";
  }

  hal::Reader reader(FLAGS_in);
  reader.Enable(hal::Msg_Type_Camera);

  int idx = 0;
  std::unique_ptr<hal::Msg> msg;
  while (frame_min != kNoRange && idx < frame_min) {
    if ((msg = reader.ReadMessage()) && msg->has_camera()) {
      ++idx;
    }
  }

  while ((frame_max == kNoRange ||
          idx <= frame_max) &&
         (msg = reader.ReadMessage())) {
    if (msg->has_camera()) {
      const hal::CameraMsg& cam_msg = msg->camera();
      for (int ii = 0; ii < cam_msg.image_size(); ++ii) {
        const hal::ImageMsg& img_msg = cam_msg.image(ii);
        SaveImage(FLAGS_out, ii, idx, cam_msg.system_time(), img_msg);
      }
      ++idx;
    }
  }
}

/** Extract certain elements of a log to a separate log file. */
void ExtractLog() {
  static const int kNoRange = -1;
  std::vector<std::string> types;
  Split(TrimQuotes(FLAGS_extract_types), ',', &types);

  int frame_min = kNoRange, frame_max = kNoRange;
  std::vector<int> frames;
  Split(TrimQuotes(FLAGS_extract_frame_range), ',', &frames);
  if (!frames.empty()) {
    CHECK_EQ(2, frames.size()) << "extract_frame_range must be frame PAIR";
    frame_min = frames[0];
    frame_max = frames[1];
    CHECK_LE(frame_min, frame_max)
        << "Minimum frame index must be <= than max frame index.";
  }

  hal::Reader reader(FLAGS_in);
  if (FLAGS_extract_types.empty()) {
    reader.EnableAll();
  } else {
    LOG(INFO) << "Extracting: ";
    for (const std::string& type : types) {
      LOG(INFO) << "\t" << type;
      reader.Enable(MsgTypeForString(type));
    }
  }

  int idx = 0;
  std::unique_ptr<hal::Msg> msg;
  while (frame_min != kNoRange && idx < frame_min) {
    if ((msg = reader.ReadMessage()) && msg->has_camera()) {
      ++idx;
    }
  }

  hal::Logger logger;
  logger.LogToFile(FLAGS_out);
  const uint32_t max_buffer_size = 5000; // This is the default in logger.
  logger.SetMaxBufferSize(max_buffer_size);
  while ((frame_max == kNoRange ||
          idx <= frame_max) &&
         (msg = reader.ReadMessage())) {
    if (msg->has_camera()) {
      ++idx;
    }

    while (logger.buffer_size() == max_buffer_size) {
      LOG(INFO) << "Hit max buffer size. Waiting 100ms.";
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!logger.LogMessage(*msg)) {
      break;
    }
  }
}

/**
 * Concatenate all the messages from the 'cat' log files into a
 * single file.
 */
void CatLogs() {
  std::vector<std::string> in;
  Split(TrimQuotes(FLAGS_cat_logs), ',', &in);

  hal::Logger logger;
  logger.LogToFile(FLAGS_out);
  for (const std::string& log : in) {
    hal::Reader reader(log);
    reader.EnableAll();

    std::unique_ptr<hal::Msg> msg;
    while ((msg = reader.ReadMessage())) {
      while (!logger.LogMessage(*msg)) {}
    }
  }
}

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_extract_log + FLAGS_extract_images + FLAGS_extract_imu +
      + FLAGS_extract_posys + !FLAGS_cat_logs.empty() != 1) {
    LOG(FATAL) << "Must choose one logtool task.";
  }

  if (FLAGS_extract_log) {
    CHECK(!FLAGS_in.empty()) << "Input file required for extraction.";
    CHECK(!FLAGS_out.empty()) << "Output file required for extraction.";
    ExtractLog();
  } else if (FLAGS_extract_images) {
    CHECK(!FLAGS_in.empty()) << "Input file required for extraction.";
    CHECK(!FLAGS_out.empty()) << "Output directory required for extraction.";
    ExtractImages();
  } else if (FLAGS_extract_imu) {
    CHECK(!FLAGS_in.empty()) << "Input file required for extraction.";
    CHECK(!FLAGS_out.empty()) << "Output directory required for extraction.";
    ExtractImu();
  } else if (FLAGS_extract_posys) {
    CHECK(!FLAGS_in.empty()) << "Input file required for extraction.";
    CHECK(!FLAGS_out.empty()) << "Output directory required for extraction.";
    ExtractPosys();
  } else if (!FLAGS_cat_logs.empty()) {
    CHECK(!FLAGS_out.empty()) << "Output file required for extraction.";
    CatLogs();
  }

  return 0;
}
