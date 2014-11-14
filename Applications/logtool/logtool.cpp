#include <iomanip>
#include <unistd.h>

#include <gflags/gflags.h>
#include <miniglog/logging.h>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif
#include <opencv.hpp>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <PbMsgs/Image.h>
#include <PbMsgs/Logger.h>
#include <PbMsgs/Reader.h>

/**
 * logtool functionality
 *
 * - [X] Extract a given set of frames
 * - [X] Extract a log into individual images.
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

inline pb::MessageType MsgTypeForString(const std::string& str) {
  static std::map<std::string, pb::MessageType> kTypeStrings = {
    {"cam", pb::Msg_Type_Camera},
    {"imu", pb::Msg_Type_IMU},
    {"encoder", pb::Msg_Type_Encoder},
    {"lidar", pb::Msg_Type_LIDAR},
    {"posys", pb::Msg_Type_Posys},
  };
  auto it = kTypeStrings.find(str);
  CHECK_NE(it, kTypeStrings.end()) << "Type string '" << str
                                   << "' does not match a known data type";
  return it->second;
}

/** Save individual file based on pb:Image type. */
inline void SaveImage(const std::string& out_dir,
                      int channel_index,
                      unsigned int frame_number,
                      const pb::ImageMsg& image) {

  // Convert index to string.
  std::string index;
  std::ostringstream convert;
  convert << channel_index;
  index = convert.str();

  std::string file_prefix = out_dir + "/";
  file_prefix = file_prefix + "channel" + index;

  convert.str("");
  convert.clear();
  convert << std::fixed << std::setfill('0') << std::setw(5) << frame_number;
  index = convert.str();

  std::string filename;

  // Use OpenCV to handle saving the file for us.
  cv::Mat cv_image = pb::WriteCvMat(image);

  if (image.type() == pb::Type::PB_FLOAT) {
    // Save floats to our own "portable depth map" format.
    filename = file_prefix + "_" + index + ".pdm";

    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    file << "P7" << std::endl;
    file << cv_image.cols << " " << cv_image.rows << std::endl;
    const size_t size = cv_image.elemSize1() * cv_image.rows * cv_image.cols;
    file << 4294967295 << std::endl;
    file.write((const char*)cv_image.data, size);
    file.close();
  } else if (image.type() == pb::Type::PB_BYTE
             || image.type() == pb::Type::PB_UNSIGNED_BYTE
             || image.type() == pb::Type::PB_SHORT
             || image.type() == pb::Type::PB_UNSIGNED_SHORT) {
    // OpenCV only supports byte/short data types with 1/3 channel images.
    filename = file_prefix + "_" + index + ".pgm";
    cv::imwrite(filename, cv_image);
  } else {
    LOG(FATAL) << "Input image type not supported for extraction.";
  }
}

/** Extracts single images out of a log file. */
void ExtractImu() {
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

  pb::Reader reader(FLAGS_in);
  reader.Enable(pb::Msg_Type_IMU);

  int idx = 0;
  std::unique_ptr<pb::Msg> msg;
  while (frame_min != kNoRange && idx < frame_min) {
    if ((msg = reader.ReadMessage()) && msg->has_camera()) {
      ++idx;
    }
  }

  while ((frame_max == kNoRange ||
          idx <= frame_max) &&
         (msg = reader.ReadMessage())) {
    if (msg->has_imu()) {
      // WRITE THE IMU
      ++idx;

    }
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

  pb::Reader reader(FLAGS_in);
  reader.Enable(pb::Msg_Type_Camera);

  int idx = 0;
  std::unique_ptr<pb::Msg> msg;
  while (frame_min != kNoRange && idx < frame_min) {
    if ((msg = reader.ReadMessage()) && msg->has_camera()) {
      ++idx;
    }
  }

  while ((frame_max == kNoRange ||
          idx <= frame_max) &&
         (msg = reader.ReadMessage())) {
    if (msg->has_camera()) {
      const pb::CameraMsg& cam_msg = msg->camera();
      for (int ii = 0; ii < cam_msg.image_size(); ++ii) {
        const pb::ImageMsg& img_msg = cam_msg.image(ii);
        SaveImage(FLAGS_out, ii, idx, img_msg);
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

  pb::Reader reader(FLAGS_in);
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
  std::unique_ptr<pb::Msg> msg;
  while (frame_min != kNoRange && idx < frame_min) {
    if ((msg = reader.ReadMessage()) && msg->has_camera()) {
      ++idx;
    }
  }

  pb::Logger logger;
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

  pb::Logger logger;
  logger.LogToFile(FLAGS_out);
  for (const std::string& log : in) {
    pb::Reader reader(log);
    reader.EnableAll();

    std::unique_ptr<pb::Msg> msg;
    while ((msg = reader.ReadMessage())) {
      while (!logger.LogMessage(*msg)) {}
    }
  }
}

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_extract_log + FLAGS_extract_images
      + !FLAGS_cat_logs.empty() != 1) {
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
  } else if (!FLAGS_cat_logs.empty()) {
    CHECK(!FLAGS_out.empty()) << "Output file required for extraction.";
    CatLogs();
  }

  return 0;
}
