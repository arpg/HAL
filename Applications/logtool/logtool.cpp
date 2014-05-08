#include <gflags/gflags.h>
#include <miniglog/logging.h>
#include <PbMsgs/Logger.h>
#include <PbMsgs/Reader.h>

/**
 * logtool functionality
 *
 * - [ ] Extract a given set of frames
 * - [ ] Concatenate multiple logs
 * - [ ] Reorder a log by timestamp
 * - [ ] Add an index
 * - [ ] Remove an index
 * - [ ] Output index, header to human-readable format
 */

DEFINE_string(in, "", "Input log file.");
DEFINE_string(out, "", "Output log file.");

DEFINE_bool(extract, false, "Enable log subset extraction.");
DEFINE_string(extract_types, "",
              "Comma-separated list of types to extract from log. "
              "Options include \"cam\", \"imu\", and \"posys\".");
DEFINE_string(extract_frame_range, "",
              "Range (inclusive) of image frames to extract from the log. "
              "Should be a comma-separated pair, e.g \"0,200\"");

DEFINE_string(cat, "",
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

/** Extract certain elements of a log to a separate log file. */
void Extract() {
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
  while (frame_min != kNoRange &&
         idx < frame_min) {
    (msg = reader.ReadMessage());    if (msg->has_camera()) {
      ++idx;
    }
  }

  pb::Logger logger;
  logger.LogToFile(FLAGS_out);
  while ((frame_max == kNoRange ||
          idx <= frame_max) &&
         (msg = reader.ReadMessage())) {
    if (msg->has_camera()) {
      ++idx;
    }
    while (!logger.LogMessage(*msg)) {}
  }
}

/**
 * Concatenate all the messages from the 'cat' log files into a
 * single file.
 */
void Cat() {
  std::vector<std::string> in;
  Split(FLAGS_cat, ',', &in);

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

  if (FLAGS_extract + !FLAGS_cat.empty() != 1) {
    LOG(FATAL) << "Must choose one logtool task.";
  }

  if (FLAGS_extract) {
    CHECK(!FLAGS_in.empty()) << "Input file required for extraction.";
    CHECK(!FLAGS_out.empty()) << "Output file required for extraction.";
    Extract();
  } else if (!FLAGS_cat.empty()) {
    CHECK(!FLAGS_out.empty()) << "Output file required for extraction.";
    Cat();
  }

  return 0;
}
