#include <gflags/gflags.h>
#include <miniglog/logging.h>

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
              "Range of image frames to extract from the log. "
              "Should be a comma-separated pair, e.g \"0,200\"");

void extract() {
  // Parse types to extract
  // Parse frame range to extract or set to -1
  // Open the input file
  // Seek to range min
  // While not at the range max
  // If we can get a message of one of the right types
}

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_extract) {
    extract();
  }

  return 0;
}
