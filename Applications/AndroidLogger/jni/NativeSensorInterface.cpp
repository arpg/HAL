#include <stdint.h>
#include <cstdio>
#include <jni.h>
#include <android/log.h>
#include <PbMsgs/Logger.h>
#include <miniglog/logging.h>

static pb::Logger& logger = pb::Logger::GetInstance();
static std::string log_file;
static pb::Msg msg;
static int image_width = 0, image_height = 0;

extern "C" {
  JNIEXPORT void JNICALL
  Java_arpg_nativesensorinterface_NativeSensorInterface_initialize
  (JNIEnv *env, jobject jobj, jstring output_dir,
   jint img_width, jint img_height)
  {
    const char* s = env->GetStringUTFChars(output_dir,NULL);
    std::string dir = s;
    env->ReleaseStringUTFChars(output_dir, s);

    log_file = logger.LogToFile(dir, "arpg");
    image_width = img_width;
    image_height = img_height;
  }

  JNIEXPORT jstring JNICALL
  Java_arpg_nativesensorinterface_NativeSensorInterface_logfile
  (JNIEnv *env, jobject jobj)
  {
    return env->NewStringUTF(log_file.c_str());
  }

  JNIEXPORT jint JNICALL
  Java_arpg_nativesensorinterface_NativeSensorInterface_num_1logged
  (JNIEnv *env, jobject jobj)
  {
    return static_cast<jint>(logger.messages_written());
  }

  JNIEXPORT jint JNICALL
  Java_arpg_nativesensorinterface_NativeSensorInterface_num_1queued
  (JNIEnv *env, jobject jobj)
  {
    return static_cast<jint>(logger.buffer_size());
  }

  JNIEXPORT void JNICALL
  Java_arpg_nativesensorinterface_NativeSensorInterface_finish
  (JNIEnv *env, jobject jobj, jboolean copy)
  {
    logger.StopLogging();
    if (!copy) return;

    std::ifstream in(log_file, std::ios::binary | std::ios::in);

    size_t last_slash = log_file.find_last_of('/');
    std::string filename = ((last_slash == std::string::npos) ? log_file :
                            log_file.substr(last_slash + 1));
    std::string output_file = "/sdcard/" + filename;
    std::ofstream out(output_file, std::ios::binary | std::ios::trunc);

    LOG(INFO) << "Moving log from " << log_file << " to " << output_file;
    out << in.rdbuf();
    in.close();
    out.close();

    // In order to preserve the log ordering, but not waste space, we
    // nuke the contents of the old log file.
    std::ofstream old_log(log_file, std::ios::trunc);
    old_log << 0;
    old_log.close();

    LOG(INFO) << "Done moving log.";
  }

  JNIEXPORT void JNICALL
  Java_arpg_nativesensorinterface_NativeSensorInterface_post_1image
  (JNIEnv *env, jobject jobj, jlong timestamp, jbyteArray bytes)
  {
    msg.Clear();

    // Scale timestamp to seconds from nanoseconds
    double sec_ts = static_cast<int64_t>(timestamp) * 1e-9;
    msg.set_timestamp(sec_ts);

    pb::CameraMsg* cam = msg.mutable_camera();
    cam->set_id(0);
    cam->set_device_time(sec_ts);

    pb::ImageMsg* img = cam->add_image();
    img->set_timestamp(sec_ts);
    img->set_width(image_width);
    img->set_height(image_height);
    img->set_format(pb::PB_LUMINANCE);

    // Size of luminance portion of image for format NV16
    int lum_size = image_width * image_height;

    int len = env->GetArrayLength(bytes);
    CHECK_GE(len, lum_size);  // Make sure the luminance is a subimage

    img->mutable_data()->resize(lum_size);
    env->GetByteArrayRegion(bytes, 0, lum_size,
                            reinterpret_cast<jbyte*>(
                                &img->mutable_data()->front()));
    logger.LogMessage(msg);
  }

  JNIEXPORT void JNICALL
  Java_arpg_nativesensorinterface_NativeSensorInterface_post_1accel
  (JNIEnv *env, jobject jobj, jlong timestamp, jfloat x, jfloat y, jfloat z)
  {
    msg.Clear();

    // Scale timestamp to seconds from nanoseconds
    double sec_ts = static_cast<int64_t>(timestamp) * 1e-9;
    msg.set_timestamp(sec_ts);

    pb::ImuMsg* imu = msg.mutable_imu();
    imu->set_id(0);
    imu->set_device_time(sec_ts);

    pb::VectorMsg* accel = imu->mutable_accel();
    accel->add_data(x);
    accel->add_data(y);
    accel->add_data(z);

    logger.LogMessage(msg);
  }

  JNIEXPORT void JNICALL
  Java_arpg_nativesensorinterface_NativeSensorInterface_post_1mag
  (JNIEnv *env, jobject jobj, jlong timestamp, jfloat x, jfloat y, jfloat z)
  {
    msg.Clear();

    // Scale timestamp to seconds from nanoseconds
    double sec_ts = static_cast<int64_t>(timestamp) * 1e-9;
    msg.set_timestamp(sec_ts);

    pb::ImuMsg* imu = msg.mutable_imu();
    imu->set_id(0);
    imu->set_device_time(sec_ts);

    pb::VectorMsg* mag = imu->mutable_mag();
    mag->add_data(x);
    mag->add_data(y);
    mag->add_data(z);

    logger.LogMessage(msg);
  }

  JNIEXPORT void JNICALL
  Java_arpg_nativesensorinterface_NativeSensorInterface_post_1gyro
  (JNIEnv *env, jobject jobj, jlong timestamp, jfloat x, jfloat y, jfloat z)
  {
    msg.Clear();

    // Scale timestamp to seconds from nanoseconds
    double sec_ts = static_cast<int64_t>(timestamp) * 1e-9;
    msg.set_timestamp(sec_ts);

    pb::ImuMsg* imu = msg.mutable_imu();
    imu->set_id(0);
    imu->set_device_time(sec_ts);

    pb::VectorMsg* gyro = imu->mutable_gyro();
    gyro->add_data(x);
    gyro->add_data(y);
    gyro->add_data(z);

    logger.LogMessage(msg);
  }

  JNIEXPORT void JNICALL
  Java_arpg_nativesensorinterface_NativeSensorInterface_post_1gps
  (JNIEnv *env, jobject jobj, jlong timestamp, jdouble lat, jdouble lon,
   jdouble alt, jfloat std)
  {
    msg.Clear();

    // Scale timestamp to seconds from nanoseconds
    double sec_ts = static_cast<int64_t>(timestamp) * 1e-9;
    msg.set_timestamp(sec_ts);

    pb::PoseMsg* pose = msg.mutable_pose();
    pose->set_type(pb::PoseMsg::LatLongAlt);
    pose->set_id(0);
    pose->set_device_time(sec_ts);

    pb::VectorMsg* pose_vec = pose->mutable_pose();
    pose_vec->add_data(lat);
    pose_vec->add_data(lon);
    pose_vec->add_data(alt);
    pose->set_std(std);

    logger.LogMessage(msg);
  }
}
