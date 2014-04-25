#include <stdint.h>
#include <jni.h>
#include <android/Log.h>
#include <PbMsgs/Logger.h>

static pb::Logger& logger = pb::Logger::GetInstance();

static pb::Msg msg;

extern "C" {
  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeSensorInterface_initialize
  (JNIEnv *env, jobject jobj, jint img_width, jint img_height)
  {
    logger.LogToFile("/data/data/arpg.androidlogger", "arpg");

    pb::ImageMsg* img = msg.mutable_camera()->add_image();
    img->set_width(img_width);
    img->set_height(img_height);
  }

  JNIEXPORT jint JNICALL
  Java_arpg_androidlogger_NativeSensorInterface_num_1logged
  (JNIEnv *env, jobject jobj)
  {
    return static_cast<jint>(logger.messages_written());
  }

  JNIEXPORT jint JNICALL
  Java_arpg_androidlogger_NativeSensorInterface_num_1queued
  (JNIEnv *env, jobject jobj)
  {
    return static_cast<jint>(logger.buffer_size());
  }

  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeSensorInterface_post_1image
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

    int len = env->GetArrayLength(bytes);
    img->mutable_data()->resize(len);

    env->GetByteArrayRegion(bytes, 0, len,
                            reinterpret_cast<jbyte*>(
                                &img->mutable_data()->front()));
    logger.LogMessage(msg);
  }

  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeSensorInterface_post_1accel
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
  Java_arpg_androidlogger_NativeSensorInterface_post_1gyro
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
  Java_arpg_androidlogger_NativeSensorInterface_post_1gps
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
