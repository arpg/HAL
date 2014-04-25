#include <stdint.h>
#include <jni.h>
#include <android/Log.h>

extern "C" {
  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeSensorInterface_initialize
  (JNIEnv *env, jobject jobj)
  {
  }

  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeSensorInterface_post_1image
  (JNIEnv *env, jobject jobj, jlong timestamp, jbyteArray bytes)
  {
    int len = env->GetArrayLength(bytes);
    char* buf = new char[len];
    env->GetByteArrayRegion(bytes, 0, len, reinterpret_cast<jbyte*>(buf));

    __android_log_print(ANDROID_LOG_INFO, "nativesensorinterface",
                        "Got an image at time %lld of size %d",
                        static_cast<int64_t>(timestamp),
                        len);

    // Do something with the image
    delete[] buf;
  }

  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeSensorInterface_post_1accel
  (JNIEnv *env, jobject jobj, jlong timestamp, jfloat x, jfloat y, jfloat z)
  {
    __android_log_print(ANDROID_LOG_INFO, "nativesensorinterface",
                        "Got accel data at %lld (%f, %f, %f)",
                        static_cast<int64_t>(timestamp),
                        static_cast<float>(x),
                        static_cast<float>(y),
                        static_cast<float>(z));
  }

  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeSensorInterface_post_1gyro
  (JNIEnv *env, jobject jobj, jlong timestamp, jfloat x, jfloat y, jfloat z)
  {
    __android_log_print(ANDROID_LOG_INFO, "nativesensorinterface",
                        "Got gyro data at %lld: (%f, %f, %f)",
                        static_cast<int64_t>(timestamp),
                        static_cast<float>(x),
                        static_cast<float>(y),
                        static_cast<float>(z));
  }

  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeSensorInterface_post_1gps
  (JNIEnv *env, jobject jobj, jlong timestamp, jdouble lat, jdouble lon,
   jdouble alt, jfloat std)
  {
    __android_log_print(ANDROID_LOG_INFO, "nativesensorinterface",
                        "Got GPS data at %lld: (%f, %f) at %f meters above "
                        "sea level w/std %f",
                        static_cast<int64_t>(timestamp),
                        static_cast<double>(lat),
                        static_cast<double>(lon),
                        static_cast<double>(alt),
                        static_cast<float>(std));
  }
}
