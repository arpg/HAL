#include <jni.h>

extern "C" {
  JNIEXPORT void JNICALL Java_edu_gwu_rpg_nativesensorinterface_NativeSensorInterface_post_image
  (JNIEnv *env, jobject jobj, jbyteArray bytes)
  {
    int len = env->GetArrayLength(bytes);
    char* buf = new char[len];
    env->GetByteArrayRegion (bytes, 0, len, reinterpret_cast<jbyte*>(buf));
    // Do something with the image
    delete[] buf;
  }

  JNIEXPORT void JNICALL Java_edu_gwu_rpg_nativesensorinterface_NativeSensorInterface_initialize
  (JNIEnv *env, jobject jobj)
  {

  }
}
