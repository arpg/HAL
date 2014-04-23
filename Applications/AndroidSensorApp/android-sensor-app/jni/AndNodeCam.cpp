#include <jni.h>
#include <AndCam.h>
#include "AndNodeCam.h"
#include <android/Log.h>


JNIEXPORT void JNICALL Java_edu_gwu_rpg_androidnodecam_AndNodeCam_sendData
  (JNIEnv *env, jobject jobj, jbyteArray bytes)
{
    int len = env->GetArrayLength (bytes);
    char* buf;
    buf = new char[len];
    env->GetByteArrayRegion (bytes, 0, len, reinterpret_cast<jbyte*>(buf));
    sendData(buf, 640, 480, 5121, 6409);
    delete[] buf;
}

JNIEXPORT void JNICALL Java_edu_gwu_rpg_androidnodecam_AndNodeCam_Initialize
  (JNIEnv *env, jobject jobj, jstring ip_address, jint port)
{
    InitializeNode(env->GetStringUTFChars(ip_address, NULL), (int) port);
}
