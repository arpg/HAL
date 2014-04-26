#include <stdint.h>
#include <jni.h>
#include <android/log.h>
#include <GLES/gl.h>

extern "C" {
  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeOpenGLRenderer_initialize
  (JNIEnv *env, jobject jobj)
  {
    __android_log_print(ANDROID_LOG_INFO, "NativeOpenGLRenderer.cpp",
                        "Intialize");
    glClearColor(0.0, 0.0, 1.0, 0.0);
  }

  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeOpenGLRenderer_draw_1frame
  (JNIEnv *env, jobject jobj)
  {
    glClear(GL_COLOR_BUFFER_BIT);
  }

  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeOpenGLRenderer_finish
  (JNIEnv *env, jobject jobj)
  {
    __android_log_print(ANDROID_LOG_INFO, "NativeOpenGLRenderer.cpp",
                        "GL finished");
  }

  JNIEXPORT void JNICALL
  Java_arpg_androidlogger_NativeOpenGLRenderer_resize
  (JNIEnv *env, jobject jobj, jint width, jint height)
  {
    __android_log_print(ANDROID_LOG_INFO, "NativeOpenGLRenderer.cpp",
                        "GL resized to %d x %d",
                        static_cast<int>(width),
                        static_cast<int>(height));
  }
}
