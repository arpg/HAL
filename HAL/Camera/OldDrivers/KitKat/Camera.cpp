#include <jni.h>

#include <android/log.h>

#include <camera/ProCamera.h>
#include <hardware/camera3.h>
#include <system/camera_metadata.h>

#include "./Camera.h"

#define CAMERA_REQUEST_COUNT 5
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "native-camera", __VA_ARGS__))

namespace android{
  class listener_t;
}

typedef struct camera_t {
  android::sp<android::ProCamera> camera;
  android::sp<android::listener_t> listener;
  int stream_id;
  camera_metadata_t *request;
  android::sp<android::CpuConsumer> consumer;
  int request_count;
  int request_id[CAMERA_REQUEST_COUNT];
  image_t image;
} camera_t;

namespace android {
class listener_t : public ProCameraListener {
  public:
    listener_t(camera_t *cam) {camera = cam; last_buffer = NULL;};
    ~listener_t() {};
    virtual void onFrameAvailable(int stream_id,
                                  const sp<CpuConsumer> &consumer) {
      status_t ret;
      CpuConsumer::LockedBuffer buf;
      if ((ret = consumer->lockNextBuffer(&buf)) == OK) {
        struct timespec up_ts, unix_ts;
        uint64_t up_time, unix_time;
        clock_gettime(CLOCK_MONOTONIC, &up_ts);
        clock_gettime(CLOCK_REALTIME, &unix_ts);
        up_time = up_ts.tv_sec * 1e9 + up_ts.tv_nsec;
        unix_time = unix_ts.tv_sec * 1e9 + unix_ts.tv_nsec;
        double timestamp = (buf.timestamp - up_time + unix_time) * 1e-9;

        // DrawBuffer
        camera->image.framecount = buf.frameNumber;
        camera->image.timestamp = timestamp;
        camera->image.width  = buf.width;
        camera->image.height = buf.height;
        camera->image.buffer = (unsigned char*)buf.data;

        // Unlock the oldest buffer
        if (last_buffer != NULL)
          consumer->unlockBuffer(*(last_buffer));
        buffer = buf;
        last_buffer = &buffer;
      }

      camera->request_id[camera->request_count] =
        camera->camera->submitRequest(camera->request, false);
      camera->request_count++;
      if (camera->request_count >= CAMERA_REQUEST_COUNT)
        camera->request_count = 0;
    };
    virtual void notify(int32_t, int32_t, int32_t){};
    virtual void onLockAcquired(){};
    virtual void onLockReleased(){};
    virtual void onLockStolen(){};
    virtual void onTriggerNotify(int32_t, int32_t, int32_t){};
    virtual void onResultReceived(int32_t, camera_metadata* request){
      free_camera_metadata(request);
    };
  private:
    camera_t *camera;
    CpuConsumer::LockedBuffer buffer;
    CpuConsumer::LockedBuffer *last_buffer;
  };
}

static void camera_update_metadata(uint32_t tag, int data_count, void *data,
                                   camera_metadata_t **request) {
  int find;
  camera_metadata_entry_t entry;
  // Find the entry associated with output streams.
  find = find_camera_metadata_entry((*request), tag, &entry);
  if (find == -ENOENT) {
    // Entry not found so add it.
    if (add_camera_metadata_entry((*request), tag, data, data_count) != android::OK) {
      camera_metadata_t *tmp = allocate_camera_metadata(100, 1000);
      append_camera_metadata(tmp, (*request));
      free_camera_metadata((*request));
      (*request) = tmp;

      add_camera_metadata_entry((*request), tag, data, data_count);
    }
  } else {
    // Update the output streams.
    update_camera_metadata_entry((*request), entry.index,
                                 data, data_count, &entry);
  }
}

camera_t *camera_alloc(int id) {
  camera_t *camera = new camera_t();
  if ((camera->camera = android::ProCamera::connect(id)) == NULL) {
    delete camera;
    return NULL;
  }
  camera->listener = new android::listener_t(camera);
  camera->camera->setListener(camera->listener);
#if 1
//def nexus5
  int format = HAL_PIXEL_FORMAT_YCbCr_420_888;
#else
  int format = HAL_PIXEL_FORMAT_YCrCb_420_SP;
#endif
  const int MAX_BUFFERS = 15;
  int32_t afMode[] = { ANDROID_CONTROL_AF_MODE_OFF };
  int32_t faceMode[] = { ANDROID_STATISTICS_FACE_DETECT_MODE_OFF };
  int32_t streams[1];

  // Create a cpu stream
  if (camera->camera->createStreamCpu(1280, 720,
                                      format, MAX_BUFFERS,
                                      &camera->consumer,
                                      &camera->stream_id) != android::OK) {
    LOGW("Camera Failed to create CPU stream.");
    delete camera;
    return NULL;
  }
  streams[0] = camera->stream_id;

  // Create a default request
  camera->camera->createDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                       &camera->request);

  // Add our cpu stream as an output
  camera_update_metadata(ANDROID_REQUEST_OUTPUT_STREAMS,
                         1, &streams, &(camera->request));

  // Disable autofocus
  camera_update_metadata(ANDROID_CONTROL_AF_MODE,
                         1, &afMode, &(camera->request));

  // Disable face Detection
  camera_update_metadata(ANDROID_STATISTICS_FACE_DETECT_MODE,
                         1, &faceMode, &(camera->request));
  return camera;
}

void camera_play(camera_t *camera) {
  if (camera == NULL || camera->camera == NULL) return;
  if (camera->camera->exclusiveTryLock() == android::OK) {
    for (int i = 0 ; i < CAMERA_REQUEST_COUNT ; i++)
      camera->request_id[i] = camera->camera->submitRequest(camera->request, false);
  }
}

void camera_lock_buffer(camera_t *camera, image_t *image) {
  if (camera == NULL || image == NULL) return;
  *image = camera->image;
}

void camera_unlock_buffer(camera_t *camera, image_t *image) {}

void camera_stop(camera_t *camera) {
  if (camera == NULL) return;

  if (camera->camera != NULL &&
      camera->camera->hasExclusiveLock())
  {
    for (int i = 0 ; i < CAMERA_REQUEST_COUNT ; i++)
      camera->camera->cancelRequest(camera->request_id[i]);
    camera->camera->exclusiveUnlock();
  }
}

void camera_free(camera_t *camera) {
  if (camera == NULL) return;

  if (camera->camera != NULL) {
    camera_stop(camera);
    camera->camera->deleteStream(camera->stream_id);
    camera->camera->disconnect();
  }
  delete camera;
}
