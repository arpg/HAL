#include "CameraListener.h"

#include <utils/StrongPointer.h>
#include <utils/String8.h>

namespace hal {

#define LOGV(...) ((void)__android_log_print(ANDROID_LOG_VERBOSE,  "HAL", __VA_ARGS__))
#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO,  "HAL", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN,  "HAL", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "HAL", __VA_ARGS__))

// number of frames for each log debug message output
#define FRAME_INTERVAL_LOG_OUTPUT 90


HALCameraListener::HALCameraListener()
    : m_nPrevFrameIdx(0), m_pBuffer(0)
{
}

HALCameraListener::~HALCameraListener()
{
    if(m_pBuffer) {
        delete[] m_pBuffer;
    }
}

void HALCameraListener::notify(int32_t /*msgType*/, int32_t /*ext1*/, int32_t /*ext2*/)
{
    LOGV("HAL::notify");
}

void HALCameraListener::postData(int32_t /*msgType*/, const android::sp<android::IMemory>& dataPtr,
                  camera_frame_metadata_t* /*metadata*/)
{
    LOGV("HAL::postData");

    ssize_t offset;
    size_t size;
    android::sp<android::IMemoryHeap> Heap = dataPtr->getMemory( &offset, &size);
    
    if(!m_pBuffer) {
        m_pBuffer = new unsigned char[size];
    }    
    
    memcpy(m_pBuffer, ((unsigned char *)Heap->base()) + offset, size);
}

void HALCameraListener::postDataTimestamp(nsecs_t /*timestamp*/, int32_t /*msgType*/, const android::sp<android::IMemory>& /*dataPtr*/)
{
    LOGV("HAL::postDataTimestamp");
}

} /* namespace */
