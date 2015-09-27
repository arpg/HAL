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
    : m_nCurrentImgId(0),
      m_nReadImgId(0),
      m_pBuffer(nullptr)
{
}

HALCameraListener::~HALCameraListener()
{
    if(m_pBuffer) {
        delete[] m_pBuffer;
    }
}

bool HALCameraListener::GetImages( hal::ImageMsg* pbImg )
{
    if(!m_pBuffer) {
        return false;
    }

    std::unique_lock<std::mutex> lock(m_Mutex);

    while( m_nCurrentImgId == m_nReadImgId ) {
        m_NewImg.wait(lock);
    }


    pbImg->set_data( m_pBuffer, m_nBuffSize );
    m_nReadImgId++;

    return true;
}


void HALCameraListener::notify(int32_t /*msgType*/, int32_t /*ext1*/, int32_t /*ext2*/)
{
}

void HALCameraListener::postData(int32_t /*msgType*/, const android::sp<android::IMemory>& dataPtr,
                  camera_frame_metadata_t* /*metadata*/)
{
    ssize_t offset;
    android::sp<android::IMemoryHeap> Heap = dataPtr->getMemory( &offset, &m_nBuffSize);

    if(!m_pBuffer) {
        m_pBuffer = new unsigned char[m_nBuffSize];
    }

    m_Mutex.lock();
    memcpy(m_pBuffer, ((unsigned char *)Heap->base()) + offset, m_nBuffSize);
    m_nCurrentImgId++;
    m_Mutex.unlock();
    m_NewImg.notify_one();
}

void HALCameraListener::postDataTimestamp(nsecs_t /*timestamp*/, int32_t /*msgType*/, const android::sp<android::IMemory>& /*dataPtr*/)
{
}

} /* namespace */
