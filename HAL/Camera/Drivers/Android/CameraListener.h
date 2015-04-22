#pragma once

#include <mutex>
#include <condition_variable>

#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <camera/Camera.h>
#pragma GCC diagnostic pop

#include <HAL/Image.pb.h>


namespace hal {

class HALCameraListener :  public android::CameraListener {
public:
    HALCameraListener();
    ~HALCameraListener();

    bool GetImages( hal::ImageMsg* pbImg );
    double GetTimestamp() { return m_dTimestamp; }

    void notify(int32_t msgType, int32_t ext1, int32_t ext2);
    void postData(int32_t msgType, const android::sp<android::IMemory>& dataPtr,
                  camera_frame_metadata_t *metadata);
    void postDataTimestamp(nsecs_t timestamp, int32_t msgType, const android::sp<android::IMemory>& dataPtr);

private:
    unsigned int                m_nCurrentImgId;
    unsigned int                m_nReadImgId;
    std::mutex                  m_Mutex;
    std::condition_variable     m_NewImg;
    double                      m_dTimestamp;
    unsigned char*              m_pBuffer;
    size_t                      m_nBuffSize;
};

} /* namespace */
