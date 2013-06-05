#pragma once

#include <camera/Camera.h>
#include <utils/StrongPointer.h>
#include <utils/String8.h>

#include <gui/SurfaceTexture.h>
#include <gui/SurfaceTextureClient.h>

#include <movidius-validation/validation-suite.h>

namespace hal {

class HALCameraListener :  public android::CameraListener {
public:
    HALCameraListener( bool bValidate = true );
    ~HALCameraListener();

    double GetTimestamp() { return m_dTimestamp; }
    unsigned char* GetBuffer() { return m_pBuffer; }

    void notify(int32_t msgType, int32_t ext1, int32_t ext2);
    void postData(int32_t msgType, const android::sp<android::IMemory>& dataPtr,
                  camera_frame_metadata_t *metadata);
    void postDataTimestamp(nsecs_t timestamp, int32_t msgType, const android::sp<android::IMemory>& dataPtr);

private:
    bool                m_bValidate;
    ValidationSuite     m_Validator;
    int                 m_nPrevFrameIdx;
    double              m_dTimestamp;
    unsigned char*      m_pBuffer;
};

} /* namespace */
