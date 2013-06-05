#include "CameraListener.h"

#include <frame-data/raisin-frame-data.h>

namespace hal {

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO,  "HAL", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN,  "HAL", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "HAL", __VA_ARGS__))

// number of frames for each log debug message output
#define FRAME_INTERVAL_LOG_OUTPUT 90


HALCameraListener::HALCameraListener( bool bValidate )
    : m_nPrevFrameIdx(0)
{
    m_bValidate = bValidate;
}

HALCameraListener::~HALCameraListener()
{

}

void HALCameraListener::notify(int32_t /*msgType*/, int32_t /*ext1*/, int32_t /*ext2*/)
{
    LOGI("HAL::notify");
}

void HALCameraListener::postData(int32_t /*msgType*/, const android::sp<android::IMemory>& dataPtr,
                  camera_frame_metadata_t* /*metadata*/)
{
    LOGI("HAL::postData");

    // from RaisinRunner
    ssize_t offset;
    size_t size;
    android::sp<android::IMemoryHeap> Heap = dataPtr->getMemory( &offset, &size);
    m_pBuffer = ((unsigned char *)Heap->base()) + offset ;

    // parse the superframe
    static frame_data::RaisinFrameData SuperFrame(1280,640);
    SuperFrame.ParseSensorPackets( m_pBuffer );
    int m_nNewFrameIdx = SuperFrame.frame_count();

    m_dTimestamp = SuperFrame.camera_timestamp(0);

    // not sure if it is worth have this or not
    if( m_bValidate ) {
        static int nTestedFrames = 0;
        static int nTears = 0;
        static int nStutters = 0;
        static int nJumps = 0;

        nTestedFrames++;
        // basic dropped frame detection
        if( m_nPrevFrameIdx == 0 ) {
            m_nPrevFrameIdx = m_nNewFrameIdx;
        }

        if((m_nNewFrameIdx - m_nPrevFrameIdx) != 1) {
        LOGE("Validation test: FAILED Frame Count Non-sequential: Prev: %d Curr: %d", m_nPrevFrameIdx, m_nNewFrameIdx);
        }
        m_nPrevFrameIdx = m_nNewFrameIdx;

        unsigned int nFrameTearResult = 0;
        nFrameTearResult = m_Validator.TestFrameTearing(SuperFrame);

        if (nFrameTearResult & 0x1 || nFrameTearResult & 0x2) {
        LOGE("Validation test: FAILED frame tearing test: code: %x", nFrameTearResult);
        nTears++;
        }
        if (!m_Validator.TestLeftRightTimestampOffset(SuperFrame)) {
        LOGE("Validation test: FAILED left-right timestamp offset test");
        }
        if (!m_Validator.TestSteadyFirmwareFramerate(SuperFrame)) {
        LOGE("Validation test: FAILED steady firmware framerate test");
        nStutters++;
        }
        if (!m_Validator.TestFrameSequenceCounts(SuperFrame)) {
        nJumps++;
        }
        if (!m_Validator.TestAccelRangeG(SuperFrame.GetAccelG(0,0))) {
        LOGE("Validation test: FAILED accel test");
        }
        if (!m_Validator.TestRaisinImuIntraframePacketInterval(SuperFrame)) {
        LOGE("Validation test: FAILED imu intraframe timing test");
        }
        if (!m_Validator.TestRaisinImuInterframePacketInterval(SuperFrame)) {
        LOGE("Validation test: FAILED imu interframe timing test");
        }
        if (!m_Validator.TestRaisinImuCameraTimeAlignment(SuperFrame)) {
        LOGE("Validation test: FAILED imu camera time alignment test");
        }
        float fRatioTorn = nTears/(float)nTestedFrames*100.0f;
        float fRatioStutters = nStutters/(float)nTestedFrames*100.0f;
        float fRatioJumps = nJumps/(float)nTestedFrames*100.0f;

        if (nTestedFrames % FRAME_INTERVAL_LOG_OUTPUT == 0 ) {
        LOGI("Validation Stats for %d images (%ds): torn: %d (%.2f%%), stutters: %d (%.2f%%), jumps: %d (%.2f%%)",
             nTestedFrames,nTestedFrames/30,
             nTears,
             fRatioTorn,
             nStutters,
             fRatioStutters,
             nJumps,
             fRatioJumps);
        }
    } else {
        if((m_nNewFrameIdx % FRAME_INTERVAL_LOG_OUTPUT) == 0) {
          LOGI("Current Frame: %d", m_nNewFrameIdx);
        }
    }
}

void HALCameraListener::postDataTimestamp(nsecs_t /*timestamp*/, int32_t /*msgType*/, const android::sp<android::IMemory>& /*dataPtr*/)
{
    LOGI("HAL::postDataTimestamp");
}

} /* namespace */
