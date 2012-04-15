/*
 * File:   CaptureDelegate.cpp
 * Author: jmf
 *
 * Created on April 10, 2012, 4:25 PM
 */

#include "CaptureDelegate.h"

#include <stdio.h>
#include <stdlib.h>

#include <sys/time.h>
#include <math.h>


// Aux Time Functions
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////

inline double Tic() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec + 1e-6 * (tv.tv_usec);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////

inline double RealTime() {
    return Tic();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////

inline double Toc(double dTic) {
    return Tic() - dTic;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double TocMS(double dTic) {
    return ( Tic() - dTic)*1000.;
}

double ttime = 0;
long unsigned int count = 0;



extern pthread_cond_t sleepCond;

CaptureDelegate::CaptureDelegate(const int bufferCount, int nNumImages, int nImageWidth, int nImageHeight) : m_refCount(0), m_frameCount(0), m_maxFrames(-1), m_timecodeFormat(0) {
    m_nImageWidth = nImageWidth;
    m_nImageHeight = nImageHeight;
    m_nNumImages = nNumImages;
    
    m_nBufferCount = bufferCount;
    pthread_mutex_init(&m_mutex, NULL);
    m_pContext = new zmq::context_t(1);
    m_pSocket = new zmq::socket_t(*m_pContext, ZMQ_PUB);
    m_pSocket->bind("tcp://*:6666");

    //fill the used and free buffers
    for (int ii = 0; ii < m_nBufferCount; ii++) {
        m_pFreeBuffers.push(new char[1920 * 1080 * 4]);
    }
}

CaptureDelegate::~CaptureDelegate() {
    pthread_mutex_destroy(&m_mutex);
}

ULONG CaptureDelegate::AddRef(void) {
    pthread_mutex_lock(&m_mutex);
    m_refCount++;
    pthread_mutex_unlock(&m_mutex);

    return (ULONG) m_refCount;
}

ULONG CaptureDelegate::Release(void) {
    pthread_mutex_lock(&m_mutex);
    m_refCount--;
    pthread_mutex_unlock(&m_mutex);

    if (m_refCount == 0) {
        delete this;
        return 0;
    }

    return (ULONG) m_refCount;
}

void Clamp(short& T) {
    if (T > 255) {
        T = 255;
    }
    if (T < 0) {
        T = 0;
    }
}

bool CaptureDelegate::GetFrame(char *&buffer, int &lengthOut)
{
    pthread_mutex_lock(&m_mutex);
    
    if(m_pUsedBuffers.empty()) {
        pthread_mutex_unlock(&m_mutex);
        lengthOut = 0;
        buffer = NULL;
        
        return false;
    }else {
        char *buffer = m_pUsedBuffers.front();
        m_pUsedBuffers.pop();
        //add this to the free buffers
        m_pFreeBuffers.push(buffer);
        
        lengthOut = m_nNumImages * m_nImageWidth * m_nImageHeight * 4;
        //std::memcpy(buffer,ptr,lengthOut);
       pthread_mutex_unlock(&m_mutex);
       
       return true;
    }
}

HRESULT CaptureDelegate::VideoInputFrameArrived(IDeckLinkVideoInputFrame* videoFrame, IDeckLinkAudioInputPacket* /* audioFrame */) {
    void* frameBytes;

    // Handle Video Frame
    if (videoFrame) {
        if (videoFrame->GetFlags() & bmdFrameHasNoInputSource) {
            fprintf(stderr, "Frame received (#%lu) - No input signal detected\n", m_frameCount);
        } else {

            double t = Tic();


            const char *timecodeString = NULL;
            if (m_timecodeFormat != 0) {
                IDeckLinkTimecode *timecode;
                if (videoFrame->GetTimecode(m_timecodeFormat, &timecode) == S_OK) {
                    timecode->GetString(&timecodeString);
                }
            }

            /*
            fprintf(stderr, "Frame received (#%lu) [%s] - Valid Frame - Height: %li - Size: %li bytes\n",
                    m_frameCount,
                    timecodeString != NULL ? timecodeString : "No timecode",
                videoFrame->GetHeight(),
                    videoFrame->GetRowBytes() * videoFrame->GetHeight());
            /**/

            if (timecodeString) {
                free((void*) timecodeString);
            }

            //if( m_pSocket ) {
            //if( m_pSocket ) {
            // this is hardcoded for now
            //int NUM_IMAGES = 1;
            //				int IMG_WIDTH = 640;
            //int IMG_WIDTH = 1280;
            //				int IMG_HEIGHT = 480;
            //int IMG_HEIGHT = 720;
            //int IMG_TYPE = 0; // equal to CV_8UC1

            // calculate image size
            //int IMG_SIZE = IMG_WIDTH * IMG_HEIGHT * 1 /* # channels */;

            // calculate total number of bytes
            unsigned long int NumBytes = m_nImageWidth * m_nImageHeight;

            // images need to be sent in the following format:
            // NumImages|Img1Width|Img1Height|Img1Format|Img1Data|Img2Width|...
            //zmq::message_t ZmqMsg(4 + (NUM_IMAGES * (12 + IMG_SIZE)));
            char *MsgPtr;
            //if there are no free buffers it means whoever is reading this
            //is not reading fast enough so we just have to rewrite over the 
            //first item
            pthread_mutex_lock(&m_mutex);
            if (m_pFreeBuffers.empty()) {
                MsgPtr = m_pUsedBuffers.front();
            } else {
                //otherwise take one of the free buffers
                MsgPtr = m_pFreeBuffers.front();
                m_pFreeBuffers.pop();
            }
            pthread_mutex_unlock(&m_mutex);

            // push number of images in message
            //memcpy( MsgPtr, &NUM_IMAGES, sizeof(NUM_IMAGES) );
            //MsgPtr += sizeof(NUM_IMAGES);

            // get frame pointer
            videoFrame->GetBytes(&frameBytes);
            unsigned char* fb = (unsigned char*) frameBytes;

            // increment pointer to start on Y0
            fb++;

            // YUV scale conversion
            float fscale = 255.0 / 219.0;

            for (int nn = 0; nn < m_nNumImages; nn++) {
                /*
                                    // push width, height and image type
                memcpy( MsgPtr, &IMG_WIDTH, sizeof(IMG_WIDTH) );
                                    MsgPtr += sizeof(IMG_WIDTH);
                                    memcpy( MsgPtr, &IMG_HEIGHT, sizeof(IMG_HEIGHT) );
                                    MsgPtr += sizeof(IMG_HEIGHT);
                                    memcpy( MsgPtr, &IMG_TYPE, sizeof(IMG_TYPE) );
                                    MsgPtr += sizeof(IMG_TYPE);
                 */
                /*
                //				for(int ii = 0; ii < 256; ii++ ) {
                                    printf("first: %d -- ", *fb);
                                    unsigned char low = round((*fb-16.0)*fscale/16.0);
                                    fb += 2;
                                    printf("second: %d --", *fb);
                                    unsigned char high = round((*fb-16)*fscale/16.0);
                                    high = high << 4;
                                    unsigned char byte = high|low;
                                    printf("lo: %d -- hi: %d -- val: %d = %c\n", low, high, byte, byte);
                                    fb += 2;
                //				}
                 */
                //*MsgPtr = byte; MsgPtr++;

                //now do the conversion and write it to the pointer
                for (unsigned long int ii = 0; ii < NumBytes; ii += 4) {
                    unsigned char Cb = *fb;
                    fb++;
                    unsigned char Y0 = *fb;
                    fb++;
                    unsigned char Cr = *fb;
                    fb++;
                    unsigned char Y1 = *fb;
                    fb++;

                    short R1 = 1.164 * (Y0 - 16) + 1.793 * (Cr - 128);
                    Clamp(R1);
                    short G1 = 1.164 * (Y0 - 16) - 0.534 * (Cr - 128) - 0.213 * (Cb - 128);
                    Clamp(G1);
                    short B1 = 1.164 * (Y0 - 16) + 2.115 * (Cb - 128);
                    Clamp(B1);
                    short R2 = 1.164 * (Y1 - 16) + 1.793 * (Cr - 128);
                    Clamp(R2);
                    short G2 = 1.164 * (Y1 - 16) - 0.534 * (Cr - 128) - 0.213 * (Cb - 128);
                    Clamp(G2);
                    short B2 = 1.164 * (Y1 - 16) + 2.115 * (Cb - 128);
                    Clamp(B2);

                    *MsgPtr = B1;
                    MsgPtr++;
                    *MsgPtr = G1;
                    MsgPtr++;
                    *MsgPtr = R1;
                    MsgPtr++;
                    *MsgPtr = B2;
                    MsgPtr++;
                    *MsgPtr = G2;
                    MsgPtr++;
                    *MsgPtr = R2;
                    MsgPtr++;

                }

            }
            
            
            //and now we add this to the tail of the used buffers
            //this is in a critical section
            pthread_mutex_lock(&m_mutex);
            m_pUsedBuffers.push(MsgPtr);
            pthread_mutex_unlock(&m_mutex);




            ttime += Toc(t);
            count++;

            if (count > 300) {
                fprintf(stderr, "Frame rate is %.2f\n", count / ttime);
                exit(0);
            }
        }

        m_frameCount++;
        if (m_maxFrames > 0 && m_frameCount >= m_maxFrames) {
            pthread_cond_signal(&sleepCond);
        }
    }

    return S_OK;
}

HRESULT CaptureDelegate::VideoInputFormatChanged(BMDVideoInputFormatChangedEvents events, IDeckLinkDisplayMode *mode, BMDDetectedVideoInputFormatFlags) {
    return S_OK;
}
