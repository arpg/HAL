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

double ttime = Tic();
long unsigned int count = 0;


CaptureDelegate::CaptureDelegate(const int bufferCount, int nNumImages, int nImageWidth, int nImageHeight) : m_maxFrames(-1), m_timecodeFormat(0) {
    m_frameCount = 0;
    m_refCount = 0;
    m_nImageWidth = nImageWidth;
    m_nImageHeight = nImageHeight;
    m_nNumImages = nNumImages;

    m_nBufferCount = bufferCount;
    pthread_mutex_init(&m_mutex, NULL);

    //fill the used and free buffers
    for (unsigned int ii = 0; ii < m_nBufferCount; ii++) {
        ImageBufferStruct *pStr = new ImageBufferStruct();
        pStr->m_pControlBuffer = new unsigned char[1280];
        pStr->m_pImageBuffer = new unsigned char[1920 * 1080 * 4];
        m_pFreeBuffers.push(pStr);
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

inline void Clamp(short& T) {
    if (T > 255) {
        T = 255;
    }else if (T < 0) {
        T = 0;
    }
}

bool CaptureDelegate::GetFrame(unsigned char *&buffer, int &lengthOut, unsigned char *&controlBuffer, int &controlLengthOut)
{
    pthread_mutex_lock(&m_mutex);

    if(m_pUsedBuffers.empty()) {
        pthread_mutex_unlock(&m_mutex);
        lengthOut = 0;
        buffer = NULL;

        return false;
    }else {
        //please don't refactor this code. You are very smart, but pop does not
        //return the front element.
        ImageBufferStruct *pStr = m_pUsedBuffers.front();
        m_pUsedBuffers.pop();
        buffer = pStr->m_pImageBuffer;
        controlBuffer = pStr->m_pControlBuffer;

        //add this to the free buffers
        m_pFreeBuffers.push(pStr);
        //fprintf(stderr,"Popped a frame. Currently %d used buffers and %d free buffers.\n",m_pUsedBuffers.size(),m_pFreeBuffers.size());
        lengthOut = m_nNumImages * m_nImageWidth * m_nImageHeight;
        controlLengthOut = 160;
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

            //double t = Tic();


            const char *timecodeString = NULL;
            if (m_timecodeFormat != 0) {
                IDeckLinkTimecode *timecode;
                if (videoFrame->GetTimecode(m_timecodeFormat, &timecode) == S_OK) {
                    timecode->GetString(&timecodeString);
                }
            }

            if (timecodeString) {
                free((void*) timecodeString);
            }

            /*
            fprintf(stderr, "Frame received (#%lu) [%s] - Valid Frame - Height: %li - Size: %li bytes\n",
                    m_frameCount,
                    timecodeString != NULL ? timecodeString : "No timecode",
                videoFrame->GetHeight(),
                    videoFrame->GetRowBytes() * videoFrame->GetHeight());
            */


            // calculate total number of bytes
            unsigned long int NumBytes = m_nImageWidth * m_nImageHeight;

            ImageBufferStruct *MsgPtr;
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

            // get frame pointer
            videoFrame->GetBytes(&frameBytes);

            unsigned char* fb = (unsigned char*) frameBytes;

            /*
            fb++;
            unsigned char *controlPtr = MsgPtr->m_pControlBuffer;
            for(int ii = 0; ii < 160; ii++ ) {
                unsigned char byte = 0;
                for( int jj = 0; jj < 8; jj++ ) {
                    //printf("%d ",*fb);
                    if( *fb > 128 ) {
                        byte |= 1;
                    }
                    byte = byte << 1;
                    fb += 2;
                }
                //printf("\n");
                *controlPtr = byte;
                controlPtr++;
            }
            fb--;

            */

            unsigned char *framePtr = MsgPtr->m_pImageBuffer;
            for (int nn = 0; nn < m_nNumImages; nn++) {
                //now do the conversion and write it to the pointer
                for (unsigned long int ii = 0; ii < NumBytes; ii += 6) {
                    unsigned char Cb = *fb;
                    fb++;
                    unsigned char Y0 = *fb;
                    fb++;
                    unsigned char Cr = *fb;
                    fb++;
                    unsigned char Y1 = *fb;
                    fb++;

                    short R1 = 1.164 * (Y0 - 16) + 1.793 * (Cr - 128);
                    //Clamp(R1);
                    short G1 = 1.164 * (Y0 - 16) - 0.534 * (Cr - 128) - 0.213 * (Cb - 128);
                    //Clamp(G1);
                    short B1 = 1.164 * (Y0 - 16) + 2.115 * (Cb - 128);
                    //Clamp(B1);
                    short R2 = 1.164 * (Y1 - 16) + 1.793 * (Cr - 128);
                    //Clamp(R2);
                    short G2 = 1.164 * (Y1 - 16) - 0.534 * (Cr - 128) - 0.213 * (Cb - 128);
                    //Clamp(G2);
                    short B2 = 1.164 * (Y1 - 16) + 2.115 * (Cb - 128);
                    //Clamp(B2);

                    *framePtr = R1;
                    framePtr++;
                    *framePtr = G1;
                    framePtr++;
                    *framePtr = B1;
                    framePtr++;
                    *framePtr = R2;
                    framePtr++;
                    *framePtr = G2;
                    framePtr++;
                    *framePtr = B2;
                    framePtr++;
/*
                    *framePtr = Y0;
                    framePtr++;
                    *framePtr = Y1;
                    framePtr++;
                    */
                }

            }




            //and now we add this to the tail of the used buffers
            //this is in a critical section
            pthread_mutex_lock(&m_mutex);
            m_pUsedBuffers.push(MsgPtr);
            //fprintf(stderr,"Pushed a frame. Currently %d used buffers and %d free buffers.\n",m_pUsedBuffers.size(),m_pFreeBuffers.size());
            pthread_mutex_unlock(&m_mutex);



            count++;
            /*
            if (count > 500) {
                fprintf(stderr, "Frame rate is %.2f\n", count / Toc(ttime));
                exit(0);
            }*/
        }
    }

    return S_OK;
}

HRESULT CaptureDelegate::VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode *, BMDDetectedVideoInputFormatFlags) {
    return S_OK;
}
