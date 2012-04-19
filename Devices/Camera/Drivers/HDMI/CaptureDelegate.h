/*
 * File:   CaptureDelegate.h
 * Author: jmf
 *
 * Created on April 10, 2012, 4:25 PM
 */

#ifndef CAPTUREDELEGATE_H
#define	CAPTUREDELEGATE_H

#include <pthread.h>
#include <queue>

#include "Blackmagic/DeckLinkAPI.h"
struct ImageBufferStruct
{
    unsigned char * m_pImageBuffer;
    unsigned char * m_pControlBuffer;
};


class CaptureDelegate : public IDeckLinkInputCallback {
public:
    CaptureDelegate(const int bufferCount, int nNumImages, int nImageWidth, int nImageHeight);
    ~CaptureDelegate();

    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID , LPVOID *) {
        return E_NOINTERFACE;
    }
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);
    virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*, BMDDetectedVideoInputFormatFlags);
    virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);
    bool GetFrame(unsigned char *&buffer, int &lengthOut, unsigned char *&controlBuffer, int &controlLengthOut);

    private:    
    unsigned int m_nBufferCount;
    unsigned long m_frameCount;
    unsigned int m_maxFrames;
    BMDTimecodeFormat m_timecodeFormat;
    ULONG m_refCount;
    pthread_mutex_t m_mutex;
    //std::vector<char*>          m_pBuffers;
    std::queue<ImageBufferStruct *> m_pUsedBuffers;
    std::queue<ImageBufferStruct *> m_pFreeBuffers;
    int m_nNumImages;
    int m_nImageHeight;
    int m_nImageWidth;
};

#endif	/* CAPTUREDELEGATE_H */

