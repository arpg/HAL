/*
 * File:   CaptureDelegate.h
 * Author: jmf
 *
 * Created on April 10, 2012, 4:25 PM
 */

#ifndef CAPTUREDELEGATE_H
#define	CAPTUREDELEGATE_H

#include <pthread.h>
#include <zmq.hpp>
#include <queue>

#include "Blackmagic/DeckLinkAPI.h"

class CaptureDelegate : public IDeckLinkInputCallback {
public:
    CaptureDelegate(const int bufferCount, int nNumImages, int nImageWidth, int nImageHeight);
    ~CaptureDelegate();

    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID *ppv) {
        return E_NOINTERFACE;
    }
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);
    virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*, BMDDetectedVideoInputFormatFlags);
    virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);
    bool GetFrame(char *&buffer, int &lengthOut);

private:
    zmq::context_t* m_pContext;
    zmq::socket_t* m_pSocket;

    unsigned int m_nBufferCount;
    unsigned long m_frameCount;
    int m_maxFrames;
    BMDTimecodeFormat m_timecodeFormat;
    ULONG m_refCount;
    pthread_mutex_t m_mutex;
    //std::vector<char*>          m_pBuffers;
    std::queue<char*> m_pUsedBuffers;
    std::queue<char*> m_pFreeBuffers;
    int m_nNumImages;
    int m_nImageHeight;
    int m_nImageWidth;
};

#endif	/* CAPTUREDELEGATE_H */

