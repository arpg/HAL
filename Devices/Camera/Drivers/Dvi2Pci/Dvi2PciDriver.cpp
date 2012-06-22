
#include "Dvi2PciDriver.h"

#include "boost/thread.hpp"
#include "opencv/cv.h"	// for Mat structure

#ifndef V2U_COUNT
#  define V2U_COUNT(array) (sizeof(array)/sizeof((array)[0]))
#endif /* V2U_COUNT */

static int g_count = 0;

///////////////////////////////////////////////////////////////////////////////
Dvi2PciDriver::Dvi2PciDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
Dvi2PciDriver::~Dvi2PciDriver()
{

}

///////////////////////////////////////////////////////////////////////////////
bool Dvi2PciDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{
    
    // if there are any matrices, delete them
    vImages.clear();

    unsigned char *buffer, *controlBuffer;
    int length, controlLength;
    
    
    
    bool gotImage = false;
        
    //block while we don't have an image
    while(gotImage == false)
    {
        pthread_mutex_lock(&m_mutex);
        if(m_pUsedBuffers.empty()) {           
            length = 0;
            controlLength = 0;
            buffer = NULL;
        } else {
            DviImageBufferStruct *pStr = m_pUsedBuffers.front();
            m_pUsedBuffers.pop();
            buffer = pStr->m_pImageBuffer;
            controlBuffer = pStr->m_pControlBuffer;

            //add this to the free buffers
            m_pFreeBuffers.push(pStr);
            //fprintf(stderr,"Popped a frame. Currently %d used buffers and %d free buffers.\n",m_pUsedBuffers.size(),m_pFreeBuffers.size());
            length = m_nNumImages * m_nImageWidth * m_nImageHeight;
            controlLength = 160;
            //std::memcpy(buffer,ptr,lengthOut);
            

            gotImage = true;
        }
        pthread_mutex_unlock(&m_mutex);
    }
    
    // allocate images if necessary
    if( vImages.size() != m_nNumImages ){
        vImages.resize( m_nNumImages );
    }
    
    g_count++;
    
    if(g_count % 2 == 0) {
        //buffer += m_nImageHeight*m_nImageWidth;
    }
    
    // now copy the images from the delegate
    for ( size_t ii = 0; ii < m_nNumImages; ii++) {
        
        vImages[ii].Image = cv::Mat(m_nImageHeight,m_nImageWidth, CV_8UC1, buffer);
        //advance the pointer forward
        //buffer += 1280*720*3/2;
        buffer += m_nImageWidth*m_nImageHeight;
    }
    return true;
}


/**
 * Prints video mode flags in human-readable form
 */
void Dvi2PciDriver::DumpVgaModeFlags(V2U_UINT32 flags, V2U_UINT32 mask)
{
    static struct v2u_vgamode_flags {
        V2U_UINT32 flag;
        const char* on;
        const char* off;
    } vgamode_flags [] = {
        {VIDEOMODE_TYPE_VALID, NULL, "INVALID"},
        {VIDEOMODE_TYPE_ENABLED, "ENABLED", "DISABLED"},
        {VIDEOMODE_TYPE_SUPPORTED, "SUPPORTED", NULL},
        {VIDEOMODE_TYPE_DIGITAL, "DIGITAL", NULL},
        {VIDEOMODE_TYPE_DUALLINK, "DUALLINK", NULL},
        {VIDEOMODE_TYPE_INTERLACED, "INTERLACED", NULL},
        {VIDEOMODE_TYPE_HSYNCPOSITIVE, "HSYNCPOSITIVE", NULL},
        {VIDEOMODE_TYPE_VSYNCPOSITIVE, "VSYNCPOSITIVE", NULL}
    };

    int flags_printed = 0;
    for (unsigned int k=0; k<V2U_COUNT(vgamode_flags); k++) {
        if (vgamode_flags[k].flag & mask) {
            const char* name = (flags & vgamode_flags[k].flag) ?
                vgamode_flags[k].on : vgamode_flags[k].off;
            if (name) {
                printf(flags_printed ? " + %s" : "%s", name);
                flags_printed |= vgamode_flags[k].flag;
            }
        }
        flags &= ~vgamode_flags[k].flag;
    }

    if (flags) {
        printf(flags_printed ? " + 0x%02X" : "0x%02X", flags);
    }
}

/**
 * Detects and prints video mode
 */
int Dvi2PciDriver::DetectVideoMode(FrmGrabber* fg, V2U_VideoMode* vm, V2U_BOOL details)
{
    if (FrmGrab_DetectVideoMode(fg, vm)) {
        if (vm->width || vm->height) {
            printf("detected %dx%d (%d.%d Hz)\n",
                        vm->width, vm->height, 
                        (vm->vfreq+50)/1000,((vm->vfreq+50)%1000)/100);

            /* Print video mode details */
            if (details) {
                V2U_Property p;
                p.key = V2UKey_CurrentVGAMode;
                if (FrmGrab_GetProperty(fg, &p)) {
                    const V2UVideoModeDescr* mode = &p.value.vesa_mode;
                    printf("  VerFreq:         %u\n", mode->VerFrequency);
                    printf("  HorAddrTime:     %hu\n", mode->HorAddrTime);
                    printf("  HorFrontPorch:   %hu\n", mode->HorFrontPorch);
                    printf("  HorSyncTime:     %hu\n", mode->HorSyncTime);
                    printf("  HorBackPorch:    %hu\n", mode->HorBackPorch);
                    printf("  VerAddrTime:     %hu\n", mode->VerAddrTime);
                    printf("  VerFrontPorch:   %hu\n", mode->VerFrontPorch);
                    printf("  VerSyncTime:     %hu\n", mode->VerSyncTime);
                    printf("  VerBackPorch:    %hu\n", mode->VerBackPorch);
                    printf("  Flags:           0x%02x", mode->Type);
                    if (mode->Type) {
                        printf(" (");
                        /* Some versions of the driver don't set VALID and
                        * ENABLED flags for digital modes - ignore them */
                        DumpVgaModeFlags(mode->Type,
                            ~(VIDEOMODE_TYPE_VALID | VIDEOMODE_TYPE_ENABLED));
                        printf(")");
                    }
                    printf("\n");
                } else {
                    printf("ERROR: failed to get video mode details\n");
                }
            }

            return 0;
        } else {
            printf("ERROR: no signal detected\n");
            return -1;
        }
    } else {
        printf("ERROR: failed to detect video mode\n");
        return -1;
    }
}

/**
 * Opens the frame grabber with either the specified serial number or
 * the network address.
 */
FrmGrabber* Dvi2PciDriver::OpenGrabber(const char* sn, const char* addr)
{
    FrmGrabber* fg;
    if (sn) {

        /* First attempt to open a local grabber, then try the network */
        fg = FrmGrabLocal_OpenSN(sn);
        if (!fg) fg = FrmGrabNet_OpenSN(sn);
        if (!fg) {
            printf("Can't find a frame grabber with s/n %s\n",sn);
        }
    } else if (addr) {

        /* Connect to the network grabber */
        fg = FrmGrabNet_OpenLocation(addr);
        if (!fg) {
            printf("Can't find a frame grabber at %s\n",addr);
        }
    } else {

        /* Try to open something - first local, then network */
        fg = FrmGrabLocal_Open();
        if (!fg) fg = FrmGrabNet_Open();
        if (!fg) {
            printf("No Epiphan frame grabber found\n");
        }
    }
    
    if(fg != NULL) {
        printf("Found capture card with serial number %s\n", FrmGrab_GetSN(fg));
    }

    return fg;
}

    
void Dvi2PciDriver::CaptureFuncHandler(Dvi2PciDriver *pUserData)
{
    pUserData->CaptureFunc();
}

bool Dvi2PciDriver::CaptureFunc()
{
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    m_nNumImages = m_pPropertyMap->GetProperty<int>( "NumImages", 1 );
    m_nImageWidth = m_pPropertyMap->GetProperty<int>( "ImageWidth", 640 );
    m_nImageHeight = m_pPropertyMap->GetProperty<int>( "ImageHeight", 480 );
    m_dFps = m_pPropertyMap->GetProperty<double>( "FPS", 60 );
    m_nBufferCount = m_pPropertyMap->GetProperty<int>("BufferCount",5);
    
    //fill the used and free buffers
    for (unsigned int ii = 0; ii < m_nBufferCount; ii++) {
        DviImageBufferStruct *pStr = new DviImageBufferStruct();
//        pStr->m_pControlBuffer = new unsigned char[1280];
        pStr->m_pImageBuffer = new unsigned char[1920 * 1080 * 4];
        m_pFreeBuffers.push(pStr);
    }

    /* Initialize frmgrab library */
    FrmGrabNet_Init();

    m_pFrameGrabber = OpenGrabber(NULL, NULL);
    
    if (m_pFrameGrabber == NULL) {
        printf("Failed to open FrameGrabber\n");
        FrmGrab_Close(m_pFrameGrabber);
        FrmGrabNet_Deinit();
        return false;
    }
    
    const char* pn = FrmGrab_GetProductName(m_pFrameGrabber);
    V2U_UINT32 caps = FrmGrab_GetCaps(m_pFrameGrabber);

    V2U_VideoMode vm;
    /* Detect video mode */
    if (FrmGrab_DetectVideoMode(m_pFrameGrabber,&vm) == false ){
        printf("No signal detected\n");
        FrmGrab_Close(m_pFrameGrabber);
        FrmGrabNet_Deinit();
        return false;
    }
    
    if( vm.width == 0 || vm.height == 0) {
        printf("No signal detected (video dimensions are 0)\n");
        FrmGrab_Close(m_pFrameGrabber);
        FrmGrabNet_Deinit();
        return false;
    }
    
    
    int result = DetectVideoMode(m_pFrameGrabber, &vm, true);
    
    printf("Starting capture thread...\n");
    /* Set up streaming (onle necessary for network grabbers) */
    FrmGrab_SetMaxFps(m_pFrameGrabber, 60.0);
    FrmGrab_Start(m_pFrameGrabber);
    
    V2U_GrabFrame2* frame = NULL;
    V2URect CropCfg;
    CropCfg.height = vm.height;
    CropCfg.width = vm.width;
    CropCfg.x = 0;
    CropCfg.y = 0;
    
    printf("Capture started...\n");
    while(1)
    {
        frame = FrmGrab_Frame(m_pFrameGrabber, V2U_GRABFRAME_FORMAT_BGR24, &CropCfg);
        if (!frame || frame->imagelen <= 0)  {
            printf("VGA2USB capture error. Skipping frame...\n");
            continue;
            //printf("VGA2USB capture error. Stopping recording.\n");
            //break;
        }
        //printf("Frame captured. Total %d bytes.\n", frame->imagelen);
        
        DviImageBufferStruct *BuffPtr;
        //if there are no free buffers it means whoever is reading this
        //is not reading fast enough so we just have to rewrite over the
        //first item
        pthread_mutex_lock(&m_mutex);
        if (m_pFreeBuffers.empty()) {
            std::cout << "Ooops.. using non-empty buffer!" << std::endl;
            BuffPtr = m_pUsedBuffers.front();
        } else {
            //otherwise take one of the free buffers
            BuffPtr = m_pFreeBuffers.front();
            m_pFreeBuffers.pop();
        }
        pthread_mutex_unlock(&m_mutex);
        
        memcpy(BuffPtr->m_pImageBuffer, frame->pixbuf, frame->imagelen);
        
        //and now we add this to the tail of the used buffers
        //this is in a critical section
        pthread_mutex_lock(&m_mutex);
        m_pUsedBuffers.push(BuffPtr);
        //fprintf(stderr,"Pushed a frame. Currently %d used buffers and %d free buffers.\n",m_pUsedBuffers.size(),m_pFreeBuffers.size());
        pthread_mutex_unlock(&m_mutex);
            
        /* Release the frame */
        FrmGrab_Release(m_pFrameGrabber, frame);
        frame = NULL;
    }
    
    FrmGrab_Close(m_pFrameGrabber);

    /* Deinitialize frmgrab library */
    FrmGrabNet_Deinit();
}


///////////////////////////////////////////////////////////////////////////////
bool Dvi2PciDriver::Init()
{
    pthread_mutex_init(&m_mutex, NULL);
    boost::thread captureThread(CaptureFuncHandler,this);
    return true;
}
