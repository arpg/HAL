
#include "Dvi2PciDriver.h"

#ifndef V2U_COUNT
#  define V2U_COUNT(array) (sizeof(array)/sizeof((array)[0]))
#endif /* V2U_COUNT */

///////////////////////////////////////////////////////////////////////////////
Dvi2PciDriver::Dvi2PciDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
Dvi2PciDriver::~Dvi2PciDriver()
{
    FrmGrab_Close(m_pFrameGrabber);
    FrmGrabNet_Deinit();
}

///////////////////////////////////////////////////////////////////////////////
bool Dvi2PciDriver::Init()
{
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    std::string     sRes        = m_pPropertyMap->GetProperty( "Resolution", "HD" );

    if( sRes == "HD" ) {
        m_nImageWidth = 1280;
        m_nImageHeight = 720;
    }

    if( sRes == "FHD" ) {
        m_nImageWidth = 1920;
        m_nImageHeight = 1084;
    }

    // always try to get max FPS
    m_dFps = 60.0;

    /* Initialize frmgrab library */
    FrmGrabNet_Init();

    m_pFrameGrabber = OpenGrabber(NULL, NULL);

    if (m_pFrameGrabber == NULL) {
        printf("Failed to open FrameGrabber!\n");
        FrmGrab_Close(m_pFrameGrabber);
        FrmGrabNet_Deinit();
    }

    //const char* pn = FrmGrab_GetProductName(m_pFrameGrabber);
    //V2U_UINT32 caps = FrmGrab_GetCaps(m_pFrameGrabber);

    V2U_VideoMode vm;
    /* Detect video mode */
    if( FrmGrab_DetectVideoMode( m_pFrameGrabber , &vm ) == false ) {
        printf("No signal detected!\n");
        FrmGrab_Close(m_pFrameGrabber);
        FrmGrabNet_Deinit();
    }

    if( vm.width == 0 || vm.height == 0) {
        printf("No signal detected (video dimensions are 0)!\n");
        FrmGrab_Close(m_pFrameGrabber);
        FrmGrabNet_Deinit();
    }


    if( DetectVideoMode(m_pFrameGrabber, &vm, true) == -1 ) {
        return false;
    }

    /* Set up streaming (only necessary for network grabbers) */
    if( FrmGrab_SetMaxFps( m_pFrameGrabber, m_dFps ) == 0 ) {
        printf("Could not set FPS.\n");
    }

    FrmGrab_Start(m_pFrameGrabber);

    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool Dvi2PciDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{
    // allocate images if necessary
    if( vImages.size() != 1 ){
        vImages.resize( 1 );
    }

    V2U_GrabFrame2* frame = FrmGrab_Frame( m_pFrameGrabber, V2U_GRABFRAME_FORMAT_BGR24, NULL );

    if (!frame || frame->imagelen <= 0)  {
        printf("VGA2USB capture error. Skipping frame...\n");
        return false;
    }
    //printf("Frame captured. Total %d bytes.\n", frame->imagelen);

    vImages[0].Image = cv::Mat( m_nImageHeight, m_nImageWidth, CV_8UC3 );

    memcpy( vImages[0].Image.data, frame->pixbuf, frame->imagelen );

    /* Release the frame */
    FrmGrab_Release(m_pFrameGrabber, frame);

    return true;
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////
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

///////////////////////////////////////////////////////////////////////////////
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
