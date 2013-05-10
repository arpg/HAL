/*
   \file CameraDevice.h

   Abstract device that represents a generic camera.

 */

#ifndef _CAMERA_DEVICE_H_
#define _CAMERA_DEVICE_H_

#include <HAL/Utils/GetPot>
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Camera/Drivers/CameraDriverRegistery.h>
#include <PbMsgs/Image.h>

namespace hal {

// Driver Creation Factory
extern CameraDriver* CreateCameraDriver( const std::string& sDriverName );

///////////////////////////////////////////////////////////////////////////////
// Generic camera device
class CameraDevice : public PropertyMap
{
    public:
        ///////////////////////////////////////////////////////////////
        CameraDevice()
            : m_pDriver(0)
        {
        }

        ///////////////////////////////////////////////////////////////
        ~CameraDevice()
        {
            _DeinitDriver();
        }

        ///////////////////////////////////////////////////////////////
        bool InitDriver(
                int                 argc,
                char**              argv,
                const std::string&  sDriver = ""
            )

        {
            _DeinitDriver();

            GetPot clArgs( argc, argv );
            _ParseCamArgs( clArgs );
            m_sDriverType = clArgs.follow( "", "-idev" );
            if( sDriver.empty() == false ) {
                m_sDriverType = sDriver;
            }
            m_pDriver = CreateCameraDriver( m_sDriverType );
            if( m_pDriver ) {
                m_pDriver->SetPropertyMap( this );
                const bool success = m_pDriver->Init();
                if( success == false ) {
                    _DeinitDriver();
                    m_sDriverType.clear();
                }
                return success;
            }
            return false;
        }

        ///////////////////////////////////////////////////////////////
        bool DriverInfo()
        {
            if( IsInitialized() ) {
                m_pDriver->PrintInfo();
                return true;
            }
            return false;
        }

        ///////////////////////////////////////////////////////////////
        bool IsInitialized()
        {
            return m_pDriver;
        }

        ///////////////////////////////////////////////////////////////
        bool Capture( pb::CameraMsg& Images )
        {
            if( m_pDriver ){
                Images.Clear();
                return m_pDriver->Capture( Images );
            }
            std::cerr << "ERROR: no driver initialized!\n";
            return false;
        }

        ///////////////////////////////////////////////////////////////
        bool Capture( pb::ImageArray& Images )
        {
            const bool ret = Capture( Images.ref() );
            Images.SelfUpdate();
            return ret;
        }



private:

        ///////////////////////////////////////////////////////////////
        void _ParseCamArgs(
                GetPot&       clArgs
                )
        {
            // general help
            if( clArgs.search( 3, "--help", "-help", "-h" ) ) {
                std::cout <<
                "\nRPG Driver Init Help\n"
                "--------------------------------\n"
                "To obtain more information on how to initialize each driver, please use:"
                "\n"
                "   -info  <driver name>\n"
                << std::endl;
                ListCameraDrivers();
                exit(0);
            }

            // driver info
            std::string     sDriverInfo    = clArgs.follow( "", "-info" );
            if( sDriverInfo.empty() == false ) {
                _DriverInfo( sDriverInfo );
                exit(0);
            }


            //--- SOURCE DIR
            SetProperty( "DataSourceDir",   clArgs.follow( ".", "-sdir"  ) );


            //--- CAMERA MODELS
            // lcmod, rcmod
            std::string     sLeftCameraModel    = clArgs.follow( "lcmod.xml", "-lcmod" );
            std::string     sRightCameraModel   = clArgs.follow( "rcmod.xml", "-rcmod" );
            SetProperty("CamModel-L",    sLeftCameraModel );
            SetProperty("CamModel-R",    sRightCameraModel );
            SetProperty("CamModel-0",    sLeftCameraModel );
            SetProperty("CamModel-1",    sRightCameraModel );

            // cmod0, cmod1, etc.
            int numCMods = 0;
            while(true) {
                std::stringstream ss;
                ss << numCMods;
                std::string arg = ss.str();
                if(!clArgs.search( ("-cmod"+arg).c_str() ) ) break;
                SetProperty("CamModel-" + arg, clArgs.follow("", ("-cmod"+arg).c_str() ) );
                numCMods++;
            }


            //--- REGEX
            // lfile, rfile
            std::string     sLeftFileRegex      = clArgs.follow( "left.*pgm", "-lfile" );
            std::string     sRightFileRegex     = clArgs.follow( "right.*pgm", "-rfile" );
            SetProperty("Channel-L",    sLeftFileRegex );
            SetProperty("Channel-R",    sRightFileRegex );
            SetProperty("Channel-0",    sLeftFileRegex );
            SetProperty("Channel-1",    sRightFileRegex );

            // chan0, chan1, etc.
            int numChans = 0;
            while(true) {
                std::stringstream ss;
                ss << numChans;
                std::string arg = ss.str();
                if(!clArgs.search( ("-chan"+arg).c_str() ) ) break;
                SetProperty("Channel-" + arg, clArgs.follow("", ("-chan"+arg).c_str() ) );
                numChans++;
            }
            SetProperty( "NumChannels", numChans == 0 ? 2 : numChans );


            //--- NODES
            int numNodes = 0;
            while(true) {
                std::stringstream ss;
                ss << numNodes;
                std::string arg = ss.str();
                if(!clArgs.search( ("-node"+arg).c_str() ) ) break;
                SetProperty("Node-" + arg, clArgs.follow("", ("-node"+arg).c_str() ) );
                numNodes++;
            }
            SetProperty( "NumNodes", numNodes );


            //--- CAMERA ID
            SetProperty( "CamId",           clArgs.follow( 0, "-camid" ) );


            //--- FILE CONTROLS
            SetProperty( "StartFrame",      clArgs.follow( 0, "-sf" ));
            SetProperty( "Loop",            clArgs.search( "-loop" ));
            SetProperty( "FPS",             clArgs.follow( 30, "-fps"  ) );
            SetProperty( "BufferSize",      clArgs.follow( 35, "-buffsize"  ) );
            SetProperty( "TimeKeeper",      clArgs.follow( "SystemTime", "-timekeeper"  ) );


            //--- RESOLUTION AND FRAMERATE
            SetProperty( "Resolution",      clArgs.follow( "VGA", "-res"  ) ); // follow format of FHD, HD, XGA, SVGA, VGA, QVGA, QQVGA, etc.
            SetProperty( "FPS",             clArgs.follow( 30, "-fps"  ) );


            //--- CAMERA POST-PROCESSING
            SetProperty( "Rectify",         clArgs.search( "-rectify" ) );
            SetProperty( "ForceGreyscale",  clArgs.search( "-greyscale" ) );
        }

        ///////////////////////////////////////////////////////////////
        bool _DriverInfo( const std::string& sDriver )
        {
            CameraDriver* pDriver;
            pDriver = CreateCameraDriver( sDriver );
            if( pDriver ){
                pDriver->PrintInfo();
                delete( pDriver );
                return true;
            }
            return false;
        }

        ///////////////////////////////////////////////////////////////
        void _DeinitDriver()
        {
            if(m_pDriver) {
                delete m_pDriver;
                m_pDriver = 0;
            }
        }



private:
        // A camera device will create and initialize a particular driver
        std::string             m_sDriverType;
        CameraDriver*           m_pDriver;
};

}

#endif
