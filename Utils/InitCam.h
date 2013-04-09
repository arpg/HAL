/*
 * File:   InitCam.h
 * Author: Juan Falquez
 *
 * Created on August 14, 2012, 11:54 AM
 */

#ifndef _INIT_CAM_H_
#define _INIT_CAM_H_

#include <sstream>
#include <RPG/Utils/GetPot>
#include <RPG/Devices/Camera/CameraDevice.h>

namespace rpg {

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    const char CAM_USAGE[] =
        "FLAGS:     -idev <input> <options>\n"
        "\n"
        "where input device can be: FileReader Bumblebee2 etc\n"
        "\n"
        "Input Specific Options:\n"
        "   FileReader:      -lfile <regular expression for left image channel>\n"
        "                    -rfile <regular expression for right image channel>\n"
        "                    -sf    <start frame [default 0]>\n"
        "                    -loop  If the driver should restart once images are consumed.\n"
        "\n"
        "   Kinect:          -no-rgb         Do not capture RGB image.\n"
        "                    -no-depth       Do not capture depth image.\n"
        "                    -align-depth    Align depth map to RGB image.\n"
        "                    -with-ir        Capture IR image.\n"
        "\n"
        "General Options:    -lcmod <left camera model xml file>\n"
        "                    -rcmod <right camera model xml file>\n"
        "                    -sdir  <source directory for images and camera model files [default '.']>\n"
        "                    -res   <resolution: FHD, HD, QVGA, etc. [default 'VGA']>\n"
        "                    -fps   <frames per second [default '30']>\n"
        "\n"
        "Example:\n"
        "./Exec  -idev FileReader  -lcmod lcmod.xml  -rcmod rcmod.xml  -lfile \"left.*pgm\"  -rfile \"right.*pgm\"\n\n";


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool ParseCamArgs(
            CameraDevice&       Cam,
            GetPot&             clArgs
            )
    {
        if( clArgs.search( 3, "--help", "-help", "-h" ) ) {
            std::cout << CAM_USAGE << std::endl;
            return false;
        }


        // get general params
        std::string     sDeviceDriver       = clArgs.follow( "", "-idev" );
        std::string     sLeftCameraModel    = clArgs.follow( "lcmod.xml", "-lcmod" );
        std::string     sRightCameraModel   = clArgs.follow( "rcmod.xml", "-rcmod" );
        std::string     sLeftFileRegex      = clArgs.follow( "left.*pgm", "-lfile" );
        std::string     sRightFileRegex     = clArgs.follow( "right.*pgm", "-rfile" );
        std::string     sSourceDir          = clArgs.follow( ".", "-sdir"  );
        unsigned int    nFPS                = clArgs.follow( 30, "-fps"  );
        std::string     sResolution         = clArgs.follow( "VGA", "-res"  ); // follow format of FHD, HD, XGA, SVGA, VGA, QVGA, QQVGA, etc.
        bool            bForceGreyscale     = clArgs.search( "-forcegreyscale" );

        //----------------------------------------------- BUMBLEBEE
        if( sDeviceDriver == "Bumblebee2" ) {
            if( sLeftCameraModel.empty() || sRightCameraModel.empty() ) {
                std::cerr << "One or more camera model files is missing!\n" << std::endl;
                std::cerr << CAM_USAGE;
                return false;
            }
            Cam.SetProperty("DataSourceDir", sSourceDir);
            Cam.SetProperty("CamModel-L",    sLeftCameraModel );
            Cam.SetProperty("CamModel-R",    sRightCameraModel );
            Cam.SetProperty("ForceGreyscale",bForceGreyscale );
            Cam.SetProperty("Rectify",       clArgs.search( "-rectify" ) );
        }

        //----------------------------------------------- FILEREADER
        if( sDeviceDriver == "FileReader") {
            Cam.SetProperty( "StartFrame",    clArgs.follow( 0, "-sf" ));
            Cam.SetProperty( "Loop",          clArgs.search( "-loop" ));

            if( sLeftCameraModel.empty() || sRightCameraModel.empty() ) {
                std::cerr << "One or more camera model files is missing!\n" << std::endl;
                std::cerr << CAM_USAGE;
                return false;
            }
            if( sLeftFileRegex.empty() || sRightFileRegex.empty() ) {
                std::cerr << "One or more file names is missing!\n" << std::endl;
                std::cerr << CAM_USAGE;
                return false;
            }
            std::string     sSourceDirImage = clArgs.follow( "", "-sdir_image"  );

            if( sSourceDirImage.empty() ) {
                Cam.SetProperty("DataSourceDir", sSourceDir );
            } else {
                Cam.SetProperty("DataSourceDir", sSourceDirImage );
            }
            Cam.SetProperty("Channel-0",     sLeftFileRegex );
            Cam.SetProperty("Channel-1",     sRightFileRegex );
            Cam.SetProperty("CamModel-L",    sLeftCameraModel );
            Cam.SetProperty("CamModel-R",    sRightCameraModel );
            Cam.SetProperty("NumChannels",   clArgs.follow(2,"-numchannels") );
            Cam.SetProperty("ForceGreyscale",bForceGreyscale );            

            /// ADDITIONAL NON-CL ARGUMENTS
            // "TimeKeeper": Name of variable that holds timestamps [default 'SystemTime'].
        }

        //----------------------------------------------- TOYOTAREADER
        if( sDeviceDriver == "ToyotaReader") {
            Cam.SetProperty( "StartFrame",    clArgs.follow( 0, "-sf" ));
            Cam.SetProperty( "Loop",          clArgs.search( "-loop" ));
            Cam.SetProperty("DataSourceDir", sSourceDir );
            Cam.SetProperty("Channel-0",     sLeftFileRegex );
            Cam.SetProperty("Channel-1",     sRightFileRegex );
            Cam.SetProperty("CamModel-L",    sLeftCameraModel );
            Cam.SetProperty("CamModel-R",    sRightCameraModel );
            Cam.SetProperty("NumChannels",   2 );
            Cam.SetProperty("ForceGreyscale",bForceGreyscale );
        }

        //----------------------------------------------- KINECT
        if( sDeviceDriver == "Kinect" ) {
            bool        bGetDepth         = !clArgs.search( "-no-depth" );
            bool        bGetRGB           = !clArgs.search( "-no-rgb" );
            bool        bGetIr            = clArgs.search( "-with-ir" );
            bool        bAlignDepth       = clArgs.search( "-align-depth" );

            Cam.SetProperty( "GetRGB", bGetRGB );
            Cam.SetProperty( "GetDepth", bGetDepth );
            Cam.SetProperty( "GetIr", bGetIr );
            Cam.SetProperty( "AlignDepth", bAlignDepth );
            Cam.SetProperty( "FPS", nFPS );
            Cam.SetProperty( "Resolution", sResolution );
            Cam.SetProperty( "ForceGreyscale",bForceGreyscale );            
        }

        //----------------------------------------------- DVI2PCI
        if( sDeviceDriver == "Dvi2Pci" ) {
            Cam.SetProperty( "Resolution", sResolution );
        }

        //----------------------------------------------- NODECAM
        if( sDeviceDriver == "NodeCam" ) {
            int numNodes = 0;
            std::stringstream ss;
            while(true) {
                ss << numNodes;
                std::string arg = ss.str();
                if(!clArgs.search( ("-n"+arg).c_str() ) ) break;
                Cam.SetProperty("Node-" + arg, clArgs.follow("", ("-n"+arg).c_str() ) );
                numNodes++;
                ss.str("");
            }
            Cam.SetProperty("NumNodes", numNodes);
        }
        
        //----------------------------------------------- OpenCV Webcam
        if( sDeviceDriver == "Webcam" ) {
            Cam.SetProperty("ForceGreyscale",bForceGreyscale );
            Cam.SetProperty("CamId", clArgs.follow( 0,"-camid" ) );
        }

        return true;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool ParseCamArgs(
            CameraDevice&       Cam,
            int                 argc,
            char**              argv
            )
    {
        GetPot clArgs( argc, argv );
        return ParseCamArgs( Cam, clArgs );
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool InitCam(
            CameraDevice&       Cam,
            GetPot&             clArgs
            )
    {
        if( ParseCamArgs( Cam, clArgs ) == false ) {
            return false;
        }

        std::string sDeviceDriver = clArgs.follow( "", 1, "-idev" );

        // init driver
        if( !Cam.InitDriver( sDeviceDriver ) ) {
            std::cerr << "Invalid input device." << std::endl;
            std::cerr << CAM_USAGE;
            return false;
        }

        return true;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool InitCam(
            CameraDevice&       Cam,
            int                 argc,
            char**              argv
            )
    {
        GetPot clArgs( argc, argv );
        return InitCam( Cam, clArgs );
    }


} /* namespace */

#endif   /* INIT_CAM_H */
