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

#include <boost/lexical_cast.hpp>

namespace rpg {

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    const char USAGE[] =
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
        "\n"
        "Example:\n"
        "./  -idev FileReader  -lcmod lcmod.xml  -rcmod rcmod.xml  -lfile \"left.*pgm\"  -rfile \"right.*pgm\"\n\n";


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool InitCam(
            CameraDevice&       Cam,
            GetPot&             clArgs
            )
    {
        if( clArgs.search( 3, "--help", "-help", "-h" ) ) {
            std::cout << USAGE << std::endl;
            return false;
        }


        // get general params
        std::string     sDeviceDriver       = clArgs.follow( "", 1, "-idev" );
        std::string     sLeftCameraModel    = clArgs.follow( "lcmod.xml", 1, "-lcmod" );
        std::string     sRightCameraModel   = clArgs.follow( "rcmod.xml", 1, "-rcmod" );
        std::string     sLeftFileRegex      = clArgs.follow( "left.*pgm", 1, "-lfile" );
        std::string     sRightFileRegex     = clArgs.follow( "right.*pgm", 1, "-rfile" );
        std::string     sSourceDir          = clArgs.follow( ".", 1, "-sdir"  );
        unsigned int    nFPS                = clArgs.follow( 30, 1, "-fps"  );
        std::string     sResolution         = clArgs.follow( "VGA", 1, "-res"  ); // follow format of XGA, SVGA, VGA, QVGA, QQVGA, etc.

        //----------------------------------------------- BUMBLEBEE
        if( sDeviceDriver == "Bumblebee2" ) {
            if( sLeftCameraModel.empty() || sRightCameraModel.empty() ) {
                std::cerr << "One or more camera model files is missing!\n" << std::endl;
                std::cerr << USAGE;
                return false;
            }
            Cam.SetProperty("DataSourceDir", sSourceDir);
            Cam.SetProperty("CamModel-L",    sLeftCameraModel );
            Cam.SetProperty("CamModel-R",    sRightCameraModel );
        }

        //----------------------------------------------- FILEREADER
        if( sDeviceDriver == "FileReader") {
            Cam.SetProperty( "StartFrame",    clArgs.follow( 0, 1, "-sf" ));
            Cam.SetProperty( "Loop",          clArgs.search( "-loop" ));

            if( sLeftCameraModel.empty() || sRightCameraModel.empty() ) {
                std::cerr << "One or more camera model files is missing!\n" << std::endl;
                std::cerr << USAGE;
                return false;
            }
            if( sLeftFileRegex.empty() || sRightFileRegex.empty() ) {
                std::cerr << "One or more file names is missing!\n" << std::endl;
                std::cerr << USAGE;
                return false;
            }
            Cam.SetProperty("DataSourceDir", sSourceDir );
            Cam.SetProperty("Channel-0",     sLeftFileRegex );
            Cam.SetProperty("Channel-1",     sRightFileRegex );
            Cam.SetProperty("CamModel-L",    sLeftCameraModel );
            Cam.SetProperty("CamModel-R",    sRightCameraModel );
            Cam.SetProperty("NumChannels",   2 );
        }

        //----------------------------------------------- TOYOTAREADER
        if( sDeviceDriver == "ToyotaReader") {
            Cam.SetProperty( "StartFrame",    clArgs.follow( 0, 1, "-sf" ));
            Cam.SetProperty( "Loop",          clArgs.search( "-loop" ));
            Cam.SetProperty("DataSourceDir", sSourceDir );
            Cam.SetProperty("Channel-0",     sLeftFileRegex );
            Cam.SetProperty("Channel-1",     sRightFileRegex );
            Cam.SetProperty("NumChannels",   2 );
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
        }

        //----------------------------------------------- NODECAM
        if( sDeviceDriver == "NodeCam" ) {
            int numNodes = 0;
            std::stringstream ss;
            while(true) {
                //std::string arg = boost::lexical_cast<std::string>(numNodes);
                ss << numNodes;
                std::string arg = ss.str();
                if(!clArgs.search( ("-n"+arg).c_str() ) ) break;
                Cam.SetProperty("Node-" + arg, clArgs.follow("", ("-n"+arg).c_str() ) );
                numNodes++;
                ss.str("");
            }
            Cam.SetProperty("NumNodes", numNodes);
        }


        // init driver
        if( !Cam.InitDriver( sDeviceDriver ) ) {
            std::cerr << "Invalid input device." << std::endl;
            std::cerr << USAGE;
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
        GetPot cl( argc, argv );
        return InitCam( Cam, cl);
    }


} /* namespace */

#endif   /* INIT_CAM_H */
