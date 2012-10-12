/*
 * File:   InitCam.h
 * Author: Juan Falquez
 *
 * Created on August 14, 2012, 11:54 AM
 */

#ifndef INIT_CAM_H
#define INIT_CAM_H

#include <RPG/Utils/GetPot>
#include <RPG/Devices/Camera/CameraDevice.h>

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
"\n"
"General Options:    -lcmod <left camera model xml file>\n"
"                    -rcmod <right camera model xml file>\n"
"                    -sdir  <source directory for images and camera model files [default '.']>\n"
"\n"
"Example:\n"
"./  -idev FileReader  -lcmod lcmod.xml  -rcmod rcmod.xml  -lfile \"left.*pgm\"  -rfile \"right.*pgm\"\n\n";


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void InitCam(
        CameraDevice&       Cam,
        GetPot&             clArgs
        )
{
    if( clArgs.search( 3, "--help", "-help", "-h" ) ) {
        std::cout << USAGE << std::endl;
        exit( 0 );
    }


    // get all params
    std::string sDeviceDriver     = clArgs.follow( "Bumblebee2", 1, "-idev" );
    std::string sLeftCameraModel  = clArgs.follow( "lcmod.xml", 1, "-lcmod" );
    std::string sRightCameraModel = clArgs.follow( "rcmod.xml", 1, "-rcmod" );
    std::string sLeftFileRegex    = clArgs.follow( "left.*pgm", 1, "-lfile" );
    std::string sRightFileRegex   = clArgs.follow( "right.*pgm", 1, "-rfile" );
    std::string sSourceDir        = clArgs.follow( ".", 1, "-sdir"  );

    //----------------------------------------------- BUMBLEBEE
    if( sDeviceDriver == "Bumblebee2" ) {
        if( sLeftCameraModel.empty() || sRightCameraModel.empty() ) {
            std::cerr << "One or more camera model files is missing!\n" << std::endl;
            std::cerr << USAGE;
            exit (0);
        }
        Cam.SetProperty("DataSourceDir", sSourceDir);
        Cam.SetProperty("CamModel-L",    sLeftCameraModel );
        Cam.SetProperty("CamModel-R",    sRightCameraModel );
    }

    //----------------------------------------------- FILEREADER
    if( sDeviceDriver == "FileReader" ) {
        if( sLeftCameraModel.empty() || sRightCameraModel.empty() ) {
            std::cerr << "One or more camera model files is missing!\n" << std::endl;
            std::cerr << USAGE;
            exit (0);
        }
        if( sLeftFileRegex.empty() || sRightFileRegex.empty() ) {
            std::cerr << "One or more file names is missing!\n" << std::endl;
            std::cerr << USAGE;
            exit(0);
        }
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

        Cam.SetProperty( "GetRGB", bGetRGB );
        Cam.SetProperty( "GetDepth", bGetDepth );
        Cam.SetProperty( "GetIr", bGetIr );
    }


    // init driver
    if( !Cam.InitDriver( sDeviceDriver ) ) {
            std::cerr << "Invalid input device." << std::endl;
            exit(0);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void InitCam(
        CameraDevice&       Cam,
        int                 argc,
        char**              argv
        )
{
    GetPot cl( argc, argv );
    InitCam( Cam, cl);
}


} /* namespace */

#endif   /* INIT_CAM_H */
