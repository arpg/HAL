#include <iostream>
#include <string>
#include <RPG/Utils/GetPot> // for command line parsing
#include <RPG/Devices/Camera/CameraDevice.h> // camera device

const char USAGE[] =
"Usage:     TestCameraDevice <options>\n"
"\n"
"Options:   -lcmod <left camera model xml file>\n"
"           -rcmod <right camera model xml file>\n"
"           -lfile <regular expression for left image channel>\n"
"           -rfile <regular expression for right image channel>\n"
"\n"
"Example:   TestCameraDevice -lcmod lcmod.xml -rcmod rcmod.xm -lfile \"left.*pgm\"  -rfile \"right.*pgm\"\n\n";


using namespace std;

/*
   get regex from cmd line, find files
 */

int main( int argc, char** argv )
{
    if( argc != 9 ){
        cout << USAGE;
        return -1;
    }

    GetPot cl(argc,argv);
    string sLeftCameraModel  = cl.follow( "", 1, "-lcmod" );
    string sRightCameraModel = cl.follow( "", 1, "-rcmod" );
    string sLeftImageFile    = cl.follow( "", 1, "-lfile" );
    string sRightImageFile   = cl.follow( "", 1, "-rfile" );


    CameraDevice cam;

    cam.SetProperty("Channel-0", sLeftImageFile );
    cam.SetProperty("Channel-1", sRightImageFile );
    cam.SetProperty("NumChannels", 2 );
    cam.SetProperty("Rate", 0.0 );

//    cam.SetProperty("Mode", "DC1394_VIDEO_MODE_160x120_YUV444" );
//    cam.SetProperty("Resolution", "" );

    cam.InitDriver( "FileReader" );
//    cam.InitDriver( "Bumblebee2" );

    cout << cam.GetProperty("Channel-0")   << endl;
    cout << cam.GetProperty("Channel-1")   << endl;
    cout << cam.GetProperty("NumChannels") << endl;
    cout << cam.GetProperty("Rate")        << endl;
    cout << cam.GetProperty("Width")        << endl;
    cout << cam.GetProperty("Height")        << endl;


    std::vector<Image> vImages;
    while( 1 ){
        cam.Capture( vImages );
        printf( "read %d images\n", (int)vImages.size() );
        sleep(1);
    }

    return 0;
}

