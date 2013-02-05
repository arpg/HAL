/*
 * File:   InitIMU.h
 * Author: Juan Falquez
 *
 * Created on February 5, 2013, 4:49 PM
 */

#ifndef _INIT_IMU_H_
#define _INIT_IMU_H_

#include <sstream>
#include <RPG/Utils/GetPot>
#include <RPG/Devices/IMU/IMUDevice.h>

namespace rpg {

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    const char IMU_USAGE[] =
        "FLAGS:     -idev <input> <options>\n"
        "\n"
        "where input device can be: IMULog MicroStrain Phidgets etc\n"
        "\n"
        "Input Specific Options:\n"
        "   IMULog:          -lfile <regular expression for left image channel>\n"
        "                    -rfile <regular expression for right image channel>\n"
        "                    -sf    <start frame [default 0]>\n"
        "                    -loop  If the driver should restart once images are consumed.\n"
        "\n"
        "General Options:    -hz_gps <hz>    Capture frequency for GPS. [default 1Hz]\n"
        "                    -hz_ahrs <hz>   Capture frequency for AHRS. [default 100Hz]\n"
        "                    -gps            Capture GPS.\n"
        "                    -ahrs           Capture AHRS.\n"
        "                    -accel          Capture accelerometer.\n"
        "                    -euler          Capture euler angles.\n"
        "                    -quat           Capture quaternian.\n"
        "                    -gyro           Capture gyro.\n"
        "                    -mag            Capture magnetometer.\n"
        "                    -pps            Capture device timestamp.\n"
        "\n"
        "Example:\n"
        "./Exec  -idev IMULog  -accel -gyro -mag \n\n";


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool ParseIMUArgs(
            IMUDevice&          IMU,
            GetPot&             clArgs
            )
    {
        if( clArgs.search( 3, "--help", "-help", "-h" ) ) {
            std::cout << IMU_USAGE << std::endl;
            return false;
        }


        // get general params
        std::string     sDeviceDriver       = clArgs.follow( "", "-idev" );
        std::string     sSourceDir          = clArgs.follow( ".", "-sdir"  );
        int             nHzGPS              = clArgs.follow( 1, "-hz_gps" );
        int             nHzAHRS             = clArgs.follow( 100, "-hz_ahrs" );
        bool            bGetGPS             = clArgs.search( "-gps" );
        bool            bGetAHRS            = clArgs.search( "-ahrs" );
        bool            bGetAccel           = clArgs.search( "-accel" );
        bool            bGetEuler           = clArgs.search( "-euler" );
        bool            bGetQuat            = clArgs.search( "-quat" );
        bool            bGetGyro            = clArgs.search( "-gyro" );
        bool            bGetMag             = clArgs.search( "-mag" );
        bool            bGetPPS             = clArgs.search( "-pps" );

        //----------------------------------------------- IMULOG
        if( sDeviceDriver == "IMULog") {

            std::string     sSourceDirIMU = clArgs.follow( "", "-sdir_imu"  );

            if( sSourceDirIMU.empty() ) {
                IMU.SetProperty("DataSourceDir", sSourceDir );
            } else {
                IMU.SetProperty("DataSourceDir", sSourceDirIMU );
            }

            // ADDITIONAL NON-CL PARAMETERS
            // "Accel": Accel file name, defaults to "/imu/accel.txt"
            // "Gyro": Gyro file name, defaults to "/imu/gyro.txt"
            // "Mag": Mag file name, defaults to "/imu/mag.txt"
            // "GPS": GPS file name, defaults to "/imu/gps.txt"
            // "Timestamp": Timestamp file name, defaults to "/imu/timestamp.txt"
        }

        //----------------------------------------------- MICROSTRAIN
        if( sDeviceDriver == "MicroStrain") {
            IMU.SetProperty("HzGPS", nHzGPS );
            IMU.SetProperty("HzAHRS", nHzAHRS );
            IMU.SetProperty("GetGPS", bGetGPS );
            IMU.SetProperty("GetAHRS", bGetAHRS );
            IMU.SetProperty("GetEuler", bGetEuler );
            IMU.SetProperty("GetQuaternion", bGetQuat );
            IMU.SetProperty("GetGyro", bGetGyro );
            IMU.SetProperty("GetMagnetometer", bGetMag );
            IMU.SetProperty("GetAccelerometer", bGetAccel );
            IMU.SetProperty("GetDeviceTimestamp", bGetPPS );
        }

        return true;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool ParseIMUArgs(
            IMUDevice&          IMU,
            int                 argc,
            char**              argv
            )
    {
        GetPot clArgs( argc, argv );
        return ParseIMUArgs( IMU, clArgs );
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool InitIMU(
            IMUDevice&          IMU,
            GetPot&             clArgs
            )
    {
        if( ParseIMUArgs( IMU, clArgs ) == false ) {
            return false;
        }

        std::string     sDeviceDriver       = clArgs.follow( "", 1, "-idev" );

        // init driver
        if( !IMU.InitDriver( sDeviceDriver ) ) {
            std::cerr << "Invalid input device." << std::endl;
            std::cerr << IMU_USAGE;
            return false;
        }

        return true;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool InitIMU(
            IMUDevice&          IMU,
            int                 argc,
            char**              argv
            )
    {
        GetPot clArgs( argc, argv );
        return InitIMU( IMU, clArgs );
    }


} /* namespace */

#endif   /* INIT_IMU_H */
