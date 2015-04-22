/* This is standard include file for lidar, depending on what lidar is there one can create a class accordingly.
 * One class is Velodyne, second would probably be hokuyo.
 */
#ifndef LIDAR_H
#define LIDAR_H

/* Headers related to the driver side*/
#include <HAL/LIDAR/LIDARDevice.h>

namespace hal
{

//This structure will contain all the correction factors for a laser. HDL-64E contains 64 lasers, so when using it
//create it such:
//              struct VelodyneLaserCorrection VarName[64]; //since NUM_LASER=64
struct VelodyneLaserCorrection
{
    double col[3];

    unsigned short minIntensity;
    unsigned short maxIntensity;

    double rotCorrection;
    double vertCorrection;
    double distCorrection;
    double distCorrectionX;
    double distCorrectionY;
    double vertOffsetCorrection;
    double horizOffsetCorrection;
    double focalDistance;
    double focalSlope;

    //Sin's and cos's of angles so that we computation is fast.
    double cos_rotCorrection;
    double sin_rotCorrection;
    double cos_vertCorrection;
    double sin_vertCorrection;

};

//Enum for how to create jet map for visualization.
enum ColoringMethod
{
    none=1,
    intensity=64,
    height=200,
    distance=120
};


class Velodyne
{
public:
    /* Here lie the Initializers of the class, they say "let there be light" */
    Velodyne();
    Velodyne(const char *calibFileName, int numLasers=64);
    void Init(const char *calibFileName, int numLasers=64);

    /* Here lie the Getters and Setters of the class, didn't do much but were guardians of the class.*/
    //NOTE: All and only getters and setters have first alphabet small.
    VelodyneLaserCorrection *getVlc() const;
    VelodyneLaserCorrection getVlc(int laserIdx) const;
    float *getPoints() const;
    unsigned char *getCol() const;
    float *getIntensities() const;
    ColoringMethod getColMethod() const;
    void setColMethod(const ColoringMethod &value);

    /*Here lie the doers, the movers and the shakers */
    void ConvertRangeToPoints(const LidarMsg& LidarData,
                              std::shared_ptr<LidarMsg> CorrectedData=nullptr);

    int getPointsSize() const;
    void setPointsSize(int value);

private:
    /* Methods */
    void ReadCalibData(const char *calibFileName);
    void SetupColorMethod();
    void ComputePoints(const LidarMsg& LidarData,
                       std::shared_ptr<LidarMsg> Points);
    void ComputeColor(int ii);
    void ComputeIntensity(const LidarMsg& LidarData,
                          std::shared_ptr<LidarMsg> Intensities);

    /* Properties */
    int mn_NumLasers; //Num of lasers in Velodyne, for HDL-64E it's 64, for HDL-32E it's 32. Imagine that!!
    int mn_PointsSize;//Number of elment in float *mp_Points.
    int mn_IntensitySize;// Number of element in float *mp_Intensities
    double md_TimeStamp;

    VelodyneLaserCorrection *vlc;//This will contain the parameters to correct the data. Each laser will have one struct, hence the array.

    float *mp_Points;//In homogenous coordinate, so each point takes 4 elements
    float *mp_Intensities;//Only one element, will be between 0-1.
    unsigned char *mp_Col;//Color takes 4 elements, so it's r,g,b,alpha, GLVbo of scenegraph works very nice in this case.
    unsigned char *mp_ColMap;//Generate a color map, this contains only r,g,b.

    ColoringMethod me_ColMethod;//Parameter stroing the coloring method.
};


//These functions will work for velodyne. And their name will start with velodyne.
}

#endif // LIDAR_H
