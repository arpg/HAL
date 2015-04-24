#include "Velodyne.h"

/* Headers related to ReadCalibData */
#include <math.h>
#include <tinyxml2.h>
#include <cmath>

using namespace tinyxml2;

namespace hal
{

// Here starts the defining of the methods, no other definitions shall compare to this. These SHALL PASS compiling and
// linking.

//Call this function in case default constructor was called.
Velodyne::Velodyne()
{
}

Velodyne::Velodyne(const char *calibFileName, int numLasers)
{
  Init(calibFileName,numLasers);
}

void Velodyne::Init(const char *calibFileName, int numLasers)
{
  this->mn_NumLasers = numLasers;
  vlc = new VelodyneLaserCorrection[numLasers];
  me_ColMethod = height;
  SetupColorMethod();

  // 144000 = 36000*4. 36000 because that's the precision for rotational position, i.e. hundredth of a degree.
  // 4 because we have because we have points in homogenous coordinate.
  // So, by having 36000*4*numLasers, we can store points for all possible directions velodyne can throw at us.
  mn_PointsSize = 144000*numLasers;
  mn_IntensitySize = 36000*numLasers;
  mp_Points = new float[mn_PointsSize];
  mp_Col = new unsigned char[mn_PointsSize];
  mp_Intensities = new float[mn_IntensitySize];
  memset(mp_Points,0,mn_PointsSize*4);
  memset(mp_Col,0,mn_PointsSize);

  ReadCalibData(calibFileName);
}

float *Velodyne::getPoints() const
{
    return mp_Points;
}

unsigned char *Velodyne::getCol() const
{
  return mp_Col;
}

float *Velodyne::getIntensities() const
{
  return mp_Intensities;
}

VelodyneLaserCorrection* Velodyne::getVlc() const
{
  return vlc;
}

VelodyneLaserCorrection Velodyne::getVlc(int laserIdx) const
{
  return vlc[laserIdx];
}

ColoringMethod Velodyne::getColMethod() const
{
  return me_ColMethod;
}

void Velodyne::setColMethod(const ColoringMethod &value)
{
  me_ColMethod = value;
  SetupColorMethod();
}

int Velodyne::getPointsSize() const
{
    return mn_PointsSize;
}

void build_jet_map( int length, unsigned char *table )
{
    int ii;
    int sectlength = length / 5;
    int sect1 = (1 * length) / 5;
    int sect2 = (2 * length) / 5;
    int sect3 = (3 * length) / 5;
    int sect4 = (4 * length) / 5;

    for(ii = 0; ii < length; ii++) {
        if( ii < sect1 ) {
            /* red is zero */
            table[ii*3 + 0] = 0;
            /* green is zero */
            table[ii*3 + 1] = 0;
            /* blue is rising */
            table[ii*3 + 2] = (128* ii) / sectlength + 127;
        }
        else if( ii < sect2 ) {
            /* red is zero */
            table[ii*3 + 0] = 0;
            /* green is rising */
            table[ii*3 + 1] = (255 * (ii - sect1)) / sectlength;
            /* blue is flat */
            table[ii*3 + 2] = 255;
        }
        else if( ii < sect3 ) {
            /* red is rising */
            table[ii*3 + 0] = (255 * (ii - sect2)) / sectlength;
            /* green is flat */
            table[ii*3 + 1] = 255;
            /* blue is falling */
            table[ii*3 + 2] = 255 - (255 * (ii - sect2)) / sectlength;
        }
        else if( ii < sect4 ) {
            /* red is flat */
            table[ii*3 + 0] = 255;
            /* green is falling */
            table[ii*3 + 1] = 255 - (255 * (ii - sect3)) / sectlength;
            /* blue is zero */
            table[ii*3 + 2] = 0;
        }
        else {
            /* red is falling */
            table[ii*3 + 0] = 255 - (128 * (ii - sect4)) / sectlength;
            /* green is zero */
            table[ii*3 + 1] = 0;
            /* blue is zero */
            table[ii*3 + 2] = 0;
        }
    }
}

void Velodyne::SetupColorMethod()
{
  //Numbers ar multiplied with 3 because, well because we want r,g,b.
  mp_ColMap = new unsigned char[me_ColMethod*3];
  build_jet_map(me_ColMethod, mp_ColMap);
}

void Velodyne::ReadCalibData(const char *calibFileName)
{
  XMLDocument doc;
  if(doc.LoadFile(calibFileName) !=0)
  {
    printf("[InitParamsFromXML] Error!! Cannot open:%s\n", calibFileName);
    return;
  }

  XMLElement *root = doc.RootElement();
#if PRINT
  printf("-%s\n",root->Name());
#endif

  XMLElement *el = root->FirstChildElement();
#if PRINT
  printf("--%s\n", el->Name());
#endif
  el = el->FirstChildElement("colors_");
  while(el)
  {
    const char *name = el->Name();
#if PRINT
    printf("---%s\n", name);
#endif

    //Don't care about this part.
    if(!strcmp(name, "colors_"))
    {
      XMLElement *col = el->FirstChildElement("item");//colors_ -> item
      XMLElement *rgb;
#if PRINT
      printf("----%s\n", col->Name());
#endif

      int i=0;
      while(col)
      {
        rgb = col->FirstChildElement()->FirstChildElement("item");//rgb -> item
#if PRINT
        printf("-----%s = %s\n", rgb->Name(), rgb->GetText());
#endif
        vlc[i].col[0] = atof(rgb->GetText());


        rgb = rgb->NextSiblingElement();//item
#if PRINT
        printf("-----%s = %s\n", rgb->Name(), rgb->GetText());
#endif
        vlc[i].col[1] = atof(rgb->GetText());

        rgb = rgb->NextSiblingElement();//item
#if PRINT
        printf("-----%s = %s\n", rgb->Name(), rgb->GetText());
#endif
        vlc[i].col[2] = atof(rgb->GetText());

        col = col->NextSiblingElement();
        i++;

      }

    }
    else if(!strcmp(name, "minIntensity_"))
    {
      XMLElement *min = el->FirstChildElement("item");
      int i=0;
      while(min)
      {
        if(i>mn_NumLasers)
        {
          fprintf(stderr, "Dude!! You said number of lasers were %d, but the calibration file has minIntensity for %d as well\n", mn_NumLasers, i);
          return;
        }
#if PRINT
        printf("----%s = %s\n", min->Name(), min->GetText());
#endif
        vlc[i].minIntensity = atoi(min->GetText());
        min = min->NextSiblingElement();
        i++;
      }
      if(i!=mn_NumLasers)
      {
        fprintf(stderr, "Dude!! You said number of lasers were %d, but the calibration file has maxIntensity only for %d lasers\n", mn_NumLasers, i);
        return;
      }
    }
    else if(!strcmp(name, "maxIntensity_"))
    {
      XMLElement *max = el->FirstChildElement("item");
      int i=0;
      while(max)
      {
        if(i>mn_NumLasers)
        {
          fprintf(stderr, "Dude!! You said number of lasers were %d, but the calibration file has maxIntensity for %d as well\n", mn_NumLasers, i);
          return;
        }
#if PRINT
        printf("----%s = %s\n", max->Name(), max->GetText());
#endif
        vlc[i].maxIntensity = atoi(max->GetText());
        max = max->NextSiblingElement();
        i++;
      }
      if(i!=mn_NumLasers)
      {
        fprintf(stderr, "Dude!! You said number of lasers were %d, but the calibration file has minIntensity only for %d lasers\n", mn_NumLasers, i);
        return;
      }
    }
    else if(!strcmp(name, "points_"))
    {
      XMLElement *point = el->FirstChildElement("item");
      XMLElement *cal;

      int i=0;
      while(point)
      {
        if(i>mn_NumLasers)
        {
          fprintf(stderr, "Dude!! You said number of lasers were %d, but the calibration file has correction values for %d as well\n", mn_NumLasers, i);
          return;
        }
        cal = point->FirstChildElement()->FirstChildElement("rotCorrection_");//cal = item->px->rotCorrection_
        vlc[i].rotCorrection = atof(cal->GetText());
        vlc[i].cos_rotCorrection = cos(vlc[i].rotCorrection*M_PI/180);
        vlc[i].sin_rotCorrection = sin(vlc[i].rotCorrection*M_PI/180);
#if PRINT
        printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

        cal = cal->NextSiblingElement();
        vlc[i].vertCorrection = atof(cal->GetText());
        vlc[i].cos_vertCorrection = cos(vlc[i].vertCorrection*M_PI/180);
        vlc[i].sin_vertCorrection = sin(vlc[i].vertCorrection*M_PI/180);
#if PRINT
        printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

        cal = cal->NextSiblingElement();
        vlc[i].distCorrection = atof(cal->GetText())/100;//since the data is in cm and we want it in meters
#if PRINT
        printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

        cal = cal->NextSiblingElement();
        vlc[i].distCorrectionX = atof(cal->GetText())/100;
#if PRINT
        printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

        cal = cal->NextSiblingElement();
        vlc[i].distCorrectionY = atof(cal->GetText())/100;
#if PRINT
        printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

        cal = cal->NextSiblingElement();
        vlc[i].vertOffsetCorrection = atof(cal->GetText())/100;
#if PRINT
        printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

        cal = cal->NextSiblingElement();
        vlc[i].horizOffsetCorrection = atof(cal->GetText())/100;
#if PRINT
        printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

        cal = cal->NextSiblingElement();
        vlc[i].focalDistance = atof(cal->GetText());
#if PRINT
        printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

        cal = cal->NextSiblingElement();
        vlc[i].focalSlope = atof(cal->GetText());
#if PRINT
        printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

        point = point->NextSiblingElement();
        i++;
      }
      if(i!=mn_NumLasers)
      {
        fprintf(stderr, "Dude!! You said number of lasers were %d, but the calibration file has correction information only for %d lasers\n", mn_NumLasers, i);
        return;
      }
    }
    el = el->NextSiblingElement();
  }
}

void Velodyne::ConvertRangeToPoints(const hal::LidarMsg& LidarData, //input
                                    std::shared_ptr<LidarMsg> CorrectedData) //output
{
  if((int)LidarData.distance().rows() != mn_NumLasers)
  {
      std::cout<<"Dude!! You said there will be "<< mn_NumLasers << " lasers"
               <<" but i am getting only "
              <<(int)LidarData.distance().rows()<<std::endl;
    return;
  }
  CorrectedData->set_system_time(LidarData.system_time());
  CorrectedData->set_device_time(LidarData.device_time());
  ComputePoints(LidarData, CorrectedData);
  ComputeIntensity(LidarData, CorrectedData);
  md_TimeStamp = LidarData.system_time();
}

void Velodyne::ComputeIntensity(const hal::LidarMsg& LidarData,
                                std::shared_ptr<LidarMsg> Intensities)
{

  hal::MatrixMsg* pbMatIntensity = nullptr;
  if(Intensities != nullptr) {
    pbMatIntensity = Intensities->mutable_intensity();
    pbMatIntensity->set_rows(1);
  }

  for (int block = 0; block < 6; ++block)
  {
    for (int laser = 0; laser < mn_NumLasers; ++laser)
    {
      VelodyneLaserCorrection vcl = vlc[laser];
      int pt_idx = block*mn_NumLasers + laser;
      double distance = LidarData.distance().data(pt_idx);
      if(distance==0)// || distance >= 120) //So that we have correspondance with points
        continue;

      double intesity = LidarData.intensity().data(pt_idx);//intesity is not a spelling mistake, can confuse with enum, simpler this way.
      double dist_raw = LidarData.distance().data(pt_idx) * 500;//The way velodyne packet intended it, at 2mm unit.

      double min_intensity = vcl.minIntensity;
      double max_intensity = vcl.maxIntensity;
      double focal_slope = vcl.focalSlope;

      float focal_offset = 256 * (1 - vcl.focalDistance/13100) * (1 - vcl.focalDistance/13100);
      intesity += focal_slope * std::abs(focal_offset - 256 * (1 - dist_raw/65535) * (1 - dist_raw/65535));
      intesity = (intesity<min_intensity)?min_intensity:intesity;
      intesity = (intesity>max_intensity)?max_intensity:intesity;

      //Scale the intensity and then assign.
      int idx = ((int)(LidarData.rotational_position().data(block)*100) * mn_NumLasers) + laser;
      double value = (intesity - min_intensity)/(max_intensity-min_intensity);
      mp_Intensities[idx] = (float)value;
      if(Intensities && pbMatIntensity)
        pbMatIntensity->add_data(value);
    }
  }
}

void Velodyne::ComputePoints(const hal::LidarMsg &LidarData,
                             std::shared_ptr<LidarMsg> Points)
{

  hal::MatrixMsg* pbMatPoint = nullptr;
  if(Points != nullptr) {
    pbMatPoint = Points->mutable_distance();
    pbMatPoint->set_rows(4);
    Points->mutable_rotational_position()->CopyFrom(
          LidarData.rotational_position());
  }

  //block contains upper and lower block, i.e. 64 lasers.
  for(int block=0; block<6; block++)
  {
    //calculating sine and cos beforehand, to save computation later
    double cos_rotation_pos=cos(LidarData.rotational_position().data(block)*M_PI/180);
    double sin_rotation_pos=sin(LidarData.rotational_position().data(block)*M_PI/180);

    for(int laser=0; laser<mn_NumLasers;laser++)
    {
      //Change in naming convention here, "_" are used to delineate words.
      //printf("--------------------------------------------------------------------\n");
      int pt_idx = block*mn_NumLasers + laser;

      //if the distance is 0, that means it was less than 0.9, so invalid, in that case we don't do anything
      //if the distance is 120 or greater it's max range, we don't know if point is there or not.
      double distance = LidarData.distance().data(pt_idx);
      double distance_raw = distance;
      if(distance==0)// || distance >= 120)
        continue;

      //getting a pointer to calibration data for this laser.
      VelodyneLaserCorrection vcl = vlc[laser];//vcl stands for Velodyne calibration of a laser.

      /* 1. Correct the distance, that is done by just adding the value with distCorrection (which is far point calibration at 25.04m).
       *    This is the distance error along the ray which a laser has.
       *    Distance from LidarMsg is already converted in meters, so was the correction factor in calibration data.
       **/
      double dist_corr = vcl.distCorrection;
      distance += dist_corr;
      //printf("pt_id = %d\ndist=%.3lf, dist_corr=%.1lf, dist_cor = %.1lf\n", pt_idx, distance, dist_corr, vc[laser].distCorrection);
      //printf("cos_rot_ang = %.1lf, sin = %.1lf\n", cos_rotation_pos, sin_rotation_pos);

      /* 2. Now we correct angles, these angles are with the front of the camera, which is +y.
         *    These values will be primarily used when we will be computing x and y coordinate.
             *    If a is angle of laser, b is correction, to correct angle we want a-b, but finally for calculationswe want cos(a-b), we have cos of a,b.
             *    So we use the identity cos(a-b) = cos(a)cos(b) + sin(a)*sin(b)
             *    Similarly for sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
             **/
      double cos_rotation_angle = cos_rotation_pos*vcl.cos_rotCorrection + sin_rotation_pos*vcl.sin_rotCorrection;
      double sin_rotation_angle = sin_rotation_pos*vcl.cos_rotCorrection - vcl.sin_rotCorrection*cos_rotation_pos;
      //printf("cos_rot_corr= %.1lf, sin = %.1lf\n", vcl.cos_rotCorrection, vcl.sin_rotCorrection);


      /* 3. Now we compute the distance in xy plane, i.e. the distance in horizontal plane and not along the ray.
         *    This is done to correct vertical and horizontal offset for the laser. Each laser is supposed to originate from a single point,
             *    which obviously doesn't happen. Vertical Offset is the offset along z-axis from xy-plane, a positive offset is towards +z.
             *    Horizontal offset is the offset in xy plane from the origin, a +ve offset is towards -x.
             *    To get the final results, some more adjustments are made in next step.
             **/
      double cos_vert_corr = vcl.cos_vertCorrection;
      double sin_vert_corr = vcl.sin_vertCorrection;
      double horiz_offset = vcl.horizOffsetCorrection;
      double vert_offset = vcl.vertOffsetCorrection;
      //now variable names are dist in corresponding planes or axis
      double xy = distance * cos_vert_corr;
      double xx = xy * sin_rotation_angle  - horiz_offset * cos_rotation_angle;
      double yy = xy * cos_rotation_angle  + horiz_offset * sin_rotation_angle;
      xx = xx<0?-xx:xx;
      yy = yy<0?-yy:yy;

      /* 4. Now, we correct for parameters distCorrectionX and distCorrectionY. Why we do this is unclear, but; what we know is what we do.
       *    The idea here is that we have correction value for near points at x=2.4m and y=1.93m, we also have distCorrection for far point
       *    calibration at 25.04m. So, to get the correction for a particular distance in x, we interpolate using values corresponding to x-axis
       *    i.e. 2.4, distCorrectionX, 25.04, distCorrection (last two apply to both cases). Similarly for y-axis using 1.93,distCorrectionY,
       *    25.04, distCorrection.
       *    For better understanding of the interpolation formulae refer to Appendix F of the manual.
       **/
      //compute the correction via interpolation
      double corr_xx = vcl.distCorrectionX + (dist_corr - vcl.distCorrectionX) * (xx-2.4)/22.64;//25.04-2.4 = 22.64
      double corr_yy = vcl.distCorrectionY + (dist_corr - vcl.distCorrectionY) * (yy-1.93)/23.11;//25.04-1.93 = 23.11

      /* 5. Next task is to extract coordinates x, y and z.
       *    To compute x and y cordinates we correct distance with corrections computed, then take projection on xy palne, then on respective axes.
       *    Ofcourse we correct for horizontal offset as well, we are not stupid you know.
       *    z is a good boy and calculating it is straight forward, projection of distance on z-axis and then correcting by vertOffset.
       **/
      //If B=distance in the follwing three formulae, then To B or Not to B is the question.
      //The X-coordinate
      xy = (distance_raw + corr_xx)*cos_vert_corr;
      xx =  xy * sin_rotation_angle  - horiz_offset * cos_rotation_angle;

      //The Y-coordinate
      xy = (distance_raw + corr_yy)*cos_vert_corr;
      yy =  xy * cos_rotation_angle  + horiz_offset * sin_rotation_angle;

      //The Z-coordinate
      double zz=distance_raw * sin_vert_corr + vert_offset + 1.5;//velodyne is porbably at 1.5m from ground, adding 1.5 to have the ground plane at z=0, exact value to be estimated later.

      //we have angular resolution of 0.01 degrees. For each rotational position (there are 36000 such positions) we have a block of 256 (64(laser) * 4(x,y,z,1) = 256) float values.
      //Angle supplied by LidarData is in degrees, so to compute a rotational position we multiply angle by 100, then we multiply by 256 to reach the offset for that block, hence
      //multiplication by 25600.
      //Adding each laser gives us data worth of 4 floats, so we multiply laser by 4 to get exact position in array.
      int idx = ((int)LidarData.rotational_position().data(block))*25600 + laser*4;//

      mp_Points[idx] = (float)xx;
      mp_Points[idx+1] = (float)yy;
      mp_Points[idx+2] = (float)zz;
      mp_Points[idx+3] = 1.0;
      ComputeColor(idx);

      if(Points != nullptr && pbMatPoint) {
        pbMatPoint->add_data(xx);
        pbMatPoint->add_data(yy);
        pbMatPoint->add_data(zz);
        pbMatPoint->add_data(1);
      }

    }
  }

}

void Velodyne::ComputeColor(int ii)
{
  int colIdx=0;
  switch (me_ColMethod)
  {
  case intensity:
      colIdx = (int)(mp_Intensities[ii]*63);
      mp_Col[ii*4] = mp_ColMap[colIdx];
      mp_Col[ii*4+1] = mp_ColMap[colIdx+1];
      mp_Col[ii*4+2] = mp_ColMap[colIdx+2];
      mp_Col[ii*4+3] = 255;
    break;
  case height:
      colIdx = floor(mp_Points[ii+2]*100)*3;
      mp_Col[ii] = mp_ColMap[colIdx];
      mp_Col[ii+1] = mp_ColMap[colIdx+1];
      mp_Col[ii+2] = mp_ColMap[colIdx+2];
      mp_Col[ii+3] = 255;
      break;
  case distance:
      //calculating distance, this will be in meter. max distance can be 119.999.... so max colIdx would be 119.
      colIdx = (int)(sqrt(mp_Points[ii]*mp_Points[ii] + mp_Points[ii+1]*mp_Points[ii+1] + mp_Points[ii+2]*mp_Points[ii+2] )/120);
      mp_Col[ii] = mp_ColMap[colIdx];
      mp_Col[ii+1] = mp_ColMap[colIdx+1];
      mp_Col[ii+2] = mp_ColMap[colIdx+2];
      mp_Col[ii+3] = 255;
    break;
  default:
      mp_Col[ii] = 0;
      mp_Col[ii+1] = 255;
      mp_Col[ii+2] = 0;
      mp_Col[ii+3] = 255;
    break;
  }
}

}
