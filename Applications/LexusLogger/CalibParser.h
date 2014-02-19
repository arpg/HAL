#define PRINT 0

#if PRINT
#include <stdio.h>
#endif

#include <tinyxml2.h>
#include <math.h>

using namespace tinyxml2;


struct VelodyneCalib
{
    double col[3];

    unsigned short minIntensity;
    unsigned short maxIntensity;

    double rotCorrection;;
    double vertCorrection;
    double distCorrection;
    double distCorrectionX;
    double distCorrectionY;
    double vertOffsetCorrection;
    double horizOffsetCorrection;
    double focalDistance;
    double focalSlope;

    //Sin's and cos's of angles so that we compute fast.
    double cos_rotCorrection;
    double sin_rotCorrection;
    double cos_vertCorrection;
    double sin_vertCorrection;

};

void ReadCalib(const char* name, VelodyneCalib* vc)
//int main()
{
    //struct VelodyneCalib vc[64];
    XMLDocument doc;
    doc.LoadFile(name);

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
		vc[i].col[0] = atof(rgb->GetText());
		

		rgb = rgb->NextSiblingElement();//item
#if PRINT
		printf("-----%s = %s\n", rgb->Name(), rgb->GetText());
#endif
		vc[i].col[1] = atof(rgb->GetText());

		rgb = rgb->NextSiblingElement();//item
#if PRINT
		printf("-----%s = %s\n", rgb->Name(), rgb->GetText());
#endif
		vc[i].col[2] = atof(rgb->GetText());

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
#if PRINT
		printf("----%s = %s\n", min->Name(), min->GetText());
#endif
		vc[i].minIntensity = atoi(min->GetText());
		min = min->NextSiblingElement();
		i++;
	    }
	}
	else if(!strcmp(name, "maxIntensity_"))
	{
	    XMLElement *max = el->FirstChildElement("item");
	    int i=0;
	    while(max)
	    {
#if PRINT
		printf("----%s = %s\n", max->Name(), max->GetText());
#endif
		vc[i].maxIntensity = atoi(max->GetText());
		max = max->NextSiblingElement();
		i++;
	    }
	}
	else if(!strcmp(name, "points_"))
	{
	    XMLElement *point = el->FirstChildElement("item");
	    XMLElement *cal;

	    int i=0;
	    while(point)
	    {
		cal = point->FirstChildElement()->FirstChildElement("rotCorrection_");//cal = item->px->rotCorrection_
		vc[i].rotCorrection = atof(cal->GetText());
		vc[i].cos_rotCorrection = cos(vc[i].rotCorrection*M_PI/180);
		vc[i].sin_rotCorrection = sin(vc[i].rotCorrection*M_PI/180);
#if PRINT
		printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

		cal = cal->NextSiblingElement();
		vc[i].vertCorrection = atof(cal->GetText());
		vc[i].cos_vertCorrection = cos(vc[i].vertCorrection*M_PI/180);
		vc[i].sin_vertCorrection = sin(vc[i].vertCorrection*M_PI/180);
#if PRINT
		printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

		cal = cal->NextSiblingElement();
		vc[i].distCorrection = atof(cal->GetText())/100;//since the data is in cm and we want it in meters
#if PRINT
		printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

		cal = cal->NextSiblingElement();
		vc[i].distCorrectionX = atof(cal->GetText())/100;
#if PRINT
		printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

		cal = cal->NextSiblingElement();
		vc[i].distCorrectionY = atof(cal->GetText())/100;
#if PRINT
		printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

		cal = cal->NextSiblingElement();
		vc[i].vertOffsetCorrection = atof(cal->GetText())/100;
#if PRINT
		printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

		cal = cal->NextSiblingElement();
		vc[i].horizOffsetCorrection = atof(cal->GetText())/100;
#if PRINT
		printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

		cal = cal->NextSiblingElement();
		vc[i].focalDistance = atof(cal->GetText());
#if PRINT
		printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

		cal = cal->NextSiblingElement();
		vc[i].focalSlope = atof(cal->GetText());
#if PRINT
		printf("-----%s = %s\n", cal->Name(), cal->GetText());
#endif

		point = point->NextSiblingElement();
		i++;
	    }
	}
	el = el->NextSiblingElement();
    }
}
