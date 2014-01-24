// Check if we need to launch Sim (Proxy that simluate device info)

#ifndef SIMLAUNCHER_H
#define SIMLAUNCHER_H

#include <tinyxml2.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <unistd.h>

using namespace std;

namespace hal
{
// flag if we launch Simulation already
bool bSimWorldRunningFlag = false;

bool GetAttribute(tinyxml2::XMLElement *pElement, const char* cAttributeName, string &sAttribute)
{
    const char* cAttribute = pElement->Attribute(cAttributeName);
    if(cAttribute==NULL)
    {
        std::cout<<"[SimLauncher.h GetAttribute()] Fatal error! cannot get attribute '"<<cAttributeName<<"' for"<< pElement->Name() << " in Sim.xml file!" <<std::endl;
        return false;
    }

    sAttribute = cAttribute;
    return true;
}

//Read the file set to Environment variable SIM.
bool ReadSimXml( string &sSimName, string &sSimPath, string &sStateKeeperName, string &sWorldPath, string &sRobotPath)
{
    char* cSIMbuf =  getenv( "SIM" );

    tinyxml2::XMLDocument t2SimDoc;
    if(t2SimDoc.LoadFile(cSIMbuf) !=0)
    {
        std::cout<<"File with simulator info(typically with name Sim.xml)"
                   "doesn't exist. Check if the path: "<<cSIMbuf
                   <<" is correct"<<std::endl;
    }

    tinyxml2::XMLElement *t2pParent = t2SimDoc.RootElement();
    tinyxml2::XMLElement *t2pElement = t2pParent->FirstChildElement();

    if(strcmp(t2pElement->Name(), "sim")==0)
    {
        if(!GetAttribute(t2pElement, "name", sSimName))
            return false;
        if(!GetAttribute(t2pElement, "path", sSimPath))
            return false;
    }

    t2pElement = t2pElement->NextSiblingElement();
    if(strcmp(t2pElement->Name(), "stateKeeper")==0)
    {
        if(!GetAttribute(t2pElement, "name", sStateKeeperName))
            return false;
        if(sStateKeeperName == "NULL")
            sStateKeeperName = "WithoutStateKeeper";
    }

    t2pElement = t2pElement->NextSiblingElement();
    if(strcmp(t2pElement->Name(), "world")==0)
        if(!GetAttribute(t2pElement, "path", sWorldPath))
            return false;

    t2pElement = t2pElement->NextSiblingElement();
    if(strcmp(t2pElement->Name(), "robot")==0)
        if(!GetAttribute(t2pElement, "path", sRobotPath))
            return false;

    return true;
}

// check if we need to launch Simulation
bool LaunchSimulationIfNeeded()
{
    if (bSimWorldRunningFlag == false)
    {

        string sSimName, sSimPath, sStateKeeperName, sWorldPath, sRobotPath;
        if(!ReadSimXml(sSimName,sSimPath, sStateKeeperName, sWorldPath, sRobotPath))
        {
            std::cout<<"As you know there was problem in parsing Sim.Xml, please Check"<<endl;
            return false;
        }
        string sCommand =sSimPath +"./LocalSim "+"-n "+sSimName+" -r "+sRobotPath+" -w "+sWorldPath+" -s "+sStateKeeperName;

        cout<<"-------------------------------------------------------------------------------------------------"<<endl
           <<"[HAL/LaunchSimulationIfNeeded] Detect SIM Environment!! Start SimWorld For Sim Device!"<<endl
          <<"World URDF File Path is "<< sWorldPath<<endl
         <<"Robot URDF File Path is "<< sRobotPath<<endl
        <<"Simulator name is "<<sSimName<<endl
        <<"The command input is "<<sCommand<<endl
        <<"-------------------------------------------------------------------------------------------------"<<endl;

        // start sim world
        int pid = fork();
        if (pid < 0)
        {
            printf("[HAL/LaunchSimulationIfNeeded] Simulator thread error!\n");
            return false;
        }
        else if( pid == 0 )
        {
            cout<<"[HAL/LaunchSimulationIfNeeded] Starting a Simulator for Sim Devices"<<endl;
            system(sCommand.c_str());
        }
        else
        {
            printf("In parent process! child process id = %d\n", pid);
            sleep(4);
            bSimWorldRunningFlag = true;
        }
    }

    return true;
}
}


#endif // SIMLAUNCHER_H
