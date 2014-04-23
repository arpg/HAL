#pragma once

#include <stdio.h>
#include <string>
#include <Node/Node.h>
#include <PbMsgs/Logger.h>
#include <PbMsgs/Image.h>
#include <PbMsgs/NodeCamMessage.pb.h>
#include <HAL/Camera/CameraDevice.h>

std::string sNodeName("LocalSim");
int gCounter = 0;
int width, height, channels;
node::node n;
std::string topic("NodeCam");

void RegsiterCamDevice(RegisterNodeCamReqMsg& req, RegisterNodeCamRepMsg& rep, void* UserData);
void sendData(char *data, int w, int h, int format, int type);
bool InitializeNode(std::string IP, int port );
