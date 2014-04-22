#pragma once

#include <stdio.h>
#include <string>
#include <Node/Node.h>
#include <PbMsgs/Logger.h>
#include <PbMsgs/Image.h>
#include <PbMsgs/NodeCamMessage.pb.h>
#include <HAL/Camera/CameraDevice.h>

using namespace std;

void RegsiterCamDevice(RegisterNodeCamReqMsg& req, RegisterNodeCamRepMsg& rep, void* UserData);
void sendData(char *data, int w, int h, int format, int type);
bool InitializeNode( string IP, int port );
