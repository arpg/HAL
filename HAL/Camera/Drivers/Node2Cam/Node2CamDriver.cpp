/*
   \file NodeCamDriver.cpp
 */
#include "Node2CamDriver.h"
#include "stdlib.h"

namespace hal
{

Node2CamDriver::Node2CamDriver(
        std::string& sDeviceName,
        std::string& sHostName)
{
    m_sDeviceName = sDeviceName;
    m_sHostName = sHostName;

    InitNode();
    std::cout<<"init Node2Cam success"<<std::endl;
}


Node2CamDriver::~Node2CamDriver()
{

}

// capture images from host
bool Node2CamDriver::Capture( pb::CameraMsg& vImages )
{
    // e.g. Proxy1/RCamera
    std::string sTopicName =m_sHostName+"/"+m_sDeviceName;

    CamMsg         Msg;
    CamMsg         TryMsg;

    // here we use max try to avoid infinia wait
    int iMaxTry=20;
    bool bSuccessFlag = false;
    while (bSuccessFlag==false||iMaxTry>0)
    {
            if(m_Node.receive(sTopicName, TryMsg)==true && TryMsg.time_step() == m_nTimeStep+1)
            {
                   Msg = TryMsg;
                   bSuccessFlag = true;
                   m_nTimeStep = TryMsg.time_step();
            }
            else
            {
                iMaxTry--;
            }
    }

    if(bSuccessFlag==false)
    {
        std::cout<<"[Node2CamDriver/Capture] Fail. Cannot receive images. form "<<sTopicName<<std::endl;
        return false;
    }

    m_nChannels = Msg.size();

    for(unsigned int ii = 0; ii != m_nChannels; ii++ )
    {
        const ImageMsg& Img = Msg.image(ii);

        if(Img.image_type()==1)//------------------------Gray
        {
            pb::ImageMsg* pbImg = vImages.add_image();
            pbImg->set_timestamp( m_nTimeStep);
            pbImg->set_width( m_nImgWidth );
            pbImg->set_height( m_nImgHeight );
            pbImg->set_type(pb::PB_UNSIGNED_SHORT);
            pbImg->set_format(pb::PB_LUMINANCE);
            pbImg->set_data( Img.image().c_str(), m_nImgWidth * m_nImgHeight );
        }
        else if(Img.image_type()==2)//-------------------RGB
        {
            pb::ImageMsg* pbImg = vImages.add_image();
            pbImg->set_timestamp( m_nTimeStep);
            pbImg->set_width( m_nImgWidth );
            pbImg->set_height( m_nImgHeight );
            pbImg->set_type(pb::PB_UNSIGNED_BYTE);
            pbImg->set_format(pb::PB_RGB);
            pbImg->set_data( Img.image().c_str(), m_nImgWidth * m_nImgHeight *3);
        }
        else if(Img.image_type()==5)//------------------Depth
        {
            pb::ImageMsg* pbImg = vImages.add_image();
            pbImg->set_timestamp( m_nTimeStep);
            pbImg->set_width( m_nImgWidth );
            pbImg->set_height( m_nImgHeight );
            pbImg->set_type(pb::PB_FLOAT);
            pbImg->set_format(pb::PB_LUMINANCE);
            pbImg->set_data( Img.image().c_str(), m_nImgWidth * m_nImgHeight *4);
        }
    }

    return true;
}

std::string Node2CamDriver::GetDeviceProperty(const std::string& sProperty)
{
    // TODO add property with suffix of camera (ie. DepthBaseline0, DepthBaseline1)
    // and return correct vector info
//    if(sProperty == hal::) {
//        return std::to_string( m_DepthBaselines[0] );
//    }
//    if(sProperty == hal::DeviceDepthFocalLength) {
//        return std::to_string( m_DepthFocalLengths[0] );
//    }
    return std::string();
}

size_t Node2CamDriver::NumChannels() const
{
    return  m_nChannels;
}

size_t Node2CamDriver::Width( size_t /*idx*/ ) const
{
    return m_nImgWidth;
}

size_t Node2CamDriver::Height( size_t /*idx*/ ) const
{
    return m_nImgHeight;
}


bool Node2CamDriver::InitNode()
{
    m_Node.set_verbocity(2); // make some noise on errors
    if(m_Node.init(m_sDeviceName)==false)
    {
        std::cerr<<"[Node2CamDriver] Cannot init Node2Cam '"<<m_sDeviceName<<"'"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"[Node2CamDriver] init Node2Cam '"<<m_sDeviceName<<"' success"<<std::endl;
    }


    // register Node2Cam with Host
    if(RegisterInHost()==false)
    {
        return false;
    }

    return true;
}

// register Node2Cam in Host
bool Node2CamDriver::RegisterInHost()
{
    RegisterCamReqMsg mRequest;
    RegisterCamRepMsg mReply;

    std::string sServiceName = m_sHostName+"/RegsiterCamDevice";
    mRequest.set_name(m_sDeviceName);
    while( m_Node.call_rpc(sServiceName, mRequest, mReply) ==false)
    {
        std::cerr << "[Node2CamDriver] Error Call Rpc method of '" << sServiceName << "'. Cannot connect to host!! Please make sure the host is running!" << std::endl;
        sleep(1);
    }

    if(mReply.regsiter_flag()==1)
    {
        m_nTimeStep =  mReply.time_step();
        m_nChannels = 2;
        m_nImgHeight = 384;
        m_nImgWidth = 512;

        std::string sTopicName =m_sHostName+"/"+m_sDeviceName;

        std::cout<<"[Node2CamDriver] Try to subscrive to topic '"<<sTopicName<<"'"<<std::endl;
        if( m_Node.subscribe(sTopicName) == false )
        {
            std::cerr << "[Node2CamDriver] Error subscribing to '" << sTopicName << "'. Please make sure "<<sTopicName<<" is running !!" << std::endl;
            return false;
        }
        else
        {
            std::cout<<"[Node2CamDriver] Subscrive to topic '"<<sTopicName<<"' success!"<<std::endl;
        }
    }
    else
    {
        std::cerr<<"[Node2CamDriver] Cannot register RPG to "<<sServiceName<<std::endl;
        return false;
    }

    return true;
}

}

