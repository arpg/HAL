/*
   \file NodeCamDriver.cpp
 */

#include "NodeCamDriver.h"

#include "Message.pb.h"

#include <boost/format.hpp>


///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
NodeCamDriver::NodeCamDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
NodeCamDriver::~NodeCamDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
bool NodeCamDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{
    msg::NodeCam Msg;

    // block until we get an image
    while( m_Node.Read( "NodeCam", Msg ) == false ) {}

    vImages.resize( Msg.image_size() );

    for( unsigned int ii = 0; ii < vImages.size(); ii++ ) {
        const msg::Image& Img = Msg.image(ii);
        const unsigned int ImgHeight = Img.image_height();
        const unsigned int ImgWidth = Img.image_width();
        vImages[ii].Image = cv::Mat( ImgHeight, ImgWidth, Img.image_type() );
        memcpy( vImages[ii].Image.data, Img.image().c_str(), Img.image_size() );

        for( int jj = 0; jj < Img.property_name_size(); jj++ ) {
            vImages[ii].Map.SetProperty( Img.property_name(jj), Img.property_val(jj) );
        }

    }

    return true;
}


///////////////////////////////////////////////////////////////////////////////
void NodeCamDriver::PrintInfo() {

    std::cout <<
    "FILEREADER\n"
    "Reads images from the disk."
    "\n"
    "Options:\n"
    "   -sdir           <source directory for images and camera model files> [default '.']\n"
    "   -lfile          <regular expression for left image channel>\n"
    "   -rfile          <regular expression for right image channel>\n"
    "   -lcmod          <left camera model xml file>\n"
    "   -rcmod          <right camera model xml file>\n"
    "   -sf             <start frame> [default 0]\n"
    "   -numchan        <number of channels> [default 2]\n"
    "   -buffsize       <size of buffer for image pre-read> [default 35]\n"
    "   -timekeeper     <name of variable holding image timestamps> [default 'SystemTime]\n"
    "\n"
    "Flags:\n"
    "   -greyscale      If the driver should return images in greyscale.\n"
    "   -loop           If the driver should restart once images are consumed.\n"
    "\n"
    "Example:\n"
    "./Exec  -idev FileReader  -lcmod lcmod.xml  -rcmod rcmod.xml  -lfile \"left.*pgm\"  -rfile \"right.*pgm\"\n\n";
}


///////////////////////////////////////////////////////////////////////////////
bool NodeCamDriver::Init()
{
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    // read property map
    m_nNumNodes       = m_pPropertyMap->GetProperty<unsigned int>( "NumNodes", 0 );

    if( m_nNumNodes < 1 ) {
        std::cerr << "ERROR: No nodes specified. Set property NumNodes." << std::endl;
        exit(1);
    }

    // subscribe to remote hose
    for( unsigned int ii = 0; ii < m_nNumNodes; ii++ ) {
        std::string sNodeID = (boost::format("Node-%d")%ii).str();
        std::string sNodeURL = m_pPropertyMap->GetProperty( sNodeID, "");

        if( m_Node.Subscribe( "NodeCam", sNodeURL ) == false ) {
            std::cerr << "Error subscribing to node '" << sNodeURL << "'." << std::endl;
        }
    }

    return true;
}
