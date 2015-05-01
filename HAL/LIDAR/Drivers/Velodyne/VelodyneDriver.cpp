#include "VelodyneDriver.h"
#include <stdio.h>

/*Socket Specific headers */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>

/* Headers from HAL */
#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
// port defaults to 2368 if not provided.
VelodyneDriver::VelodyneDriver(int port)
    : m_running(false), m_callback(nullptr), m_port(port), m_socketDescriptor(0)
{
    //open the socket and stuff.
    struct sockaddr_in si_me;

    //Creating socket.
    if ((m_socketDescriptor=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
	throw DeviceException(strerror(errno));

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(m_port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    //Binding the Socket.
    if (bind(m_socketDescriptor, (sockaddr *)&si_me, sizeof(si_me))==-1)
	throw DeviceException(strerror(errno));
}


/////////////////////////////////////////////////////////////////////////////////////////
void VelodyneDriver::_ThreadFunc()
{
    hal::LidarMsg pbMsg;
	struct sockaddr_in si_other;
	char buf[BUFLEN];
	int slen=sizeof(si_other);

    while( m_running ) {
	pbMsg.Clear();

	//TODO: all the code for recieving the packets and pack it into PbMsg.
	if (recvfrom(m_socketDescriptor, buf, BUFLEN, 0, (sockaddr *)&si_other, (socklen_t *)&slen)==-1)
	    throw DeviceException(strerror(errno));

	//Now we start transfering the data to PbMsg.
	//Assuming recvfrom ignores the 42 byte udp header, hence size of data packet is 1206.
	pbMsg.set_system_time(Tic());//before doing anything else.

	hal::VectorMsg *pbVec = pbMsg.mutable_rotational_position();
	hal::MatrixMsg *pbMatDist = pbMsg.mutable_distance();
	hal::MatrixMsg *pbMatIntensity = pbMsg.mutable_intensity();
	int offset=0; //This Variable will tell where data for block starts, each block will be of 100 bytes.



    /* Data contains 12 blocks (6 upper, 6 lower), each block contains
     * 32 lasers, we read data in this format. ALTHOUGH, each pair of upper
     * and lower block have same rotational position, so it can be interpreted
     * as 6 blocks of 64 lasers. LidarMsg should be thought of as a matrix of
     * 64x6, each column representing block and each row laser in that block.
     */
    pbMatDist->set_rows(64);//64 lasers
    pbMatIntensity->set_rows(64);
	for(int block=0; block<12; block++)
	{
	    /*100, which consists of 2byte block id, 2 byte rotational position and 96 byts (32x3) of distance and intensity information from laser. Upper and lower block should alternate. */
	    offset = block*100;

	    /*First two bytes are upper/lower block, ignoring it*/

	    /* Next two bytes are rotational position*/
	    if(block%2==0)
	    {
		unsigned short *rot_pos = (unsigned short*)(buf+offset+2);
		pbVec->add_data( ( (*rot_pos)/(double)100 ) );
	    }

	    /* Next 32x3 for the 32 lasers in each block */
	    int laser_offset = offset+4;//this points to start of laser data
	    for(int laser=0; laser<32; laser++)
	    {
		unsigned short *si_dist = (unsigned short*)(buf + laser_offset);
		double dist = (double)*si_dist;
		if(dist<450)//If value is less than 450, i.e. 0.9 m, reading is not be trusted
		    dist=0;
		else
		    dist = dist/500;//converting to meters, each increment is 2mm, hence division by 500.

		pbMatDist->add_data(dist);
		pbMatIntensity->add_data((double)*(buf + laser_offset + 1));
		laser_offset +=3;//size of data for each laser is 3.
	    }

	    /* Converting the gps timestamp from big endian to little endian */
	    char little_endian_gps[4];
	    little_endian_gps[0] = *(buf+1204);
	    little_endian_gps[1] = *(buf+1203);
	    little_endian_gps[2] = *(buf+1202);
	    little_endian_gps[3] = *(buf+1201);
	    int *gps_time = (int*)little_endian_gps;
	    pbMsg.set_device_time((double)( (*gps_time)/1e+6 ));//device time in secs.
	}

	// Code for packet recieving and packaging ends.

	//Now call the callback function with the data.
	m_callback(pbMsg);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
VelodyneDriver::~VelodyneDriver()
{
    m_running = false;
    if( m_callbackThread.joinable() ) {
        m_callbackThread.join();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void VelodyneDriver::RegisterLIDARDataCallback(LIDARDriverDataCallback callback)
{
    m_callback = callback;
    m_running = true;
    m_callbackThread = std::thread( &VelodyneDriver::_ThreadFunc, this );
}
