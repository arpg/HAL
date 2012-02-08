#ifndef _HERMES_H_
#define _HERMES_H_

#include <map>
#include <utility>
#include <zmq.hpp>
#include <google/protobuf/message.h>

enum EndpointType
{
    // this matches ZMQ's defines.. perhaps not needed?
    EP_PUBLISH   = 1,
    EP_SUBSCRIBE = 2,
    EP_REQUEST   = 3,
    EP_REPLY     = 4
};

class Hermes {
	    public:
                Hermes();
                ~Hermes();
                bool AddEndpoint( std::string Node, EndpointType eType );
                bool RemoveEndpoint( std::string Node );
                bool Bind( std::string Node, std::string Host );
                bool Connect( std::string Node, std::string Host );
                bool Send( std::string Node, char *pData, unsigned int nData );
    		unsigned int Receive( std::string Node, char *pBuffer, unsigned int nBufferSize );

    		unsigned int GetEndpointCount() { return m_mEndpoints.size(); }

	    private:
		EndpointType                                            m_eType;
    		zmq::context_t*                                         m_pContext;
                std::map < std::string, zmq::socket_t* >                m_mEndpoints;
                std::map < std::string, google::protobuf::Message* pb > m_vMessages;
};

#endif