#ifndef _HERMES_H_
#define _HERMES_H_

#include <vector>
#include <zmq.hpp>
#include "ZmqEndpoint.h"

class Hermes {
	    public:
            Hermes();
    		unsigned int AddEndpoint();
    		void RemoveEndpoint(int id);
    		void Send(int id, char *pData, int nData);
    		bool Receive(int id,char *pBuffer,char nBufferLength,int &nReceived); //this will NEVER block, since if it blocks the entire ZmqWrapper class will block

    		unsigned int GetEndpointCount() { return m_vEndpoints.size(); }

	    private:
    		zmq::context_t*  			m_pContext;
    		std::vector<ZmqEndpoint> 	m_vEndpoints;
            // some vector of protobufs
};

#endif
