#include "Hermes.h"

Hermes::Hermes()
{
    m_pContext = new zmq::context_t(1);;
}


Hermes::~Hermes()
{
    std::map < std::string, zmq::socket_t* >::iterator it;
    
    for( it = m_mEndpoints.begin(); it != m_mEndpoints.end(); it++ ) {
        delete (*it).second;
        m_mEndpoints.erase( it );
    }
}


bool Hermes::AddEndpoint( std::string Node, EndpointType eType  ) {
    std::pair< std::map < std::string, zmq::socket_t* >::iterator, bool > ret;
    
    zmq::socket_t* Socket = new zmq::socket_t( *m_pContext, eType );
    
    ret = m_mEndpoints.insert ( std::pair < std::string, zmq::socket_t* > ( Node, Socket ) );
    return ret.second;
}


bool Hermes::RemoveEndpoint( std::string Node ) {
    std::map < std::string, zmq::socket_t* >::iterator ep;
    
    ep = m_mEndpoints.find( Node );
    if( ep != m_mEndpoints.end() ) {
        delete (*ep).second;
        m_mEndpoints.erase( ep );
        return true;
    } else {
        return false;
    }
}


bool Hermes::Bind( std::string Node, std::string Host ) {
    std::map < std::string, zmq::socket_t* >::iterator ep;
    
    ep = m_mEndpoints.find( Node );
    if( ep == m_mEndpoints.end() ) {
        return false;
    } else {
        zmq::socket_t* pSocket = (*ep).second;
        try {
            pSocket->bind( Host.c_str() );
        }
        catch( zmq::error_t error ) {
            return false;
        }
        return true;
    }
    
}


bool Hermes::Connect( std::string Node, std::string Host ) {
    std::map < std::string, zmq::socket_t* >::iterator ep;
    
    ep = m_mEndpoints.find( Node );
    if( ep == m_mEndpoints.end() ) {
        return false;
    } else {
        zmq::socket_t* pSocket = (*ep).second;
        try {
            pSocket->connect( Host.c_str() );
        }
        catch( zmq::error_t error ) {
            return false;
        }
        return true;
    }
    
}


bool Hermes::Send( std::string Node, char *pData, unsigned int nData ) {
    std::map < std::string, zmq::socket_t* >::iterator ep;
    
    ep = m_mEndpoints.find( Node );
    if( ep == m_mEndpoints.end() ) {
        return false;
    } else {
        zmq::socket_t* pSocket = (*ep).second;
        zmq::message_t message( nData );
        std::memcpy( message.data(), pData, nData );
        pSocket->send( message );
        return true;
    }
    
}


