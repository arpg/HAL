/*
   \file Node.h

   Lovely minimal networking system with RPC and Pub-Sub support using google
   protocal buffers and ZermMQ. 

   $Id$
 */

#ifndef _NODE_H_
#define _NODE_H_

#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/thread.hpp>
#include <zmq.hpp>
#include <google/protobuf/message.h>
#include <typeinfo>

namespace rpg
{
    class Node
    {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        typedef void(*FuncPtr)( google::protobuf::Message&, google::protobuf::Message& );


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        struct RPC {
            FuncPtr                     RpcFunc;
            google::protobuf::Message*  ReqMsg;
            google::protobuf::Message*  RepMsg;
        };

        public:

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Node(
                int nPort = 1337       //< Input: RPC Port.. randomized in the future
            )
        {
            // init context
            m_pContext = new zmq::context_t(1);
            m_pRpcSocket = new zmq::socket_t( *m_pContext, ZMQ_REP );
            std::ostringstream address;
            address << "tcp://*:" << nPort;
            std::cout << "[Node] RPC listener at " << address.str() << std::endl;
            m_pRpcSocket->bind( address.str().c_str() );
            m_mHosts[ address.str() ] = m_pRpcSocket;
            // run listener in a different thread...
            boost::thread RPCThread( RPCThreadFunc, this );
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ~Node()
        {
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        static void RPCThreadFunc( Node *pThis )
        {
            while( 1 ) {
                // wait for request
                zmq::message_t ZmqReq;
                if( !pThis->m_pRpcSocket->recv( &ZmqReq ) ) {
                    // error receiving
                    exit(1);
                }
                // obtain "header" which contains function name
                unsigned char FuncNameSize;
                memcpy( &FuncNameSize, ZmqReq.data(), sizeof(FuncNameSize) );
                std::string FuncName( (char*)(ZmqReq.data()) + sizeof(FuncNameSize), FuncNameSize );

                // prepare reply message
                unsigned int PbOffset = sizeof(FuncNameSize) + FuncNameSize;
                unsigned int PbByteSize = ZmqReq.size() - PbOffset;

                // look-up function
                std::map < std::string, RPC* >::iterator it;

                it = pThis->m_mRpcTable.find( FuncName );
                if( it != pThis->m_mRpcTable.end() ) {

                    RPC* pRPC = it->second;
                    FuncPtr Func = pRPC->RpcFunc;
                    google::protobuf::Message* Req = pRPC->ReqMsg;
                    google::protobuf::Message* Rep = pRPC->RepMsg;

                    if( !Req->ParseFromArray( (char*)ZmqReq.data() + PbOffset, PbByteSize ) ) {
                        continue;
                    }

                    // call function
                    (*Func)( *(Req),*(Rep) );

                    // send reply
                    zmq::message_t ZmqRep( Rep->ByteSize() );
                    if( !Rep->SerializeToArray( ZmqRep.data(), Rep->ByteSize() ) ) {
                        // error serializing protobuf to ZMQ message
                    }
                    pThis->m_pRpcSocket->send( ZmqRep );
                } else {
                    // send empty reply
                    zmq::message_t ZmqRep(0);
                    pThis->m_pRpcSocket->send( ZmqRep );
                }
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        template <class Req, class Rep>
            bool Register(
                    const std::string& sName,		//< Input: Function name
                    void(*pFunc)(Req&,Rep&)			//< Input: Function pointer
                    )
            {
                // check if function with that name is already registered
                std::map < std::string, RPC* >::iterator it;
                it = m_mRpcTable.find( sName );
                if( it != m_mRpcTable.end() ) {
                    return false;
                }
                else {
                    RPC* pRPC = new RPC;
                    pRPC->RpcFunc = (FuncPtr)pFunc;
                    pRPC->ReqMsg = new Req;
                    pRPC->RepMsg = new Rep;
                    m_mRpcTable[ sName ] = pRPC;
                    return true;
                }
            }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Call(
                const std::string& sHost,                   //< Input: Remote host address <IP>:<PORT>
                const std::string& sFuncName,               //< Input: Remote function to call
                const google::protobuf::Message& MsgReq,    //< Input: Protobuf message containing request
                google::protobuf::Message& MsgRep           //< Output: Protobuf message holding reply
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator it;
            zmq::socket_t* pSock;

            // check if socket is already open for this host
            it = m_mHosts.find( sHost );
            if( it != m_mHosts.end() ) {
                // socket is already open, lets use it
                pSock = it->second;
            } else {
                // socket is not open, lets open one
                pSock = new zmq::socket_t( *m_pContext, ZMQ_REQ );

                // lets connect using the socket
                try {
                    pSock->connect( ("tcp://" + sHost).c_str() );
                }
                catch( zmq::error_t error ) {
                    // oops, an error occurred lets rollback
                    delete pSock;
                    return false;
                }
                m_mHosts[ sHost ] = pSock;
            }
            // prepare to append function information
            std::string FuncName = sFuncName;
            if( sFuncName.size() > 254 ) {
                FuncName.resize(254);
            }
            unsigned char FuncNameSize = FuncName.size();

            // prepare message
            zmq::message_t ZmqReq( sizeof(FuncNameSize) + FuncNameSize + MsgReq.ByteSize() );
            std::memcpy( ZmqReq.data(), &FuncNameSize, sizeof(FuncNameSize) );
            std::memcpy( (char*)ZmqReq.data() + sizeof(FuncNameSize), FuncName.c_str(), FuncNameSize );
            if( !MsgReq.SerializeToArray( (char*)ZmqReq.data() + sizeof(FuncNameSize) + FuncNameSize, MsgReq.ByteSize() ) ) {
                // error serializing protobuf to ZMQ message
                return false;
            }

            // send request
            if( !pSock->send( ZmqReq ) ) {
                // error sending request
                return false;
            }

            // wait reply
            zmq::message_t ZmqRep;
            if( !pSock->recv( &ZmqRep ) ) {
                // error receiving
                return false;
            }
            if( !MsgRep.ParseFromArray( ZmqRep.data(), ZmqRep.size() ) ) {
                // bad protobuf format
                return false;
            }
            return true;
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Publish(
                const std::string& sTopic,      	//< Input: Topic name
                unsigned int nPort              	//< Input: 
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator it;

            // check if socket is already open for this topic
            it = m_mTopics.find( sTopic );
            if( it != m_mTopics.end() ) {
                // socket is already open, return false
                return false;
            } else {
                // no socket open.. lets open a new one

                // check if port is already in use
                std::ostringstream address;
                address << "tcp://*:" << nPort;
                it = m_mHosts.find( address.str() );
                if( it != m_mHosts.end() ) {
                    // port is in use, return false
                    return false;
                }

                // create socket
                zmq::socket_t* pSock = new zmq::socket_t( *m_pContext, ZMQ_PUB );
                try {
                    std::cout << "[Node] Publishing topic '" << sTopic << "' on " << address.str() << std::endl;
                    pSock->bind(address.str().c_str());
                }
                catch( zmq::error_t error ) {
                    // oops, an error occurred lets rollback
                    delete pSock;
                    return false;
                }
                m_mHosts[ address.str() ] = pSock;
                m_mTopics[ sTopic ] = pSock;
                return true;
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Write(
                const std::string& sTopic,          	//< Input: Topic to write to
                const google::protobuf::Message& Msg  	//< Input: Message to send
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator it;

            // check if socket is already open for this topic
            it = m_mTopics.find( sTopic );
            if( it == m_mTopics.end() ) {
                // no socket found
                return false;
            }
            else {
                zmq::socket_t* pSock = it->second;
                zmq::message_t ZmqMsg( Msg.ByteSize() );
                if( !Msg.SerializeToArray( ZmqMsg.data(), Msg.ByteSize() ) ) {
                    // error serializing protobuf to ZMQ message
                    return false;
                }
                return pSock->send( ZmqMsg );
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Subscribe(
                const std::string& sTopic,      //< Input: Topic to subscribe to
                const std::string& sHost        //< Input: for now, host info.. later, just the topic
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator it;

            // check if socket is already open for this topic
            it = m_mTopics.find( sTopic );
            if( it != m_mTopics.end() ) {
                // subscription for that topic already exists
                return false;
            }
            else {
                // check if host+port is already in use
                it = m_mHosts.find( sHost );
                if( it != m_mHosts.end() ) {
                    // port is in use, return false
                    return false;
                }
                // create socket
                zmq::socket_t* pSock = new zmq::socket_t( *m_pContext, ZMQ_SUB );

                // lets connect using the socket
                try {
                    pSock->setsockopt( ZMQ_SUBSCRIBE, NULL, 0 );
                    pSock->connect( ("tcp://" + sHost).c_str() );
                }
                catch( zmq::error_t error ) {
                    // oops, an error occurred lets rollback
                    delete pSock;
                    return false;
                }
                m_mHosts[ sHost ] = pSock;
                m_mTopics[ sTopic ] = pSock;
                return true;
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Read(
                const std::string& sTopic,             	//< Input: Topic to read
                google::protobuf::Message& Msg  	//< Output: Message read
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator it;

            // check if socket is already open for this topic
            it = m_mTopics.find( sTopic );
            if( it == m_mTopics.end() ) {
                // no socket found
                return false;
            }
            else {
                zmq::socket_t* pSock = it->second;
                zmq::message_t ZmqMsg;
                if( !pSock->recv( &ZmqMsg, ZMQ_NOBLOCK ) ) {
                    return false;
                }
                if( !Msg.ParseFromArray( ZmqMsg.data(), ZmqMsg.size() ) ) {
                    return false;
                }
                return true;
            }
        }


        private:

        zmq::context_t*                         m_pContext;     // global context

        std::map< std::string, zmq::socket_t* > m_mHosts;       // map of hosts + endpoints

        // Variables for Topics (PubSubs)
        std::map< std::string, zmq::socket_t* > m_mTopics;      // map of topics + endpoints

        // Variables for RPC (ReqRep)
        zmq::socket_t*                          m_pRpcSocket;   // global RPC socket
        std::map< std::string, RPC* >           m_mRpcTable;    // map of functions + RPC structs

    };

}

#endif
