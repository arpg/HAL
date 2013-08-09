#pragma once

// hack to enable sleep_for (GCC < 4.8)
#define _GLIBCXX_USE_NANOSLEEP

#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <thread>
#include <chrono>

#include <zmq.hpp>

#include <google/protobuf/message.h>

namespace rpg
{

    class Node
    {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        typedef void(*FuncPtr)( google::protobuf::Message&, google::protobuf::Message&, void * );


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        struct RPC {
            FuncPtr                     RpcFunc;
            google::protobuf::Message*  ReqMsg;
            google::protobuf::Message*  RepMsg;
            void *                      UserData;
        };


    public:

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /*



        Node( const Node& rhs )
        {
            this->m_pContext   = rhs.m_pContext;
            this->m_mHosts     = rhs.m_mHosts;
            this->m_mTopics    = rhs.m_mTopics;
            this->m_pRpcSocket = rhs.m_pRpcSocket;
            this->m_mRpcTable  = rhs.m_mRpcTable;
        }
        */

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Node()
        {
            // no rpc version... not good
            m_pContext = new zmq::context_t(1);
            m_pRpcSocket = NULL;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Node(
            int nPort       //< Input: RPC Port.. randomized in the future
            )
        {
            // init context
            try {
                m_pContext = new zmq::context_t(1);
                m_pRpcSocket = NULL;
                m_pRpcSocket = new zmq::socket_t( *m_pContext, ZMQ_REP );
            }
            catch( zmq::error_t error ) {
                std::cerr << "Node: Exception caught " << error.what() << std::endl;
            }
            std::ostringstream address;
            address << "tcp://*:" << nPort;
            std::cout << "[Node] RPC listener at " << address.str() << std::endl;

            try {
                m_pRpcSocket->bind( address.str().c_str() );
            }
            catch( zmq::error_t error ) {
                std::cerr << "Node: Exception caught " << error.what() << std::endl;
            }
            m_mHosts[ address.str() ] = m_pRpcSocket;
            // run listener in a different thread...
            m_RPCThread = std::thread( &RPCThreadFunc, this );
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ~Node()
        {
            if( m_RPCThread.joinable() ) {
                m_RPCThread.join();
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        static void RPCThreadFunc( Node *pThis )
        {
            while( 1 ) {
                // wait for request
                zmq::message_t ZmqReq;
                try {
                    if( pThis->m_pRpcSocket->recv( &ZmqReq ) == false ) {
                        // error receiving
                        exit(1);
                    }
                }
                catch( zmq::error_t error ) {
                    std::cerr << "Node: Exception caught " << error.what() << std::endl;
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
                    (*Func)( *(Req),*(Rep), pRPC->UserData );

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
                const std::string& sName,				//< Input: Function name
                void(*pFunc)(Req&,Rep&,void*),			//< Input: Function pointer
                void * userData							//< Input: User data passed to the function
                )
        {
            // check if we have RPC enabled
            if( !m_pRpcSocket ) {
                std::cout << "[Node] Error. No RPC listener is running!" << std::endl;
                return false;
            }
            // check if function with that name is already registered
            std::map < std::string, RPC* >::iterator it;
            it = m_mRpcTable.find( sName );
            if( it != m_mRpcTable.end() ) {
                return false;
            } else {
                RPC* pRPC = new RPC;
                pRPC->RpcFunc = (FuncPtr)pFunc;
                pRPC->ReqMsg = new Req;
                pRPC->RepMsg = new Rep;
                pRPC->UserData = userData;
                m_mRpcTable[ sName ] = pRPC;
                return true;
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Call(
                const std::string& sHost,                   //< Input: Remote host address <IP>:<PORT>
                const std::string& sFuncName,               //< Input: Remote function to call
                const google::protobuf::Message& MsgReq,    //< Input: Protobuf message containing request
                google::protobuf::Message& MsgRep,          //< Output: Protobuf message holding reply
                unsigned int TimeOut = 100					//< Input: Number of miliseconds to wait for reply
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
            if( pSock->send( ZmqReq, ZMQ_NOBLOCK ) == false ) {
                // error sending request
                // delete socket -- see "Lazy pirate ZMQ"
                pSock->close();
                it = m_mHosts.find( sHost );
                m_mHosts.erase(it);
                return false;
            }

            // wait reply
            unsigned int nCount = 0;
            zmq::message_t ZmqRep;
            while( pSock->recv( &ZmqRep, ZMQ_NOBLOCK ) == false && nCount < TimeOut ) {
                nCount++;
                std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ) );
            }
            if( nCount == TimeOut ) {
                // timeout... error receiving
                // delete socket -- see "Lazy pirate ZMQ"
                pSock->close();
                it = m_mHosts.find( sHost );
                m_mHosts.erase(it);
                return false;
            }
            if( MsgRep.ParseFromArray( ZmqRep.data(), ZmqRep.size() ) == false ) {
                // bad protobuf format
                return false;
            }
            return true;
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Publish(
                const std::string& sTopic,      	//< Input: Topic name
                unsigned int nPort              	//< Input: for now port to publish, later it is random
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
            } else {
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
        bool Write(
                const std::string&	sTopic,          	//< Input: Topic to write to
                zmq::message_t&		Msg					//< Input: Message to send
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator it;

            // check if socket is already open for this topic
            it = m_mTopics.find( sTopic );
            if( it == m_mTopics.end() ) {
                // no socket found
                return false;
            } else {
                zmq::socket_t* pSock = it->second;
                return pSock->send( Msg );
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
            } else {
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
                const std::string& sTopic,          //< Input: Topic to read
                google::protobuf::Message& Msg  	//< Output: Message read
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator it;

            // check if socket is already open for this topic
            it = m_mTopics.find( sTopic );
            if( it == m_mTopics.end() ) {
                // no socket found
                return false;
            } else {
                zmq::socket_t* pSock = it->second;
                zmq::message_t ZmqMsg;
                if( pSock->recv( &ZmqMsg, ZMQ_NOBLOCK ) == false ) {
                    // nothing to read
                    return false;
                }
                if( !Msg.ParseFromArray( ZmqMsg.data(), ZmqMsg.size() ) ) {
                    return false;
                }
                return true;
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ReadBlocking(
                const std::string& sTopic,          //< Input: Topic to read
                google::protobuf::Message& Msg  	//< Output: Message read
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator it;

            // check if socket is already open for this topic
            it = m_mTopics.find( sTopic );
            if( it == m_mTopics.end() ) {
                // no socket found
                return false;
            } else {
                zmq::socket_t* pSock = it->second;
                zmq::message_t ZmqMsg;
                if( pSock->recv( &ZmqMsg ) == false ) {
                    // nothing to read
                    return false;
                }
                if( !Msg.ParseFromArray( ZmqMsg.data(), ZmqMsg.size() ) ) {
                    return false;
                }
                return true;
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Read(
                const std::string& sTopic,          //< Input: Topic to read
                zmq::message_t& ZmqMsg				//< Output: ZMQ Output message
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator it;

            // check if socket is already open for this topic
            it = m_mTopics.find( sTopic );
            if( it == m_mTopics.end() ) {
                // no socket found
                return false;
            } else {
                zmq::socket_t* pSock = it->second;
//                zmq::message_t ZmqMsg(pOut, nSize, NULL, NULL);
                if( pSock->recv( &ZmqMsg, ZMQ_NOBLOCK ) == false ) {
                    // nothing to read
                    return false;
                }
                return true;
            }
        }


    private:

        std::thread                             m_RPCThread;

        zmq::context_t*                         m_pContext;     // global context

        std::map< std::string, zmq::socket_t* > m_mHosts;       // map of hosts + endpoints

        // Variables for Topics (PubSubs)
        std::map< std::string, zmq::socket_t* > m_mTopics;      // map of topics + endpoints

        // Variables for RPC (ReqRep)
        zmq::socket_t*                          m_pRpcSocket;   // global RPC socket
        std::map< std::string, RPC* >           m_mRpcTable;    // map of functions + RPC structs

    };

}
