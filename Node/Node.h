///*

//   rpg::node is a simple peer-to-peer networking tool providing pub/sub and rpc functionality.

//   As ever, the goal for node it so be simple, yet powerful.

//   As an example, here is how the API can be used:

//   rpg::node n1;
//   n1.init( "NodeName1" );
//   n1.advertise( "LeftImage" );
//   n1.publish( "LeftImage", data ); // data needs to be a zmq msg or google pb

//   ...

//   rpg::node n2;
//   n2.init( "NodeName2" );
//   n2.subscribe( "NodeName2/LeftImage" ); // allow explicit subscription
//   n2.receive( "NodeName1/LeftImage", data ); // blocking call

//   ...

//   rpg::node n3;
//   n3.init("name3");
//   n3.provide_rpc( "MyFunc", MyFunc );

//   ...

//   n1.call_rpc( "name3/MyFunc", arg, res );

// */


#ifndef _NODE_H_
#define _NODE_H_

#include <stdio.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <signal.h>

#include <map>
#include <string>
#include <sstream>
#include <iostream>

#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/crc.hpp>  // for boost::crc_32_type

#include "zmq.hpp"
#include <zmq.h>

#include <google/protobuf/message.h>

#include <NodeMessages.pb.h>
#include "ZeroConf.h"
#include "PrintMessage.h"


typedef boost::shared_ptr<zmq::socket_t> NodeSocket;


///////////////////////////////////////////////////////////////////////////////////////
typedef void(*FuncPtr)( google::protobuf::Message&, google::protobuf::Message&, void * );

///////////////////////////////////////////////////////////////////////////////////////
struct RPC {
    FuncPtr                     RpcFunc;
    google::protobuf::Message*  ReqMsg;
    google::protobuf::Message*  RepMsg;
    void*                       UserData;
};

namespace rpg
{
    class node;
}

// global list of all alocated nodes -- we will call _BroadcastExit on shutdown
extern std::vector<rpg::node*> g_vNodes;

namespace rpg
{
    zmq::context_t* _InitSingleton();

//    class node : public boost::mutex
    class node
    {
        public:
            ///////////////////////////////////////////////////////////////////////////////////////
            /// node constructor you can call wo initialization.  user MUST call init at some point.
            node()
            {
                m_dHeartbeatWaitTresh = 10;
                m_dGetResourceTableMaxWait = 0.8;// maximum of timeout. Default value is 0.1.
                m_uResourceTableVersion = 1;
                m_bInitDone = false;
                m_dSendRecvMaxWait=1; // maximum wait time for publish and recv method, millionsecond---------------------------------------------------------------------------add by luma
            }

            ///////////////////////////////////////////////////////////////////////////////////////
            ~node()
            {
                _BroadcastExit();
            }

            ///////////////////////////////////////////////////////////////////////////////////////
            void set_verbocity( int nLevel )
            {
                rpg::PrintHandlerSetErrorLevel( nLevel );
            }

            ///////////////////////////////////////////////////////////////////////////////////////
            bool init(
                    std::string  sNodeName //< Input: Node identifier
                    )
            {
                m_sNodeName = sNodeName;
                m_sHostIP = _GetHostIP();
                m_pContext = _InitSingleton();

                // install signal handlers so we can exit and tell other nodes about it..
                signal( SIGHUP       , NodeSignalHandler );
                signal( SIGINT       , NodeSignalHandler );
                signal( SIGQUIT      , NodeSignalHandler );
                signal( SIGILL       , NodeSignalHandler );
                signal( SIGTRAP      , NodeSignalHandler );
                signal( SIGABRT      , NodeSignalHandler );
                signal( SIGFPE       , NodeSignalHandler );
                signal( SIGKILL      , NodeSignalHandler );
                signal( SIGBUS       , NodeSignalHandler );
                signal( SIGSEGV      , NodeSignalHandler );
                signal( SIGSYS       , NodeSignalHandler );
                signal( SIGPIPE      , NodeSignalHandler );
                signal( SIGALRM      , NodeSignalHandler );
                signal( SIGTERM      , NodeSignalHandler );
                signal( SIGURG       , NodeSignalHandler );
                signal( SIGSTOP      , NodeSignalHandler );
                signal( SIGTSTP      , NodeSignalHandler );
                signal( SIGCONT      , NodeSignalHandler );
                signal( SIGCHLD      , NodeSignalHandler );
                signal( SIGTTIN      , NodeSignalHandler );
                signal( SIGTTOU      , NodeSignalHandler );
                signal( SIGIO        , NodeSignalHandler );
                signal( SIGXCPU      , NodeSignalHandler );
                signal( SIGXFSZ      , NodeSignalHandler );
                signal( SIGVTALRM    , NodeSignalHandler );
                signal( SIGPROF      , NodeSignalHandler );
                signal( SIGUSR1      , NodeSignalHandler );
                signal( SIGUSR2      , NodeSignalHandler );

                g_vNodes.push_back( this );

                m_bInitDone = true;
                PrintMessage( 1, "[Node] finished registering signals\n" );
                // RPC server socket
                m_pSocket = NodeSocket( new zmq::socket_t( *m_pContext, ZMQ_REP ) );
                m_nPort = _BindRandomPort( m_pSocket );

                // register with zeroconf
                if( !m_ZeroConf.RegisterService( "hermes_" + m_sNodeName, "_hermes._tcp", m_nPort ) ){
                    PrintError( "[Node] ERROR registering node '%s' with ZeroConf -- make sure the name is unique\n",
                           m_sNodeName.c_str() );
                    return false;
                }

                // register special calls for distributing the node-table
                this->provide_rpc( "Heartbeat", &_HeartbeatFunc, this );
                this->provide_rpc( "GetResourceTable", &_GetResourceTableFunc, this );
                this->provide_rpc( "SetResourceTable", &_SetResourceTableFunc, this );
                this->provide_rpc( "DeleteFromResourceTable", &_DeleteFromResourceTableFunc, this );

                boost::mutex::scoped_lock lock(m_Mutex); // careful
                m_RPCThread = boost::thread( _RPCThreadFunc, this );// run RPC listener in his own thread.
                m_HeartbeatThread = boost::thread( _HeartbeatThreadFunc, this );// run RPC listener in his own thread.

                // ask avahi who's around, and get their resource tables, also build our table
                _UpdateNodeRegistery();
                PrintMessage( 1, "[Node] finished updating node registry\n" );

                PrintMessage( 1, "[Node] '%s' started at '%s\n", m_sNodeName.c_str(), m_sHostIP.c_str() );

                // propagate changes
                _PropagateResourceTable();
                PrintMessage( 1, "[Node] finished propagating resource table\n" );

                _PrintResourceLocatorTable();

                return true;
            }

            ///////////////////////////////////////////////////////////////////////////////////////
            /// Specialization to register a remote procedure call for the
            // "int f(string)" signature.
            bool provide_rpc(
                    const std::string&       sName,   //< Input: Function name
                    int (*pFunc)(const std::string&)  //< Input: Function pointer
                    )
            {
                return provide_rpc( sName, _IntStringFunc, (void*)pFunc );
            }

            ///////////////////////////////////////////////////////////////////////////////////////
            /// Register a remote procedure call.
            template <class Req, class Rep> bool provide_rpc(
                    const std::string&       sName, //< Input: Function name
                    void (*pFunc)(Req&,Rep&,void*),	//< Input: Function pointer
                    void*  pUserData  //< Input: User data passed to the function
                    )
            {
                boost::mutex::scoped_lock lock(m_Mutex); // careful

                assert( m_bInitDone );
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
                    pRPC->UserData = pUserData;
                    m_mRpcTable[ sName ] = pRPC;
                    std::string sRpcResource = "rpc://" + m_sNodeName + "/" + sName;
                    m_mResourceTable[ sRpcResource ] = _GetAddress();
                    return true;
                }
            }

            ///////////////////////////////////////////////////////////////////////////////////////
            /// Make a remote procedure call like "node->func()".
            // This is a specialization for the "int f(string)" signature
            bool call_rpc(
                    const std::string& sRpcResource, //< Input: Remote node name and rpc method
                    const std::string& sInput,    //< Input: input to rpc method
                    int& nResult                  //< Output:
                    )
            {
                std::string sNode = _ParseNodeName( sRpcResource );
                std::string sFuncName = _ParseRpcName( sRpcResource );
                msg::String req;
                msg::Int rep;
                req.set_value( sInput );
                bool bRes = call_rpc( sNode, sFuncName, req, rep );
                nResult = rep.value();
                return bRes;
            }

            ///////////////////////////////////////////////////////////////////////////////////////
            /// Make a remote procedure call like "node->func()".
            bool call_rpc(
                    const std::string&               sRpcResource,//< Input:  node/rpc
                    const google::protobuf::Message& MsgReq,   //< Input: Protobuf message request
                    google::protobuf::Message&       MsgRep,  //< Output: Protobuf message reply
                    unsigned int                     TimeOut = 0//< Input: ms to wait for reply
                    )
            {
                std::string sNode = _ParseNodeName( sRpcResource );
                std::string sFuncName = _ParseRpcName( sRpcResource );
                return call_rpc( sNode, sFuncName, MsgReq, MsgRep, TimeOut );
            }

            ///////////////////////////////////////////////////////////////////////////////////////
            /// Make a remote procedure call like "node->func()".
            bool call_rpc(
                    const std::string&                sNode,      //< Input: Remote node name
                    const std::string&                sFuncName,  //< Input: Remote function to call
                    const google::protobuf::Message&  MsgReq,     //< Input: Protobuf message request
                    google::protobuf::Message&        MsgRep,     //< Output: Protobuf message reply
                    unsigned int                      TimeOut = 0//< Input: ms to wait for reply
                    )
            {
                assert( m_bInitDone );
                std::map<std::string,std::string>::iterator it;
                NodeSocket pSock;

                boost::mutex::scoped_lock lock(m_Mutex); // careful

                // make sure we know about this method
                std::string sRpcResource =  "rpc://"+sNode+"/"+sFuncName;
                it = m_mResourceTable.find( sRpcResource );
                if( it == m_mResourceTable.end() ){
                    // unknown method, fail
                    return false;
                }
                std::string sHostAndPort = m_mResourceTable[ sRpcResource ];

                // check if socket is already open for this host
                std::map<std::string,TimedNodeSocket>::iterator sockit;
                sockit = m_mRpcSockets.find( sNode );
                if( sockit != m_mRpcSockets.end() ) {
                    // socket is already open, lets use it
                    pSock = sockit->second.m_pSocket;
                }
                else {
                    pSock = NodeSocket( new zmq::socket_t( *m_pContext, ZMQ_REQ ) );
                    // lets connect using the socket
                    try {
                        pSock->connect( ("tcp://"+sHostAndPort).c_str() );
                    }
                    catch( zmq::error_t error ) {
                        return false;
                    }
                    m_mRpcSockets[ sNode ] = TimedNodeSocket(pSock);
                }

                // once we have pSock, we can let the resource table change. yay smart pointers
                lock.unlock();

                // TODO: what happens if this client tries to Delete himself?

                return call_rpc( pSock, sFuncName, MsgReq, MsgRep, TimeOut );
            }

            ///////////////////////////////////////////////////////////////////////////////////////
            /// Make a remote procedure call like "node->func()" -- with out node name resolution.
            // this is the main API most calls boil down to.
            bool call_rpc(
                    NodeSocket                        pSock,
                    const std::string&                sFuncName,  //< Input: Remote function to call
                    const google::protobuf::Message&  MsgReq,     //< Input: Protobuf message request
                    google::protobuf::Message&        MsgRep,     //< Output: Protobuf message reply
                    unsigned int                      nTimeoutMS = 0 //< Input: ms to wait for reply
                    )
            {
                assert( m_bInitDone );

                // prepare to append function information (clip function name size)
                std::string sFName = sFuncName;
                if( sFName.size() > 254 ) {
                    sFName.resize(254);
                }
                unsigned char n = sFName.size();

                // prepare message
                zmq::message_t ZmqReq( sizeof(n) + n + MsgReq.ByteSize() );
                std::memcpy( ZmqReq.data(), &n, sizeof(n) );
                std::memcpy( (char*)ZmqReq.data() + sizeof(n), sFName.c_str(), n );
                if( !MsgReq.SerializeToArray( (char*)ZmqReq.data() + sizeof(n) + n, MsgReq.ByteSize() ) ) {
                    // error serializing protobuf to ZMQ message
                    return false;
                }

                // send request
                pSock->setsockopt(ZMQ_SNDTIMEO,&m_dSendRecvMaxWait,sizeof(m_dSendRecvMaxWait));// -------------------------------------------------------------------------------------------------- add by luma 2

                try {
                    if( pSock->send( ZmqReq ) == false ) {
                        //if( pSock->send( ZmqReq ) != 0 ) {
                        PrintError( "Error: %s\n", strerror(errno));
                        // error sending request
                        return false;
                    }
                    }
                    catch( zmq::error_t error ) {
                        std::string sErr = error.what();
                        PrintError( " zmq->send() -- %s", sErr.c_str() );
                        return false;
                    }


                    zmq::message_t ZmqRep;
                    double dStartTime = _TicMS();

                    pSock->setsockopt(ZMQ_RCVTIMEO,&m_dSendRecvMaxWait,sizeof(m_dSendRecvMaxWait));// --------------------------------------------------------------------------------------------- add by luma 2

                     try{
                         bool bStatus = false;
                         while( bStatus == false ) {
                             if( nTimeoutMS == 0 ){ // block
                                 bStatus = pSock->recv( &ZmqRep );
                             }
                             else{
                                 bStatus = pSock->recv( &ZmqRep );
                                 double dTimeTaken = _TocMS(dStartTime);
                                 if( dTimeTaken >= nTimeoutMS ){
                                     // timeout... error receiving
                                     PrintMessage( 1, "[Node] Warning: Call timed out waiting for reply (%.2f msms > %.2f msms).", dTimeTaken, nTimeoutMS );
                                     return false;
                                 }
                                 usleep(100); // wait a bit
                             }
                         }
                     }
                    catch( zmq::error_t error ) {
                        std::string sErr = error.what();
                        PrintError( " zmq->recv() -- %s", sErr.c_str() );
                        return false;
                    }

                    if( MsgRep.ParseFromArray( ZmqRep.data(), ZmqRep.size() ) == false ) {
                        // bad protobuf format
                        return false;
                    }

                    return true;
                }


                /////////////////////////////////////////////////////////////////////////////////////
                ///  Tell all other nodes we publish this topic
                bool advertise(
                        const std::string&				sTopic        	//< Input: Topic name
                        )
                {
                    boost::mutex::scoped_lock lock(m_Mutex); // don't let anyone touch the shared resource table...

                    assert( m_bInitDone );
                    std::string sTopicResource = "topic://" + m_sNodeName + "/" + sTopic;

                    // check if socket is already open for this topic
                    std::map<std::string,NodeSocket>::iterator it;
                    it = m_mTopicSockets.find( sTopicResource );
                    if( it != m_mTopicSockets.end() ) {
                        PrintMessage( 1, "[Node] ERROR: resource socket is already open, return false\n" );
                        return false;
                    }
                    else {
                        // no socket open.. lets open a new one
                        // check if port is already in use
                        NodeSocket pSock( new zmq::socket_t( *m_pContext, ZMQ_PUB ) );
                        int nPort = _BindRandomPort( pSock );
                        std::string sAddr = _GetAddress( m_sHostIP, nPort );
                        PrintMessage( 1, "[Node] Publishing topic '%s' on %s\n",
                                sTopic.c_str(), sAddr.c_str() );

                        // updae node table and socket table
//                       lock();
//                        boost::mutex::scoped_lock lock(m_Mutex); // careful
                        m_mResourceTable[ sTopicResource ] = sAddr;
                        m_mTopicSockets[ sTopicResource ] = pSock;
//                        lock.unlock();
//                        unlock();

                        // propagate changes (quorum write)
                        _PropagateResourceTable();
                        //                        _PrintResourceLocatorTable();
//                        printf("advertise complete\n");
                        return true;
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// Send data
                bool publish(
                        const std::string&					sTopic,     //< Input: Topic to write to
                        const google::protobuf::Message&	Msg			//< Input: Message to send
                        )
                {
                    assert( m_bInitDone );
                    std::string sTopicResource = "topic://" + m_sNodeName + "/" + sTopic;

                    // check if socket is already open for this topic
                    std::map<std::string,NodeSocket >::iterator it;
                    it = m_mTopicSockets.find( sTopicResource );
                    if( it == m_mTopicSockets.end() ) {
                        // no socket found
                        return false;
                    } else {
                            NodeSocket pSock = it->second;
                            zmq::message_t ZmqMsg( Msg.ByteSize() );
                            if( !Msg.SerializeToArray( ZmqMsg.data(), Msg.ByteSize() ) ) {
                                // error serializing protobuf to ZMQ message
                                return false;
                            }

                            pSock->setsockopt(ZMQ_SNDTIMEO,&m_dSendRecvMaxWait,sizeof(m_dSendRecvMaxWait));// ----------------------------------------------------------------------------------------- add by luma
//                            double dStartTime=_TicMS();
                            try
                            {
                                if(pSock->send(ZmqMsg)==true)
                                {
//                                    printf("[Node publish] Publish Protobuf success. used time %.2f ms\n",_TocMS(dStartTime));
                                    return true;
                                }
                                else
                                {
//                                    printf("[Node publish] Publish Protobuf Fail. used time %.2f ms\n",_TocMS(dStartTime));
                                    return false;
                                }
                            }
                            catch( zmq::error_t error )
                            {
                                return false;
                            }
                     }
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// Send data.
                bool publish(
                        const std::string&				sTopic,			//< Input: Topic to write to
                        zmq::message_t&					Msg				//< Input: Message to send
                        )
                {
                    assert( m_bInitDone );
                    std::string sTopicResource = "topic://" + m_sNodeName + "/" + sTopic;

                    // check if socket is already open for this topic
                    std::map<std::string,NodeSocket>::iterator it;
                    it = m_mTopicSockets.find( sTopicResource );
                    if( it == m_mTopicSockets.end() ) {
                        // no socket found
                        return false;
                    } else {
                        NodeSocket pSock = it->second;

                        pSock->setsockopt(ZMQ_SNDTIMEO,&m_dSendRecvMaxWait,sizeof(m_dSendRecvMaxWait));// --------------------------------------------------------------------------------------------- add by luma
//                        double dStartTime=_TicMS();

                        try {
                            if(pSock->send( Msg)==true)
                            {
//                                printf("[Node Publish] Publish zmq success. used time %.2f ms. \n",_TocMS(dStartTime));
                                return true;
                            }
                            else
                            {
//                                printf("[Node Publish] Publish zmq failed. used time %.2f ms. \n",_TocMS(dStartTime));
                                return false;
                            }
                        }
                        catch( zmq::error_t error ) {
                            return false;
                        }
                    }
                }


                ///////////////////////////////////////////////////////////////////////////////////////
                /// subscribe to topic will open socket for read
                bool subscribe(
                        const std::string&	sResource  //< Input: Node resource: "NodeName/Topic"
                        )
                {
                    boost::mutex::scoped_lock lock(m_Mutex); // don't let anyone touch the shared resource table...

                    std::string	sTopicResource = "topic://" + sResource;
                    assert( m_bInitDone );
                    // check if socket is already open for this topic
                    std::map<std::string,NodeSocket>::iterator it;
                    it = m_mTopicSockets.find( sTopicResource );
                    if( it != m_mTopicSockets.end() ) {
                        PrintMessage( 1, "[Node] ERROR: subscription for that topic already exists\n" );
                        return false;
                    } else {
                        // lets find this node's IP
                        std::map < std::string, std::string >::iterator its;
                        its = m_mResourceTable.find( sTopicResource );
                        if( its == m_mResourceTable.end() ) {
                            PrintMessage( 1, "[Node] Resource '%s' not found on cache table.\n",
                                    sTopicResource.c_str() );
                            return false;
                        }
                        else {
                            // create socket
                            NodeSocket pSock( new zmq::socket_t( *m_pContext, ZMQ_SUB ) );
                            // lets connect using the socket
                            try {
                                pSock->setsockopt( ZMQ_SUBSCRIBE, NULL, 0 );
                                pSock->connect( ("tcp://" + its->second).c_str() );
                            }
                            catch( zmq::error_t error ) {
                                return false;
                            }
                            m_mTopicSockets[ sTopicResource ] = pSock;
                            return true;
                        }
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// Consume data from publisher
                bool receive(
                        const std::string& sResource, //< Input: Node resource: "NodeName/Topic"
                        google::protobuf::Message& Msg   //< Output: Message read
                        )
                {
                    std::string	sTopicResource = "topic://" + sResource;
                    assert( m_bInitDone );
                    // check if socket is already open for this topic
                    std::map<std::string,NodeSocket>::iterator it;
                    it = m_mTopicSockets.find( sTopicResource );
                    if( it == m_mTopicSockets.end() ) {
                        // no socket found
                        return false;
                    } else {
                        NodeSocket pSock = it->second;
                        zmq::message_t ZmqMsg;

                        pSock->setsockopt(ZMQ_RCVTIMEO,&m_dSendRecvMaxWait,sizeof(m_dSendRecvMaxWait));// --------------------------------------------------------------------------------------------- add by luma
//                        double dStartTime=_TicMS();
                        try {
                            if( pSock->recv( &ZmqMsg) == true )
                            {
//                                printf("[Node Receive] Receive Protobuf success. used time %.2f ms. \n",_TocMS(dStartTime));
                            }
                            else
                            {
//                                printf("[Node Receive] Receive Protobuf Fail. used time %.2f ms. \n",_TocMS(dStartTime));
                                return false;
                            }
                        }
                        catch( zmq::error_t error ) {
                            return false;
                        }

                        if( !Msg.ParseFromArray( ZmqMsg.data(), ZmqMsg.size() ) ) {
                            return false;
                        }
                        return true;
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// Consume data form publisher
                bool receive(
                        const std::string& sResource,  //< Input: Node resource: "NodeName/Topic"
                        zmq::message_t&    ZmqMsg //< Output: ZMQ Output message
                        )
                {
                    std::string	sTopicResource = "topic://" + sResource;
                    assert( m_bInitDone );
                    // check if socket is already open for this topic
                    std::map<std::string,NodeSocket>::iterator it;
                    it = m_mTopicSockets.find( sTopicResource );
                    if( it == m_mTopicSockets.end() ) {
                        // no socket found
                        return false;
                    }
                    else {
                        NodeSocket pSock = it->second;

//                        double dStartTime=_TicMS();

                        pSock->setsockopt(ZMQ_RCVTIMEO,&m_dSendRecvMaxWait,sizeof(m_dSendRecvMaxWait));// --------------------------------------------------------------------------------------------- add by luma
                        try {
                            if( pSock->recv( &ZmqMsg) == false )
                            {
//                              printf("[Node Receive] Receive zmq Fail. used time %.2f ms. \n",_TocMS(dStartTime));
                                return false;
                            }
                            else
                            {
//                                printf("[Node Receive] Receive zmq success. used time %.2f ms. \n",_TocMS(dStartTime));
                                return true;
                            }
                        }
                        catch( zmq::error_t error ) {
                            return false;
                        }
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// Figure out the network name of this machine
                const char* _GetHostIP( const std::string& sPreferredInterface = "eth" )
                {
                    // orderd list of interfaces we perfer... all so
                    // the interface matches what Zeroconf says.. this
                    // is a hack. should instead re-map what zeroconf
                    // says to some standard...
                    std::vector<std::string> vIfs;
                    vIfs.push_back( sPreferredInterface );
                    vIfs.push_back( "eth" );
                    vIfs.push_back( "en" );
                    vIfs.push_back( "wlan" );

                    struct ifaddrs *ifaddr, *ifa;
                    char host[NI_MAXHOST];

                    std::vector<std::pair<std::string,std::string> > vIfAndIP;

                    if( getifaddrs(&ifaddr) == -1 ){
                        perror("getifaddrs");
                    }

                    for( ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next ){
                        if (ifa->ifa_addr == NULL){
                            continue;
                        }
                        int s = getnameinfo( ifa->ifa_addr, sizeof(struct sockaddr_in), host,
                                NI_MAXHOST, NULL, 0, NI_NUMERICHOST );
                        if( s == 0 && ifa->ifa_addr->sa_family == AF_INET ){
                            vIfAndIP.push_back( std::pair<std::string,std::string>(ifa->ifa_name,host) );
                        }
                    }
                    freeifaddrs(ifaddr);

                    std::string sIP = "127.0.0.1";
                    std::string sIf;
                    for( size_t ii = 0; ii < vIfs.size(); ii++ ){
                        std::string sPreferredIf = vIfs[ii];
                        for( size_t n = 0; n < vIfAndIP.size(); n++ ){
                            sIf = vIfAndIP[n].first;
                            if( sIf.find(sPreferredIf) != std::string::npos ){ // found a prefered interface
                                sIP = vIfAndIP[n].second;
                            }
                        }
                    }

                    return sIP.c_str();

                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// Heartbeat, called by client _DoHeartbeat to check if he is up-to-date.
                static void _HeartbeatFunc(
                        msg::HeartbeatRequest& req,
                        msg::HeartbeatResponse& rep, void* pUserData
                        )
                {
                    ((node*)pUserData)->HeartbeatFunc( req, rep );
                }
                void HeartbeatFunc(
                        msg::HeartbeatRequest& req,
                        msg::HeartbeatResponse& rep
                        )
                {
                    rep.set_checksum( _ResourceTableCRC() );
                    rep.set_version( m_uResourceTableVersion );

                    // 1) if we are behind the remote client, do nothing
                    // -- the next RPC call from the client will give
                    // us an updated table.
                    // 2) if client is behind, send him our table (TODO: send a diff instead)
                    if( req.version() < m_uResourceTableVersion ){
                        _BuildResoruceTableMessage( *rep.mutable_resource_table() );
                    }
                    // 3) if version match, do nothing -- this is the default case
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// here we are replying to a remote node who has asked for a copy of our node table
                static void _GetResourceTableFunc(
                        msg::GetTableRequest& req,
                        msg::GetTableResponse& rep, void* pUserData
                        )
                {
                    ((node*)pUserData)->GetResourceTableFunc( req, rep );
                }
                void GetResourceTableFunc(
                        msg::GetTableRequest& req,
                        msg::GetTableResponse& rep
                        )
                {
                    std::map<std::string,std::string>::iterator it;
//                       printf("GetResourc rep.urls_size() = %d\n", (int)rep.urls_size() );
                    msg::ResourceTable* pTable = rep.mutable_resource_table(); // add the resource table to the message
                    pTable->set_version( m_uResourceTableVersion );
                    pTable->set_checksum( _ResourceTableCRC() );
                    rep.set_sender_name( m_sNodeName );
                    for( it = m_mResourceTable.begin(); it != m_mResourceTable.end(); it++ ){
                        msg::ResourceLocator* pMsg = pTable->add_urls();// add new url to end of table
                        pMsg->set_resource( it->first );
                        pMsg->set_address( it->second );
//                      printf("adding %s:%s\n", it->first.c_str(), it->second.c_str() );
                    }
                    PrintMessage( 1, "[Node] GetResourceTableFunc() called -- will send %d resouces back\n",
                            (int)rep.resource_table().urls_size() );
                    _PrintResourceLocatorTable();
                    // ok at this point the NodeTable protobuf is ready to send back to the caller
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// being asked by other nodes to remove references to them from our resource table
                static void _DeleteFromResourceTableFunc(
                        msg::DeleteFromTableRequest& req,
                        msg::DeleteFromTableResponse& rep,
                        void* pUserData
                        )
                {
                    ((node*)pUserData)->DeleteFromResourceTableFunc( req, rep );
                }
                void DeleteFromResourceTableFunc(
                        msg::DeleteFromTableRequest& req,
                        msg::DeleteFromTableResponse& rep
                        )
                {

                    boost::mutex::scoped_lock lock(m_Mutex); // careful

                    PrintMessage( 2, "[Node] DeleteFromResourceTableFunc() called by '%s'"
                            " to delete %d resources\n",  req.requesting_node_name().c_str(), (int)req.urls_to_delete_size() );
                    //                            _PrintResourceLocatorTable();
                    for( int ii = 0; ii < req.urls_to_delete_size(); ii++ ){
                        const msg::ResourceLocator& m = req.urls_to_delete(ii);
                        std::map<std::string,std::string>::iterator it;
                        it = m_mResourceTable.find( m.resource() );

                        if( it != m_mResourceTable.end() ){
                            // printf("Deleting %s\n", m.resource().c_str() );
                            m_mResourceTable.erase(it);
                        }
                        else{
                            // printf("Resource %s not found\n", m.resource().c_str() );
                        }
                    }

                    // also remove from RPC socket map.
                    std::map<std::string,TimedNodeSocket>::iterator sockit;
                    sockit = m_mRpcSockets.find( req.requesting_node_name() );
                    if( sockit != m_mRpcSockets.end() ){
                        m_mRpcSockets.erase(sockit); // this is ok, because no one else can touch this stuff right now...
                    }

                    // also remove from m_mTopicSockets, ---------------------------------------------------------------------------------------------------add by Luma
                    std::map<std::string,NodeSocket > ::iterator topicit;
                    for(topicit=m_mTopicSockets.begin();topicit!=m_mTopicSockets.end();topicit++)
                    {
                        if (topicit->first.find(req.requesting_node_name()) != std::string::npos)
                        {
                            m_mTopicSockets.erase(topicit);
                        }
                    }

                    // ok at this point we have updated our node table
                    _PrintResourceLocatorTable();
                    //_PrintRpcSockets();
                }


                ///////////////////////////////////////////////////////////////////////////////////////
                /// here we are receiving an update to our node table from a remote node
                static void _SetResourceTableFunc( msg::SetTableRequest& req, msg::SetTableResponse& rep, void* pUserData )
                {
                    ((node*)pUserData)->SetResourceTableFunc( req, rep );
                }
                void SetResourceTableFunc( msg::SetTableRequest& req, msg::SetTableResponse& rep )
                {
                    boost::mutex::scoped_lock lock(m_Mutex); // careful

                    PrintMessage( 1, "[Node] SetResourceTableFunc() called by '%s' to share %d resources\n",
                            req.requesting_node_name().c_str(), (int)req.resource_table().urls_size() );

                    // open RPC socket if necessary -- e.g. if the client is new:
                    _ConnectRpcSocket( req.requesting_node_name(), req.requesting_node_addr() );

                    // verify that the new clients resource table is newer than ours
                    if( req.resource_table().version() <= m_uResourceTableVersion ){
                        PrintMessage( 1, "[Node] WARNING '%s' sent an outdated resource table with version %d\n",
                                req.requesting_node_name().c_str(), req.resource_table().version() );
                    }

                    // else all good, just adopt the remote resource table version
                    m_uResourceTableVersion = req.resource_table().version();

                    //  update the rest of the table:
                    for( int ii = 0; ii < req.resource_table().urls_size(); ii++ ){
                        const msg::ResourceLocator& m = req.resource_table().urls(ii);
                        m_mResourceTable[ m.resource() ] = m.address();
                    }

                    // set appropraite reply to the remote calling client
                    rep.set_version( m_uResourceTableVersion );
                    rep.set_checksum( _ResourceTableCRC() );

                    // ok at this point we have updated our node table
                    _PrintResourceLocatorTable();
                    printf("set resource table complete..\n");
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// Send a heartbeat message to peers.
                static void _HeartbeatThreadFunc( node *pThis ) { pThis->HeartbeatThreadFunc(); }
                void HeartbeatThreadFunc()
                {
                    std::string sAddr = _GetAddress(m_sHostIP,m_nPort);
                    PrintMessage( 9, "[Node] Starting Heartbeat Thread at %s\n", sAddr.c_str() );
                    while( 1 ){
                        // ping all nodes we haven't heard from recently
                        boost::mutex::scoped_lock lock(m_Mutex);
                        std::map<std::string,TimedNodeSocket>::iterator it;
                        for( it = m_mRpcSockets.begin() ; it != m_mRpcSockets.end(); it++ ) {
                            double dElapsedTime = _Toc(it->second.m_dLastHeartbeatTime);
                            if( dElapsedTime > m_dHeartbeatWaitTresh ){ // haven't heard from this guy in a while
                                PrintMessage( 9, "[Node] sending heartbeat to '%s'\n", it->first.c_str() );
                                double dTimeout = m_dGetResourceTableMaxWait*1e3;
                                msg::HeartbeatRequest hbreq;
                                msg::HeartbeatResponse hbrep;
                                hbreq.set_checksum( _ResourceTableCRC() );
                                hbreq.set_version( m_uResourceTableVersion );
                                if( !call_rpc( it->second.m_pSocket, "Heartbeat", hbreq, hbrep, dTimeout ) ){
                                    PrintMessage( 1,  "[Node] WARNING: haven't heard from '%s' for %.2f ms seconds\n",
                                            it->first.c_str(), _Tic()-it->second.m_dLastHeartbeatTime );
                                    continue;
                                }

                                it->second.m_dLastHeartbeatTime = _Tic();

                                // if remote resource table is newer than ours...
                                if( hbrep.version() > m_uResourceTableVersion ){
                                    // ...update resource table with data from remote node:
                                    for( int ii = 0; ii < hbrep.resource_table().urls_size(); ii++ ){
                                        msg::ResourceLocator r = hbrep.resource_table().urls(ii);
                                        m_mResourceTable[ r.resource() ] = r.address();
                                    }
                                    m_uResourceTableVersion = hbrep.version();
                                }
                                // if our resource table is more current, send it out:
                                if( hbrep.version() < m_uResourceTableVersion ){
                                    msg::SetTableRequest streq;
                                    msg::SetTableResponse strep;
                                    _BuildResoruceTableMessage( *streq.mutable_resource_table() );
                                    streq.set_requesting_node_name( m_sNodeName );
                                    streq.set_requesting_node_addr( _GetAddress() );
                                    if( !call_rpc( it->second.m_pSocket, "SetResourceTable", streq, strep ) ){
                                        PrintError( "[Node] ERROR: sending resource table\n");
                                    }
                                }

                                /// serious error!
                                if( _ResourceTableCRC() != hbrep.checksum() ){
                                    PrintError( "[Node] ERROR: node '%s' checksum error (%d != %d)\n",
                                            it->first.c_str() , _ResourceTableCRC(), hbrep.checksum() );
                                }
                            }
                        }
                        lock.unlock();
                        /// wait a bit
                        usleep( 10000 );
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                static void _RPCThreadFunc( node *pThis ) { pThis->RPCThreadFunc(); }
                void RPCThreadFunc()
                {
                    std::string sAddr = _GetAddress(m_sHostIP,m_nPort);
                    PrintMessage( 1, "[Node] Starting RPC server at %s\n", sAddr.c_str() );

                    while( 1 ) {

                        // wait for request
                        zmq::message_t ZmqReq;

                        try{
                            if( m_pSocket->recv( &ZmqReq ) == false ) {
                                // error receiving
                                PrintMessage( 1, "[Node] WARNING! RPC listener was terminated.\n" );
                                exit(1);
                            }
                        }
                        catch( zmq::error_t error ) {
                            std::string sErr = error.what();
                            PrintError( " zmq->recv() -- %s", sErr.c_str() );
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

                        it = m_mRpcTable.find( FuncName );
                        if( it != m_mRpcTable.end() ) {

                            RPC* pRPC = it->second;
                            FuncPtr Func = pRPC->RpcFunc;
                            google::protobuf::Message* Req = pRPC->ReqMsg;
                            google::protobuf::Message* Rep = pRPC->RepMsg;

                            Rep->Clear();

                            if( !Req->ParseFromArray( (char*)(ZmqReq.data()) + PbOffset, PbByteSize ) )
                            {
                                continue;
                            }

                            // call function
                            (*Func)( *(Req),*(Rep), pRPC->UserData );

                            // send reply
                            zmq::message_t ZmqRep( Rep->ByteSize() );
                            if( !Rep->SerializeToArray( ZmqRep.data(), Rep->ByteSize() ) ) {
                                // error serializing protobuf to ZMQ message
                            }
                            m_pSocket->send( ZmqRep );
                        } else {
                            // send empty reply
                            zmq::message_t ZmqRep(0);
                            m_pSocket->send( ZmqRep );
                        }
                    }
                }

                /////////////////////////////////////////////////////////////////////////////////////// --------------------------------------------------------------------add by luma
                // this function return the name of all connect client
                std::vector<std::string>  GetSubscribeClientName()
                {
                    boost::mutex::scoped_lock lock(m_Mutex); // careful

                    std::vector<std::string> vClientNames;
                    std::map<std::string,NodeSocket >::iterator it;
                    for(it=m_mTopicSockets.begin();it!=m_mTopicSockets.end();it++)
                    {
                        if(it->first.find("StateKeeper")==std::string::npos)
                        {
                            std::string sSubString=it->first.substr(8,it->first.size());
                            vClientNames.push_back(sSubString.substr(0,sSubString.find("/")));
                        }
                    }
                    return vClientNames;
                }
                ///////////////////////////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////////////////////////

            private:

                ///////////////////////////////////////////////////////////////////////////////////////
                /// Build a protobuf containing all the resources we know of, and the CRC.
                msg::ResourceTable _BuildResoruceTableMessage(
                        msg::ResourceTable& t
                        )
                {
                    std::map<std::string,std::string>::iterator it;
                    for( it = m_mResourceTable.begin(); it != m_mResourceTable.end(); it++ ){
                        msg::ResourceLocator* pMsg = t.add_urls();// add new url to end of table
                        pMsg->set_resource( it->first );
                        pMsg->set_address( it->second );
                    }
                    t.set_checksum( _ResourceTableCRC() );
                    t.set_version( m_uResourceTableVersion );
                    return t;
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// Checksum of the ResourceTable (used by nodes to check they are up-to-date).
                uint32_t _ResourceTableCRC()
                {
                    boost::crc_32_type crc;
                    std::string vNames;
                    std::map<std::string,std::string>::iterator it;
                    for( it = m_mResourceTable.begin() ; it != m_mResourceTable.end(); it++ ) {
                        vNames += it->first;
                    }
                    crc.process_bytes( vNames.c_str(), vNames.size() );
                    return crc.checksum();
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// Here we loop through our list of known ndoes and call their SetNodeTable functions,
                // passing them our node table.
                void _PropagateResourceTable()
                {
                    std::map<std::string,TimedNodeSocket>::iterator it;
                    msg::SetTableRequest req;
                    msg::SetTableResponse rep;

                    _BuildResoruceTableMessage( *req.mutable_resource_table() );
                    req.set_requesting_node_name( m_sNodeName );
                    req.set_requesting_node_addr( _GetAddress() );

                    for( it = m_mRpcSockets.begin(); it != m_mRpcSockets.end(); it++ ){

                        if( it->second.m_pSocket == m_pSocket ){
                            continue; // don't send to self
                        }

                        //printf(" calling node '%s' SetResourceTable rpc method\n", it->first.c_str() );
                        if( !call_rpc( it->second.m_pSocket, "SetResourceTable", req, rep ) ){
                            PrintError( "Error: sending resource table\n");
                        }
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// Ask zeroconf for list of nodes on the network and ask each for their resource
                //  table.
                void _UpdateNodeRegistery()
                {
                    std::vector<ZeroConfRecord> vRecords;
                    vRecords = m_ZeroConf.BrowseForServiceType( "_hermes._tcp" );
                    PrintMessage( 1, "[Node] looking for hermes.tcp \n" );
                    while( vRecords.size() == 0 ){
                        PrintMessage( 1, "[Node] waiting for _hermes._tcp to appear in ZeroConf registery\n");
                        vRecords = m_ZeroConf.BrowseForServiceType( "_hermes._tcp" );
                        usleep(1000);
                    }

                    // Report all the nodes registered with Avahi:
                    std::vector<ZeroConfURL> urls;
                    for( size_t ii = 0; ii < vRecords.size(); ii++ ){
                        ZeroConfRecord r = vRecords[ii];
                        std::vector<ZeroConfURL> new_urls
                            = m_ZeroConf.ResolveService( r.m_sServiceName, r.m_sRegType );
                        for(size_t jj = 0; jj < new_urls.size(); jj++ ){
                            urls.push_back( new_urls[jj] );
                            ZeroConfURL url = new_urls[jj];
                        }
                    }

                    PrintMessage( 1, "[Node] found %d URLs for nodes\n", (int)urls.size() );
                    for( size_t ii = 0; ii < urls.size(); ii++ ){
                        ZeroConfURL url = urls[ii];
                        std::string sAddr = _GetAddress( url.m_sHost, url.m_nPort );
                        std::string sMyAddr = _GetAddress();
                        if( sAddr == sMyAddr ){ // don't send a message to ourselves...
                            PrintMessage( 1, "     node at %s:%d (my URL)\n", url.Host(), url.Port() );
                        }
                        else{
                            PrintMessage( 1, "     node at %s:%d\n", url.Host(), url.Port() );
                        }
                    }

                    unsigned int uLatestResourceTableVersion = 0;

                    for( size_t ii = 0; ii < urls.size(); ii++ ){
                        ZeroConfURL url = urls[ii];

                        std::string sAddr = _GetAddress( url.m_sHost, url.m_nPort );
                        std::string sMyAddr = _GetAddress();
                        if( sAddr == sMyAddr ){ // don't send a message to ourselves...
                            continue;
                        }

                        // connect to the service, if we can
                        std::map<std::string,std::string>::iterator it = m_mResourceTable.find( sAddr );
                        if( it == m_mResourceTable.end() ) { // not in registery, connect to this node
                            std::string sZmqAddr = _ZmqAddress( url.Host(), url.Port() );
                            NodeSocket pSock = NodeSocket( new zmq::socket_t(*m_pContext,ZMQ_REQ) );
                            try {
                                pSock->connect( sZmqAddr.c_str() );
                            }
                            catch( zmq::error_t error ) {
                                std::string sErr = error.what();
                                PrintError( "ERROR: zmq->connect() -- %s", sErr.c_str() );

                            }
                            PrintMessage( 1, "[Node] '%s' connected to remote node: %s:%d\n",
                                    m_sNodeName.c_str(), url.Host(), url.Port() );

                            /// update our registery
                            msg::GetTableResponse rep; // we get this back
                            msg::GetTableRequest req;
                            req.set_requesting_node_name( m_sNodeName );
                            req.set_requesting_node_addr( _GetAddress() );
                            double dTimeout = m_dGetResourceTableMaxWait*1e3; // wait half a second, max
                            if( !call_rpc( pSock, "GetResourceTable", req, rep, dTimeout ) ){
                                PrintError("Error: Failed when asking for remote node name\n");
                            }
                            PrintMessage( 1, "[Node]    heard back from '%s' about %d resources\n",
                                    rep.sender_name().c_str(), rep.resource_table().urls_size() );

                            // ok, now we have the nodes name to record his socket
                            m_mRpcSockets[ rep.sender_name() ] = TimedNodeSocket(pSock);
                            // push these into our resource table:
                            for( int ii = 0; ii < rep.resource_table().urls_size(); ii++ ){
                                msg::ResourceLocator r = rep.resource_table().urls(ii);
                                m_mResourceTable[ r.resource() ] = r.address();
                            }

                            // keep track of incoming resource table versions
                            if( uLatestResourceTableVersion == 0 ){ // catch the initial case
                                uLatestResourceTableVersion = rep.resource_table().version();
                            }
                            if( uLatestResourceTableVersion != rep.resource_table().version() ){
                                // error, in this case, we have received different resource table
                                // versions from at least two different nodes... there is a conflict.
                                PrintMessage( 1, "[Node] WARNING resource table conflict\n" );
                            }
                        }
                        m_uResourceTableVersion = uLatestResourceTableVersion + 1;
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// print the node table
                void _PrintResourceLocatorTable()
                {
                    PrintMessage( 1, "--------------- RESOURCE TABLE (ver %d, crc %X) --------------\n",
                            m_uResourceTableVersion, _ResourceTableCRC() );
                    PrintMessage( 1, "%-40sURL\n", "RESOURCE");
                    std::map<std::string,std::string>::iterator it;
                    for( it = m_mResourceTable.begin() ; it != m_mResourceTable.end(); it++ ) {
                        PrintMessage( 1, "%-40s%s\n", it->first.c_str(), it->second.c_str() );
                    }

                    PrintMessage( 1, "\n\n");

                }

                ///////////////////////////////////////////////////////////////////////////////////////
                /// print the rpc sockets
                void _PrintRpcSockets()
                {
                    PrintMessage( 1, "----------- RPC SOCKETS OPEN ----------------\n");
                    std::map<std::string,TimedNodeSocket>::iterator it;
                    for( it = m_mRpcSockets.begin() ; it != m_mRpcSockets.end(); it++ ) {
                        PrintMessage( 1, "%s\n", it->first.c_str() );
                    }
                    PrintMessage( 1, "\n\n");
                }


                ///////////////////////////////////////////////////////////////////////////////////////
                int _BindRandomPort( NodeSocket& pSock )
                {
                    std::ostringstream address;
                    int nPort = 5555;
                    address << "tcp://*:" << nPort;
                    while(1) {
                        try {
//                            pSock->setsockopt(ZMQ_RCVTIMEO,&m_dSendRecvMaxWait,sizeof(m_dSendRecvMaxWait));// --------------------------------------------------------------------------------------------- add by luma
//                            pSock->setsockopt(ZMQ_SNDTIMEO,&m_dSendRecvMaxWait,sizeof(m_dSendRecvMaxWait));// --------------------------------------------------------------------------------------------- add by luma

                            pSock->bind( address.str().c_str() );
                            break;
                        }
                        catch( zmq::error_t error ) {
                            address.str("");
                            nPort++;
                            address << "tcp://*:" << nPort;
                        }
                    }
                    return nPort;
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                std::string _GetAddress()
                {
                    return _GetAddress( m_sHostIP, m_nPort );
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                std::string _GetAddress( const std::string& sHostIP, const int nPort )
                {
                    std::ostringstream address;
                    address << sHostIP << ":" << nPort;
                    return address.str();
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                std::string _ZmqAddress()
                {
                    return _ZmqAddress( m_sHostIP, m_nPort );
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                std::string _ZmqAddress( const std::string& sHostIP, const int nPort )
                {
                    std::ostringstream address;
                    address << "tcp://"<< sHostIP << ":" << nPort;
                    return address.str();
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                double _Tic()
                {
                    struct timeval tv;
                    gettimeofday(&tv, 0);
                    return tv.tv_sec  + 1e-6 * (tv.tv_usec);
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                double _Toc( double dSec )
                {
                    return _Tic() - dSec;
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                double _TicMS()
                {
                    struct timeval tv;
                    gettimeofday(&tv, 0);
                    double dTSec = tv.tv_sec  + 1e-6 * (tv.tv_usec);
                    return dTSec*1e3;
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                double _TocMS( double dMS )
                {
                    return _TicMS() - dMS;
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                ///
                std::string _ParseNodeName( const std::string& sResource )
                {
                    std::string sNodeName;
                    size_t found = sResource.find_first_of('/');
                    if( found != std::string::npos ){
                        sNodeName = sResource.substr(0,found);
                    }
                    return sNodeName;
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                ///
                std::string _ParseRpcName( const std::string& sResource )
                {
                    std::string sRpcName;
                    size_t found = sResource.find_first_of('/');
                    if( found != std::string::npos ){
                        sRpcName = sResource.substr(found+1);
                    }
                    return sRpcName;
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                // ensure we have a connection
                void _ConnectRpcSocket( const std::string& sNodeName, const std::string& sNodeAddr )
                {
                    // open RPC socket if necessary:
                    std::map<std::string,TimedNodeSocket>::iterator sockit;
                    sockit = m_mRpcSockets.find( sNodeName );
                    if( sockit == m_mRpcSockets.end() ){
                        std::string sZmqAddr = "tcp://" + sNodeAddr;
                        NodeSocket pSock = NodeSocket( new zmq::socket_t(*m_pContext,ZMQ_REQ) );
                        try {
                            pSock->connect( sZmqAddr.c_str() );
                        }
                        catch( zmq::error_t error ) {
                            std::string sErr = error.what();
                            PrintError( "Error zmq->connect() -- %s", sErr.c_str() );
                        }
//                        PrintMessage( 1, "Connected to remote node: %s\n", sZmqAddr.c_str() );

                        // record the new connection
                        m_mRpcSockets[ sNodeName ] = TimedNodeSocket(pSock);
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                static void NodeSignalHandler( int nSig )
                {

                    for( size_t ii = 0; ii < g_vNodes.size(); ii++ ){
                        g_vNodes[ii]->_BroadcastExit();
                    }
                    switch( nSig ){
                        case SIGINT : PrintMessage( 1, "[Node] caught SIGINT\n"); break;
                        case SIGTERM: PrintMessage( 1, "[Node] caught SIGTERM\n"); break;
                        case SIGSTOP: PrintMessage( 1, "[Node] caught SIGSTOP\n"); break;
                        case SIGSEGV: PrintMessage( 1, "[Node] caught SIGSEGV\n"); break;
                    }
                    printf("exit success.");
                    exit(-1);
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                void _BroadcastExit()
                {
//                    boost::mutex::scoped_lock lock(m_Mutex); // careful ------------------------------------------------------------- we should not use lock here because of the following call_rpc method. (rpc method also use this lock)

                    // collect all resources we provide
                    msg::DeleteFromTableRequest req;
                    msg::DeleteFromTableResponse rep;

                    req.set_requesting_node_name( m_sNodeName );
                    req.set_requesting_node_addr( _GetAddress() );
                    std::map<std::string,std::string>::iterator it;
                    for( it = m_mResourceTable.begin(); it != m_mResourceTable.end(); it++ ){
                        if( it->first.find( "rpc://"+m_sNodeName ) != std::string::npos
                                || it->first.find( "topic://"+m_sNodeName ) != std::string::npos ){
                            msg::ResourceLocator* pMsg = req.add_urls_to_delete();
                            pMsg->set_resource( it->first );
                            pMsg->set_address( it->second );
                        }
                    }

                    //_PrintRpcSockets();

                    // ask all known nodes to remove us:
                    std::map<std::string,TimedNodeSocket> tmp = m_mRpcSockets;
                    std::map<std::string,TimedNodeSocket>::iterator sockit;
                    for( sockit = tmp.begin(); sockit != tmp.end(); sockit++ ){
                        if( sockit->second.m_pSocket == m_pSocket ){
                            continue;
                        }
                        PrintMessage( 1, "[Node '%s']  Calling DeleteFromResource to remove %d resources\n",
                                m_sNodeName.c_str(), (int)req.urls_to_delete_size() );
                        if( !call_rpc( sockit->second.m_pSocket, "DeleteFromResourceTable", req, rep ) ){
                            PrintMessage( 1, "ERROR: calling remote DeleteFromResourceTable\n" );
                        }
                    }
                }

                ///////////////////////////////////////////////////////////////////////////////////////
                // signature for simplified int f(void) style functions
                static void _IntStringFunc( msg::String& sStr, msg::Int& nInt, void* pUserData )
                {
                    int (*pFunc)(const std::string&) = (int (*)(const std::string&))pUserData;
                    nInt.set_value( (*pFunc)(sStr.value()) );
                }



                ///////////////////////////////////////////////////////////////////////////////////////
                    private:

                /// used to time socket communications
                struct TimedNodeSocket
                {
                    TimedNodeSocket(){};
                    TimedNodeSocket( NodeSocket pSocket )
                    {
                        m_pSocket = pSocket;
                    }

                    TimedNodeSocket& operator=( const TimedNodeSocket& RHS )
                    {
                        if (this == &RHS){
                            return *this;
                        }
                        m_pSocket = RHS.m_pSocket;
                        m_dLastHeartbeatTime = RHS.m_dLastHeartbeatTime;
                        return *this;
                    }
                    NodeSocket  m_pSocket;
                    double      m_dLastHeartbeatTime;
                };

                // NB a "resource" is a nodename, node/rpc or node/topic URL
                std::map<std::string,std::string>       m_mResourceTable; // resource to host:port map
                std::map<std::string,NodeSocket >       m_mTopicSockets;  // resource to socket map
                std::map<std::string,TimedNodeSocket >  m_mRpcSockets;    // nodename to socket map

                std::map<std::string,RPC*>              m_mRpcTable;      // function to RPC structs map

                ZeroConf                                m_ZeroConf;       // for automatic server discovery
                NodeSocket                              m_pSocket;        // global socket, (RPC too)
                zmq::context_t*                         m_pContext;       // global context
                std::string                             m_sHostIP;        // node's machine IP
                int                                     m_nPort;          // node's RPC port
                std::string                             m_sNodeName;      // node unique name
                bool                                    m_bInitDone;      // initialized?
                boost::thread                           m_RPCThread;      // Thread for handling rpc
                boost::thread                           m_HeartbeatThread;// Thread for handling heartbeats
                double                                  m_dGetResourceTableMaxWait;  // Max timeout wait
                double                                  m_dHeartbeatWaitTresh;
                unsigned int                            m_uResourceTableVersion;
                boost::mutex                            m_Mutex;
                int                                     m_dSendRecvMaxWait;// send and receive message max wait-------------------------------------add by luma
                }; // end class Node
                }  // end namespace rpg

#endif
