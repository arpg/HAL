//
//   hal::node is a simple peer-to-peer networking tool providing
//   pub/sub and rpc functionality.  As ever, the goal for node it so
//   be simple, yet powerful.  As an example, here is how the API can
//   be used:
//
//   hal::node n1;
//   n1.init("NodeName1");
//   n1.advertise("LeftImage");
//   n1.publish("LeftImage", data); // data needs to be a zmq msg or google pb
//
//   ...
//
//   hal::node n2;
//   n2.init("NodeName2");
//   n2.subscribe("NodeName2/LeftImage"); // allow explicit subscription
//   n2.receive("NodeName1/LeftImage", data); // blocking call
//
//   ...
//
//   hal::node n3;
//   n3.init("name3");
//   n3.provide_rpc("MyFunc", MyFunc);
//
//   ...
//
//   n1.call_rpc("name3/MyFunc", arg, res);
//
#ifndef _HAL_NODE_NODE_H_
#define _HAL_NODE_NODE_H_

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <zmq.h>
#include <google/protobuf/message.h>

#include <Node/NodeConfig.h>
#include <Node/NodeMessages.pb.h>
#include <Node/zmq.hpp>
#include <Node/ZeroConf.h>

typedef std::shared_ptr<zmq::socket_t> NodeSocket;

typedef void(*FuncPtr)(google::protobuf::Message&,
                       google::protobuf::Message&,
                       void *);

typedef std::function<void(google::protobuf::Message&,
                           google::protobuf::Message&,
                           void *)> RPCFunction;

struct RPC {
  RPCFunction RpcFunc;
  google::protobuf::Message* ReqMsg;
  google::protobuf::Message* RepMsg;
  void* UserData;
};

namespace hal { class node; }

// global list of all alocated nodes -- we will call _BroadcastExit on shutdown
extern std::vector<hal::node*> g_vNodes;

namespace hal {

struct TimedNodeSocket;

// The port to use by default if we can't autodiscover
#define NODE_DEFAULT_PORT 1776U

zmq::context_t* _InitSingleton();

class node {
 public:
  /// node constructor you can call wo initialization.  user MUST call
  /// init at some point.
  ///
  /// @param use_autodiscovery Use Avahi to autodiscover other nodes
  ///        (if available)
  explicit node(bool use_auto_discovery = true);
  virtual ~node();
  void set_verbosity(int nLevel);

  ///
  /// Input: Node identifier
  bool init(std::string node_name);

  /// Specialization to register a remote procedure call for the
  // "int f(string)" signature.
  bool provide_rpc(const std::string& sName,   //< Input: Function name
                   int (*pFunc)(const std::string&)  //< Input: Function pointer
                   );

  /// Register a remote procedure call.
  template <class Req, class Rep>
  bool provide_rpc(const std::string& sName, //< Input: Function name
                   void (*pFunc)(Req&, Rep&, void*), //< Input: Function pointer
                   void* pUserData //< Input: User data passed to the function
                   ) {
    std::lock_guard<std::mutex> lock(mutex_); // careful

    assert(init_done_);
    // check if function with that name is already registered
    auto it = rpc_table_.find(sName);
    if (it != rpc_table_.end()) return false;

    RPC* pRPC = new RPC;
    pRPC->RpcFunc = (FuncPtr)pFunc;
    pRPC->ReqMsg = new Req;
    pRPC->RepMsg = new Rep;
    pRPC->UserData = pUserData;
    rpc_table_[sName] = pRPC;
    std::string rpc_resource = "rpc://" + node_name_ + "/" + sName;
    resource_table_[rpc_resource] = _GetAddress();
    return true;
  }

  /// Make a remote procedure call like "node->func()".
  // This is a specialization for the "int f(string)" signature
  bool call_rpc(
      const std::string& rpc_resource, //< Input: Remote node name and rpc method
      const std::string& input,    //< Input: input to rpc method
      int& result                  //< Output:
                );

  /// Make a remote procedure call like "node->func()".
  bool call_rpc(
      const std::string&               rpc_resource,//< Input:  node/rpc
      const google::protobuf::Message& msg_req,   //< Input: Protobuf message request
      google::protobuf::Message&       msg_rep,  //< Output: Protobuf message reply
      unsigned int                     time_out = 0//< Input: ms to wait for reply
                );

  /// Make a remote procedure call like "node->func()".
  bool call_rpc(
      const std::string&                node_name,      //< Input: Remote node name
      const std::string&                function,  //< Input: Remote function to call
      const google::protobuf::Message&  msg_req,     //< Input: Protobuf message request
      google::protobuf::Message&        msg_rep,     //< Output: Protobuf message reply
      unsigned int                      time_out = 0//< Input: ms to wait for reply
                );

  /// Make a remote procedure call like "node->func()" -- with out node name resolution.
  // this is the main API most calls boil down to.
  bool call_rpc(NodeSocket socket,
                std::shared_ptr<std::mutex> socket_mutex,
                const std::string& function,  //< Input: Remote function to call
                const google::protobuf::Message& msg_req,     //< Input: Protobuf message request
                google::protobuf::Message& msg_rep,     //< Output: Protobuf message reply
                unsigned int nTimeoutMS = 0 //< Input: ms to wait for reply
                );

  ///  Tell all other nodes we publish this topic
  /// Input: Topic name
  bool advertise(const std::string& sTopic);

  /// Send data
  bool publish(const std::string& sTopic,     //< Input: Topic to write to
               const google::protobuf::Message&  Msg //< Input: Message to send
               );

  /// Send data.
  bool publish(const std::string& sTopic, //< Input: Topic to write to
               zmq::message_t& Msg //< Input: Message to send
               );

  /// subscribe to topic will open socket for read
  /// @param [in] Node resource: "NodeName/Topic"
  bool subscribe(const std::string& sResource);

  /// Consume data from publisher
  bool receive(const std::string& sResource, //< Input: Node resource: "NodeName/Topic"
               google::protobuf::Message& Msg   //< Output: Message read
               );

  /// Consume data form publisher
  bool receive(const std::string& sResource, //< Input: Node resource: "NodeName/Topic"
               zmq::message_t& ZmqMsg //< Output: ZMQ Output message
               );

  /// Figure out the network name of this machine
  std::string _GetHostIP(const std::string& sPreferredInterface = "eth");

  /// Heartbeat, called by client _DoHeartbeat to check if he is up-to-date.
  static void _HeartbeatFunc(msg::HeartbeatRequest& req,
                             msg::HeartbeatResponse& rep,
                             void* pUserData);

  void HeartbeatFunc(msg::HeartbeatRequest& req,
                     msg::HeartbeatResponse& rep);

  /// here we are replying to a remote node who has asked for a copy
  /// of our node table
  static void _GetResourceTableFunc(msg::GetTableRequest& req,
                                    msg::GetTableResponse& rep,
                                    void* pUserData);

  void GetResourceTableFunc(msg::GetTableRequest& req,
                            msg::GetTableResponse& rep);

  /// being asked by other nodes to remove references to them from our
  /// resource table
  static void _DeleteFromResourceTableFunc(msg::DeleteFromTableRequest& req,
                                           msg::DeleteFromTableResponse& rep,
                                           void* pUserData);

  void DeleteFromResourceTableFunc(msg::DeleteFromTableRequest& req,
                                   msg::DeleteFromTableResponse& rep);

  /// here we are receiving an update to our node table from a remote node
  static void _SetResourceTableFunc(msg::SetTableRequest& req,
                                    msg::SetTableResponse& rep,
                                    void* pUserData);

  void SetResourceTableFunc(msg::SetTableRequest& req,
                            msg::SetTableResponse& rep);

  void HeartbeatThread();
  void RPCThread();

  // this function return the name of all connect client
  std::vector<std::string> GetSubscribeClientName();

  /// Connect to another node at a given hostname string and port
  ///
  /// The GetTableResponse will be filled out with the response from
  /// the connected node. This includes the Node name to be used for
  /// RPC nodes.
  ///
  /// Returns whether the connection was successful.
  bool ConnectNode(const std::string& host, uint16_t port,
                   msg::GetTableResponse* rep);

  /// Disconnect from the desired node
  void DisconnectNode(const std::string& node_name);

  bool using_auto_discovery() const {
    return use_auto_discovery_;
  }

  void set_using_auto_discovery(bool use_auto) {
    use_auto_discovery_ = use_auto;
  }

  // Set the port for this Node to use. Can only be called BEFORE init().
  void set_bind_port(uint16_t port) {
    if (initialized_) {
      std::cerr << "Only call set_bind_port before init()!" << std::endl;
      abort();
    }
    port_ = port;
    use_fixed_port_ = true;
  }

  // Get the port this node is listening on.
  uint16_t bind_port() {
    return port_;
  }

 private:
  /// Build a protobuf containing all the resources we know of, and the CRC.
  msg::ResourceTable _BuildResourceTableMessage(msg::ResourceTable& t);

  /// Checksum of the ResourceTable (used by nodes to check they are
  /// up-to-date).
  uint32_t _ResourceTableCRC();

  /// Here we loop through our list of known ndoes and call their
  // SetNodeTable functions, passing them our node table.
  void _PropagateResourceTable();

  /// Ask zeroconf for list of nodes on the network and ask each for
  //  their resource table.
  void _UpdateNodeRegistery();

  /// print the node table
  void _PrintResourceLocatorTable();

  /// print the rpc sockets
  void _PrintRpcSockets();

  // Bind the given socket on a certain port
  bool _BindPort(uint16_t port, const NodeSocket& socket);

  // Find an available port number and bind to it.
  int _BindRandomPort(const NodeSocket& socket);

  std::string _GetAddress() const;
  std::string _GetAddress(const std::string& sHostIP, const int nPort) const;

  std::string _ZmqAddress() const;
  std::string _ZmqAddress(const std::string& sHostIP, const int nPort) const;

  double _Tic();
  double _Toc(double dSec);
  double _TicMS();
  double _TocMS(double dMS);
  ///
  std::string _ParseNodeName(const std::string& sResource);
  ///
  std::string _ParseRpcName(const std::string& sResource);

  // ensure we have a connection
  void _ConnectRpcSocket(const std::string& node_name_name,
                         const std::string& node_nameAddr);

  static void NodeSignalHandler(int nSig);

  void _BroadcastExit();

  // signature for simplified int f(void) style functions
  static void _IntStringFunc(msg::String& sStr,
                             msg::Int& nInt, void* pUserData);

  /** Get the mutex associated with the socket connected to a certain node */
  std::shared_ptr<std::mutex> rpc_mutex(const std::string& node);

  void BuildDeleteFromTableRequest(msg::DeleteFromTableRequest* msg) const;

 private:
  // NB a "resource" is a nodename, node/rpc or node/topic URL
  // resource to host:port map
  std::map<std::string, std::string> resource_table_;

  // resource to socket map
  std::map<std::string, NodeSocket> topic_sockets_;

  // nodename to socket map
  std::map<std::string, TimedNodeSocket> rpc_sockets_;

  // Each socket should only be used by one thread at a time
  std::map<std::string, std::shared_ptr<std::mutex> > rpc_mutex_;

  // function to RPC structs map
  std::map<std::string, RPC*> rpc_table_;

  // for automatic server discovery
  ZeroConf zero_conf_;

  // global socket, (RPC too)
  NodeSocket socket_;

  // global context
  zmq::context_t* context_;

  // node's machine IP
  std::string host_ip_;

  // node's RPC port
  uint16_t port_ = NODE_DEFAULT_PORT;

  // node unique name
  std::string node_name_;

  // initialized?
  bool init_done_;

  // Thread for handling rpc
  std::thread rpc_thread_;

  // Thread for handling heartbeats
  std::thread heartbeat_thread_;

  // Max timeout wait
  double get_resource_table_max_wait_;
  double heartbeat_wait_thresh_;

  // Timeout w/o heartbeat before death is declared in seconds.
  double heartbeat_death_timeout_ = 10.0;
  unsigned int resource_table_version_;
  mutable std::mutex mutex_;

  // send and receive message max wait
  int send_recv_max_wait_ = 3000;

  // Should we use autodiscovery to find other nodes
  bool use_auto_discovery_ = true;

  // Has a port number been set for this Node to use
  bool use_fixed_port_ = false;

  // Has this Node been initialized yet?
  bool initialized_ = false;

  int debug_level_ = 0;

  bool exiting_ = false;
};

}  // end namespace hal

#endif  // _HAL_NODE_NODE_H_
