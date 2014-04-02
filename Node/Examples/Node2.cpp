/*
   Exampel calling rpc methods in Node1, and subscribing to Node1 topic

   Run Node1 then run this guy.

 */

#include <Node/Node.h>
#include "ExampleMessage.pb.h"

#include <stdio.h>
#include <chrono>
#include <thread>

void Node1Topic2Callback(std::shared_ptr<google::protobuf::Message> mMsg) {
  std::shared_ptr<Msg> msg;
  msg = std::dynamic_pointer_cast<Msg>(mMsg);
  printf("In Callback Got '%s'\n", msg->value().c_str());//'%s'.\n", mMsg.value().c_str());
}

int main()
{

    hal::node n;
    n.set_verbosity( 2 ); // make some noise on errors
    n.init("Node2");

    // subscribe to Node1's topic
    if( n.subscribe( "Node1/Node1Topic" ) == false ) {
        printf("Error subscribing to Node1Topic.\n");
    }

    // this is using callback.
    if( n.subscribe( "Node1/Node1Topic2" ) == false ) {
      printf("Error subscribing to Node1Topic2.\n");
    }
    std::function<void(std::shared_ptr<google::protobuf::Message>)> func_ptr =
        Node1Topic2Callback;
    if( !n.RegisterCallback<Msg>("Node1/Node1Topic2", Node1Topic2Callback))
      printf("Topic ain't there, can't register callback\n");
    printf("Registered I guess\n");

    if( n.advertise( "Node2Topic" ) == false ) {
        printf("Error subscribing to topic.\n");
    }

    unsigned int nCount = 0;

    // test the "easy" api -- call Node1->SimpleRpcMethod("test")
    int res;
    n.call_rpc( "Node1/SimpleRpcMethod", "test", res );
    printf("Got %d back from 'Node1/SimpleRpcMethod'\n", res);

    // now make a mistake
    n.call_rpc( "Node1/SimpleRpcMethod2", "test", res );

    while(1) {
        Msg mMsg;
        n.receive( "Node1/Node1Topic", mMsg ); // blocking call
        printf("Got '%s'.\n", mMsg.value().c_str());

        nCount++;
        if( nCount == 3 ) {
            printf("--- Sending RPC message! ---\n");
            mMsg.set_value( "Bye!" );
            n.call_rpc( "Node1/RpcMethod", mMsg, mMsg );
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
