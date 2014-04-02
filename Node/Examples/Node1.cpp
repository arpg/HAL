/*
   This node provides some RPC methods and also publishes some data.

   Start this guy then try Node2.

 */

#include "ExampleMessage.pb.h"
#include <Node/Node.h>

#include <stdio.h>
#include <chrono>
#include <thread>

using namespace std;

string gText = "Hello";

///////////////////////////////////////////////////////////
// nice simple remote procedure call
int SimpleRpcMethod( const std::string& sStr )
{
    printf("int SimpleRpcPMethod( \"%s\" ) called\n", sStr.c_str() );
    return 1;
}

///////////////////////////////////////////////////////////
void RpcMethod( Msg& Req, Msg& Rep, void* )
{
    // print value we got from network
    printf("\n--- Incoming message! The value within: %s ---\n", Req.value().c_str());

    // do something
    gText = Req.value();

    // prepare reply message
    Rep.set_value("Value set!");
}

///////////////////////////////////////////////////////////
int main( int, char** )
{
    string sNodeName = "Node1";

    // initialize node
    hal::node n;

    n.set_verbosity(2); // be a bit noisy
    n.init( sNodeName );

    // set up a publisher
    n.advertise( "Node1Topic" );
    cout << "Node1Topic advertized." << endl;
    n.advertise("Node1Topic2");

    // set up a remote procedure call
    n.provide_rpc( "RpcMethod", &RpcMethod, NULL );

    // easy api
    n.provide_rpc( "SimpleRpcMethod", &SimpleRpcMethod );

    while(1){
        Msg PubMsg;
        PubMsg.set_value( gText );
        n.publish( "Node1Topic", PubMsg );
        n.publish( "Node1Topic2", PubMsg );
        cout << "Sending '" << gText << "'" << endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
