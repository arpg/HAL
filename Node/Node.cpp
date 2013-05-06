#include "Node.h"

std::vector<rpg::node*> g_vNodes;

namespace rpg
{
    zmq::context_t* _InitSingleton()
    {
        setenv("AVAHI_COMPAT_NOWARN","1",1); // not ideal! we should apparently port away from avahi-compat... ug
        static zmq::context_t* pContext = NULL;
        if( pContext == NULL ){
            pContext = new zmq::context_t(1);
        }
        return pContext;
    }

}  // end namespace rpg


