#include "Hermes.h"


Hermes::Hermes()
{

}

unsigned int Hermes::AddEndpoint() {
	ZmqEndpoint zEndpoint;

	// do some more stuff

	m_vEndpoints.push_back(zEndpoint);
	return m_vEndpoints.size() - 1;
}
