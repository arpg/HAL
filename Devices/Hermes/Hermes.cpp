#include "Hermes.h"


Hermes::Hermes()
{

}

unsigned int Hermes::AddEndpoint() {
	ZmqEndpoint zEndpoint;

	m_vEndpoints.push_back(zEndpoint);
	return m_vEndpoints.size() - 1;
}
