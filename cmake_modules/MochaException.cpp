/*
 * MochaException.cpp
 *
 *  Created on: Feb 6, 2012
 *      Author: nima
 */

#include "MochaException.h"

namespace Mochaccino {

MochaException::MochaException(const char *what): m_pWhat(what) {
}

MochaException::~MochaException() throw()
{

}

const char* MochaException::what()
{
	return m_pWhat;
}

} /* namespace Mochaccino */
