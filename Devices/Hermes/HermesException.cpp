#include "hermesexception.h"

HermesException::HermesException(const char *what): m_pWhat(what) {
}

HermesException::~HermesException() throw()
{

}

const char* HermesException::what()
{
    return m_pWhat;
}
