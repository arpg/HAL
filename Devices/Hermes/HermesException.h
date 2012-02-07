#ifndef HERMESEXCEPTION_H
#define HERMESEXCEPTION_H

#include <exception>

class HermesException : public exception
{
private:
    const char *m_pWhat;
public:
    HermesException(const char *what);
    virtual ~HermesException()  throw() ;
    virtual const char* what();
};

#endif // HERMESEXCEPTION_H
