/*
 *  \file PrintMessage.cpp
 *
 *  Printing utility functions.
 *
 *  $Id$
 */ 

#ifndef _RPGPP_PRINT_MESSAGE_H_
#define _RPGPP_PRINT_MESSAGE_H_

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#ifndef PrintMessage
/// PrintMessage is a macro for efficiency. Calls amount to an
// extra if, and they can be turned off completely if necessary.
#   ifdef SILENCE_RPG_PRINT_HANDLER
#       define PrintMessage( nErrorLevel, ... )
#   else
#       define PrintMessage( nErrorLevel, ... ) \
           if( ((int)nErrorLevel) <= rpg::PrintHandlerGetErrorLevel() ) { \
               rpg::PrintVaradicMessage( nErrorLevel, __VA_ARGS__ ); \
           }
#   endif
#endif

namespace rpg
{
    int _GetSetErrorLevel( int nLevel = -1 );
    int PrintHandlerGetErrorLevel();
    void PrintHandlerSetErrorLevel( int nLevel );
    void PrintVaradicMessage( int nVerbosityLevel, const char *sMsg, ... );
    void PrintError( const char *sMsg, ... );
    bool PassVerbosityLevel( int nLevel );
    std::string StringFormat( const char *fmt, ... );

    template <class T> inline std::string ValToStr( const T val )
    {
        std::ostringstream ss;
        ss << std::setiosflags(std::ios::fixed) << std::setprecision(9) << val;
        return ss.str();
    }

    template <class T> inline T StrToVal( const std::string& sStr )
    {
        std::stringstream ss;
        ss << sStr;
        T t;
        ss >> t;
        return t;
    }
}

#endif 

