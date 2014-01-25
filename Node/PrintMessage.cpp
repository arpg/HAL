/*
 *  \file PrintMessage.cpp
 *
 *  Printing utility functions.
 *
 *  $Id$
 */

#include "PrintMessage.h"

#include <string>
#include <vector>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

namespace hal {

int _GetSetErrorLevel( int nLevel ) {
  static int nPrintHandlerGlobalErrorLevel = 0;
  //  if nLevel is GE 0, must be a "Set" call
  if( nLevel >= 0 ){
    nPrintHandlerGlobalErrorLevel = nLevel;
  }
  return nPrintHandlerGlobalErrorLevel;
}

int PrintHandlerGetErrorLevel() {
  return hal::_GetSetErrorLevel();
}

void PrintHandlerSetErrorLevel( int nLevel ) {
  hal::_GetSetErrorLevel( nLevel );
}

void PrintVaradicMessage( int nVerbosityLevel, const char *sMsg, ... ) {
  if( nVerbosityLevel <= PrintHandlerGetErrorLevel() ){
    if( sMsg == NULL ){
      return;
    }

    char *pBuffer = NULL;
    va_list ap;
    memset( &ap, 0, sizeof( ap ) );

    va_start( ap, sMsg );
    {
      int nResult = INT_MAX;
      int nLength = 2048;
      while( nResult >= nLength ) {
        if( pBuffer != NULL ) {
          delete[] pBuffer;
        }
        nLength *= 2;
        pBuffer = new char[nLength + 1];
        memset( pBuffer, 0, nLength + 1 );
        nResult = vsnprintf( pBuffer, nLength, sMsg, ap );
      }
    }
    va_end( ap );

    fputs( pBuffer, stdout );
    fflush( stdout );
    delete[] pBuffer;
  }
}

void PrintError( const char *sMsg, ... ) {
  if( sMsg == NULL ){
    return;
  }

  char *pBuffer = NULL;
  va_list ap;
  memset( &ap, 0, sizeof( ap ) );
  va_start( ap, sMsg );
  {
    int nResult = INT_MAX;
    int nLength = 2048;
    while( nResult >= nLength ) {
      nLength *= 2;
      if( pBuffer != NULL ) {
        delete[] pBuffer;
      }
      pBuffer = new char[nLength + 1];
      memset( pBuffer, 0, nLength + 1 );
      nResult = vsnprintf( pBuffer, nLength, sMsg, ap );
    }
  }
  va_end( ap );

  fputs( pBuffer, stderr );
  fflush( stderr );
  delete[] pBuffer;
}

bool PassVerbosityLevel( int nLevel ) {
  return nLevel <= hal::PrintHandlerGetErrorLevel();
}

std::string StringFormat( const char *fmt, ... ) {
  va_list args;
  va_start( args, fmt );

  if( !fmt ) {
    return "";
  }

  int nResult = INT_MAX;
  int nLength = 2048;
  char *pBuffer = NULL;
  while( nResult >= nLength ) {
    nLength *= 2;
    if( pBuffer != NULL ) {
      delete[] pBuffer;
    }
    pBuffer = new char[nLength + 1];
    memset( pBuffer, 0, nLength + 1 );
    nResult = vsnprintf( pBuffer, nLength, fmt, args );
  }
  std::string s( pBuffer );
  delete[] pBuffer;

  va_end(args);
  return s;
}
} // end rpg namespace
