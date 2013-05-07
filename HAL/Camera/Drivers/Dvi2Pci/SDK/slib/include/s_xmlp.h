/*
 * $Id: s_xmlp.h,v 1.4 2006/03/28 23:47:16 slava Exp $
 *
 * Copyright (C) 2001-2006 by Slava Monich
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met: 
 *
 *   1.Redistributions of source code must retain the above copyright 
 *     notice, this list of conditions and the following disclaimer. 
 *   2.Redistributions in binary form must reproduce the above copyright 
 *     notice, this list of conditions and the following disclaimer 
 *     in the documentation and/or other materials provided with the 
 *     distribution. 
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING 
 * IN ANY WAY OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE, EVEN 
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 *
 * The views and conclusions contained in the software and documentation 
 * are those of the authors and should not be interpreted as representing 
 * any official policies, either expressed or implied.
 */

#ifndef _SLAVA_XML_PRIVATE_H_
#define _SLAVA_XML_PRIVATE_H_ 1

#include "s_xml.h"

/* Set of attributes */
struct _XMLAttr {
    Char * storage;     /* storage for string pairs */
    int size;           /* total size of the data in the storage, bytes */
    int * off;          /* array of offsets to name/value pairs */
    int n;              /* number of name/value pairs */
};

/* Internal functions */
extern void XML_AttrDestroy P_((XMLAttr * atts));

#endif /* _SLAVA_XML_PRIVATE_H_ */

/*
 * HISTORY:
 *
 * $Log: s_xmlp.h,v $
 * Revision 1.4  2006/03/28 23:47:16  slava
 * o updated XML parsing internals. New public functions: XML_AttrClone,
 *   XML_AttrDelete and XML_AttrCopy.
 *
 * Revision 1.3  2005/02/19 02:37:10  slava
 * o include SLIB headers as user includes (i.e. "s_lib.h") rather than
 *   system includes (i.e. <s_lib.h>). This helps with generation of
 *   dependencies
 *
 * Revision 1.2  2003/01/28 20:01:44  slava
 * o added 'storage' and 'size' fields to the XMLAttr structure
 *
 * Revision 1.1  2003/01/05 06:21:51  slava
 * o moved the functions that reference XML parser into a separate file,
 *   so that we don't have to link with expat if we aren't parsing any XML
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
