/*
 * $Id: s_xml.h,v 1.15 2007/01/27 02:43:28 slava Exp $
 *
 * Copyright (C) 2001-2007 by Slava Monich
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

#ifndef _SLAVA_XML_H_
#define _SLAVA_XML_H_

#include "s_def.h"
#include "s_strbuf.h"
#include "s_file.h"

/*
 * This file defines interface between an application that needs to parse
 * XML files and some abstract non-validating XML parser. The actual parser 
 * being used by SLIB may change but this interface (hopefully) won't. This
 * interface defines a minimal subset of typicall XML parser functionality, 
 * necessary and sufficient to parse simple XML-compliant data files.
 */

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* XML file normally begins with something like this */
#define XML_PROLOG "<?xml version='1.0' encoding='utf-8' standalone='yes'?>"

/* 
 * SAX style event handlers.
 *
 * NOTE: in the "characterData" notification, the string does not have 
 * to be NULL terminated. In fact, it's usually not NULL terminated. The 
 * length of the data block (in characters) if given as the third parameter.
 */
typedef struct _XMLAttr XMLAttr;
typedef void (* XMLStartElemCB) P_((void * ctx, Str tag, const XMLAttr * a));
typedef void (* XMLEndElemCB) P_((void * ctx, Str tag));
typedef void (* XMLCharDataCB) P_((void * ctx, const wchar_t * s, int len));
extern const XMLAttr xmlEmptyAttr;  /* empty attribute list */

/*
 * This structure defines event handlers for the most intersting SAX 
 * events that carry some useful information (rather than formatting 
 * details). 
 */
typedef struct _XMLCallbacks {
    XMLStartElemCB startElem;       /* start element */
    XMLEndElemCB   endElem;         /* end element */
    XMLCharDataCB  charData;        /* character data */
} XMLCallbacks;

/* Access to XML attributes */
extern XMLAttr * XML_AttrClone P_((const XMLAttr * atts));
extern void XML_AttrDelete P_((XMLAttr * atts));
extern Bool XML_AttrCopy P_((XMLAttr * dest, const XMLAttr * src));
extern int XML_AttrCount P_((const XMLAttr * atts));
extern Str XML_AttrValue P_((const XMLAttr * atts, Str name));
extern Str XML_AttrValueAt P_((const XMLAttr * atts, int pos));
extern Str XML_AttrNameAt P_((const XMLAttr * atts, int pos));

/* Run XML parser */
extern Bool XML_ParseStream P_((File* f, const XMLCallbacks* cb, void * ctx));
extern Bool XML_ParseFile P_((Str path, const XMLCallbacks * cb, void * ctx));
#define XML_RunParser XML_ParseStream /* for backward compatibility */

/* 
 * XML parsing mechanism that allows embedding pieces of opaque XML content 
 * into XML documents. Note that the root handler does not know the name of 
 * the tag that invoked it. If XMLEndTag callback is NULL, the memory pointed
 * to by the XMLTag pointer is deallocated with MEM_Free
 *
 * See comments for XML_Handle function below for more information on how
 * to use these callbacks.
 */

typedef struct _XMLTag XMLTag;

typedef XMLTag * (* XMLHandleRoot) P_((void * ctx,Str tag,const XMLAttr * a));
typedef XMLTag * (* XMLHandleTag) P_((XMLTag * p,Str name,const XMLAttr * a));
typedef Bool (* XMLHandleChars) P_((XMLTag * tag, const wchar_t * s, int n));
typedef Bool (* XMLEndTag) P_((XMLTag * tag, XMLTag * parent));

/* public part of XMLTag context */
struct _XMLTag {
    XMLHandleTag handleTag;         /* tag handler at this level */
    XMLHandleChars handleChars;     /* handle characters */
    XMLEndTag endTag;               /* end of tag handler */
};

/*
 * XML_Handle parses well-formed XML stream returning the root tag context
 * on success, NULL on failure. The endTag callback provided by the root tag
 * handler can be used to deallocate the root tag can context without any
 * side effects. However, in order to extract any useful information from the 
 * root tag context, each object providing root tag handler would normally
 * provide a function that would exract such information from the root tag 
 * (and deallocate everything else).
 *
 * The last parameter is a context parameter passed to the root tag callback
 * as the first argument. The code calling XML_Handle or the XMLHandleRoot
 * callback directly must know that XMLHandleRoot expects as a context
 * parameter. Typically, this is not a problem, but the use of this parameter
 * is discouraged. The XML stream should be self-sufficient, i.e. it should
 * contain all the information necessary to completely parse it. The context
 * parameter exists primarily to support for legacy XML file formats that 
 * require extra parsing information in addition to the information contained
 * in the XML stream itself.
 *
 * The top level tag (second parameter) can be NULL in which case any root
 * tag will be handled. Otherwise, the parsing will fail if the top level
 * tag does not match.
 */
extern XMLTag * XML_Handle P_((File * f,Str tag,XMLHandleRoot cb,void * ctx));

/* Utilities */
extern Bool XML_DontFreeTagCB P_((XMLTag * tag, XMLTag * parent));
extern Str  XML_Escape P_((StrBuf * sb, Str s));
extern Str  XML_Escape2 P_((StrBuf * sb, Str s, Bool escapeNonPrint));
extern Bool XML_OpenTag P_((File * out, Str tag, Bool eol));
extern Bool XML_CloseTag P_((File * out, Str tag, Bool eol));
extern Bool XML_StartTag P_((File * out, Str tag));
extern Bool XML_EndTag P_((File * out, Bool eol));
extern Bool XML_WriteAttr P_((File * out, Str attr, Str value));
extern Bool XML_WriteAttrNoEsc P_((File * out, Str attr, Str value));
extern Bool XML_WriteDoubleAttr P_((File * out, Str attr, double value));
extern Bool XML_WriteFloatAttr P_((File * out, Str attr, float value));
extern Bool XML_WriteIntAttr P_((File * out, Str attr, int value));
extern Bool XML_WriteI64Attr P_((File * out, Str attr, I64s value));
extern Bool XML_WriteU64Attr P_((File * out, Str attr, I64u value));
extern Bool XML_WriteUIntAttr P_((File * out, Str attr, unsigned int value));
extern Bool XML_WriteHexAttr P_((File * out, Str attr, int value));
extern Bool XML_MatchAttr P_((const XMLAttr * a, Str name, Str value));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_XML_H_ */

/*
 * HISTORY:
 *
 * $Log: s_xml.h,v $
 * Revision 1.15  2007/01/27 02:43:28  slava
 * o added XML_Escape2 and XML_WriteAttrNoEsc functions
 *
 * Revision 1.14  2006/10/05 17:11:49  slava
 * o added XML_WriteFloatAttr function
 *
 * Revision 1.13  2006/03/28 23:47:16  slava
 * o updated XML parsing internals. New public functions: XML_AttrClone,
 *   XML_AttrDelete and XML_AttrCopy.
 *
 * Revision 1.12  2004/03/17 19:58:12  slava
 * o added XML_MatchAttr function and xmlEmptyAttr constant
 *
 * Revision 1.11  2004/03/16 22:18:46  slava
 * o added XML_WriteDoubleAttr function
 *
 * Revision 1.10  2003/01/28 19:58:06  slava
 * o XMLHandleRoot callback now takes tag name as a parameter
 *
 * Revision 1.9  2003/01/06 07:16:01  slava
 * o XML_OpenTag and XML_CloseTag take care of indentation
 * o added a number of XML_WriteXxxAttr functions for writing attributes
 *   of various types.
 *
 * Revision 1.8  2003/01/05 16:58:12  slava
 * o added XML_StartTag, XML_EndTag and XML_WriteAttr utilities
 *
 * Revision 1.7  2003/01/05 05:00:00  slava
 * o added XML_OpenTag and XML_CloseTag
 *
 * Revision 1.6  2002/12/30 22:49:22  slava
 * o implemented XML parsing mechanism that allows embedding pieces of
 *   opaque XML content into the XML stream
 *
 * Revision 1.5  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.4  2002/09/23 02:07:39  slava
 * o switched to Expat release 1.95.5
 *
 * Revision 1.3  2002/05/22 04:19:10  slava
 * o fixed include statements after s_sbuf.h was renamed into s_strbuf.h
 *
 * Revision 1.2  2001/10/09 05:56:08  slava
 * o more XML utilities
 *
 * Revision 1.1  2001/10/08 08:25:36  slava
 * o support for parsing XML files
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
