/*
 * $Id: s_dom.h,v 1.3 2004/03/17 19:53:40 slava Exp $
 *
 * Copyright (C) 2001-2004 by Slava Monich
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

#ifndef _SLAVA_DOM_H_
#define _SLAVA_DOM_H_

#include "s_xml.h"

/*
 * This module implements a simple read-only DOM tree API.
 * Useful for loading small XML files.
 */

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* node in the DOM tree */
typedef struct _DOMNode  DOMNode;
typedef struct _DOMChunk DOMChunk;

/* access to the parent node */
extern DOMNode * DOM_Parent P_((DOMNode * node));

/* access to the child nodes */
extern DOMNode * DOM_FirstNode P_((DOMNode * node));
extern DOMNode * DOM_LastNode P_((DOMNode * node));
extern DOMNode * DOM_FindFirst P_((DOMNode * node, Str tag));
extern DOMNode * DOM_FindLast P_((DOMNode * node, Str tag));

/* access to the sibling nodes */
extern DOMNode * DOM_Next P_((DOMNode * node));
extern DOMNode * DOM_Prev P_((DOMNode * node));
extern DOMNode * DOM_FindNext P_((DOMNode * node, Str tag));
extern DOMNode * DOM_FindPrev P_((DOMNode * node, Str tag));

/* node contents */
extern int DOM_ChunkCount P_((DOMNode * node));
extern int DOM_NodeCount P_((const DOMNode * node));
extern Str DOM_CharData P_((const DOMNode * node));
extern Str DOM_TagName P_((const DOMNode * node));
extern const XMLAttr * DOM_Attr P_((const DOMNode * node));
extern void DOM_Delete P_((DOMNode * node));

/* chunks is a piece of text + optional node after the text */
extern DOMChunk * DOM_FirstChunk(DOMNode * node);
extern DOMChunk * DOM_NextChunk(DOMChunk * chunk);
extern DOMNode * DOM_ChunkNode(DOMChunk * chunk);
extern Str DOM_ChunkText(const DOMChunk * chunk);

/* parsing */
extern XMLTag *  DOM_RootCB P_((void * ctx, Str tag, const XMLAttr * a));
extern XMLTag *  DOM_TagsCB P_((void * ctx, Str tag, const XMLAttr * a));
extern DOMNode * DOM_FromTag P_((XMLTag * tag));
extern DOMNode * DOM_Load P_((Str file));
extern DOMNode * DOM_Read P_((File * in));
extern DOMNode * DOM_LoadTags P_((Str file));
extern DOMNode * DOM_ReadTags P_((File * in));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_DOM_H_ */

/*
 * HISTORY:
 *
 * $Log: s_dom.h,v $
 * Revision 1.3  2004/03/17 19:53:40  slava
 * o added DOM_TagsCB and DOM_LoadTags functions that only keep the tags
 *   and their attributes, ignoring XML character data
 *
 * Revision 1.2  2003/01/28 22:37:44  slava
 * o added something called DOMChunk which allows perfectly accurate
 *   reconstruction of the original XML document (except for comments)
 *
 * Revision 1.1  2003/01/28 20:08:06  slava
 * o DOM tree
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
