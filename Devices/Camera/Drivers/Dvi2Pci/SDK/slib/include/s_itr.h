/*
 * $Id: s_itr.h,v 1.8 2008/01/27 16:39:13 slava Exp $
 *
 * Copyright (C) 2000-2008 by Slava Monich
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

#ifndef _SLAVA_ITERATOR_H_
#define _SLAVA_ITERATOR_H_

#include "s_def.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*
 * Generic iterator. Keep in mind that the iterators are not thread
 * safe and normally do not provide any protection against concurrent
 * modifications of the collection. That is, if you modify the collection
 * between the two calls to the iterator, the results are undefined. The 
 * only iterator function that you can safely call after you have modified
 * the collection (in any way other than calling ITR_Remove), is ITR_Delete 
 */

/* data types */
typedef struct _Iterator Iterator;
typedef void * IElement;
typedef const void * IElementC;

/* functions */
extern IElement ITR_Next P_((Iterator * itr));
extern IElement ITR_Last P_((Iterator * itr));
extern Bool ITR_HasNext P_((Iterator * itr));
extern Bool ITR_Remove P_((Iterator * itr));
extern void ITR_Delete P_((Iterator * itr));

/* general purpose iterators */
extern Iterator * ITR_Empty P_((void));
extern Iterator * ITR_Singleton P_((IElementC element));
extern Iterator * ITR_Array P_((IElementC array[], int size));
extern Iterator * ITR_Combine P_((Iterator * iterators[], int count));

/* filtered iterator */
typedef IElement (*ItrFilterNext)    P_((Iterator * target, void * ctx));
typedef Bool     (*ItrFilterRemove)  P_((Iterator * target, void * ctx));
extern Iterator * ITR_Filter P_((Iterator * target, ItrFilterNext fnext,
                                ItrFilterRemove fremove, void * ctx));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_ITERATOR_H_ */

/*
 * HISTORY:
 *
 * $Log: s_itr.h,v $
 * Revision 1.8  2008/01/27 16:39:13  slava
 * o removed unnecessary const in ITR_Array declaration
 *
 * Revision 1.7  2006/10/02 04:46:45  slava
 * o added IElementC type
 * o ITR_Singleton now takes IElementC as a parameter
 * o replaced ConstPtr with IElementC in ITR_Array declaration
 *
 * Revision 1.6  2006/03/20 06:50:25  slava
 * o added singleton iterator to complete the picture
 *
 * Revision 1.5  2006/03/20 02:23:12  slava
 * o added "composite" iterator which combines several iterators into one.
 *   The new iterator is created by ITR_Combine function.
 *
 * Revision 1.4  2006/03/19 09:58:09  slava
 * o fixed gcc compilation warning (shadow declaration)
 *
 * Revision 1.3  2006/03/19 09:33:21  slava
 * o added ITR_Last function and general purpose "filter" iterator (ITR_Filter)
 *
 * Revision 1.2  2004/03/15 20:13:18  slava
 * o added general purpose iterators (empty iterator and array iterator)
 *
 * Revision 1.1  2003/11/08 20:47:43  slava
 * o added iterators
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
