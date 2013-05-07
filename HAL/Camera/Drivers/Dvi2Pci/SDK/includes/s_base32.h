/*
 * $Id: s_base32.h,v 1.2 2009/04/09 21:54:56 slava Exp $
 *
 * Copyright (C) 2000-2004 by Slava Monich
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

#ifndef _SLAVA_BASE32_H_
#define _SLAVA_BASE32_H_

#include "s_buf.h"
#include "s_file.h"
#include "s_strbuf.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*
 * Flags for the encoding functions
 */
#define BASE32_PAD       0x01  /* write the trailing pad characters */
#define BASE32_LOWERCASE 0x02  /* lowercase the output */

/*
 * BASE32 encoding functions.
 */
extern Char * BASE32_Encode P_((const void * data, size_t size, int flags));
extern Str BASE32_EncodeStr P_((const void * data, size_t size, StrBuf * sb,
    int flags));

/*
 * BASE32 decoding functions. The "strict" decoding is case sensitive.
 */
extern Bool   BASE32_Decode P_((Str base64, Buffer * out));
extern Bool   BASE32_StrictDecode P_((Str base64, Buffer * out));
extern Bool   BASE32_DecodeFile P_((File * in, Buffer * out));
extern Bool   BASE32_StrictDecodeFile P_((File * in, Buffer * out));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_BASE32_H_ */

/*
 * HISTORY:
 *
 * $Log: s_base32.h,v $
 * Revision 1.2  2009/04/09 21:54:56  slava
 * o use size_t instead of int where it's appropriate
 *
 * Revision 1.1  2004/08/18 02:52:01  slava
 * o added support for BASE32 encoding
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
