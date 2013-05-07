/*
 * $Id: s_md.h,v 1.5 2009/11/07 09:12:53 slava Exp $
 *
 * Copyright (C) 2000-2009 by Slava Monich
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

#ifndef _SLAVA_MD_H_
#define _SLAVA_MD_H_

#include "s_def.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*
 * Generic message digest framework. 
 */
typedef struct _Digest Digest;
extern void DIGEST_Init   P_((Digest * d));
extern Str  DIGEST_Name   P_((const Digest * d));
extern int  DIGEST_Size   P_((const Digest * d));
extern void DIGEST_Update P_((Digest * d, const void * data, size_t size));
extern void DIGEST_Finish P_((Digest * d, void * out));
extern void DIGEST_Delete P_((Digest * d));

/*
 * SHA-1 message digest.
 * http://www.itl.nist.gov/fipspubs/fip180-1.htm
 */
#define SHA1_DIGEST_SIZE  20

extern Digest * SHA1_Create P_((void));
extern void SHA1_Digest P_((const void * in, size_t n, void * out));

/*
 * MD-5 message digest. 
 * http://www.ietf.org/rfc/rfc1321.txt
 */
#define MD5_DIGEST_SIZE  16

extern Digest * MD5_Create P_((void));
extern void MD5_Digest P_((const void * in, size_t n, void * out));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_MD_H_ */

/*
 * HISTORY:
 *
 * $Log: s_md.h,v $
 * Revision 1.5  2009/11/07 09:12:53  slava
 * o replaced a few more int's with size_t
 *
 * Revision 1.4  2009/04/09 21:17:17  slava
 * o user size_t instead of int where it's appropriate
 *
 * Revision 1.3  2004/07/31 18:14:19  slava
 * o added DIGEST_Name function
 *
 * Revision 1.2  2004/07/31 07:27:03  slava
 * o implemented MD-5 digest (RFC 1321)
 *
 * Revision 1.1  2004/07/29 17:09:37  slava
 * o message digest framework
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
