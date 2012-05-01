/*
 * $Id: s_random.h,v 1.21 2006/01/08 08:37:35 slava Exp $
 *
 * Copyright (C) 2000-2006 by Slava Monich
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

#ifndef _SLAVA_RANDOM_H_
#define _SLAVA_RANDOM_H_

#include "s_def.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* data types */
typedef struct _Random Random;
typedef I64s Seed;

/* 
 * Random number generator algorithm.
 *
 * The next() function generates the next pseudorandom number for any of 
 * the RANDOM_NextXxx functions. The general contract is that it 
 * returns a 32 int value and if the argument bits is between 1 
 * and 32 (inclusive), then that many low-order bits of the returned 
 * value will be (approximately) independently chosen bit values, 
 * each of which is (approximately) equally likely to be 0 or 1.
 *
 * State of the generator is defined as a seed that can be used 
 * to restart the sequence from the current point. Such thing may 
 * not exist. If RNG does not support a notion of state, it must 
 * leave rng_state as NULL. RANDOM_HasState() can be used to test 
 * whether the algorithm has state. The default algorithm supports 
 * it of course.
 */
typedef void * (*RngInit)  P_((Random * r));
typedef void   (*RngSeed)  P_((void * rng, Seed s));
typedef I32s   (*RngNext)  P_((void * rng, int bits));
typedef Seed   (*RngState) P_((void * rng));
typedef void   (*RngFree)  P_((void * rng));

typedef struct _RNG {
    RngInit  rng_init;  /* creates an instance of RNG */
    RngSeed  rng_seed;  /* seeds the generator */
    RngNext  rng_next;  /* returns the next pseudorandom number */
    RngState rng_state; /* returns the state of the generator */
    RngFree  rng_free;  /* deallocates the instance */
} RNG;

/* global operations */
extern void     RANDOM_InitModule P_((void));
extern void     RANDOM_Shutdown   P_((void));
extern Random * RANDOM_GetRandom  P_((void));
extern Seed     RANDOM_GenSeed    P_((void));

/* operations on a particular instance of Random */
extern Random * RANDOM_CreateRNG  P_((const RNG * rng));
extern void     RANDOM_Delete     P_((Random * r));
extern Bool     RANDOM_IsSync     P_((const Random * r));
extern void     RANDOM_SetSync    P_((Random * r, Bool syn));
extern Bool     RANDOM_HasState   P_((const Random * r));
extern Seed     RANDOM_GetState   P_((const Random * r));
extern Seed     RANDOM_GetSeed    P_((const Random * r));
extern Seed     RANDOM_SetSeed    P_((Random * r, Seed seed));
extern I32s     RANDOM_NextI32    P_((Random * r));
extern I32s     RANDOM_NextInt32  P_((Random * r, int n));
extern I64s     RANDOM_NextI64    P_((Random * r));
extern I64s     RANDOM_NextInt64  P_((Random * r, I64s n));
extern Bool     RANDOM_NextBool   P_((Random * r));

#ifndef __KERNEL__
extern float    RANDOM_NextFloat  P_((Random * r));
extern double   RANDOM_NextDouble P_((Random * r));
extern double   RANDOM_NextGauss  P_((Random * r));
#endif /* __KERNEL__ */

#define RANDOM_Create()       RANDOM_CreateRNG(NULL)
#define RANDOM_NextInt(_r,_n) RANDOM_NextInt32(_r,_n)

/* operations on the shared instance of Random */
#define RAND_GetSeed()      RANDOM_GetSeed(RANDOM_GetRandom())
#define RAND_GetState()     RANDOM_GetState(RANDOM_GetRandom())
#define RAND_SetSeed(_seed) RANDOM_SetSeed(RANDOM_GetRandom(),_seed)
#define RAND_NextI32()      RANDOM_NextI32(RANDOM_GetRandom())
#define RAND_NextInt(_n)    RANDOM_NextInt(RANDOM_GetRandom(),_n)
#define RAND_NextInt32(_n)  RANDOM_NextInt32(RANDOM_GetRandom(),_n)
#define RAND_NextI64()      RANDOM_NextI64(RANDOM_GetRandom())
#define RAND_NextInt64(_n)  RANDOM_NextInt64(RANDOM_GetRandom(),_n)
#define RAND_NextBool()     RANDOM_NextBool(RANDOM_GetRandom())

#ifndef __KERNEL__
#define RAND_NextFloat()    RANDOM_NextFloat(RANDOM_GetRandom())
#define RAND_NextDouble()   RANDOM_NextDouble(RANDOM_GetRandom())
#define RAND_NextGauss()    RANDOM_NextGauss(RANDOM_GetRandom())
#endif /* __KERNEL__ */

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_RANDOM_H_ */

/*
 * HISTORY:
 *
 * $Log: s_random.h,v $
 * Revision 1.21  2006/01/08 08:37:35  slava
 * o added RAND_GetState() macro
 *
 * Revision 1.20  2003/12/01 03:01:01  slava
 * o disable use of floating point data types in kernel mode
 *
 * Revision 1.19  2003/07/28 03:12:09  slava
 * o added RAND_GetSeed() macro
 *
 * Revision 1.18  2003/01/20 19:02:46  slava
 * o changed XXX_InitModule and XXX_Shutdown (should be XXX_DeinitModule?)
 *   functions such that they can be called more than once. Obviously,
 *   only the first XXX_InitModule and the last XXX_Shutdown calls will
 *   actually do something; all other calls merely increment/decrement
 *   the "reference count". This addresses the problem of initializing/
 *   deinitializing slib in case if it's being used by multiple static
 *   libraries which may (but don't have to) be compiled into a single
 *   application. The caller of each XXX_InitModule function is now
 *   responsible for calling the respective XXX_Shutdown function when
 *   it no longer needs the services provided by the XXX module. The
 *   access to the internal "reference count" of each module is not
 *   synchronized, meaning that all the XXX_InitModule and XXX_Shutdown
 *   calls must be made under the circumstances that make race conditions
 *   impossible
 *
 * Revision 1.17  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.16  2002/08/12 05:13:27  slava
 * o use (void) rather than () to declare functions that have
 *   no arguments. The problem with () is that compilers interpret
 *   it as an *unspecified* argument list, but (void) clearly
 *   indicates that function takes no arguments. Improves compile
 *   time error checking
 *
 * Revision 1.15  2002/01/18 05:51:02  slava
 * o RANDOM_HasState, RANDOM_GetState and RANDOM_GetSeed should take const
 *   pointer as a parameter
 *
 * Revision 1.14  2001/10/09 05:55:42  slava
 * o added RANDOM_HasState() and RANDOM_GetState() functions
 *
 * Revision 1.13  2001/10/08 05:17:52  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.12  2001/10/07 16:07:42  slava
 * o RANDOM_Free() has been renamed into RANDOM_Delete() for consistency
 *   with other slib modules
 *
 * Revision 1.11  2001/06/12 18:56:52  slava
 * o added RANDOM_GenSeed() function
 *
 * Revision 1.10  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.9  2001/05/27 10:48:17  slava
 * o renamed fields of the RNG structure (added rng_ prefix)
 *
 * Revision 1.8  2001/03/25 19:21:08  slava
 * o added RANDOM_NextInt64() function
 *
 * Revision 1.7  2001/01/06 20:18:27  slava
 * o added RANDOM_IsSync() and RANDOM_SetSync() functions
 *
 * Revision 1.6  2000/12/17 16:09:47  slava
 * o added extern "C" {} so that slib could be used from a C++ program
 *
 * Revision 1.5  2000/11/10 00:18:16  slava
 * o added RANDOM_GetSeed() declaration
 *
 * Revision 1.4  2000/11/05 01:40:47  slava
 * o made pseudorandom number generation algorithm "pluggable"
 *
 * Revision 1.3  2000/11/04 16:12:21  slava
 * o renamed RANDOM_Destory() into RANDOM_Free() for consistency. The
 *   convention is that Xxx_Create() and Xxx_Free() allocate and deallocate
 *   Xxx objects, while Xxx_Init() and Xxx_Destroy() initialize and destroy
 *   the objects allocated by the caller (usually as a part of other data
 *   structure). Therefore, RANDOM_Destroy() was quite misleading.
 *   RANDOM_Destroy() still exists but is now a static function used
 *   internally by the module
 *
 * Revision 1.2  2000/08/23 05:34:44  slava
 * o renamed RANDOM_Delete() into RANDOM_Destroy()
 * o RANDOM_InitModule() now returns NULL
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
