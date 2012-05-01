/*
 * $Id: s_random.c,v 1.31 2006/03/19 09:28:23 slava Exp $
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

#include "s_random.h"
#include "s_mutex.h"
#include "s_util.h"
#include "s_libp.h"
#include "s_mem.h"

#include <math.h>
#include <float.h>

#define DBL_MANT_DIG1 (DBL_MANT_DIG/2)
#define DBL_MANT_DIG2 (DBL_MANT_DIG-(DBL_MANT_DIG/2))

/* certain assumptions made about the size of mantissa */
COMPILE_ASSERT(DBL_MANT_DIG < 64);
COMPILE_ASSERT(FLT_MANT_DIG < 32);

/*
 * Random generator's context.
 */
struct _Random {
#ifndef __KERNEL__
    double nextGaussian;        /* next random gaussian value */
    Bool   haveNextGaussian;    /* True if we have the above */
#endif /* __KERNEL__ */

    Seed   seed;                /* see that inialized this sequence */

    RNG    rng;                 /* random number generation algorithm */
    void * ctx;                 /* algorithm context */

    Bool   syn;                 /* True to synchronize the generator */
    Mutex  mutex;               /* synchronization mutex */
};

#define NEXT_RANDOM(_r,_bits) ((_r)->rng.rng_next((_r)->ctx, _bits))

/**
 * Default RNG implementation is a linear congruential pseudorandom number 
 * generator, as defined by D. H. Lehmer and described by Donald E. Knuth in
 * "The Art of Computer Programming", Volume 2: "Seminumerical Algorithms", 
 * section 3.2.1.
 */
typedef struct _RNG_Lehmer_Data {
    I64s multiplier;
    I64s addend;
    I64s mask;
    I64s seed;
} RNG_Lehmer_Data;

STATIC void * RNG_Lehmer_Init(Random * rnd)
{
    RNG_Lehmer_Data * r;
    UNREF(rnd);
    
    r = MEM_New(RNG_Lehmer_Data);
    if (r) {
        r->multiplier = __INT64_C(0x5DEECE66D);
        r->addend = __INT64_C(0xB);
        r->mask = (__INT64_C(1) << 48) - 1;
        r->seed = RANDOM_GenSeed();
    }
    return r;
}

STATIC void RNG_Lehmer_Seed(void * rng, Seed s)
{
    RNG_Lehmer_Data * r =  (RNG_Lehmer_Data*)rng;
    r->seed = (s ^ r->multiplier) & r->mask;
}

STATIC I32s RNG_Lehmer_Next(void * rng, int bits)
{
    RNG_Lehmer_Data * r =  (RNG_Lehmer_Data*)rng;
    I64u nextseed = (r->seed * r->multiplier + r->addend) & r->mask;
    r->seed = nextseed;
    return (I32s)(nextseed >> (48 - bits));
}

STATIC Seed RNG_Lehmer_State(void * rng)
{
    RNG_Lehmer_Data * r =  (RNG_Lehmer_Data*)rng;
    return (r->seed ^ r->multiplier) & r->mask;
}

STATIC void RNG_Lehmer_Free(void * rng)
{
    MEM_Free(rng);
}

STATIC RNG RNG_Lehmer = {
    RNG_Lehmer_Init,
    RNG_Lehmer_Seed,
    RNG_Lehmer_Next,
    RNG_Lehmer_State,
    RNG_Lehmer_Free
};

/*
 * Global instance of Random
 */
STATIC Random RANDOM_random = {0};
STATIC int    RANDOM_initCount = 0;
STATIC Seed   RANDOM_lastSeed = 0; /* last auto-generated seed */

/**
 * Initialize random number generator context.
 */
STATIC Bool RANDOM_Init(Random * r, const RNG * algorithm)
{
    const RNG * rng = (algorithm ? algorithm : (&RNG_Lehmer));
    ASSERT(rng->rng_next);
    ASSERT(rng->rng_seed);
    if (MUTEX_Init(&r->mutex)) {
        void * ctx = NULL;
        if (!rng->rng_init || (ctx = (*(rng->rng_init))(r)) != NULL) {
#ifndef __KERNEL__
            r->nextGaussian = 0;
            r->haveNextGaussian = False;
#endif /* __KERNEL__ */
            r->rng = (*rng);
            r->ctx = ctx;
            r->syn = True;     /* synchronize by default */
            RANDOM_SetSeed(r, RANDOM_GenSeed());
            return True;
        }
        MUTEX_Destroy(&r->mutex);
    }
    return False;
}

/**
 * Deallocate resources used by the random generator context
 */
STATIC void RANDOM_Destroy(Random * r)
{
    if (r->rng.rng_free) (*(r->rng.rng_free))(r->ctx);
    MUTEX_Destroy(&r->mutex);
}

/**
 * Initialize the module.
 */
void RANDOM_InitModule()
{
    if ((RANDOM_initCount++) == 0) {
        MEM_InitModule();
        if (RANDOM_Init(&RANDOM_random, NULL)) {
            return;
        }

        /* unrecoverable error */
        SLIB_Abort(TEXT("RANDOM"));
    }
}

/**
 * Cleanup the module.
 */
void RANDOM_Shutdown()
{
    ASSERT(RANDOM_initCount > 0);
    if ((--RANDOM_initCount) == 0) {
        RANDOM_Destroy(&RANDOM_random);
        MEM_Shutdown();
    }
}

/**
 * Get single global instance of Random.
 */
Random * RANDOM_GetRandom()
{
    ASSERT(RANDOM_initCount > 0);
    return &RANDOM_random;
}

/**
 * Generates seed based on current time
 */
Seed RANDOM_GenSeed()
{

    /*
     * this is prone to race conditions, although very unlikely.
     * usually all initialization occurs on the same thread so
     * it's not a problem
     */
    Seed s = TIME_Now();
    if (RANDOM_lastSeed >= s) {
        s = ++RANDOM_lastSeed;
    } else {
        RANDOM_lastSeed = s;
    }

    return s;
}

/**
 * Allocate random number generator context.
 * Returns NULL is memory allocation fails.
 * The seed is initialized with the value returned by time()
 */
Random * RANDOM_CreateRNG(const RNG * rng)
{
    Random * r = MEM_New(Random);
    if (r) {
        if (!RANDOM_Init(r, rng)) {
            MEM_Free(r);
            r = NULL;
        }
    }
    return r;
}

/**
 * Deletes random number generator context.
 */
void RANDOM_Delete(Random * r)
{
    if (r) {
        RANDOM_Destroy(r);
        MEM_Free(r);
    }
}

/**
 * Returns True if operations on this Random object are being synchronized
 * internally. 
 *
 * NOTE: in multithread environment synchronization is required in order 
 * to produce statistically correct results. However, in cases when all 
 * operations on this instance of Random are being performed by the same 
 * thread OR if application provides external synchronization, you may 
 * want to turn off the internal Random synchronization and somewhat 
 * improve performance.
 */
Bool RANDOM_IsSync(const Random * r)
{
    return r->syn;
}

/**
 * Turns on/off the internal synchronization. By default it's on. See 
 * also comments for RANDOM_IsSync() above
 */
void RANDOM_SetSync(Random * r, Bool syn)
{
    r->syn = syn;
}

/**
 * Sets the seed of this random number generator. 
 */
Seed RANDOM_SetSeed(Random * r, Seed s)
{
    Bool unlock = BoolValue(r->syn && MUTEX_Lock(&r->mutex)); 
    r->seed = s;
    (*(r->rng.rng_seed))(r->ctx, s);
#ifndef __KERNEL__
    r->haveNextGaussian = False;
#endif /* __KERNEL__ */
    if (unlock) MUTEX_Unlock(&r->mutex);
    return s;
}

/**
 * Returns the seed that initiated the current pseudorandom number sequence
 */
Seed RANDOM_GetSeed(const Random * r)
{
    return r->seed;
}

/**
 * Returns the seed that can be used to restart the current pseudorandom 
 * number sequence from its current point.
 */
Seed RANDOM_GetState(const Random * r)
{
    Seed state = (Seed)0;
    if (r->rng.rng_state) {
        Bool unlock = BoolValue(r->syn && MUTEX_Lock((Mutex*)(&r->mutex))); 
        state = (*(r->rng.rng_state))(r->ctx);
        if (unlock) MUTEX_Unlock((Mutex*)(&r->mutex));
    }
    return state;
}

/**
 * Returns True if RNG supports the notion of state.
 */
Bool RANDOM_HasState(const Random * r)
{
    return BoolValue(r->rng.rng_state != NULL);
}

/**
 * Returns the next pseudorandom, uniformly distributed 32-bit integer
 * value from this random number generator's sequence. 
 */
I32s RANDOM_NextI32(Random * r)
{
    Bool unlock = BoolValue(r->syn && MUTEX_Lock(&r->mutex)); 
    I32s next =  NEXT_RANDOM(r,32); 
    if (unlock) MUTEX_Unlock(&r->mutex);
    return next;
}

/**
 * Returns a pseudorandom, uniformly distributed 32-bit integer value
 * between 0 (inclusive) and the specified value (exclusive), drawn 
 * from this random number generator's sequence.
 */
I32s RANDOM_NextInt32(Random * r, int n)
{
    Bool unlock = BoolValue(r->syn && MUTEX_Lock(&r->mutex)); 
    I32s bits, val;
    ASSERT(n>0);

    if ((n & -n) == n) { /* i.e., n is a power of 2 */
        I64s next = (I64s)NEXT_RANDOM(r,31);
        if (unlock) MUTEX_Unlock(&r->mutex);
        return (I32s)((n * next) >> 31);
    }

    do {
        bits = NEXT_RANDOM(r,31);
        val = bits % n;
    } while (bits - val + (n-1) < 0);
    
    if (unlock) MUTEX_Unlock(&r->mutex);
    return val;
}

/**
 * Returns the next pseudorandom, uniformly distributed 64-bit integer
 * value from this random number generator's sequence. 
 */
I64s RANDOM_NextI64(Random * r)
{
    Bool unlock = BoolValue(r->syn && MUTEX_Lock(&r->mutex)); 
    I64s next = ((I64s)(NEXT_RANDOM(r,32)) << 32) + NEXT_RANDOM(r,32);
    if (unlock) MUTEX_Unlock(&r->mutex);
    return next;
}

/**
 * Returns a pseudorandom, uniformly distributed 64-bit integer value
 * between 0 (inclusive) and the specified value (exclusive), drawn 
 * from this random number generator's sequence.
 */
I64s RANDOM_NextInt64(Random * r, I64s n)
{
    if (n <= INT_MAX) {
        return (I64s)RANDOM_NextInt32(r, (int)n);
    } else {
        /* this is essentially cut&pasted from RANDOM_NextInt32.
         * I haven't done any formal testing of whether it actually 
         * works as expected. */
        Bool unlock = BoolValue(r->syn && MUTEX_Lock(&r->mutex)); 
        I64s bits, val;
        ASSERT(n>0);

        do {
            bits = ((I64u)(NEXT_RANDOM(r,31)) << 32) + NEXT_RANDOM(r,32);
            val = bits % n;
        } while (bits - val + (n-1) < 0);
    
        if (unlock) MUTEX_Unlock(&r->mutex);
        return val;
    }
}

/**
 * Returns the next pseudorandom, uniformly distributed boolean 
 * value from this random number generator's sequence. 
 */
Bool RANDOM_NextBool(Random * r)
{
    Bool unlock = BoolValue(r->syn && MUTEX_Lock(&r->mutex)); 
    Bool next = BoolValue(NEXT_RANDOM(r,1));
    if (unlock) MUTEX_Unlock(&r->mutex);
    return next;
}

/*
 * The code that uses floating point numbers is not being compiled 
 * in NT kernel mode configuration.
 */
#ifndef __KERNEL__

/**
 * Returns the next pseudorandom, uniformly distributed float
 * value between 0.0 and 1.0 from this random number generator's 
 * sequence.
 *
 * This code is not being compiled in NT kernel configuration because
 * floating point arithmetics is not available there.
 */
float RANDOM_NextFloat(Random * r)
{
    Bool unlock = BoolValue(r->syn && MUTEX_Lock(&r->mutex)); 
    I32s i = NEXT_RANDOM(r,FLT_MANT_DIG);
    float f = i / ((float)(1 << FLT_MANT_DIG));
    if (unlock) MUTEX_Unlock(&r->mutex);
    return f;
}

/**
 * Returns the next pseudorandom, uniformly distributed double
 * value between 0.0 and 1.0 from this random number generator's 
 * sequence.
 */
STATIC double nextDouble(Random * r)
{
    I64s l = ((I64s)(NEXT_RANDOM(r,DBL_MANT_DIG1)) << DBL_MANT_DIG2) + 
        NEXT_RANDOM(r,DBL_MANT_DIG2);
    return (l / (double)(__INT64_C(1) << DBL_MANT_DIG));
}

/**
 * Returns the next pseudorandom, uniformly distributed double
 * value between 0.0 and 1.0 from this random number generator's 
 * sequence.
 */
double RANDOM_NextDouble(Random * r)
{
    Bool unlock = BoolValue(r->syn && MUTEX_Lock(&r->mutex)); 
    double d = nextDouble(r);
    if (unlock) MUTEX_Unlock(&r->mutex);
    return d;
}

/**
 * Returns the next pseudorandom, Gaussian ("normally") distributed
 * double value with mean 0.0 and standard deviation 1.0 from this 
 * random number generator's sequence.
 *
 * This uses the *polar method* of G. E. P. Box, M. E. Muller, 
 * and G. Marsaglia, as described by Donald E. Knuth in "The Art
 * of Computer Programming", Volume 2: Seminumerical Algorithms, 
 * section 3.4.1, subsection C, algorithm P. Note that algorithm 
 * generates two independent values at the cost of only one call 
 * to log() and one call to sqrt(). 
 *
 * To be honest, this code was stolen from Java class java.util.Random
 * if you haven't noticed yet.
 */
double RANDOM_NextGauss(Random * r)
{
    Bool unlock = BoolValue(r->syn && MUTEX_Lock(&r->mutex)); 
    double d;

    /* See Knuth, ACP, Section 3.4.1 Algorithm C. */
    if (r->haveNextGaussian) {
        r->haveNextGaussian = False;
        d = r->nextGaussian;
    } else {
        double v1, v2, s, x;
        do { 
            v1 = 2 * nextDouble(r) - 1; /* between -1 and 1 */
            v2 = 2 * nextDouble(r) - 1; /* between -1 and 1 */
            s = v1 * v1 + v2 * v2;
        } while (s >= 1);
        x = sqrt(-2 * log(s)/s);
        r->nextGaussian = v2 * x;
        r->haveNextGaussian = True;
        d = v1 * x;
    }

    if (unlock) MUTEX_Unlock(&r->mutex);
    return d;
}

#endif /* __KERNEL__ */

/*
 * HISTORY:
 *
 * $Log: s_random.c,v $
 * Revision 1.31  2006/03/19 09:28:23  slava
 * o use DBL_MANT_DIG and FLT_MANT_DIG defines from <float.h> instead of
 *   hardcoded numbers as these numbers can be platform dependent (can they?).
 *
 * Revision 1.30  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.29  2004/04/08 12:21:43  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.28  2003/12/01 03:01:01  slava
 * o disable use of floating point data types in kernel mode
 *
 * Revision 1.27  2003/11/02 17:46:38  slava
 * o no floating point in NT kernel mode environment
 *
 * Revision 1.26  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.25  2003/04/12 18:46:55  slava
 * o initialize/deinitialize the MEM module in RANDOM_InitModule/Shutdown
 *
 * Revision 1.24  2003/01/20 19:02:46  slava
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
 * Revision 1.23  2002/08/27 11:52:03  slava
 * o some reformatting
 *
 * Revision 1.22  2002/05/29 08:29:49  slava
 * o removed EVENT_InitModule and MUTEX_InitModule functions
 *
 * Revision 1.21  2002/01/18 05:51:02  slava
 * o RANDOM_HasState, RANDOM_GetState and RANDOM_GetSeed should take const
 *   pointer as a parameter
 *
 * Revision 1.20  2001/11/28 10:03:56  slava
 * o <math.h> is no longer being include from s_os.h
 *
 * Revision 1.19  2001/10/09 05:55:42  slava
 * o added RANDOM_HasState() and RANDOM_GetState() functions
 *
 * Revision 1.18  2001/10/08 05:17:52  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.17  2001/10/07 16:07:42  slava
 * o RANDOM_Free() has been renamed into RANDOM_Delete() for consistency
 *   with other slib modules
 *
 * Revision 1.16  2001/06/12 18:56:52  slava
 * o added RANDOM_GenSeed() function
 *
 * Revision 1.15  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.14  2001/05/27 10:48:17  slava
 * o renamed fields of the RNG structure (added rng_ prefix)
 *
 * Revision 1.13  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.12  2001/03/25 19:21:08  slava
 * o added RANDOM_NextInt64() function
 *
 * Revision 1.11  2001/01/06 20:18:27  slava
 * o added RANDOM_IsSync() and RANDOM_SetSync() functions
 *
 * Revision 1.10  2000/11/10 00:19:37  slava
 * o implemented RANDOM_GetSeed()
 * o fixed a bug that allowed multiple Random objects have the same
 *   default seed
 *
 * Revision 1.9  2000/11/05 02:10:39  slava
 * o fixed a stupid bug in RANDOM_Init()
 *
 * Revision 1.8  2000/11/05 01:40:47  slava
 * o made pseudorandom number generation algorithm "pluggable"
 *
 * Revision 1.7  2000/11/04 16:12:21  slava
 * o renamed RANDOM_Destory() into RANDOM_Free() for consistency. The
 *   convention is that Xxx_Create() and Xxx_Free() allocate and deallocate
 *   Xxx objects, while Xxx_Init() and Xxx_Destroy() initialize and destroy
 *   the objects allocated by the caller (usually as a part of other data
 *   structure). Therefore, RANDOM_Destroy() was quite misleading.
 *   RANDOM_Destroy() still exists but is now a static function used
 *   internally by the module
 *
 * Revision 1.6  2000/11/01 13:25:05  slava
 * o replaced TRUE/FALSE with True/False
 *
 * Revision 1.5  2000/11/01 10:25:14  slava
 * o replaced New, NewArray and ReallocArray macros with MEM_New,
 *   MEM_NewArray and MEM_ReallocArray, respectively
 *
 * Revision 1.4  2000/08/24 01:35:15  slava
 * o fixed a deadlock in RANDOM_NextGauss() caused by attempt to recursively
 *   acquire the mutex
 *
 * Revision 1.3  2000/08/23 05:35:45  slava
 * o made it thread-safe
 *
 * Revision 1.2  2000/08/19 11:47:35  slava
 * o <math.h> is now included from s_os.h
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
