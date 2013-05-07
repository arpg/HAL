/*
 * $Id: s_hist.h,v 1.14 2006/10/21 21:45:28 slava Exp $
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

#ifndef _SLAVA_HIST_H_
#define _SLAVA_HIST_H_

#include "s_def.h"
#include "s_file.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* contents of one bin. */
#ifdef __SYMBIAN32__
#  undef _HBIN_64_
#else /* !__SYMBIAN32__ */
#  define _HBIN_64_ 1
#endif /* !__SYMBIAN32__ */

#ifdef _HBIN_64_
   typedef I64u HBin;
#  define MAX_HBIN_VALUE  UINT64_MAX
#  define HBIN_FORMAT I64U_FORMAT
#else /* !_HBIN_64_ */
   typedef I32u HBin;
#  define MAX_HBIN_VALUE  UINT_MAX
#  define HBIN_FORMAT "%u"
#endif /* !_HBIN_64_ */

/*
 * Flags:
 *
 * H_OVERFLOW   - this flag is set in Histogram->flags if an overflow is 
 *                detected in one of the bins of the histogram.
 *
 * H_AUTO_RANGE - this bit in HistAxis->flags marks the axes that have
 *                "auto range" feature enabled. If any axis has this flag,
 *                it's also set in Histogram->flags
 *
 * H_MAX_VALUE  - this flag is set in Histogram->flags if maxval field 
 *                contains a valid up-to-date value
 */
#define H_OVERFLOW   0x0001  /* overflow detected */
#define H_AUTO_RANGE 0x0002  /* range is detected automatically */
#define H_MAX_VALUE  0x0004  /* maxval field contains a valid value */

/* 
 * Histogram descriptor for one axis. Multi-dimensional histograms need 
 * more than one of those, one per each axis. 
 */
typedef struct _HistAxis {
    size_t nbins;       /* number of bins */
    Char * label;       /* name of the axis */
    double interval;    /* size of the interval */
    double min;         /* lower edge, inclusive */
    double max;         /* upper edge, exclusive */
    int flags;          /* flags */
} HistAxis;

/* 
 * N-dimensional histogram. Can be one-dimensional too. However, the
 * Hist1D histogram and functions that work with it are better optimized 
 * for one-dimensional case, use those whenever posisble. Note that the 
 * size of the Histogram memory block allocated for N-dimensional histogram,
 * where N >= 2, is greater than the size of this structure.
 */
typedef struct _Histogram {
    I16u dim;           /* number of dimensions */
    I16u flags;         /* flags */
    size_t totalbins;   /* total number of bins */
    size_t usedbins;    /* number of used bins */
    HBin total;         /* total number of entries */
    HBin missed;        /* total attempts to put an out-of-bounds value */
    HBin maxval;        /* maximum value */
    HBin * bins;        /* histogram contents */
    Char * title;       /* histogram title */
    HistAxis axis[1];   /* variable size array, must be the last */
} Histogram;

/* 
 * one-dimensional histogram. The size of this structure is fixed, so it 
 * may be allocated on stack, initialized by HIST1D_Init and deinitialized
 * by HIST1D_Destroy, unlike multi-dimensional histograms, which must be
 * allocated dynamically.
 */
typedef union _Hist1D {
    Histogram hist;     /* histogram data */
} Hist1D;

/* content of a single bin (used by enumeration) */
typedef struct _HBinData {
    const Histogram * hist;     /* the histogram containing the data */
    HBin count;                 /* the contents of the bin */
    unsigned int bin[1];        /* identifies the bin (variable size) */
} HBinData;

/*
 * HistCB - callback for HIST_Examine. It's called for each non-empty
 *          bin of the histogram. This is more efficient than requesting
 *          the content of every single bin with HIST_Get
 */
typedef Bool (*HistCB) P_((const HBinData * data, void * ctx));

/*
 * These are the functions for type-checking macro parameters of the macros
 * at compile time. We only need to do this in debug build.
 */
#if DEBUG
extern Hist1D * HIST1D_Cast P_((Hist1D * h));
extern Histogram * HIST_Cast P_((Histogram * h));
extern const Hist1D * HIST1D_CastC P_((const Hist1D * h));
extern const Histogram * HIST_CastC P_((const Histogram * h));
#else  /* !DEBUG */
#  define HIST1D_Cast(_h) (_h)
#  define HIST1D_CastC(_h) (_h)
#  define HIST_Cast(_h)  (_h)
#  define HIST_CastC(_h)  (_h)
#endif /* !DEBUG */

/* functions to work with one-dimensional histogram */
extern Hist1D * HIST1D_Create P_((Str s, int n, double min, double max));
extern Bool HIST1D_Init P_((Hist1D * h, Str s, int n, double min, double max));
extern void HIST1D_Destroy P_((Hist1D * h));
extern void HIST1D_Delete P_((Hist1D * h));

extern Bool HIST1D_IsIn P_((const Hist1D * h, double d));
extern Bool HIST1D_Put P_((Hist1D * h, double value));
extern Bool HIST1D_PutAll P_((Hist1D * h, const double * values, int count));
extern HBin HIST1D_Get P_((const Hist1D * h, int i));
extern void HIST1D_Set P_((Hist1D * h, int i, HBin value));
extern int  HIST1D_GetSize P_((const Hist1D * h));
extern Hist1D * HIST1D_Read P_((File * in));
extern Hist1D * HIST1D_Load P_((Str fname, IODesc io));
extern Bool HIST1D_Write P_((const Hist1D * h, File * out));
extern Bool HIST1D_Save P_((const Hist1D * h, Str fname, IODesc io));

/*
 * The following functions are implemented as macros:
 *
 * extern void HIST1D_Reset P_((Hist1D * h));
 * extern HBin HIST1D_MaxValue P_((const Hist1D * h));
 * extern Str  HIST1D_GetTitle P_((const Hist1D * h));
 * extern Bool HIST1D_SetTitle P_((Hist1D * h, Str title));
 * extern Str  HIST1D_GetLabel P_((const Hist1D * h));
 * extern Bool HIST1D_SetLabel P_((Hist1D * h, Str label));
 * extern Bool HIST1D_IsOver P_((const Hist1D * h));
 * extern HBin HIST1D_GetTotal P_((const Hist1D * h));
 * extern HBin HIST1D_GetMissed P_((const Hist1D * h));
 * extern int  HIST1D_GetUsedBins P_((const Hist1D * h));
 * extern double HIST1D_GetMin P_((const Hist1D * h));
 * extern double HIST1D_GetMax P_((const Hist1D * h));
 */
#define HIST1D_Reset(_h) HIST_Reset(&HIST1D_Cast(_h)->hist)
#define HIST1D_MaxValue(_h) HIST_MaxValue(&HIST1D_Cast(_h)->hist)
#define HIST1D_GetTitle(_h) (HIST1D_CastC(_h)->hist.title)
#define HIST1D_SetTitle(_h,_s) HIST_SetTitle(&HIST1D_Cast(_h)->hist.title),_s)
#define HIST1D_GetLabel(_h) HIST_GetLabel(&HIST1D_CastC(_h)->hist,0)
#define HIST1D_SetLabel(_h,_s) HIST_SetLabel(&HIST1D_Cast(_h)->hist,0,_s)
#define HIST1D_IsOver(_h) (((HIST1D_CastC(_h)->hist.flags) & H_OVERFLOW) != 0)
#define HIST1D_GetTotal(_h) (HIST1D_CastC(_h)->hist.total)
#define HIST1D_GetMissed(_h) (HIST1D_CastC(_h)->hist.missed)
#define HIST1D_GetUsedBins(_h) (HIST1D_CastC(_h)->hist.usedbins)
#define HIST1D_GetMin(_h) (HIST1D_CastC(_h)->hist.axis[0].min)
#define HIST1D_GetMax(_h) (HIST1D_CastC(_h)->hist.axis[0].max)

/* functions to work with multi-dimensional histogram */
extern Histogram * HIST_Create P_((Str s, int dim, const int n[],
    const double min[], const double max[]));
extern void HIST_Delete P_((Histogram * h));

extern double HIST_GetMin P_((const Histogram * h, int i));
extern double HIST_GetMax P_((const Histogram * h, int i));
extern Bool HIST_SetTitle P_((Histogram * h, Str title));
extern int  HIST_GetSize P_((const Histogram * h, int i));
extern Str  HIST_GetLabel P_((const Histogram * h, int i));
extern Bool HIST_SetLabel P_((Histogram * h, int i, Str label));
extern void HIST_Reset P_((Histogram * h));
extern HBin HIST_MaxValue P_((const Histogram * h));
extern Bool HIST_IsIn P_((const Histogram * h, const double x[]));
extern Bool HIST_Put P_((Histogram * h, const double value[]));
extern void HIST_Set P_((Histogram * h, const int i[], HBin value));
extern HBin HIST_Get P_((const Histogram * h, const int i[]));
extern void HIST_Examine P_((const Histogram * h, HistCB cb, void * ctx));
extern Histogram * HIST_Read P_((File * in));
extern Histogram * HIST_Load P_((Str fname, IODesc io));
extern Bool HIST_Write P_((const Histogram * h, File * out));
extern Bool HIST_Save P_((const Histogram * h, Str fname, IODesc io));

/*
 * The following functions are implemented as macros:
 *
 * extern int  HIST_GetDim P_((const Histogram * h))
 * extern Str  HIST_GetTitle P_((const Histogram * h))
 * extern Bool HIST_IsOver P_((const Histogram * h));
 * extern HBin HIST_GetTotal P_((const Histogram * h));
 * extern HBin HIST_GetMissed P_((const Histogram * h));
 * extern int  HIST_GetUsedBins P_((const Histogram * h));
 */
#define HIST_GetDim(_h) (HIST_CastC(_h)->dim)
#define HIST_GetTitle(_h) (HIST_CastC(_h)->title)
#define HIST_IsOver(_h) (((HIST_CastC(_h)->flags) & H_OVERFLOW) != 0)
#define HIST_GetTotal(_h) (HIST_CastC(_h)->total)
#define HIST_GetMissed(_h) (HIST_CastC(_h)->missed)
#define HIST_GetUsedBins(_h) (HIST_CastC(_h)->usedbins)

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_HIST_H_ */

/*
 * HISTORY:
 *
 * $Log: s_hist.h,v $
 * Revision 1.14  2006/10/21 21:45:28  slava
 * o more signed/unsigned madness
 *
 * Revision 1.13  2006/10/21 21:33:05  slava
 * o fixed gcc complains about incompatible pointer types
 *
 * Revision 1.12  2006/10/20 19:25:24  slava
 * o serialization of histograms
 *
 * Revision 1.11  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.10  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.9  2002/12/09 04:22:04  slava
 * o added HIST_MaxValue and HIST_Set functions
 *
 * Revision 1.8  2002/12/07 22:38:30  slava
 * o added HIST_GetUsedBins and HIST1D_GetUsedBins macros
 *
 * Revision 1.7  2002/12/05 01:28:06  slava
 * o added HIST1D_GetLabel and HIST1D_SetLabel macros
 *
 * Revision 1.6  2002/12/01 06:48:09  slava
 * o fixed HIST1D_GetSize, HIST1D_GetMin and HIST1D_GetMax macros
 *
 * Revision 1.5  2002/12/01 03:14:38  slava
 * o added HIST_Examine function
 *
 * Revision 1.4  2002/11/30 08:02:22  slava
 * o major rewrite. more changes to follow
 *
 * Revision 1.3  2002/05/22 04:19:10  slava
 * o fixed include statements after s_sbuf.h was renamed into s_strbuf.h
 *
 * Revision 1.2  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.1  2001/01/12 06:52:53  slava
 * o support for histogramming
 *
 * Local Variables:
 * mode:C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
