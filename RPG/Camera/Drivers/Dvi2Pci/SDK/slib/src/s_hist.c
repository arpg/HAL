/*
 * $Id: s_hist.c,v 1.31 2010/12/09 12:23:11 slava Exp $
 *
 * Copyright (C) 2001-2010 by Slava Monich
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

#include "s_hist.h"
#include "s_util.h"
#include "s_futil.h"
#include "s_mem.h"

/* Flags that survive the reset */
#define H_PERMANENT_FLAGS H_AUTO_RANGE

/* values from returned by HIST_ComputePos */
#define HPOS_MISSED      -1
#define HPOS_CAN_STRETCH -2

/* serialization stuff */
#define HIST_SIGNATURE_SIZE 4
STATIC const char histSignature[] = {'H','I','S','T'};
STATIC const char histAxisSignature[] = {'A','X','I','S'};
STATIC const char histBinsSignature[] = {'B','I','N','S'};

#ifdef _HBIN_64_
#  define HBin_Read(_in,_h)     FILE_ReadMultiByte64(_in,_h)
#  define HBin_Write(_out,_h)   FILE_WriteMultiByte64(_out,_h)
#  define HBin_Size(_h)         FILE_MultiByteSize64(_h)
#else /* !_HBIN_64_ */
#  define HBin_Read(_in,_h)     FILE_ReadMultiByte32(_in,_h)
#  define HBin_Write(_out,_h)   FILE_WriteMultiByte32(_out,_h)
#  define HBin_Size(_h)         FILE_MultiByteSize32(_h)
#endif /* !_HBIN_64_ */

/*
 * These are the functions for compile time type-checking macro 
 * parameters. We only need to do this in debug build.
 */
#if DEBUG
Hist1D * HIST1D_Cast(Hist1D * h) { return h; }
Histogram * HIST_Cast(Histogram * h) { return h; }
const Hist1D * HIST1D_CastC(const Hist1D * h) { return h; }
const Histogram * HIST_CastC(const Histogram * h) { return h; }
#endif /* !DEBUG */

/* Enumeration context */
typedef struct _HistEnum {
    unsigned int nbins;        /* number of bins found so far */
    HistCB cb;                 /* the callback to call */
    void * ctx;                /* callback's context */
    HBinData data;             /* the rest of the data (variable size) */
} HistEnum;

/*==========================================================================*
 *               MULTI DIMENSIONAL HISTOGRAM
 *==========================================================================*/

/**
 * Initializes a histogram
 */
STATIC Bool HIST_Init(Histogram * h, Str s, int dim, const int n[], 
                      const double min[], const double max[])
{
    int i;
    size_t size = OFFSET(Histogram,axis) + sizeof(HistAxis)*dim;

    /* initialize the Histogram structure */
    memset(h, 0, size);

    /* calculate total number of bins */
    h->totalbins = 1;
    for (i=0; i<dim && h->totalbins; i++) {
        ASSERT(n[i] >= 0);
        h->totalbins *= n[i];
    }

    /* allocate memory for bins */
    if (h->totalbins) h->bins = MEM_NewArray(HBin,h->totalbins);
    if (!h->totalbins || h->bins) {

        /* copy the title if we have one */
        if (!s || (h->title = STRING_Dup(s)) != NULL) {

            h->dim = dim;
            for (i=0; i<dim; i++) {
                HistAxis * a = h->axis + i;
                ASSERT(min[i] < max[i]);
                a->nbins = n[i];
                a->label = NULL;
                a->min = min[i];
                a->max = max[i];
                if (a->nbins <= 0) {
                    a->nbins = 0;
                    a->flags |= H_AUTO_RANGE;
                    h->flags |= H_AUTO_RANGE;
                    a->interval = a->max - a->min;
                } else {
                    a->interval = (a->max - a->min)/a->nbins;
                }
            }

            /* this will clear the array of bins */
            HIST_Reset(h);
            return True;

        } else {
            MEM_Free(h->bins);
        }
    }
    return False;
}

/**
 * Allocates a new histogram
 */
Histogram * HIST_Create(Str s, int dim, const int n[], const double min[], 
                        const double max[])
{
    ASSERT(dim > 0);
    ASSERT(n);
    ASSERT(min);
    ASSERT(max);
    if (dim > 0 && n && min && max) {
        size_t size = OFFSET(Histogram,axis) + sizeof(HistAxis)*dim;
        Histogram * h = (Histogram*)MEM_Alloc(size);
        if (h) {
            if (HIST_Init(h, s, dim, n, min, max)) {
                return h;
            }
            MEM_Free(h);
        }
    }
    return NULL;
}

/**
 * Destroys the contents of the histogram
 */
STATIC void HIST_Destroy(Histogram * h)
{
    int i;
    MEM_Free(h->title);
    for (i=0; i<h->dim; i++) {
        MEM_Free(h->axis[i].label);
    }
    if (h->bins) {
        MEM_Free(h->bins);
        h->bins = NULL;
    }
}

/**
 * Delete the histogram
 */
void HIST_Delete(Histogram * h)
{
    if (h) {
        HIST_Destroy(h);
        MEM_Free(h);
    }
}

/**
 * Sets title for the histogram
 */
Bool HIST_SetTitle(Histogram * h, Str title)
{
    Char * copy = STRING_Dup(title);
    if (copy) {
        MEM_Free(h->title);
        h->title = copy;
        return True;
    } else {
        return False;
    }
}

/**
 * Sets the label for the i-th axis. If label is NULL, it gets reset
 * to its default value
 */
Bool HIST_SetLabel(Histogram * h, int i, Str label)
{
    ASSERT(i < h->dim);
    if (i < h->dim) {
        HistAxis * a = h->axis + i;
        MEM_Free(a->label);
        if (label) {
            a->label = STRING_Dup(label);
            return BoolValue(a->label != NULL);
        } else {
            a->label = NULL;
            return True;
        }
    }
    return False;
}

/**
 * Returns the label for the i-th axis. Unless set externally, the axis
 * are labeled X, Y and Z for dimensions <= 3, or X1, X2, X3, X4, ... for
 * dimensions >= 4 
 */
Str HIST_GetLabel(const Histogram * h, int i)
{
    ASSERT(i < h->dim);
    if (i < h->dim) {
        const HistAxis * a = h->axis + i;
        if (!a->label) {
            static Str STANDARD_LABELS [] = {TEXT("X"), TEXT("Y"), TEXT("Z")};
            if (h->dim <= COUNT(STANDARD_LABELS)) {
                return STANDARD_LABELS[i];
            } else {
                /* allocate default label */
                HistAxis * axis = (HistAxis*)a; /* cheating... */
                StrBuf32 buf;
                STRBUF_InitBufXXX(&buf);
                STRBUF_Format(&buf.sb, TEXT("X%d"), (i+1));
                axis->label = STRBUF_Dup(&buf.sb);
                STRBUF_Destroy(&buf.sb);
            }
        }
        return a->label;
    }
    return NULL;
}

/**
 * Resets one axis of the histogram.
 */
STATIC void HIST_ResetAxis(HistAxis * a)
{
    a->flags &= H_PERMANENT_FLAGS;
    if (a->flags & H_AUTO_RANGE) {
        a->max = a->min;
        a->nbins = 0;
    }
}

/**
 * Resets the histogram.
 */
void HIST_Reset(Histogram * h)
{
    int i;
    h->missed  = 
    h->total = 0;
    h->usedbins = 0;
    h->maxval = 0;
    h->flags &= H_PERMANENT_FLAGS;
    h->flags |= H_MAX_VALUE;
    for (i=0; i<h->dim; i++) {
        HIST_ResetAxis(h->axis + i);
    }
    if (h->flags & H_AUTO_RANGE) {
        MEM_Free(h->bins);
        h->bins = NULL;
        h->totalbins = 0;
    } else {
        memset(h->bins, 0, sizeof(HBin) * h->totalbins);
    }
}

/**
 * Returns the minimum value.
 */
double HIST_GetMin(const Histogram * h, int i)
{
    ASSERT(h);
    ASSERT(i >= 0);
    ASSERT(i < h->dim);
    if (h && i >= 0 && i < h->dim) {
        return h->axis[i].min;
    }
    return 0.0;
}

/**
 * Returns the maximum value.
 */
double HIST_GetMax(const Histogram * h, int i)
{
    ASSERT(h);
    ASSERT(i >= 0);
    ASSERT(i < h->dim);
    if (h && i >= 0 && i < h->dim) {
        return h->axis[i].max;
    }
    return 0.0;
}

/**
 * Returns number of bins along ith coordinate.
 */
int HIST_GetSize(const Histogram * h, int i)
{
    ASSERT(h);
    ASSERT(i >= 0);
    ASSERT(i < h->dim);
    if (h && i >= 0 && i < h->dim) {
        return h->axis[i].nbins;
    }
    return 0;
}

/**
 * Tests if the specified value falls into the histogram range
 */
Bool HIST_IsIn(const Histogram * h, const double value [])
{
    ASSERT(h);
    if (h) {
        int i;
        for (i=0; i<h->dim; i++) {
            /* If the axis has H_AUTO_RANGE flag set, everything is in */
            const HistAxis * a = h->axis + i;
            if (!(a->flags & H_AUTO_RANGE)) {
                if (value[i] < a->min || value[i] >= a->max) {
                    return False;
                }
            }
        }
        return True;
    }
    return False;
}

/**
 * Recursive helper for HIST_Examine
 */
STATIC void HIST_Examine2(HistEnum * e, int k)
{
    const Histogram * h = e->data.hist;
    const HistAxis * a = h->axis + k;
    unsigned int * bin = e->data.bin;
    if ((k+1) == e->data.hist->dim) {

        /* calculate start index */
        int i;
        int pos = 0;
        int step = h->totalbins;
        for (i=0; i<(h->dim-1); i++) {
            step /= h->axis[i].nbins;
            pos += bin[i]*step;
        }

        /* scan the bins */
        for (bin[k]=0; e->nbins < h->usedbins && bin[k]<a->nbins; bin[k]++) {
            e->data.count = h->bins[pos + bin[k]];
            if (e->data.count) {
                e->nbins++;
                if (!e->cb(&e->data, e->ctx)) {
                    /* stop the enumeration */
                    e->nbins = h->usedbins;
                }
            }
        }

    } else {
        for (bin[k]=0; e->nbins < h->usedbins && bin[k]<a->nbins; bin[k]++) {
            /* scan the next dimension */
            HIST_Examine2(e, k+1);
        }
    }
}

/**
 * Calls the specified callback function for each non-empty bin of the 
 * histogram. This is more efficient than requesting the content of every 
 * single bin with HIST_Get
 */
void HIST_Examine(const Histogram * h, HistCB cb, void * ctx)
{
    if (h->total > 0) {
        int size = sizeof(HistEnum) + (h->dim-1)*sizeof(int);
        HistEnum * e = (HistEnum*)MEM_Alloc(size);
        if (e) {
            memset(e, 0, size);
            e->cb = cb;
            e->ctx = ctx;
            e->data.hist = h;
            HIST_Examine2(e, 0);
            MEM_Free(e);
        }
    }
}

/**
 * Increments the specified bin value and sets "overflow" flag if it's 
 * already reached MAX_BIN_VALUE value. ASSERTs when histogram is overflown 
 * for the first time
 */
STATIC HBin HIST_Inc(Histogram * h, HBin x)
{
    if (x == MAX_HBIN_VALUE) {
        /* 
         * Histograms should not overflow. They are pretty much useless when
         * they do. This ASSERT only fires once when overflow occurs for the 
         * first time.
         */
        ASSMSG("Histogram overflow!");   
        h->flags |= H_OVERFLOW;
    } else {
        x++;
    }
    return x;
}

/**
 * Calculates the index of the bin where we put the specified value.
 * If the value is out of bounds, returns negative value
 *
 * Should this be a macro?
 */
STATIC int HIST_BinIndex(const HistAxis * a, double x)
{
    if (x >= a->min && x < a->max && a->nbins > 0) {
        return (int)((x - a->min)/a->interval);
    } else {
        return -1;
    }
}

/**
 * Returns the position of the bin that matches the specified value.
 * If the value if out of bounds of the histogram, one of the following
 * (negative) values is returned:
 *
 * HPOS_MISSED (-1)       this is a miss
 * HPOS_CAN_STRETCH (-2)  we missed but the histogram is configured such that
 *                        we may dynamically stretch it and count this value 
 */
STATIC int HIST_ComputePos(const Histogram * h, const double value[])
{
    int i;
    int pos = 0;
    int step = h->totalbins;
    Bool missed = False;
    for (i=0; i<h->dim; i++) {
        const HistAxis * a = h->axis + i;
        int n = 0;
        if (!a->nbins || (n = HIST_BinIndex(a, value[i])) < 0) {
            if (a->flags & H_AUTO_RANGE) {
                missed = True;
            } else {
                /* it's out, cannot stretch */
                return HPOS_MISSED;
            }
        }
        if (step) {
            step /= a->nbins;
            pos += n*step;
        }
    }
    return (missed ? HPOS_CAN_STRETCH : pos);
}

/**
 * Recursive helper for resampling the histogram.
 */
STATIC void HIST_Stretch2(
    const Histogram * h,    /* the histogram beig stretched */
    HBin * newbuf,          /* new histogram buffer */
    int i,                  /* axis index */
    const int k[],          /* bins added, + up, - down */
    int start1,             /* start of the region in the old buffer */
    int start2,             /* start of the region in the new buffer */
    int size1,              /* size of the region in the old buffer */
    int size2)              /* size of the region in the new buffer */
{
    int n = ABS(k[i]);
    const HistAxis * a = h->axis + i;
    int nbins = a->nbins + n;
    int nextSize1 = size1/a->nbins;
    int nextSize2 = size2/nbins;
    int skip = nextSize2 * n;
    int offset = 0;

    ASSERT(!(size1%a->nbins));
    ASSERT(!(size2%nbins));

    /* clear unused part of the buffer */
    if (k[i] < 0) {
        memset(newbuf + start2, 0, sizeof(HBin) * skip);
        offset = skip;
    } else if (k[i] > 0) {
        memset(newbuf + start2 + size2 - skip, 0, sizeof(HBin) * skip);
    }
    
    if (i == (h->dim-1)) {
        /* copy data from the old buffer */
        int copyBytes = sizeof(HBin) * a->nbins;
        memcpy(newbuf + start2 + offset, h->bins + start1, copyBytes);
    } else {
        /* recurse */
        unsigned int j;
        for (j=0; j<a->nbins; j++) {
            HIST_Stretch2(h, newbuf, i+1, k, 
                          start1 + j * nextSize1,
                          start2 + offset + j * nextSize2, 
                          nextSize1, nextSize2);
        }
    }
}

/**
 * Resamples the histogram so that the specified value would fit in.
 * Does not actually updates the statistics. Returns False if it fails 
 * to reallocate the buffer.
 */
STATIC Bool HIST_Stretch(Histogram * h, const double value[])
{
    int k0;
    int * k = ((h->dim == 1) ? &k0 : MEM_NewArray(int,h->dim));
    if (k) {
        HBin * newbuf;
        
        /* determine the amount of memory we need */
        int i;
        size_t n = 1;
        memset(k, 0, sizeof(*k) * h->dim);
        for (i=0; i<h->dim; i++) {
            const HistAxis * a = h->axis + i;
            if (HIST_BinIndex(a, value[i]) >= 0) {
                n *= a->nbins;
            } else {
                const double x = value[i];
                ASSERT(a->flags & H_AUTO_RANGE);

                /* first check for overflow */
                if (x >= a->max) {
                    if ((x - a->min)/a->interval > (double)(INT_MAX-1)) {
                        if (h->dim > 1) MEM_Free(k);
                        return False;
                    }
                } else {
                    ASSERT(x < a->min);
                    if ((a->max - x)/a->interval > (double)(INT_MAX-1)) {
                        if (h->dim > 1) MEM_Free(k);
                        return False;
                    }
                }

                if (a->nbins) {
                    if (x >= a->max) {
                        k[i] = (int)((x - a->max)/a->interval)+1;
                        ASSERT(k[i] > 0);
                    } else {
                        k[i] = (int)((x - a->min)/a->interval);
                        if (x < (a->min+k[i]*a->interval)) k[i]--;
                    }
                    n *= a->nbins + ABS(k[i]);
                } else {
                    k[i] = 1;
                }
            }
        }

        /* allocate new buffer and resample the histogram */
        ASSERT(n > h->totalbins);
        newbuf = MEM_NewArray(HBin,n);
        if (newbuf) {
            if (!h->totalbins) {
                /* the first hit */
                memset(newbuf, 0, sizeof(HBin) * n);
            } else {
                HIST_Stretch2(h, newbuf, 0, k, 0, 0, h->totalbins, n);
            }
                
            /* update the axis */
            for (i=0; i<h->dim; i++) {
                HistAxis * a = h->axis + i;
                if (!a->nbins) {
                    ASSERT(a->min == a->max);
                    if (value[i] >= a->min) {
                        int add = (int)((value[i] - a->min)/a->interval);
                        a->max = a->min = a->min + a->interval * add;
                    } else {
                        int add = (int)((a->min - value[i])/a->interval) + 1;
                        a->max = a->min = a->min - a->interval * add;
                    }
                }
                if (k[i] > 0) {
                    a->nbins += k[i];
                    a->max += k[i] * a->interval;
                } else if (k[i] < 0) {
                    a->nbins -= k[i];
                    a->min += k[i] * a->interval;
                }
            }

            /* switch to the new buffer */
            if (h->dim > 1) MEM_Free(k);
            MEM_Free(h->bins);
            h->bins = newbuf;
            h->totalbins = n;
            return True;
        }
        if (h->dim > 1) MEM_Free(k);
    }
    return False;
}

/**
 * Adds another value to the statistics. If the value is outside of the
 * histogram's range, incremented the "missed" count, otherwise increments
 * the "total" count and the value of the appropriate bin. Returns True is
 * the value was counted, False if overflow has occured.
 */
Bool HIST_Put(Histogram * h, const double value[])
{
    if (!(h->flags & H_OVERFLOW)) {
        int pos = HIST_ComputePos(h, value);

        /* stretch the histogram if necessary */
        if (pos == HPOS_CAN_STRETCH) {

            /* resample the histogram */
            if (HIST_Stretch(h, value)) {

                /* recalculate the position. This time it must succeed */
                pos = HIST_ComputePos(h, value);
                ASSERT(pos >= 0);
            } else {
                /* count this as a miss */
                pos = HPOS_MISSED;
            }
        }

        if (pos == HPOS_MISSED) {

            /* 
             * do not set the "overflow" flag if "miss" count gets 
             * overflown. This counter usually does not affect the 
             * statistics we are collecting except for some rare cases. 
             * If we will ever need this counter for anything rather 
             * than strictly informational purposes, this behavior can 
             * be made configurable.
             */
            if (h->missed != MAX_HBIN_VALUE) {
                h->missed++;
            }
        } else {
            HBin binValue = HIST_Inc(h, h->bins[pos]);
            h->total = HIST_Inc(h, h->total);
            if (!h->bins[pos]) h->usedbins++;
            if ((h->flags & H_MAX_VALUE) && (binValue > h->maxval)) {
                h->maxval = binValue;
            }
            h->bins[pos] = binValue;
            if (!(h->flags & H_OVERFLOW)) {
                return True;
            } else {
                return False;
            }
        }
    }
    return False;
}

/**
 * Returns the index of the specified bin.
 */
STATIC int HIST_FlatBinIndex(const Histogram * h, const int n [])
{
    ASSERT(h);
    if (h) {
        unsigned int i, pos = 0, step = h->totalbins;
        for (i=0; i<h->dim; i++) {
            ASSERT(n[i] >= 0 && ((unsigned int)(n[i])) < h->axis[i].nbins);
            if (n[i] >= 0 && ((unsigned int)(n[i])) < h->axis[i].nbins) {
                step /= h->axis[i].nbins;
                pos += n[i]*step;
            } else {
                return -1;
            }
        }
        ASSERT(pos < h->totalbins);
        return pos;
    }
    return -1;
}

/**
 * Returns the contents of the specified bin.
 */
HBin HIST_Get(const Histogram * h, const int i [])
{
    int pos = HIST_FlatBinIndex(h, i);
    return ((pos >= 0) ? h->bins[pos] : 0);
}

/**
 * Sets the contents of the specified bin
 */
void HIST_Set(Histogram * h, const int i [], HBin value)
{
    long pos = HIST_FlatBinIndex(h, i);
    if (pos >= 0) {
        HBin oldval = h->bins[pos];
        if (oldval != value) {
            h->bins[pos] = value;
            if (value == 0) {
                h->usedbins--;
            } else if (h->flags & H_MAX_VALUE) {
                if (value > h->maxval) {
                    /* update maximum value */
                    h->maxval = value;
                } else if (oldval == h->maxval) {
                    /* maximum value is unknown */
                    h->flags &= ~H_MAX_VALUE;
                }
            }
        }
    }
}

/**
 * Returns the maximum bin value. Note that in some cases this function
 * has to calculate this value, i.e. h->maxval does not always contains
 * the right value.
 */
HBin HIST_MaxValue(const Histogram * h)
{
    if (!(h->flags & H_MAX_VALUE)) {
        unsigned int count = 0;
        const HBin * p;
        HBin maxval = 0;
        for (p = h->bins; count<h->usedbins; p++) {
            if (*p) {
                if (*p > maxval) maxval = *p;
                count++;
            }
        }
        ((Histogram*)h)->maxval = maxval;
        ((Histogram*)h)->flags |= H_MAX_VALUE;
    }
    return h->maxval;
}

/**
 * Functions below write histogram to a binary stream. Stream starts with
 * the histogram header:
 *
 *   +-------+-------+-------+-------+
 *   |  'H'  |  'I'  |  'S'  |  'T'  |
 *   +-------+-------+-------+-------+
 *   | (MB) size of remaining part of the header
 *   +-------+-------+-------+-------
 *   |    <--|-- sizeof(HBin), one byte
 *   +-------+-------+-------+-------
 *   | (MB) number of dimensions
 *   +-------+-------+-------+-------
 *   | (MB) flags (H_OVERFLOW)
 *   +-------+-------+-------+-------
 *   | (MB) number of attempts to put an out-of-bounds value
 *   +-------+-------+-------+-------
 *   | Zero-terminated histogram title
 *   +-------+-------+-------+-------
 *
 * The histogram header is followed by a number of axis records. Number of
 * these records equals number of dimensions in the histogram header.
 *
 *   +-------+-------+-------+-------+
 *   |  'A'  |  'X'  |  'I'  |  'S'  |
 *   +-------+-------+-------+-------+
 *   | (MB) size of remaining part of the header
 *   +-------+-------+-------+-------
 *   | (MB) number of bins along this axis
 *   +-------+-------+-------+-------+-------+-------+-------+-------+
 *   | Lower edge, BIG endian double                                 |
 *   +-------+-------+-------+-------+-------+-------+-------+-------+
 *   | Upper edge, BIG endian double                                 |
 *   +-------+-------+-------+-------+-------+-------+-------+-------+
 *   | (MB) flags (H_AUTO_RANGE)
 *   +-------+-------+-------+-------
 *   | Zero-terminated label
 *   +-------+-------+-------+-------
 *
 * And finally, here comes the contents of the histogram. Number of entries
 * must match histogram dimension and number of intervals along each axis.
 *
 *   +-------+-------+-------+-------+
 *   |  'B'  |  'I'  |  'N'  |  'S'  |
 *   +-------+-------+-------+-------+
 *   | (MB) size of remaining part of the header
 *   +-------+-------+-------+-------
 *   | (MB) bin values...
 *   +-------+-------+-------+-------
 */

STATIC Bool HIST_WriteHeader(const Histogram * h, File * out)
{
    if (FILE_WriteAll(out, histSignature, HIST_SIGNATURE_SIZE)) {
        char * title = 
#ifdef UNICODE
            STRING_ToMultiByte(h->title);
#else  /* !UNICODE */
            h->title;
#endif /* !UNICODE */
        int titleLen = (title ? strlen(title) : 0);
        I8u sizeofBin = sizeof(HBin);
        I16u flags = h->flags & (~(H_OVERFLOW|H_MAX_VALUE));
        size_t nbytes = 2 + titleLen +
            FILE_MultiByteSizeInt(h->dim) +
            FILE_MultiByteSizeInt(flags) +
            FILE_MultiByteSize64(h->missed);

        if (FILE_WriteMultiByteLong(out, nbytes)) {
#if DEBUG_TRACE
            size_t byteCount = FILE_BytesWritten(out);
#endif /* DEBUG_TRACE */
            if (FILE_WriteAll(out, &sizeofBin, 1) &&
                FILE_WriteMultiByteInt(out, h->dim) &&
                FILE_WriteMultiByteInt(out,flags) &&
                FILE_WriteMultiByte64(out, h->missed) &&
                FILE_WriteAll(out, title ? title : "", titleLen+1)) {
#ifdef UNICODE
                MEM_Free(title);
#endif /* UNICODE */
                ASSERT((FILE_BytesWritten(out)-byteCount) == nbytes);
                return True;
            }
        }
#ifdef UNICODE
        MEM_Free(title);
#endif /* UNICODE */
    }
    return False;
}

STATIC Bool HIST_WriteAxis(const HistAxis * a, File * out)
{
    if (FILE_WriteAll(out, histAxisSignature, HIST_SIGNATURE_SIZE)) {
        char * label = 
#ifdef UNICODE
            STRING_ToMultiByte(a->label);
#else  /* !UNICODE */
            a->label;
#endif /* !UNICODE */
        int labelLen = (label ? strlen(label) : 0);
        size_t nbytes = 2 * sizeof(double) + labelLen + 1 +
            FILE_MultiByteSizeSizeT(a->nbins) + 
            FILE_MultiByteSizeInt(a->flags);

        if (FILE_WriteMultiByteLong(out, nbytes)) {
#if DEBUG_TRACE
            size_t byteCount = FILE_BytesWritten(out);
#endif /* DEBUG_TRACE */
            if (FILE_WriteMultiByteSizeT(out, a->nbins) &&
                FILE_WriteF64B(out, a->min) &&
                FILE_WriteF64B(out, a->max) &&
                FILE_WriteMultiByteInt(out, a->flags) &&
                FILE_WriteAll(out, label ? label : "", labelLen+1)) {
#ifdef UNICODE
                MEM_Free(label);
#endif /* UNICODE */
                ASSERT((FILE_BytesWritten(out)-byteCount) == nbytes);
                return True;
            }
        }
#ifdef UNICODE
        MEM_Free(label);
#endif /* UNICODE */
    }
    return False;
}

STATIC Bool HIST_WriteAxes(const Histogram * h, File * out)
{
    I16u i;
    for (i=0; i<h->dim; i++) {
        if (!HIST_WriteAxis(h->axis + i, out)) {
            return False;
        }
    }
    return True;
}

STATIC Bool HIST_WriteValues(const Histogram * h, File * out)
{
    /* calculate total size of the block */
    size_t i, nbytes = 0;
    for (i=0; i<h->totalbins; i++) {
        nbytes += HBin_Size(h->bins[i]);
    }

    if (FILE_WriteAll(out, histBinsSignature, HIST_SIGNATURE_SIZE) &&
        FILE_WriteMultiByteLong(out, nbytes)) {
#if DEBUG_TRACE
        size_t byteCount = FILE_BytesWritten(out);
#endif /* DEBUG_TRACE */
        for (i=0; i<h->totalbins; i++) {
            if (!HBin_Write(out, h->bins[i])) {
                return False;
            }
        }
        ASSERT((FILE_BytesWritten(out)-byteCount) == nbytes);
        return True;
    }
    return False;
}


/**
 * Writes n-dimentional histogram to the stream. See comments above on the
 * format of the stream
 */
Bool HIST_Write(const Histogram * h, File * out)
{
    return HIST_WriteHeader(h, out) &&
           HIST_WriteAxes(h, out) &&
           HIST_WriteValues(h, out);
}

/**
 * HIST_Save callback
 */
STATIC Bool HIST_FileSaveCB(File * out, Str fname, void * ctx)
{
    return HIST_Write(ctx, out);
}

/**
 * Saves histogram to a binary file
 */
Bool HIST_Save(const Histogram * h, Str fname, IODesc io)
{
    /* Note: FILE_Save write text files, we must use FILE_Save2 */
    return FILE_Save2(fname, HIST_FileSaveCB, (void*)h, False, io);
}

/**
 * Reads zero-terminated UTF-8 string from the stream.
 */
STATIC Char * HIST_ReadString(File * in)
{
    int c;
    Char * s;
    StrBuf32 buf;
    STRBUF_InitBufXXX(&buf);
    while ((c = FILE_Getc(in)) > 0) {
        if (!STRBUF_AppendChar(&buf.sb, (char)c)) {
            STRBUF_Destroy(&buf.sb);
            return NULL;
        }
    }
    s = ((c == 0) ? STRBUF_Dup(&buf.sb) : NULL);
    STRBUF_Destroy(&buf.sb);
    return s;
}

/**
 * Functions below read the histogram header from the stream, allocate
 * and initialize the histogram object.
 *
 *   +-------+-------+-------+-------+
 *   |  'H'  |  'I'  |  'S'  |  'T'  |
 *   +-------+-------+-------+-------+
 *   | (MB) size of remaining part of the header
 *   +-------+-------+-------+-------
 *   |    <--|-- sizeof(HBin), one byte
 *   +-------+-------+-------+-------
 *   | (MB) number of dimensions
 *   +-------+-------+-------+-------
 *   | (MB) flags (H_OVERFLOW)
 *   +-------+-------+-------+-------
 *   | (MB) number of attempts to put an out-of-bounds value
 *   +-------+-------+-------+-------
 *   | Zero-terminated histogram title
 *   +-------+-------+-------+-------
 *
 * The Histogram object returned by this function is not fully initialized.
 * It only contains the fields found in the histogram stream header.
 */
STATIC Histogram * HIST_ReadHeaderStream(File * in)
{
    int dim;
    int binSize = FILE_Getc(in);
    if (binSize == sizeof(HBin) && FILE_ReadMultiByteInt(in,&dim) && dim>0) {
        size_t size = OFFSET(Histogram,axis) + sizeof(HistAxis)*dim;
        Histogram * h = MEM_Alloc(size);
        if (h) {
            I32u flags;
            memset(h, 0, size);
            h->dim = dim;
            if (FILE_ReadMultiByte32(in, &flags) &&
                HBin_Read(in, &h->missed)) {
                h->title = HIST_ReadString(in);
                if (h->title) {
                    h->flags = (I16u)flags;
                    return h;
                }
            }
            MEM_Free(h);
        }
    }
    return NULL;
}

STATIC Histogram * HIST_ReadHeader(File * in)
{
    char signature[HIST_SIGNATURE_SIZE];
    if (FILE_ReadAll(in,signature,sizeof(signature))) {
        if (!memcmp(signature, histSignature, sizeof(signature))) {
            size_t headerSize;
            if (FILE_ReadMultiByteSizeT(in, &headerSize)) {
                File * hstream = FILE_SubStream(in, headerSize);
                if (hstream) {
                    Histogram * h = HIST_ReadHeaderStream(hstream);
                    FILE_Skip(hstream, headerSize - FILE_BytesRead(hstream));
                    FILE_Detach(hstream);
                    FILE_Close(hstream);
                    return h;
                }
            }
        }
    } else {
        FILE_PushBack(in, signature,sizeof(signature));
    }
    return NULL;
}

/**
 * Functions below read the axis blocks from the stream. Each block has the
 * following format:
 *
 *   +-------+-------+-------+-------+
 *   |  'A'  |  'X'  |  'I'  |  'S'  |
 *   +-------+-------+-------+-------+
 *   | (MB) size of remaining part of the header
 *   +-------+-------+-------+-------
 *   | (MB) number of bins along this axis
 *   +-------+-------+-------+-------+-------+-------+-------+-------+
 *   | Lower edge, BIG endian double                                 |
 *   +-------+-------+-------+-------+-------+-------+-------+-------+
 *   | Upper edge, BIG endian double                                 |
 *   +-------+-------+-------+-------+-------+-------+-------+-------+
 *   | (MB) flags (H_AUTO_RANGE)
 *   +-------+-------+-------+-------
 *   | Zero-terminated label
 *   +-------+-------+-------+-------
 */
STATIC Bool HIST_ReadAxesBlock(HistAxis * a, File * in)
{
    if (FILE_ReadMultiByteSizeT(in, &a->nbins) && a->nbins > 0 &&
        FILE_ReadF64B(in, &a->min) && 
        FILE_ReadF64B(in, &a->max) && a->min < a->max &&
        FILE_ReadMultiByteInt(in, &a->flags) &&
        (a->label = HIST_ReadString(in)) != NULL) {
        return True;
    }
    return False;
}

STATIC Bool HIST_ReadAxis(Histogram * h, File * in)
{
    I16u i;
    h->totalbins = 1;
    for (i=0; i<h->dim; i++) {
        char signature[HIST_SIGNATURE_SIZE];
        if (FILE_ReadAll(in,signature,sizeof(signature))) {
            if (!memcmp(signature, histAxisSignature, sizeof(signature))) {
                size_t blockSize;
                if (FILE_ReadMultiByteSizeT(in, &blockSize)) {
                    File * astream = FILE_SubStream(in, blockSize);
                    if (astream) {
                        Bool ok = HIST_ReadAxesBlock(h->axis + i, astream);
                        FILE_Skip(astream, blockSize - FILE_BytesRead(astream));
                        FILE_Detach(astream);
                        FILE_Close(astream);
                        if (ok) {
                            h->totalbins *= h->axis[i].nbins;
                            continue;
                        }
                    }
                }
            }
        }
        return False;
    }

    /* now we know how much memory we need for the histogram contents */
    h->bins = MEM_NewArray(HBin,h->totalbins);
    if (h->bins) {
        memset(h->bins, 0, sizeof(HBin) * h->totalbins);
        return True;
    }
    return False;
}

/**
 * Reads the histogram values from the stream.
 *
 *   +-------+-------+-------+-------+
 *   |  'B'  |  'I'  |  'N'  |  'S'  |
 *   +-------+-------+-------+-------+
 *   | (MB) size of remaining part of the header
 *   +-------+-------+-------+-------
 *   | (MB) bin values...
 *   +-------+-------+-------+-------
 */
STATIC Bool HIST_ReadContents(Histogram * h, File * in)
{
    size_t i;
    HBin * b = h->bins;
    for (i=0; i<h->totalbins; i++, b++) {
        if (HBin_Read(in, b)) {
            if (*b) {
                h->usedbins++;
                if ((MAX_HBIN_VALUE - h->total) >= (*b)) {
                    h->total += *b;
                } else {
                    ASSMSG("Histogram overflow!");   
                    h->total = MAX_HBIN_VALUE;
                    h->flags |= H_OVERFLOW;
                }
                if (*b > h->maxval) {
                    h->maxval = *b;
                }
            }
            continue;
        }
        return False;
    }
    h->flags |= H_MAX_VALUE;
    return True;
}

STATIC Bool HIST_ReadValues(Histogram * h, File * in)
{
    char signature[HIST_SIGNATURE_SIZE];
    if (FILE_ReadAll(in,signature,sizeof(signature))) {
        if (!memcmp(signature, histBinsSignature, sizeof(signature))) {
            size_t headerSize;
            if (FILE_ReadMultiByteSizeT(in, &headerSize)) {
                File * hstream = FILE_SubStream(in, headerSize);
                if (hstream) {
                    Bool ok = HIST_ReadContents(h, hstream) &&
                        (headerSize == FILE_BytesRead(hstream));
                    FILE_Detach(hstream);
                    FILE_Close(hstream);
                    return ok;
                }
            }
        }
    }
    return False;
}

/**
 * Reads the histogram from the stream.
 */
Histogram * HIST_Read(File * in)
{
    Histogram * h = HIST_ReadHeader(in);
    if (h) {
        if (HIST_ReadAxis(h, in) &&
            HIST_ReadValues(h, in)) {
            return h;
        }
        HIST_Delete(h);
    }
    return NULL;
}

/**
 * Reads histogram from the binary file specified by name.
 */
Histogram * HIST_Load(Str fname, IODesc io)
{
    Histogram * h = NULL;
    File * in = FILE_Open(fname, READ_BINARY_MODE, io);
    if (in) {
        h = HIST_Read(in);
        FILE_Close(in);
    } else {
        TRACE1("SLIB: cannot open file %s\n",fname);
    }
    return h;
}

/*==========================================================================*
 *               ONE DIMENSIONAL HISTOGRAM
 *==========================================================================*/

/**
 * Allocates and initializes one-dimensional histogram
 */
Hist1D * HIST1D_Create(Str s, int n, double min, double max)
{
    Hist1D * h = MEM_New(Hist1D);
    if (h) {
        if (HIST1D_Init(h, s, n, min, max)) {
            return h;
        }
        MEM_Free(h);
    }
    return  NULL;
}

/**
 * Initializes one-dimensional histogram
 */
Bool HIST1D_Init(Hist1D * h, Str s, int n, double min, double max)
{
    return HIST_Init(&h->hist, s, 1, &n, &min, &max);
}

/**
 * Deallocates all the memory referenced by the histogram.
 */
void HIST1D_Destroy(Hist1D * h)
{
    HIST_Destroy(&h->hist);
}

/**
 * Deallocates all the memory referenced by the histogram and deallocates 
 * the histogram itself. The histogram is assumed to have been created 
 * with HIST1D_Create() call.
 */
void HIST1D_Delete(Hist1D * h)
{
    if (h) {
        ASSERT(h->hist.dim == 1);
        HIST1D_Destroy(h);
        MEM_Free(h);
    }
}

/**
 * Tests if the specified value falls into the histogram range
 */
Bool HIST1D_IsIn(const Hist1D * h, double value)
{
    ASSERT(h->hist.dim == 1);
    if (value >= h->hist.axis[0].min && value < h->hist.axis[0].max) {
        return True;
    } else {
        return False;
    }
}

/**
 * Adds another value to the statistics. If the value is outside of the
 * histogram's range, incremented the "missed" count, otherwise increments
 * the "total" count and the value of the appropriate bin. Returns True is
 * the value was counted, False if overflow has occured.
 */
Bool HIST1D_Put(Hist1D * h, double value)
{
    ASSERT(h->hist.dim == 1);
    return HIST_Put(&h->hist, &value);
}

/**
 * Puts multiple values into the histogram. Returns True on success, False if
 * an overflow occurs.
 */
Bool HIST1D_PutAll(Hist1D * h, const double * values, int count)
{
    int i;
    ASSERT(h->hist.dim == 1);
    for (i=0; i<count; i++) {
        if (!HIST1D_Put(h, values[i])) {
            return False;
        }
    }
    return True;
}

/**
 * Returns the contents of the ith bin.
 */
HBin HIST1D_Get(const Hist1D * h, int i)
{
    ASSERT(i >= 0);
    ASSERT(((unsigned int)i) < h->hist.axis[0].nbins);
    ASSERT(h->hist.dim == 1);
    if (i >= 0 && ((unsigned int)i) < h->hist.axis[0].nbins) {
        return h->hist.bins[i];
    }
    ASSMSG2("HIST1D: Index %d out of bounds 0..%lu",i,
        (unsigned long)h->hist.axis[0].nbins-1);
    return 0;
}

/**
 * Sets the value of the i-th bin
 */
void HIST1D_Set(Hist1D * h, int i, HBin value)
{
    ASSERT(h->hist.dim == 1);
    HIST_Set(&h->hist, &i, value);    
}

/**
 * Returns number of bins in one-dimensional histogram
 */
int HIST1D_GetSize(const Hist1D * h)
{
    ASSERT(h->hist.dim == 1);
    return HIST_GetSize(&h->hist, 0);
}

/**
 * Reads one-dimentional histogram from the stream
 */
COMPILE_ASSERT(OFFSET(Hist1D,hist) == 0);
Hist1D * HIST1D_Read(File * in)
{
    Histogram * h = HIST_Read(in);
    if (h) {
        if (h->dim == 1) {
            return (Hist1D*)h;
        } else {
            TRACE1("SLIB: unexpected number of demensions (%d)\n",h->dim);
            HIST_Delete(h);
        }
    }
    return NULL;
}

/**
 * Writes one-dimentional histogram to the stream
 */
Bool HIST1D_Write(const Hist1D * h, File * out)
{
    ASSERT(h->hist.dim == 1);
    return HIST_Write(&h->hist, out);
}

/**
 * Saves histogram to a binary file
 */
Bool HIST1D_Save(const Hist1D * h, Str fname, IODesc io)
{
    /* Note: FILE_Save write text files, we must use FILE_Save2 */
    return FILE_Save2(fname, HIST_FileSaveCB, (void*)&h->hist, False, io);
}

/**
 * Reads histogram from the binary file specified by name.
 */
Hist1D * HIST1D_Load(Str fname, IODesc io)
{
    Hist1D * h = NULL;
    File * in = FILE_Open(fname, READ_BINARY_MODE, io);
    if (in) {
        h = HIST1D_Read(in);
        FILE_Close(in);
    } else {
        TRACE1("SLIB: cannot open file %s\n",fname);
    }
    return h;
}

/*
 * HISTORY:
 *
 * $Log: s_hist.c,v $
 * Revision 1.31  2010/12/09 12:23:11  slava
 * o minor tweaks
 *
 * Revision 1.30  2010/09/28 19:47:12  slava
 * o fixing format/argument mismatch issues
 *
 * Revision 1.29  2010/09/25 09:55:06  slava
 * o added PRINTF_ATTR macro which assigns printf-like characteristics to the
 *   declared function and enables gcc to check the format string against the
 *   parameters.
 *
 * Revision 1.28  2009/03/25 22:08:04  slava
 * o made it easier to compile release version of slib with debug trace
 *
 * Revision 1.27  2007/03/16 01:22:47  slava
 * o removed erroneous assert
 *
 * Revision 1.26  2007/03/15 22:51:57  slava
 * o removed unused variable
 *
 * Revision 1.25  2007/03/15 17:57:01  slava
 * o better way to handle integer overflow
 *
 * Revision 1.24  2007/03/15 02:03:42  slava
 * o handle overflow in HIST_Stretch
 *
 * Revision 1.23  2006/11/03 17:06:49  slava
 * o special handling of size_t data type. Linux defines it as
 *   "unsigned long int" which gcc compiler considers different
 *   enough from "unsigned long" to generate tons of useless warnings.
 *
 * Revision 1.22  2006/10/21 21:50:58  slava
 * o and more incompatible pointer types...
 *
 * Revision 1.21  2006/10/21 21:45:29  slava
 * o more signed/unsigned madness
 *
 * Revision 1.20  2006/10/21 21:33:05  slava
 * o fixed gcc complains about incompatible pointer types
 *
 * Revision 1.19  2006/10/20 23:02:45  slava
 * o cleaned up comments a bit
 *
 * Revision 1.18  2006/10/20 22:52:45  slava
 * o fixed gcc compilation warning in HIST1D_Read. COMPILE_ASSERT was inside
 *   the function, causing "nested extern declaration of `_compile_time_assert'"
 *   warning.
 *
 * Revision 1.17  2006/10/20 19:25:25  slava
 * o serialization of histograms
 *
 * Revision 1.16  2005/02/19 03:36:13  slava
 * o fixed compilation warnings produced by C++ compiler
 *
 * Revision 1.15  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.14  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.13  2003/01/24 02:38:18  slava
 * o cleaned up ASSERT messages (don't need newline)
 *
 * Revision 1.12  2003/01/05 17:32:05  slava
 * o fixed compilation error in Windows CE environment
 *
 * Revision 1.11  2002/12/14 18:28:39  slava
 * o fixed gcc compilation warnings
 *
 * Revision 1.10  2002/12/09 04:22:04  slava
 * o added HIST_MaxValue and HIST_Set functions
 *
 * Revision 1.9  2002/12/05 01:32:14  slava
 * o implemented dynamic resizing. if number of bins is zero, the axis is
 *   assumed to be dynamically resizable and the difference between the
 *   minimum and the maximum is interpreted as the width of a single bin
 *
 * Revision 1.8  2002/12/01 03:14:38  slava
 * o added HIST_Examine function
 *
 * Revision 1.7  2002/11/30 08:11:09  slava
 * o fixed a warning (unused variable)
 *
 * Revision 1.6  2002/11/30 08:02:23  slava
 * o major rewrite. more changes to follow
 *
 * Revision 1.5  2002/08/27 11:52:03  slava
 * o some reformatting
 *
 * Revision 1.4  2001/10/08 05:17:51  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.3  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.2  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.1  2001/01/12 06:52:53  slava
 * o support for histogramming
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
