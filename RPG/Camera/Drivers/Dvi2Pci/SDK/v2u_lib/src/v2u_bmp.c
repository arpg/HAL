/****************************************************************************
 *
 * $Id: v2u_bmp.c 11808 2010-12-07 15:55:23Z monich $
 *
 * Copyright (C) 2003-2010 Epiphan Systems Inc. All rights reserved.
 *
 * Implements the v2u_write_bmp function
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "v2u_save.h"
#include "v2u_util.h"

#define BMP_FILE_HEADER_SIZE 14
#define BMP_INFO_HEADER_SIZE 40

#define _BI_RGB 0
#define _BI_BITFIELDS 3

/**
 * Writes 32-bit number in little endian byte order to the output stream.
 * Returns zero on failure, non-zero on success.
 */
static V2U_BOOL v2u_write_i16(FILE * out, short n)
{
    return fputc(n & 0xff, out) != EOF &&
           fputc((n >> 8) & 0xff, out) != EOF;
}

/**
 * Writes 32-bit number in little endian byte order to the output stream.
 * Returns zero on failure, non-zero on success.
 */
static V2U_BOOL v2u_write_i32(FILE * out, int n)
{
    return fputc(n & 0xff, out) != EOF &&
           fputc((n >> 8) & 0xff, out) != EOF &&
           fputc((n >> 16) & 0xff, out) != EOF &&
           fputc((n >> 24) & 0xff, out) != EOF;
}

/**
 * Converts one row from RGB24 to BGR24
 */
static void v2u_convert_rgb24(void* dst,const void* src,int row,int w,int h)
{
    int i;
    const V2U_BYTE* rgb = ((V2U_BYTE*)src) + 3*w*row;
    V2U_BYTE* bgr = dst;
    for (i=0; i<w; i++) {
        *bgr++ = rgb[2];
        *bgr++ = rgb[1];
        *bgr++ = rgb[0];
        rgb += 3;
    }
}

/**
 * Converts one row from ARGB32 to BGR24
 */
static void v2u_convert_argb32(void* dst,const void* src,int row,int w,int h)
{
    const V2U_BYTE* argb = ((V2U_BYTE*)src) + 4*w*row;
    V2U_BYTE* bgr = dst;
    int i;
    for (i=0; i<w; i++) {
        *bgr++ = argb[3];
        *bgr++ = argb[2];
        *bgr++ = argb[1];
        argb += 4;
    }
}

/**
 * Writes BMP file to the specified stream. Returns V2U_FALSE on failure,
 * V2U_TRUE on success. The fmt parameter defined the input pixel format.
 * The following formats are supported:
 *
 *   V2U_GRABFRAME_FORMAT_Y8
 *   V2U_GRABFRAME_FORMAT_RGB4
 *   V2U_GRABFRAME_FORMAT_RGB8
 *   V2U_GRABFRAME_FORMAT_RGB16
 *   V2U_GRABFRAME_FORMAT_BGR16
 *   V2U_GRABFRAME_FORMAT_RGB24
 *   V2U_GRABFRAME_FORMAT_BGR24
 *   V2U_GRABFRAME_FORMAT_ARGB32
 *   V2U_GRABFRAME_FORMAT_YUY2
 *   V2U_GRABFRAME_FORMAT_2VUY
 *   V2U_GRABFRAME_FORMAT_YV12
 *   V2U_GRABFRAME_FORMAT_I420
 *
 * possibly, with V2U_GRABFRAME_BOTTOM_UP_FLAG bit set. The pixels are assumed
 * to be tightly packed, i.e. the line size is NOT aligned at 32-bit boundary.
 */
V2U_BOOL v2u_write_bmp(FILE* out, int w, int h, int format, const void* pix)
{
    int bmpsize, offbits;
    unsigned int biCompression, biClrUsed = 0, palsize = 0;
    unsigned int rowsize, padding, datasize, filelength;
    void (*converter)(void* dst,const void* src,int row,int w,int h) = NULL;
    const void* palette = NULL;

    int bpp = V2UPALETTE_2_BPP(format);
    const int bottomUp = (format & V2U_GRABFRAME_BOTTOM_UP_FLAG) != 0;

    switch (format & V2U_GRABFRAME_FORMAT_MASK) {
    case V2U_GRABFRAME_FORMAT_RGB4:
        biCompression = _BI_RGB;
        biClrUsed = 16;
        palsize = 4*biClrUsed;
        palette = v2u_palette_4bpp;
        break;

    case V2U_GRABFRAME_FORMAT_RGB8:
        biCompression = _BI_RGB;
        biClrUsed = 256;
        palsize = 4*biClrUsed;
        palette = v2u_palette_8bpp;
        break;

    case V2U_GRABFRAME_FORMAT_Y8:
        biCompression = _BI_RGB;
        biClrUsed = 256;
        palsize = 4*biClrUsed;
        palette = v2u_palette_y8;
        break;

    case V2U_GRABFRAME_FORMAT_RGB16:
        palsize = 12;
        palette = v2u_mask_rgb16;
        biCompression = _BI_BITFIELDS;
        break;

    case V2U_GRABFRAME_FORMAT_BGR16:
        palsize = 12;
        palette = v2u_mask_bgr16;
        biCompression = _BI_BITFIELDS;
        break;

    case V2U_GRABFRAME_FORMAT_RGB24:
        converter = v2u_convert_rgb24;
        biCompression = _BI_RGB;
        break;

    case V2U_GRABFRAME_FORMAT_BGR24:
        biCompression = _BI_RGB;
        break;

    case V2U_GRABFRAME_FORMAT_ARGB32:
        converter = v2u_convert_argb32;
        biCompression = _BI_RGB;
        bpp = 24;
        break;

    case V2U_GRABFRAME_FORMAT_YUY2:
        converter = v2u_convert_yuy2_to_bgr24;
        biCompression = _BI_RGB;
        bpp = 24;
        break;

    case V2U_GRABFRAME_FORMAT_2VUY:
        converter = v2u_convert_2vuy_to_bgr24;
        biCompression = _BI_RGB;
        bpp = 24;
        break;

    case V2U_GRABFRAME_FORMAT_YV12:
        converter = v2u_convert_yv12_to_bgr24;
        biCompression = _BI_RGB;
        bpp = 24;
        break;

    case V2U_GRABFRAME_FORMAT_I420:
        converter = v2u_convert_i420_to_bgr24;
        biCompression = _BI_RGB;
        bpp = 24;
        break;

    case V2U_GRABFRAME_FORMAT_NV12:
        converter = v2u_convert_nv12_to_bgr24;
        biCompression = _BI_RGB;
        bpp = 24;
        break;

    default:
       return V2U_FALSE;
    }

    bmpsize = BMP_INFO_HEADER_SIZE + palsize;
    rowsize = (w + ((bpp < 8) ? 1 : 0))*bpp/8;
    padding = ((rowsize + 3) & (-4)) - rowsize;
    datasize = h*rowsize;

    offbits = BMP_FILE_HEADER_SIZE + bmpsize;
    filelength = offbits + datasize;

    /* Write file header */
    if (fputs("BM", out) == EOF ||              /* signature */
        !v2u_write_i32(out, filelength) ||      /* bfSize */
        !v2u_write_i32(out, 0) ||               /* bfReserved1/bfReserved2 */
        !v2u_write_i32(out, offbits) ||         /* bfOffBits */

        /* Write BITMAP Header */
        !v2u_write_i32(out,BMP_INFO_HEADER_SIZE) || /* DWORD biSize */
        !v2u_write_i32(out, w) ||               /* LONG  biWidth */
        !v2u_write_i32(out, h) ||               /* LONG  biHeight */
        !v2u_write_i16(out, 1) ||               /* WORD  biPlanes */
        !v2u_write_i16(out, (short)bpp) ||      /* WORD  biBitCount */
        !v2u_write_i32(out, biCompression) ||   /* DWORD biCompression */
        !v2u_write_i32(out, datasize) ||        /* DWORD biSizeImage */
        !v2u_write_i32(out, 2835) ||            /* LONG  biXPelsPerMeter */
        !v2u_write_i32(out, 2835) ||            /* LONG  biYPelsPerMeter */
        !v2u_write_i32(out, biClrUsed) ||       /* DWORD biClrUsed */
        !v2u_write_i32(out, 0)) {               /* DWORD biClrImportant */

        return V2U_FALSE;        
    }

    /* Write the RGB masks or palette */
    if (palette) {
        if (fwrite(palette, 1, palsize, out) < palsize) {
            return V2U_FALSE;
        }
    }

    /* Write the bitmap data */
    if (bottomUp && !converter && !padding) {
        if (fwrite(pix, 1, datasize, out) < datasize) {
            return V2U_FALSE;
        }
    } else {
        int y, zero = 0;
        const int srcRowSize = w*V2UPALETTE_2_BPP(format)/8;
        V2U_UINT8* convertBuf = (converter ? malloc(3*w) : NULL);
        V2U_BOOL result = V2U_TRUE;

        for (y=0; y<h; y++) {
            const void * buf;
            const int row = bottomUp ? y : (h-y-1);

            if (convertBuf) {
                converter(convertBuf, pix, row, w, h);
                buf = convertBuf;
            } else {
                buf = ((V2U_UINT8*)pix) + row*srcRowSize;
            }

            if (fwrite(buf, 1, rowsize, out) < rowsize) {
                result = V2U_FALSE;
                break;
            }

            if (padding && fwrite(&zero, 1, padding, out) < padding) {
                result = V2U_FALSE;
                break;
            }
        }

        free(convertBuf);
        return result;
    }

    return V2U_TRUE;
}
