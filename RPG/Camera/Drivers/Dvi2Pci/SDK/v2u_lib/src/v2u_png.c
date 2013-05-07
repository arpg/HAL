/****************************************************************************
 *
 * $Id: v2u_png.c 10849 2010-09-06 10:45:37Z monich $
 *
 * Copyright (C) 2003-2010 Epiphan Systems Inc. All rights reserved.
 *
 * Implements the v2u_write_png function
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "png.h"

#include <setjmp.h>

#include "v2u_save.h"
#include "v2u_util.h"

typedef struct _PngWrite {
    jmp_buf jmp;                        /* for return to caller */
} PngWrite;

/* Error handler */
static void v2u_png_error(png_structp png, png_const_charp msg)
{
    PngWrite* context = png_get_error_ptr(png);
    if (!context) {
        /* we are completely hosed */
        exit(99);
    }

    /* return to caller */
    longjmp(context->jmp, 1);
}

/*
 * Writes PNG file to the specified stream. Returns V2U_FALSE on failure,
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
V2U_BOOL v2u_write_png(FILE* out, int w, int h, int fmt, const void* buf)
{
    png_byte* pngRow;
    V2U_BOOL ok = V2U_FALSE;
    const BmpPalEntry* palette = NULL;
    int colorType, paletteSize = 0, pngRowSize;
    const V2U_BOOL bottomUp = (fmt & V2U_GRABFRAME_BOTTOM_UP_FLAG) != 0;
    const int bpp = V2UPALETTE_2_BPP(fmt);

    fmt &= V2U_GRABFRAME_FORMAT_MASK;
    switch (fmt) {
    case V2U_GRABFRAME_FORMAT_Y8:
    case V2U_GRABFRAME_FORMAT_RGB8:
        pngRowSize = 0; /* Directly copy the data from the input buffer */
        colorType = PNG_COLOR_TYPE_PALETTE;
        paletteSize = 256;
        palette = (fmt == V2U_GRABFRAME_FORMAT_Y8) ?
            v2u_palette_y8 :    /* V2U_GRABFRAME_FORMAT_Y8 */
            v2u_palette_8bpp;   /* V2U_GRABFRAME_FORMAT_RGB8 */
        break;

    case V2U_GRABFRAME_FORMAT_RGB4:
        pngRowSize = w;
        colorType = PNG_COLOR_TYPE_PALETTE;
        paletteSize = 16;
        palette = v2u_palette_4bpp;
        break;

    case V2U_GRABFRAME_FORMAT_RGB24:
        pngRowSize = 0; /* Directly copy the data from the input buffer */
        colorType = PNG_COLOR_TYPE_RGB;
        break;

    default:
        pngRowSize = 3*w;
        colorType = PNG_COLOR_TYPE_RGB;
        break;
    }

    pngRow = (pngRowSize > 0) ? malloc(pngRowSize) : 0;
    if (pngRow || !pngRowSize) {
        PngWrite context;
        png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, 
            &context, v2u_png_error, NULL);
        if (png) {
            png_infop pnginfo = png_create_info_struct(png);
            if (pnginfo) {
                if (!setjmp(context.jmp)) {
                    int i, y, n = 0, rowsize = (w+((bpp<8)?1:0))*bpp/8;
                    time_t t;
                    png_time pngtime;
                    png_text txt[4];
                    png_init_io(png, out);
                    png_set_compression_level(png, 5);

                    png_set_IHDR(png, pnginfo, w, h, 8, colorType,
                        PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, 
                        PNG_FILTER_TYPE_DEFAULT);

                    if (palette) {
                        png_color pngPalette[256];
                        for (i=0; i<paletteSize; i++) {
                            pngPalette[i].red = palette[i].rgbRed;
                            pngPalette[i].green = palette[i].rgbGreen;
                            pngPalette[i].blue = palette[i].rgbBlue;
                        }
                        png_set_PLTE(png, pnginfo, pngPalette, paletteSize);
                    }

                    /* Image information */
                    txt[n].key = (char*)"Title";
                    txt[n].text = (char*)"VGA capture";
                    txt[n++].compression = PNG_TEXT_COMPRESSION_NONE;
                    txt[n].key = (char*)"Author";
                    txt[n].text = (char*)"VGA2USB";
                    txt[n++].compression = PNG_TEXT_COMPRESSION_NONE;
                    txt[n].key = (char*)"URL";
                    txt[n].text = (char*)"http://www.epiphan.com/vga2usb/";
                    txt[n++].compression = PNG_TEXT_COMPRESSION_NONE;
                    png_set_text(png, pnginfo, txt, n);

                    /* Time */
                    time(&t);
                    png_convert_from_time_t(&pngtime, t);
                    png_set_tIME(png, pnginfo, &pngtime);

                    /* Actually write the file */
                    png_write_info(png, pnginfo);
                    png_set_packing(png);

                    ok = V2U_TRUE;

                    for (y=0; y<h && ok; y++) {
                        const int nrow = (bottomUp ? (h-y-1) : y);
                        const png_byte* row = ((png_byte*)buf) + nrow*rowsize;
                        if (!pngRowSize) {
                            /* Directly copy the data from the input buffer */
                            png_write_row(png, (png_bytep)row);
                        } else if (fmt == V2U_GRABFRAME_FORMAT_RGB4) {
                            /* Unpack 4bpp data into pngRow */
                            const png_byte* src = row;
                            png_byte* dest = pngRow;
                            for (i=0; i<w-1; i+=2) {
                                const unsigned char pixel = (*src++);
                                *dest++ = (pixel >> 4);
                                *dest++ = (pixel & 0x0f);
                            }
                            if (i<w) {
                                *dest = ((*src) >> 4);
                            }
                            png_write_row(png, pngRow);
                        } else {
                            /* Convert into RGB24 */
                            if (v2u_convert_to_rgb24(pngRow,buf,fmt,nrow,w,h)) {
                                png_write_row(png, pngRow);
                            } else {
                                ok = V2U_FALSE;
                            }
                        }
                    }
                    png_write_end(png, pnginfo);
                }  else {
                    ok = V2U_FALSE;
                }
            }
            png_destroy_write_struct(&png, &pnginfo);
        }
        free(pngRow);
    }

    return ok;
}
