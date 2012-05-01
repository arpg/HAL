/****************************************************************************
 *
 * $Id: v2u_jpeg.c 10816 2010-09-02 07:58:50Z monich $
 *
 * Copyright (C) 2003-2010 Epiphan Systems Inc. All rights reserved.
 *
 * Implements the v2u_write_jpeg function
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>

#include "jpeglib.h"

#include "v2u_save.h"
#include "v2u_util.h"

#define JPEG_BUF_SIZE 0x4000
#define JPEG_QUALITY  85

// JPEG destination context
typedef struct _V2U_JPEG_DEST_MRG {
    struct jpeg_destination_mgr pub;    /* public fields */
    JOCTET * buffer;                    /* start of buffer */
    size_t bufsize;                     /* buffer size */
    FILE * out;                         /* output file */
} V2U_JPEG_DEST_MRG;

typedef struct _V2U_JPEG_ERR_MRG {
    struct jpeg_error_mgr pub;          /* public fields */
    jmp_buf jmp;                        /* for return to caller */
} V2U_JPEG_ERR_MRG;

static void v2u_jpeg_init(j_compress_ptr cinfo)
{
    V2U_JPEG_DEST_MRG* dest = (V2U_JPEG_DEST_MRG*)cinfo->dest;
    dest->pub.next_output_byte = dest->buffer;
    dest->pub.free_in_buffer = dest->bufsize;
}

static boolean v2u_jpeg_flush(j_compress_ptr cinfo)
{
    V2U_JPEG_DEST_MRG* dest = (V2U_JPEG_DEST_MRG*)cinfo->dest;
    if (fwrite(dest->buffer, 1, dest->bufsize, dest->out) == dest->bufsize) {
        dest->pub.next_output_byte = dest->buffer;
        dest->pub.free_in_buffer = dest->bufsize;
        return V2U_TRUE;
    } else {
        return V2U_FALSE;
    }
}

static void v2u_jpeg_destroy(j_compress_ptr cinfo)
{
    V2U_JPEG_DEST_MRG* dest = (V2U_JPEG_DEST_MRG*)cinfo->dest;
    size_t datacount = dest->bufsize - dest->pub.free_in_buffer;
    if (datacount > 0) {
        fwrite(dest->buffer, 1, datacount, dest->out);
    }
    fflush(dest->out);
}

static void v2u_jpeg_error_exit(j_common_ptr cinfo)
{
    V2U_JPEG_ERR_MRG* err = (V2U_JPEG_ERR_MRG*)cinfo->err;
    cinfo->err->output_message(cinfo);
    longjmp(err->jmp, 1);
}

/*
 * Writes JPEG file to the specified stream. Returns zero on failure, 
 * non-zero on success. The fmt parameter defined the input pixel format.
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
V2U_BOOL v2u_write_jpeg(FILE* out, int w, int h, int format, const void* pix)
{
    V2U_BOOL result = V2U_FALSE;
    const V2U_BOOL bottomUp = (format & V2U_GRABFRAME_BOTTOM_UP_FLAG) != 0;
    JSAMPLE * jpegrow = calloc(3*w, sizeof(JSAMPLE));
    if (jpegrow) {
        JSAMPLE* jpegbuf = calloc(JPEG_BUF_SIZE, sizeof(JSAMPLE));
        if (jpegbuf) {
            int y;
            V2U_JPEG_ERR_MRG err;
            V2U_JPEG_DEST_MRG dest;
            struct jpeg_compress_struct cinfo;

            /* Initialize JPEG compression */        
            memset(&dest, 0, sizeof(dest));
            dest.pub.init_destination = v2u_jpeg_init;
            dest.pub.empty_output_buffer = v2u_jpeg_flush;
            dest.pub.term_destination = v2u_jpeg_destroy;
            dest.out = out;
            dest.bufsize = JPEG_BUF_SIZE;
            dest.buffer = jpegbuf;

            /* Step 1: allocate and initialize JPEG compressor */
            cinfo.err = jpeg_std_error(&err.pub);
            err.pub.error_exit = v2u_jpeg_error_exit;
            jpeg_create_compress(&cinfo);
            if (setjmp(err.jmp)) {

                /* If we get here, the JPEG code signaled an error */
                result = V2U_FALSE;

            } else {

                /* Step 2: specify data destination */
                cinfo.dest = &dest.pub;

                /* Step 3: set parameters for compression */
                cinfo.image_width = w;
                cinfo.image_height = h;
                cinfo.input_components = 3;
                cinfo.in_color_space = JCS_RGB;
                jpeg_set_defaults(&cinfo);
                jpeg_set_quality(&cinfo, JPEG_QUALITY, V2U_TRUE);

                /* Step 4: Start compressor */
                jpeg_start_compress(&cinfo, V2U_TRUE);

                /* Step 5: while (scan lines remain to be written) */
                /*           jpeg_write_scanlines(...); */
                result = V2U_TRUE;
                while ((y = cinfo.next_scanline) < h) {
                    const int nrow = (bottomUp ? (h-y-1) : y);
                    if (v2u_convert_to_rgb24(jpegrow,pix,format,nrow,w,h)) {
                        jpeg_write_scanlines(&cinfo, &jpegrow, 1);
                    } else {
                        result = V2U_FALSE;
                        break;
                    }
                }

                /* Step 6: Finish compression */
                jpeg_finish_compress(&cinfo);
            }
            jpeg_destroy_compress(&cinfo);
            free(jpegbuf);
        }
        free(jpegrow);
    }
    return result;
}
