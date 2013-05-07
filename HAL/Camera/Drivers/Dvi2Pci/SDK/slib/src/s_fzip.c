/*
 * $Id: s_fzip.c,v 1.28 2009/10/08 14:32:11 slava Exp $
 *
 * Copyright (C) 2001-2009 by Slava Monich
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

#include "s_util.h"
#include "s_fio.h"
#include "s_mem.h"

#include <zlib.h>

/* OS code part of GZ file header */
#ifdef _WIN32
#  define OS_CODE 0x0b
#elif defined(_UNIX)
#  define OS_CODE 0x03
#else
#  define OS_CODE 0x00
#endif

/* default input/output buffer size */
#define Z_BUFSIZE 16384

/*==========================================================================*
 *              G Z I P P E D     F I L E    I O
 *==========================================================================*/

typedef struct _Zip {
    File file;              /* shared File structure */
    File * f;               /* the file performing actual IO */
    int inbufsize;          /* input buffer size */
    int outbufsize;         /* output buffer size */
    z_stream * in;          /* zlib input context */
    z_stream * out;         /* zlib output context */
    I8u * inbuf;            /* input buffer */
    I8u * outbuf;           /* output buffer */
    I32u  incrc;            /* crc32 of uncompressed data */
    I32u  outcrc;           /* crc32 of compressed data */
    int   compression;      /* compression level */
    int   zflags;           /* flags, see below */

#define ZIP_IN      0x0001  /* compress input stream */
#define ZIP_OUT     0x0002  /* compress output stream */
#define ZIP_ZHDR    0x0008  /* read and write ZLIB header */
#define ZIP_GZIP    0x0004  /* read and write GZIP header */
#define ZIP_FINISH  0x0010  /* we have finished writing */
#define ZIP_EOF     0x0020  /* end of file reported to the caller */
#define ZIP_IN_EOF  0x0040  /* low level end of file */
#define ZIP_IN_END  0x0080  /* end of decompression */
#define ZIP_IN_ERR  0x0100  /* input error */
#define ZIP_OUT_ERR 0x0200  /* output error */

} Zip;

/* gzip flag byte */
#define GZ_ASCII_FLAG  0x01 /* bit 0 set: file probably ascii text */
#define GZ_HEAD_CRC    0x02 /* bit 1 set: header CRC present */
#define GZ_EXTRA_FIELD 0x04 /* bit 2 set: extra field present */
#define GZ_ORIG_NAME   0x08 /* bit 3 set: original file name present */
#define GZ_COMMENT     0x10 /* bit 4 set: file comment present */
#define GZ_RESERVED    0xE0 /* bits 5..7: reserved */

STATIC int GzMagic[2] = {0x1f, 0x8b};   /* gzip magic header */

/* I/O handlers */
STATIC int    ZipRead   P_((File * f, void * buf, int len));
STATIC int    ZipWrite  P_((File * f, const void * buf, int len));
STATIC Bool   ZipFlush  P_((File * f));
STATIC Bool   ZipEof    P_((File * f));
STATIC File * ZipTarget P_((File * f));
STATIC void   ZipDetach P_((File * f));
STATIC void   ZipClose  P_((File * f));
STATIC void   ZipFree   P_((File * f));

/*
 * Table of I/O handlers
 */
STATIC const FileIO ZipIO = {
    NULL                /* open     */,
    NULL                /* reopen   */,
    NULL                /* setparam */,
    ZipRead             /* read     */,
    ZipWrite            /* write    */,
    NULL                /* skip     */,
    ZipFlush            /* flush    */,
    ZipEof              /* eof      */,
    NULL                /* fd       */,
    ZipTarget           /* target   */,
    ZipDetach           /* detach   */,
    ZipClose            /* close    */,
    ZipFree             /* free     */,
    0                   /* flags    */
};

/**
 * Memory allocation/deallocation functions for use by zlib
 */
STATIC voidpf ZipMemAlloc(voidpf opaque, uInt items, uInt size)
{
    UNREF(opaque);
    return MEM_Alloc(items * size);
}

STATIC void ZipMemFree(voidpf opaque, voidpf address)
{
    UNREF(opaque);
    MEM_Free(address);
}

/**
 * Converts File * pointer into Zip * pointer
 */
STATIC Zip * ZipCast(File * f)
{
    ASSERT(f);
    if (f) {
        ASSERT(f->io == &ZipIO);
        if (f->io == &ZipIO) {
            return CAST(f,Zip,file);
        }
    }
    return NULL;
}

/**
 * Reads a byte from a gz_stream; update next_in and avail_in. 
 * Return zipEOF for end of file. Assumes that the stream zf has been 
 * sucessfully opened for reading.
 */
STATIC int ZipGetByte(Zip * zf)
{
    if (!(zf->zflags & (ZIP_IN_ERR | ZIP_IN_EOF))) {
        if (zf->in->avail_in == 0) {
            int bytesToRead = (FILE_CanBlock(zf->f) ? 1 : zf->inbufsize);
            int nbytes = FILE_Read(zf->f, zf->inbuf, bytesToRead);
            if (nbytes == 0) {
                zf->zflags |= ZIP_IN_EOF;
                return EOF;
            } else if (nbytes < 0) {
                zf->zflags |= ZIP_IN_ERR;
                return EOF;
            }
            zf->in->avail_in = nbytes;
            zf->in->next_in = zf->inbuf;
        }
        zf->in->avail_in--;
        return *((zf->in->next_in)++);
    }
    return EOF;
}

/**
 * Push the most recently read byte back into ZLIB input buffer
 */
STATIC Bool ZipPushBack(Zip * zf)
{
    ASSERT(zf->in->next_in > zf->inbuf);
    if (zf->in->next_in > zf->inbuf) {
        zf->in->next_in--;
        zf->in->avail_in++;
        return True;
    }
    return False;
}

/**
 * Checks the gzip header of a zf stream opened for reading.  Assumes that 
 * the stream zf has already been created sucessfully; zf->in->avail_in is 
 * zero for the first time, but may be non-zero for concatenated .gz files.
 */
STATIC Bool ZipSkipHeader(Zip * zf)
{
    int method; /* method byte */
    int flags;  /* flags byte */
    int len;    /* buyes read so far */
    int c;

    /* check the gzip magic header */
    if ((c = ZipGetByte(zf)) != GzMagic[0]) {
        ZipPushBack(zf);
        TRACE("ZIP: no magic[0]\n");
        return False;
    } else if ((c = ZipGetByte(zf)) != GzMagic[1]) {
        ZipPushBack(zf);
        ZipPushBack(zf);
        TRACE("ZIP: no magic[1]\n");
        return False;
    }

    method = ZipGetByte(zf);
    flags = ZipGetByte(zf);
    if (method != Z_DEFLATED || (flags & GZ_RESERVED) != 0) {
        TRACE("ZIP: unsupported method/flags\n");
        return False;
    }

    /* discard time, xflags and OS code: */
    for (len = 0; len<6; len++) (void)ZipGetByte(zf);

    /* skip the extra field */
    if ((flags & GZ_EXTRA_FIELD) != 0) { 
        len  =  ZipGetByte(zf);
        len +=  ZipGetByte(zf)<<8;

        /* len is garbage if EOF but the loop below will quit anyway */
        while (len-- != 0 && ZipGetByte(zf) != EOF) NOTHING;
    }

    /* skip the original file name */
    if ((flags & GZ_ORIG_NAME) != 0) { 
        while ((c = ZipGetByte(zf)) != 0  && c != EOF) NOTHING;
    }

    /* skip the .gz file comment */
    if ((flags & GZ_COMMENT) != 0) {   
        while ((c = ZipGetByte(zf)) != 0  && c != EOF) NOTHING;
    }

    /* skip the header crc */
    if ((flags & GZ_HEAD_CRC) != 0) {  
        for (len=0; len<2; len++) (void)ZipGetByte(zf);
    }

    return BoolValue((zf->zflags & (ZIP_IN_ERR | ZIP_IN_EOF)) == 0);
}

/**
 * Initializes the input zlib context 
 */
STATIC Bool ZipInitIn(Zip * zf)
{

    /* allocate buffer */
    zf->inbuf = (I8u*)MEM_Alloc(zf->inbufsize);
    if (zf->inbuf) {

        /* allocate zlib context */
        zf->in = MEM_New(z_stream);
        if (zf->in) {
            int bits = ((zf->zflags & ZIP_ZHDR) ? (-MAX_WBITS) : MAX_WBITS);

            /* tell zlib to use our memory allocation functions */
            memset(zf->in, 0, sizeof(*zf->in));
            zf->in->zalloc = ZipMemAlloc;
            zf->in->zfree = ZipMemFree;
            zf->in->next_in = zf->inbuf;
            if (inflateInit2(zf->in, bits) == Z_OK) {

                /* skip .gz header */
                if (!(zf->zflags & ZIP_GZIP) || ZipSkipHeader(zf)) {
                    return True;
                }
                inflateEnd(zf->in);
            }
            MEM_Free(zf->in);
            zf->in = NULL;
        }
        MEM_Free(zf->inbuf);
        zf->inbuf = NULL;
    }
    zf->zflags |= ZIP_IN_ERR;
    return False;
}

/**
 * Reads and uncompresses up to len bytes from the compressed stream.
 * Returns -1 in case of error, 0 if end of file has been reached (-1 on
 * subequent calls)
 */
STATIC int ZipRead(File * f, void * buf, int len)
{
    Zip * zf = ZipCast(f);

    /* check if input is transparent */
    if (!(zf->zflags & ZIP_IN)) {
        int nbytes = FILE_Read(zf->f, buf, len);
        if (nbytes == 0) {
            zf->zflags |= (ZIP_IN_EOF | ZIP_EOF);
        } else if (nbytes < 0) {
            zf->zflags |= ZIP_IN_ERR;
        }
        return nbytes;
    }

    /* check for error or end-of-file condition */
    if (zf->zflags & (ZIP_IN_ERR | ZIP_EOF)) {
        return -1;
    }

    /* report end-of-file condition to the caller */
    if (zf->zflags & ZIP_IN_END) {
        zf->zflags |= ZIP_EOF;
        return 0;
    }

    /* check if we can read from the low level file */
    if (!FILE_AllowsReads(zf->f)) {
        zf->zflags |= ZIP_IN_ERR;
        return -1;
    }

    /* "lazy" allocation of the input context */
    if (!zf->in) {
        if (!ZipInitIn(zf)) {
            return -1;
        }
    }

    /* read and inflate the data */
    zf->in->next_out = (Bytef*)buf;
    zf->in->avail_out = len;
    while (zf->in->avail_out > 0 && !(zf->zflags & (ZIP_IN_ERR|ZIP_IN_END))) {
        int zerr;
        if (zf->in->avail_in == 0 && !(zf->zflags & ZIP_IN_EOF)) {
            int bytesToRead = (FILE_CanBlock(zf->f) ? 1 : zf->inbufsize);
            int nbytes = FILE_Read(zf->f, zf->inbuf, bytesToRead);
            if (nbytes == 0) {
                zf->zflags |= ZIP_IN_EOF;
            } else if (nbytes < 0) {
                zf->zflags |= ZIP_IN_ERR;
                break;
            }
            zf->in->avail_in = nbytes;
            zf->in->next_in = zf->inbuf;
        }
        zerr = inflate(zf->in, Z_NO_FLUSH);
        if (zerr == Z_STREAM_END) {
            inflateReset(zf->in);
            zf->zflags |= ZIP_IN_END;
        } else  if (zerr != Z_OK) {
            zf->zflags |= ZIP_IN_ERR;
            break;
        }
    }

    /* 
     * return error or end-of-file condition if nothing has been 
     * decompressed, otherwise return the amount we have successfully
     * inflated.
     */
    if (len == (int)zf->in->avail_out) {
        if (zf->zflags & ZIP_IN_EOF) {
            zf->zflags |= ZIP_EOF;
            return 0;
        }
        if (zf->zflags & ZIP_IN_ERR) {
            return -1;
        }
        /* what would this mean? there's no data and yet it's not an error */
        zf->zflags |= ZIP_EOF;
        return 0;
    } else {
        zf->incrc = crc32(zf->incrc, (Bytef*)buf, len - zf->in->avail_out);
        return (len - zf->in->avail_out);
    }
}

/**
 * Initializes the output zlib context 
 */
STATIC Bool ZipInitOut(Zip * zf)
{

    /* allocate buffer */
    zf->outbuf = (I8u*)MEM_Alloc(zf->outbufsize);
    if (zf->outbuf) {

        /* allocate zlib context */
        zf->out = MEM_New(z_stream);
        if (zf->out) {
            int zerr;
            int bits = ((zf->zflags & ZIP_ZHDR) ? (-MAX_WBITS) : MAX_WBITS);

            /* tell zlib to use our memory allocation functions */
            memset(zf->out, 0, sizeof(*zf->out));
            zf->out->zalloc = ZipMemAlloc;
            zf->out->zfree = ZipMemFree;

            /* windowBits is passed < 0 to suppress zlib header */
            zerr = deflateInit2(zf->out, zf->compression,
                                         Z_DEFLATED, bits, 8,
                                         Z_DEFAULT_STRATEGY);

            if (zerr == Z_OK) {
                if (zf->zflags & ZIP_GZIP) {
                    /* write a very simple .gz header */
                    I8u hdr[10];
                    memset(hdr, 0, sizeof(hdr));
                    hdr[0] = (I8u)GzMagic[0];
                    hdr[1] = (I8u)GzMagic[1];
                    hdr[2] = Z_DEFLATED;
                    hdr[9] = OS_CODE;
                    if (FILE_Write(zf->f,hdr,sizeof(hdr)) == sizeof(hdr)) {
                        FILE_Flush(zf->f);
                        zf->out->next_out = zf->outbuf;
                        zf->out->avail_out = zf->outbufsize;
                        return True;
                    }
                } else {
                    /* not writing the header */
                    zf->out->next_out = zf->outbuf;
                    zf->out->avail_out = zf->outbufsize;
                    return True;
                }
                deflateEnd(zf->out);
            }
            MEM_Free(zf->out);
            zf->out = NULL;
        }
        MEM_Free(zf->outbuf);
        zf->outbuf = NULL;
    }
    zf->zflags |= ZIP_OUT_ERR;
    return False;
}

/**
 * Compresses and writes len bytes
 */
STATIC int ZipWrite(File * f, const void * buf, int len)
{
    Zip * zf = ZipCast(f);

    /* check if output is transparent */
    if (!(zf->zflags & ZIP_OUT)) {
        int nbytes = FILE_Write(zf->f, buf, len);
        if (nbytes < 0) zf->zflags |= ZIP_OUT_ERR;
        return nbytes;
    }

    /* check for errors (ZIP_OUT_ERR) and caller's stupidity (ZIP_FINISH) */
    ASSERT(!(zf->zflags & ZIP_FINISH));
    if (zf->zflags & (ZIP_OUT_ERR | ZIP_FINISH)) {
        return -1;
    }

    /* check if we can write to the low level file */
    if (!FILE_AllowsWrites(zf->f)) {
        zf->zflags |= ZIP_OUT_ERR;
        return -1;
    }

    /* "lazy" allocation of the input context */
    if (!zf->out) {
        if (!ZipInitOut(zf)) {
            return -1;
        }
    }

    /* deflate and write data */
    zf->out->next_in = (Bytef*)buf;
    zf->out->avail_in = len;
    while (zf->out->avail_in != 0) {
        if (zf->out->avail_out == 0) {
            zf->out->next_out = zf->outbuf;
            if (!FILE_WriteAll(zf->f, zf->outbuf, zf->outbufsize)) {
                zf->zflags |= ZIP_OUT_ERR;
                return -1;
            }
            zf->out->avail_out = zf->outbufsize;
        }
        if (deflate(zf->out, Z_NO_FLUSH) != Z_OK) break;
    }
    zf->outcrc = crc32(zf->outcrc, (const Bytef *)buf, len);
    return (len - zf->out->avail_in);
}

/**
 * Sends all the data compressed so far to the underlying stream.
 */
STATIC Bool ZipFlush2(Zip * zf, int flush)
{
    if (zf->out) {
        int zerr = Z_OK;
        Bool done = False;

        ASSERT(!zf->out->avail_in);
        zf->out->avail_in = 0; /* should be zero already anyway */

        for (;;) {
            int len = zf->outbufsize - zf->out->avail_out;
            if (len != 0) {
                if (FILE_Write(zf->f, zf->outbuf, len) != len) {
                    zf->zflags |= ZIP_OUT_ERR;
                    return False;
                }

                zf->out->next_out = zf->outbuf;
                zf->out->avail_out = zf->outbufsize;
            }

            if (done) break;
            zerr = deflate(zf->out, flush);

            /* ignore the second of two consecutive flushes */
            if (len == 0 && zerr == Z_BUF_ERROR) zerr = Z_OK;

            /* deflate has finished flushing only when it hasn't used 
             * up all the available space in the output buffer
             */
            if (zf->out->avail_out || zerr == Z_STREAM_END) {
                done = True;
            }
            if (zerr != Z_OK && zerr != Z_STREAM_END) break;
        }
        return BoolValue(FILE_Flush(zf->f) && (zerr == Z_STREAM_END));
    }
    return True;
}

STATIC Bool ZipFlush(File * f)
{
    Zip * zf = ZipCast(f);
    return (zf ? ZipFlush2(zf, Z_SYNC_FLUSH) : False);
}

STATIC Bool ZipEof(File * f)
{
    Zip * zf = ZipCast(f);
    return (zf ? FILE_Eof(zf->f) : False);
}

STATIC File * ZipTarget(File * f)
{
    Zip * zf = ZipCast(f);
    return (zf ? zf->f : NULL);
}

STATIC void ZipDetach2(Zip * zf)
{
    if (zf->f) {
        if (zf->out && !(zf->zflags & ZIP_FINISH)) {
            ZipFlush2(zf, Z_FINISH);
        }
        zf->f = NULL;
    }
    if (zf->in) {
        inflateEnd(zf->in);
        MEM_Free(zf->in);
        zf->in = NULL;
    }
    if (zf->out) {
        deflateEnd(zf->out);
        MEM_Free(zf->out);
        zf->out = NULL;
    }
    if (zf->inbuf) {
        MEM_Free(zf->inbuf);
        zf->inbuf = NULL;
    }
    if (zf->outbuf) {
        MEM_Free(zf->outbuf);
        zf->outbuf = NULL;
    }
}

STATIC void ZipDetach(File * f)
{
    Zip * zf = ZipCast(f);
    if (zf) ZipDetach2(zf);
}

/**
 * Flushes the deflator and the underlying stream. 
 * Closes the low level stream. No I/O is possible 
 * after this function has been invoked.
 */
STATIC void ZipClose(File * f)
{
    Zip * zf = ZipCast(f);
    if (zf) {
        if (zf->f) {
            if (zf->out && !(zf->zflags & ZIP_FINISH)) {
                ZipFlush2(zf, Z_FINISH);
            }
            FILE_Close(zf->f);
            zf->f = NULL;
        }
        ZipDetach2(zf); /* deallocates buffers */
    }
}

STATIC void ZipFree(File * f)
{
    Zip * zf = ZipCast(f);
    if (zf) {
        ASSERT(!(f->flags & FILE_IS_OPEN));
        MEM_Free(zf);
    }
}

/**
 * Creates a new Zip IO context.
 * Compresses output data and decompresses input data.
 * Compression levels are defined in zlib.h:
 *
 * #define Z_NO_COMPRESSION         0
 * #define Z_BEST_SPEED             1
 * #define Z_BEST_COMPRESSION       9
 * #define Z_DEFAULT_COMPRESSION  (-1)
 */
File * FILE_Zip2(File * f, int flags, int level)
{
    ASSERT(f);
    ASSERT(!(flags & (~0x000f)));
    if (f) {
        Zip * zf = MEM_New(Zip);
        if (zf) {
            memset(zf, 0, sizeof(*zf));
            if (FILE_Init(&zf->file, TEXT("zip"), True, &ZipIO)) {
                zf->f = f;
                zf->outbufsize = Z_BUFSIZE;
                zf->inbufsize = (FILE_CanBlock(f) ? 1 : Z_BUFSIZE);
                zf->compression = level;
                if (flags & FILE_ZIP_IN) zf->zflags |= ZIP_IN;
                if (flags & FILE_ZIP_OUT) zf->zflags |= ZIP_OUT;
                if (flags & FILE_ZIP_ZHDR) zf->zflags |= ZIP_ZHDR;
                if (flags & FILE_ZIP_GZIP) zf->zflags |= ZIP_GZIP;
                return &zf->file;
            }
            MEM_Free(zf);
        }
    }
    return NULL;
}

/**
 * Creates a new Zip IO context.
 * Compresses output data and decompresses input data.
 * Uses default zlib compression level to compress output data.
 */
File * FILE_Zip(File * f, int flags)
{
    return FILE_Zip2(f, flags, Z_DEFAULT_COMPRESSION);
}

/**
 * Finishes deflation by calling deflate(out,Z_FINISH)
 * No more deflation is possible after this call.
 */
Bool FILE_ZipFinish(File * f)
{
    Zip * zf = ZipCast(f);
    if (zf) {
        ASSERT(!(zf->zflags & ZIP_FINISH));
        if (!(zf->zflags & ZIP_FINISH)) {
            ZipFlush2(zf, Z_FINISH);
            zf->zflags |= ZIP_FINISH;
            return True;
        }
    }
    return False;
}

/*
 * HISTORY:
 *
 * $Log: s_fzip.c,v $
 * Revision 1.28  2009/10/08 14:32:11  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.27  2008/11/10 22:26:56  slava
 * o added new function FILE_Zip2 which allows the caller to specify desired
 *   compression level of the compressor stream
 *
 * Revision 1.26  2008/09/24 11:59:04  slava
 * o replaced Z_BEST_COMPRESSION with Z_DEFAULT_COMPRESSION. Should probably
 *   make it configurable...
 *
 * Revision 1.25  2008/09/24 11:54:17  slava
 * o added another check for FILE_CAN_BLOCK on the underlying file
 *
 * Revision 1.24  2008/09/03 09:37:02  slava
 * o removed static FIO_BLOCKING_READ flag and added FILE_CAN_BLOCK flag that
 *   can be set on individual streams. This is a lot more flexible. Added
 *   FILE_CanBlock function.
 *
 * Revision 1.23  2008/09/01 12:08:15  slava
 * o replaced FileIO::fileIO field with flags. Added FIO_BLOCKING_READ flag
 *   which marks I/O methods whose read callback can block before the end of
 *   stream is reached (i.e. socket I/O).
 *
 * Revision 1.22  2008/09/01 10:57:21  slava
 * o added 'skip' I/O callback which can be implemented by those I/O methods
 *   which can efficiently skip input data without actually reading and
 *   copying anything. Currently, such callback is only implemented for
 *   in-memory I/O, but it can also be done for file I/O using fseek.
 *
 * Revision 1.21  2006/10/12 16:23:24  slava
 * o made ZipIO static
 *
 * Revision 1.20  2005/03/11 22:31:37  slava
 * o handle the situation when we run out of input data, but decompression
 *   is not finished yet. Before, we could truncate the output data.
 *
 * Revision 1.19  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.18  2004/12/03 04:47:43  slava
 * o fixed a bug in ZipRead, it could discard some successfully inflated
 *   data after a read or inflate error. That wasn't right. Everything
 *   that has been successfully decompressed must be returned to the
 *   caller, and only then we can signal an error or end-of-file condition.
 *
 * Revision 1.17  2004/07/20 22:36:21  slava
 * o set the FILE_IS_ATTACHED flag for the File objects created by FILE_Zip.
 *   otherwise there is no way to detach the compressor from the underlying
 *   stream
 *
 * Revision 1.16  2004/04/08 01:44:18  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.15  2004/04/04 15:46:36  slava
 * o don't close the underlying stream on detach
 *
 * Revision 1.14  2003/08/28 17:16:23  slava
 * o replaced tabs with spaces
 *
 * Revision 1.13  2003/06/02 18:23:44  slava
 * o added recovery mechanism for the case if .gz header is missing
 *
 * Revision 1.12  2003/05/27 13:00:06  slava
 * o tell zlib to use our memory allocation functions
 *
 * Revision 1.11  2003/05/27 05:44:03  slava
 * o fixed erroneous ASSERT
 *
 * Revision 1.10  2003/05/27 05:22:27  slava
 * o renamed FILE_Compress into FILE_Zip, added FILE_ZipFinish
 *
 * Revision 1.9  2003/05/21 00:11:04  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.8  2003/01/08 17:37:48  slava
 * o allow to pass NULL path argument to FILE_Init. In that case, FILE_Name
 *   will return the name of the target file. This rule recursively applies
 *   to the target file as well.
 *
 * Revision 1.7  2003/01/08 17:15:43  slava
 * o removed ZIP_GetTarget and WRAP_GetTarget functions, replaced them
 *   with generic FILE_Target
 *
 * Revision 1.6  2002/12/30 21:42:21  slava
 * o added fileIO field to the FileIO structure. This field is set to
 *   True if the File stream performs file based I/O (as opposed to socket
 *   or in-memory I/O)
 *
 * Revision 1.5  2002/08/27 11:52:03  slava
 * o some reformatting
 *
 * Revision 1.4  2002/01/31 01:10:08  slava
 * o do not include <zutil.h> as it may not be available on some platforms
 *
 * Revision 1.3  2002/01/25 04:09:10  slava
 * o fixed compilation warnings
 *
 * Revision 1.2  2002/01/25 03:11:01  slava
 * o added generic compressor/decompressor stream, created by FILE_Compress
 *
 * Revision 1.1  2001/12/23 21:17:00  slava
 * o moved each file I/O implemenetation into a separate file. This prevents
 *   unnecessary linkage with zlib and socket library and makes executables
 *   smaller. Most linkers are not very good in removing dead references
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
