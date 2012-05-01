/*
 * $Id: s_file.c,v 1.60 2010/12/19 17:51:46 slava Exp $
 *
 * Copyright (C) 2000-2010 by Slava Monich
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

#include "s_fio.h"
#include "s_util.h"
#include "s_mem.h"

/**
 * Initializes the shared portion of all file structures. The path parameter
 * may be NULL, which leaves name buffer empty, which in turn will cause
 * FILE_Name to query the target file name.
 */
Bool FILE_Init(File * f, Str path, Bool attach, IODesc io)
{
    f->io = io;
    f->flags = FILE_IS_OPEN | FILE_CAN_READ | FILE_CAN_WRITE;
    if (attach) f->flags |= FILE_IS_ATTACHED;

    STRBUF_Init(&f->buf);
    f->pushed = 0;
    f->bytesRead = 0;
    f->bytesWritten = 0;

    /* copy the file name. that's the only operation that can fail */
    if (path) {
        f->name = STRING_Dup(path);
        return f->name ? True : False;
    } else {
        f->name = NULL;
        return True;
    }
}

/**
 * Internal function. Called when the caller needs to undo what FILE_Init
 * has done.
 */
void FILE_Destroy(File * f)
{
    STRBUF_Destroy(&f->buf);
    MEM_Free(f->name);
}

/**
 * Open a file. Returns NULL if file cannot be opened or memory 
 * allocation fails.
 */
File * FILE_Open(Str path, const char * mode, IODesc io)
{
    File * f = NULL;

    if (!io) {
#ifdef  __KERNEL__
        ASSMSG("No default I/O in kernel mode!");
        return NULL;
#else /* ! __KERNEL__ */
        io = PlainFile;
#endif /* ! __KERNEL__ */
    }

    ASSERT(io->open);
    if (io->open) {
        f = (*io->open)(path, mode);
        if (f) {
            if (FILE_Init(f, path, False, io)) {
                return f;
            }
            f->io->close(f);
            f->io->free(f);
        }
    }

    return NULL;
}

/**
 * Reopens the file with specified mode. FILE_Close() is still necessary
 * even if this call fails.
 */
Bool FILE_Reopen(File * f, Str path, const char * mode)
{
    ASSERT(f);
    if (f && (f->flags & FILE_IS_OPEN) && 
        f->io->reopen && f->io->reopen(f, path, mode)) {
        f->bytesRead = 0;
        f->bytesWritten = 0;
        MEM_Free(f->name);
        if (path) {
            f->name = STRING_Dup(path);
            if (f->name) {
                return True;
            } else {
                FILE_Finish(f); /* no more I/O */
            }
        } else {
            f->name = NULL;
            return True;
        }
    }
    return False;
}

/**
 * Sets a file type specific I/O parameter.
 */
Bool FILE_SetParam(File * f, Str name, void * value)
{
    ASSERT(f);
    if (f && (f->flags & FILE_IS_OPEN) && f->io->setparam) {
        return f->io->setparam(f, name, value);
    }
    return False;
}

/**
 * Set the file name returned by FILE_GetName. This call doesn't affect
 * the actual file I/O. It's used for logging purposes etc.
 */
Bool FILE_SetName(File * f, Str name)
{
    if (f) {
        if (name) {
            Char * copy = STRING_Dup(name);
            if (copy) {
                MEM_Free(f->name);
                f->name = copy;
                return True;
            }
        } else {
            MEM_Free(f->name);
            f->name = NULL;
            return True;
        }
    }
    return False;
}

/**
 * Returns the file name. The returned pointer is valid until next 
 * FILE_Close(), FILE_Reopen(), FILE_Detach() or FILE_SetName() call.
 * If this file does not have a name (passed NULL path to FILE_Init)
 * then the name of the target file will be returned. Returns NULL if
 * it doesn't find non-empty file name in the the chain of streams.
 */
Str FILE_GetName(File * f)
{
    ASSERT(f);
    while (f) {
        if (f->name && f->name[0]) {
            return f->name;
        }
        f = FILE_Target(f);
    }

    return NULL;
}

/**
 * Same as the above, only it returns empty string instead of NULL if it
 * fails to determine the file name.
 */
Str FILE_Name(File * f)
{
    Str name = FILE_GetName(f);
    return (name ? name : TEXT(""));
}

/**
 * Returns the flags
 */
int FILE_Flags(File * f)
{
    ASSERT(f);
    return (f->flags & FILE_PUBLIC_FLAGS);
}

/**
 * Set flags
 */
void FILE_SetFlag(File * f, int flags)
{
    ASSERT(f);
    f->flags |= (flags & FILE_PUBLIC_FLAGS);
}

/**
 * Clear flags
 */
void FILE_ClrFlag(File * f, int flags)
{
    ASSERT(f);
    f->flags &= (~(flags & FILE_PUBLIC_FLAGS));
}

/**
 * Reads data from the stream. Returns the number of bytes actually read,
 * which may be less than the requested count if an error occurs or if the
 * end of the file is encountered before reaching the requested count.
 * Returns (-1) if an error occurs and no data has been read.
 */
int FILE_Read(File * f, void * buf, int len)
{
    ASSERT(f);
    ASSERT(len >= 0);
    if (CAN_READ(f) && len >= 0) {
        int n1 = 0;
        I8u * dest = (I8u*)buf;
        while (f->pushed > 0 && n1 < len) {
            dest[n1++] = f->pushBack[--(f->pushed)];
            f->bytesRead++;
        }
        if (n1 == 0) {
            n1 = f->io->read(f, buf, len);
            if (n1 > 0) f->bytesRead += n1;
            return n1;
        } else {
            if (FILE_Eof(f)) {
                return n1;
            } else {
                int n2 = f->io->read(f, dest + n1, len - n1);
                if (n2 >= 0) {
                    f->bytesRead += n2;
                    return (n1 + n2);
                } else {
                    return n1;
                }
            }
        }
    } else {
        return (-1);
    }
}

/**
 * Reads data from stream. Returns True if all requested bytes have been
 * read from the stream, or False in case of an error, end of file condition
 * or partial read.
 */
Bool FILE_ReadAll(File * f, void * buf, int len)
{
    int n = FILE_Read(f, buf, len);
    if (n == len) {
        return True;
    } else {
        ASSERT(n < len);
        return False;
    }
}

/**
 * Pushes back some data that will be read from the file next time
 * FILE_Read is invoked. NOTE that the size of the pushback buffer
 * is very small. It's only guaranteed to be enough to push back
 * one character, so that one FILE_Ungetc call will always succeed.
 * Returns number of bytes actually pushed back to the file's buffer.
 * 
 * NOTE that the data are being placed to the pushback buffer starting
 * from the LAST byte in the provided buffer. That is, if this function
 * returns n, it means that the LAST n bytes have been pushed back.
 */
int FILE_PushBack(File * f, const void * buf, int len)
{
    int n = 0;
    ASSERT(len >= 0);
    if (len > 0) {
        ASSERT(f->bytesRead >= (size_t)len);
        if (CAN_READ(f)) {
            const I8u * src = (I8u*)buf;
            while (n < len && f->pushed < (int)sizeof(f->pushBack)) {
                f->pushBack[(f->pushed)++] = src[len - (n++) - 1];
                f->bytesRead--;
            }
        }
    }
    return n;
}

/**
 * Writes data to the stream. Returns the number of bytes actually written, 
 * which may be less than the buffer length if an error occurs. Returns (-1) 
 * if an error occurs and no data has been written.
 */
int FILE_Write(File * f, const void * buf, int len)
{
    ASSERT(f);
    if (CAN_WRITE(f)) {
        int n = f->io->write(f, buf, len);
        if (n > 0) f->bytesWritten += n;
        return n;
    } else {
        return (-1);
    }
}

/**
 * Writes data to the stream. Returns True if the entire buffer has been 
 * written to the file, or False if less than len bytes have been written.
 */
Bool FILE_WriteAll(File * f, const void * buf, int len)
{
    ASSERT(f);
    if (CAN_WRITE(f)) {
        int n = f->io->write(f, buf, len);
        if (n > 0) f->bytesWritten += n;
        if (n == len) {
            return True;
        }
        ASSERT(n < len);
    }
    return False;
}

/**
 * Reads and discards the specified number of bytes from the stream.
 * Returns the number of bytes actually skipped 
 */
size_t FILE_Skip(File * f, size_t skip)
{
    size_t count = 0;
    if (skip > 0) {
        int n;
        if (f->io->skip) {

            /* this I/O method has an optimized skip callback, use it */
            while (skip > (size_t)INT_MAX) {

                /* FileSkip can't skip more than INT_MAX at a time */
                n = f->io->skip(f, INT_MAX);
                if (n <= 0) {
                    return count;
                } else {
                    f->bytesRead += n;
                    count += n;
                    skip -= n;
                }
            }

            if (skip > 0) {
                n = f->io->skip(f, (int)skip);
                if (n > 0) {
                    f->bytesRead += n;
                    count += n;
                }
            }

        } else {

            /* just read and discard the data */
            int minibuf[64];
            void* buf;

            /* alloc 12K buffer if we are skipping large amounts of data */
            size_t bufsize = 12000;
            if (skip <= (int)(sizeof(minibuf)*2)) {
                buf = minibuf;
                bufsize = sizeof(minibuf);
            } else {
                if (skip < bufsize) bufsize = skip;
                buf = MEM_Alloc(bufsize);
                if (!buf) {
                    buf = minibuf;
                    bufsize = sizeof(minibuf);
                }
            }

            /* read full buffers */
            while (skip > bufsize) {
                n = FILE_Read(f, buf, (int)bufsize);
                if (n <= 0) {
                    /* deallocate temporary buffer */
                    if (buf != minibuf) MEM_Free(buf);
                    return count;
                 } else {
                    count += n;
                    skip -= n;
                }
            }

            /* read the rest */
            if (skip > 0) {
                n = FILE_Read(f, buf, (int)skip);
                if (n > 0) count += n;
            }

            /* deallocate temporary buffer */
            if (buf != minibuf) MEM_Free(buf);
        }
    }
    return count;
}

/**
 * Reads and discards the specified number of bytes from the stream.
 * Returns True if the requested number of bytes have been skipped.
 */
Bool FILE_SkipAll(File * f, size_t skip)
{
    size_t n = FILE_Skip(f, skip);
    if (n == skip) {
        return True;
    } else {
        ASSERT(n < skip);
        return False;
    }
}

/**
 * Prints formatted data to the stream. Returns the number of characters 
 * actually written to the stream, or (-1) if it fails.
 */
int FILE_Printf(File * f, Str format, ...)
{
    int nbytes;
    va_list va;
    va_start(va, format);
    nbytes = FILE_VaPrintf(f, format, va);
    va_end(va);        
    return nbytes;
}

/**
 * Prints formatted data to the stream. Returns the number of characters 
 * actually written to the stream, or (-1) if it fails.
 */
int FILE_VaPrintf(File * f, Str format, va_list va)
{
    int nbytes = -1;
    ASSERT(f);
    if (CAN_WRITE(f)) {
        if (STRBUF_FormatVa(&f->buf, format, va)) {
            nbytes = f->io->write(f, f->buf.s, (int)f->buf.len);
            if (nbytes > 0) f->bytesWritten += nbytes;
        }
    }
    return nbytes;
}

/**
 * Writes a string to the stream. Returns True if string was written 
 * successfully, False if no data was written and MayBe if only part 
 * of the string was written to the stream.
 */
Bool FILE_Puts(File * f, Str s)
{
    ASSERT(f);
    ASSERT(s);
    if (CAN_WRITE(f)) {
        if (!*s) {
            return True;
        } else {
            size_t nbytes;
#ifdef UNICODE
            size_t n = wcstombs(NULL,s,0) + 1;
            char * bytes = NULL;
            if (((int)n) > 0) {

                /*
                 * use the string buffer for storing the result of 
                 * translating UNICODE string into multibyte 
                 */

                STRBUF_SetLength(&f->buf, 0);
                if (STRBUF_Alloc(&f->buf,(n+1)/2)) {
                    bytes = (char*)(f->buf.s);
                    n = wcstombs(bytes,s,n);
                    if (n == ((size_t)-1)) {
                        return False;
                    }
                    bytes[n] = 0;
                } else {
                    return False;
                }
            }
#else
            size_t n = StrLen(s);
            const char * bytes = s;
#endif /* ! UNICODE */

            nbytes = f->io->write(f, bytes, (int)n);

#ifdef UNICODE
            if (n > 0) f->buf.s[0] = 0;
#endif /* UNICODE */
            if (nbytes > 0) { 
                f->bytesWritten += nbytes;
                if (nbytes == n) {
                    return True;
                } else {
                    return MayBe;   /* partial write */
                }
            }
        }
    }
    return False;
}

/**
 * Reads a string from the stream. False is returned to indicate an I/O
 * error, end-of-file condition or detection of binary data (i.e. a zero
 * has been received from the stream)
 */
Bool FILE_Gets(File * f, Char * buf, size_t len)
{
    Char * s = buf;
    if (!buf || !len || !CAN_READ(f)) return False;
    while (len-- > 1) {
        int c = FILE_Getc(f);
        if (c == 0) {
            FILE_Ungetc(f, (Char)c);
            break; /* binary data */
        }
        if ((c == EOF) || ((*s++ = (Char)c) == '\n')) break;
    }
    *s = 0;
    return BoolValue(s != buf || len == 0);
}

/**
 * Writes a character to the stream. Returns True on success, False on error
 */
Bool FILE_Putc(File * f, int c)
{
    Char s[2];
    s[0] = (Char)c;
    s[1] = 0;

    /* If the character is zero, this function won't do what you want.
     * Besides, it probably means that you are writing binary data
     * rather than characters. */
    ASSERT(c);
    return FILE_Puts(f, s);
}

/**
 * Reads one byte from the stream. Returns the byte read as an int,
 * or EOF in case of error.
 */
int FILE_GetByte(File * f)
{
    ASSERT(f);
    if (CAN_READ(f)) {
        char ch = 0;
        int nbytes = FILE_Read(f, &ch, 1);
        if (nbytes == 1) {
            return (((int)ch) & 0xff);
        }
    }
    return EOF;
}

/**
 * Writes one byte to the stream. Returns True on success, False on error
 */
Bool FILE_PutByte(File * f, int b)
{
    I8u buf = (I8u)b;
    return (FILE_Write(f, &buf, 1) == 1);
}

/**
 * Reads a character to the stream. Returns the character read,
 * or EOF in case of error.
 */
int FILE_Getc(File * f)
{
    ASSERT(f);
    if (CAN_READ(f)) {
#ifndef UNICODE
        char ch = 0;
        int nbytes = FILE_Read(f, &ch, 1);
        if (nbytes == 1) {
            return (((int)ch) & 0xff);
        }
#else  /* UNICODE */
        char mbs[5];
        int i, n = COUNT(mbs)-1;
        wchar_t ws[2];
        for (i=0; i<n; i++) {
            if (FILE_Read(f, mbs+i, 1) != 1) return EOF;
            mbs[i+1] = 0;
            if (mbstowcs(ws, mbs, COUNT(ws)) == 1) {
                return ws[0];
            }
        }
        ASSMSG("Invalid multibyte sequence?");
#endif /* UNICODE */
    }
    return EOF;
}

/**
 * Pushes c back to stream, where it is  available for subsequent read 
 * operations. Pushed-back characters will be returned in reverse order;
 * only one pushback is guaranteed.
 */
Bool FILE_Ungetc(File * f, Char c)
{
    Bool ok = False;
    ASSERT(f);
    if (CAN_READ(f)) {
#ifndef UNICODE
        ok = BoolValue(FILE_PushBack(f, &c, 1) == 1);
#else  /* UNICODE */
        int n;
        char mbs[COUNT(f->pushBack)+1];
        int avail = COUNT(f->pushBack) - f->pushed;
        wchar_t ws[2];
        ws[0] = c;
        ws[1] = 0;
        n = (int)wcstombs(mbs, ws, avail + 1);
        if (n > 0 && n <= avail) {
            ok = BoolValue(FILE_PushBack(f, mbs, n));
            ASSERT(ok);
        }
#endif /* UNICODE */
    }
    return ok;
}

/**
 * Flushes the stream. Returns True on success.
 */
Bool FILE_Flush(File * f)
{
    ASSERT(f);
    return ((f && (f->flags & FILE_IS_OPEN) && f->io->flush) ? 
            f->io->flush(f) : False);
}

/**
 * Tests if end of file has been reached
 */
Bool FILE_Eof(File * f)
{
    ASSERT(f);
    return ((f && (f->flags & FILE_IS_OPEN)) ? f->io->eof(f) : True);
}

/**
 * This function returns the underlying file descriptor for this File object.
 * If this File object is not associated with a file descriptor, returns -1
 */
int FILE_Fd(File * f)
{
    ASSERT(f);
    if (f && f->io->fd) {
        return f->io->fd(f);
    }
    return -1;
}

/**
 * This call is similar to FILE_Fd, except that it skips File wrappers
 * until it files a real File object associated with a file descriptor.
 * Returns -1 if it doesn't find any.
 */
int FILE_TargetFd(File * f)
{
    while (f) {
        if (f->io->fd) {
            int fd = f->io->fd(f);
            if (fd >= 0) {
                return fd;
            }
        }
        f = FILE_Target(f);
    }
    return -1;
}

/**
 * If this stream redirects (possibly modified) data to another file,
 * this method returns the target file. Examples of such streams are
 * ZIP and WRAP streams
 */
File * FILE_Target(File * f)
{
    ASSERT(f);
    if (f && f->io->target) {
        return f->io->target(f);
    }
    return NULL;
}

/**
 * Detach the File structure from the underlying socket/file descriptor, etc.
 * The File must have been created with one of the FILE_Attach* calls.
 */
void FILE_Detach(File * f)
{
    ASSERT(f);
    if (f) {
        if (f->flags & FILE_IS_ATTACHED) {
            f->io->detach(f);
            f->pushed = 0;
            f->flags &= (~(FILE_IS_OPEN | FILE_IS_ATTACHED));
        }
    }    
}

/**
 * Closes the file but doesn't deallocate the File structure. Any I/O
 * operation will return failure after this call.
 */
void FILE_Finish(File * f)
{
    ASSERT(f);
    if (f) {
        if (f->flags & FILE_IS_OPEN) {
            f->io->close(f);
            f->pushed = 0;
            f->flags &= (~FILE_IS_OPEN);
        }
    }    
}

/**
 * Closes the file. After this call returns, the File structure is 
 * no longer usable.
 */
void FILE_Close(File * f)
{
    ASSERT(f);
    if (f) {
        FileFree ff = f->io->free;
        FILE_Finish(f);
        MEM_Free(f->name);
        STRBUF_Destroy(&f->buf);
        (*ff)(f);
    }
}

/**
 * Returns the number of bytes read from the file
 */
size_t FILE_BytesRead(File * f)
{
    return f->bytesRead;
}

/**
 * Returns the number of bytes written to the file
 */
size_t FILE_BytesWritten(File * f)
{
    return f->bytesWritten;
}

/**
 * Tests if this stream performs file based I/O. If this function return 
 * True, then the string returned by FILE_Name probably represents a file
 * on disk. Otherwise, it can be pretty much anything.
 */
Bool FILE_IsFileIO(File * f)
{
    ASSERT(f);
    while (f) {
        if (f->io->flags & FIO_FILE_BASED) {
            return True;
        }
        f = FILE_Target(f);
    }
    return False;
}

/**
 * Tests whether reads from this stream can block before the end of stream.
 */
Bool FILE_CanBlock(File * f)
{
    ASSERT(f);
    while (f) {
        if (f->flags & FILE_CAN_BLOCK) {
            return True;
        }
        f = FILE_Target(f);
    }
    return False;
}

/*==========================================================================*
 *              U T I L I T I E S
 *==========================================================================*/

#define STACK_BUF_SIZE 1024  /* size of the buffer we allocate on stack */

/**
 * Copies the data from one stream and writes it into another. Returns number
 * of bytes copied, -1 if nothing has been written due to an error. There's 
 * no return value to detect partial copy.
 */
int FILE_Copy(File * in, File * out)
{
    int n, total = 0;
    I8u data[STACK_BUF_SIZE];
    while ((n = FILE_Read(in, data, sizeof(data))) > 0) {
        int nread = n;
        n = FILE_Write(out, data, nread);
        if (n > 0) {
            total += n;
            if (n == nread) {
                continue;
            }
        }
        break;
    }
    return ((total > 0) ? total : ((n == 0) ? 0 : -1));
}

/**
 * Copies no more than max bytes from one stream to another. Negative
 * max size means to copy all data until end of file. Returns number
 * of bytes successfully read from the input file and written to the
 * output file. Note that number of bytes read from the input file
 * may be greater than the number of bytes written to the output file.
 */
int FILE_CopyN(File * in, File * out, int max)
{
    if (max < 0) {
        return FILE_Copy(in, out);
    } else if (max == 0) {
        return FILE_Read(in, NULL, 0);
    } else {
        int n, total = 0;
        I8u data[STACK_BUF_SIZE];
        int chunk = MIN(STACK_BUF_SIZE,max);
        while ((n = FILE_Read(in, data, chunk)) > 0) {
            int nread = n;
            n = FILE_Write(out, data, nread);
            if (n > 0) {
                total += n;
                if (n == nread && max > total) {
                    chunk = MIN(STACK_BUF_SIZE,max-total);
                    continue;
                }
            }
            break;
        }
        return total;
    }
}

/**
 * Reads a line from the stream. Returns the pointer to the string buffer,
 * NULL if nothing was read from the file (that includes both end-of-line
 * and error conditions)
 */
Str FILE_ReadLine(File * in, StrBuf * sb) 
{
    STRBUF_Clear(sb);
    if (sb->alloc > 1 || STRBUF_Alloc(sb, 63)) {
        while (FILE_Gets(in, sb->s + sb->len, sb->alloc - sb->len)) {
            sb->len += StrLen(sb->s + sb->len);
            if (sb->len == 0) {
                ASSERT(False);
                return STRBUF_Text(sb);
            } else if (sb->s[sb->len-1] == '\n') {
                sb->s[--(sb->len)] = 0;
                if (sb->len > 0 && sb->s[sb->len-1] == '\r') {
                    sb->s[--(sb->len)] = 0;
                }
                return STRBUF_Text(sb);
            }

            /* we don't see '\n' in the end, continue reading */
            if (!STRBUF_Alloc(sb, sb->len * 2)) {
                return NULL;
            }
        }
        return ((sb->len > 0) ? STRBUF_Text(sb) : NULL);
    }
    return NULL;
}

/**
 * Reads the contents of the file into a buffer. Returns number of bytes
 * stored in the buffer. If max parameter is >= 0, reads no more than
 * max bytes
 */
int FILE_ReadData(File * in, Buffer * out, int max)
{
    I8u data[STACK_BUF_SIZE];
    size_t initial = BUFFER_Size(out);
    size_t bufsiz = initial;

    /* size of the first chunk to read */
    int chunk = ((max<0) ? STACK_BUF_SIZE : MIN(STACK_BUF_SIZE,max));
    while (BUFFER_EnsureCapacity(out, bufsiz + chunk, True)) {
        int maxsiz = (int)MIN(out->alloc - bufsiz, (size_t)chunk);
        int nbytes = FILE_Read(in, data, maxsiz);
        if (nbytes > 0) {
            /* this must succeed since memory has been pre-allocated */
            BUFFER_Put(out, data, nbytes, True);
            bufsiz += nbytes;
            ASSERT(bufsiz == BUFFER_Size(out));

            /* calculate the size of the next chunk */
            if (max < 0) {
                chunk = STACK_BUF_SIZE;
            } else {
                ASSERT(max >= nbytes);
                max -= nbytes;
                if (max == 0) {
                    break;
                } else {
                    chunk = MIN(STACK_BUF_SIZE,max);
                }
            }
        } else {
            break;
        }
    }
    return (int)(bufsiz - initial);
}

/**
 * Dump the data to the stream 
 */
void FILE_Dump(File* out, const void* buf, size_t off, size_t len, size_t max)
{

#define PRINTABLE(c) (IsPrint(c) ? (c) : '.')
#define BYTES_PER_LINE  16

    size_t i,j;
    const size_t maxCount = MIN(len,max);
    const unsigned char* data = (unsigned char*)buf;
    Char line[100];

    for (i=0; i<maxCount; i += BYTES_PER_LINE) {
        Char tmp[8];
        const Char* sep = TEXT("    ");
        Sprintf(line, TEXT("   %04lx: "), (unsigned long)(i+off));

        /* hex bytes */
        for (j=i; j < (i+BYTES_PER_LINE); j++) {
            if (j < maxCount) {
                Sprintf(tmp, TEXT("%02x "), (unsigned int)data[j]);
                StrCat(line, tmp);
            } else if (j == maxCount && j < len) {
                StrCat(line, TEXT(" ..."));
                sep = TEXT("   ");
            } else {
                StrCat(line, TEXT("   "));
            }
        }

        /* ASCII letters */
        StrCat(line, sep);
        tmp[1] = 0;
        for (j =i; j<(i+BYTES_PER_LINE); j++) {
            if (j<maxCount) {
                tmp[0] = PRINTABLE(data[j]);
                StrCat(line, tmp);
            } else {
                StrCat(line, TEXT(" "));
            }
        }

        FILE_Printf(out, TEXT("%s\n"), line);
    }

    /* if the data was truncated, print the ellipses ... */
    if (i < len) {
        Sprintf(line, TEXT("   %04lx: "), (unsigned long)(i+off));
        StrCat(line, TEXT("..."));
        FILE_Printf(out, TEXT("%s\n"), line);
    }
}

/*
 * HISTORY:
 *
 * $Log: s_file.c,v $
 * Revision 1.60  2010/12/19 17:51:46  slava
 * o don't ASSERT if we are pushing back zero bytes
 *
 * Revision 1.59  2010/10/23 15:55:25  slava
 * o added FILE_Dump function
 *
 * Revision 1.58  2009/12/28 12:52:10  slava
 * o fixed 64-bit compilation warning in FILE_Skip
 *
 * Revision 1.57  2009/12/26 13:57:15  slava
 * o updated FILE_Skip to take size_t as the number of bytes to skip
 *
 * Revision 1.56  2009/12/26 12:21:32  slava
 * o working on 64-bit issues
 *
 * Revision 1.55  2009/10/08 14:32:11  slava
 * o added FILE_Fd() and FILE_TargetFd() functions
 *
 * Revision 1.54  2008/09/03 12:22:21  slava
 * o added FILE_PutByte function
 *
 * Revision 1.53  2008/09/03 09:37:02  slava
 * o removed static FIO_BLOCKING_READ flag and added FILE_CAN_BLOCK flag that
 *   can be set on individual streams. This is a lot more flexible. Added
 *   FILE_CanBlock function.
 *
 * Revision 1.52  2008/09/01 12:08:15  slava
 * o replaced FileIO::fileIO field with flags. Added FIO_BLOCKING_READ flag
 *   which marks I/O methods whose read callback can block before the end of
 *   stream is reached (i.e. socket I/O).
 *
 * Revision 1.51  2008/09/01 10:57:21  slava
 * o added 'skip' I/O callback which can be implemented by those I/O methods
 *   which can efficiently skip input data without actually reading and
 *   copying anything. Currently, such callback is only implemented for
 *   in-memory I/O, but it can also be done for file I/O using fseek.
 *
 * Revision 1.50  2007/03/05 02:01:57  slava
 * o added FILE_SetName() function. This function only affects the output
 *   of FILE_Name() and FILE_GetName(). It has no effect on the actual I/O.
 *
 * Revision 1.49  2007/02/08 17:43:59  slava
 * o added internal function FILE_Destroy
 *
 * Revision 1.48  2007/01/26 16:45:19  slava
 * o added FILE_GetByte function. Unlike FILE_Getc it does the same thing in
 *   both Unicode and non-Unicode builds.
 *
 * Revision 1.47  2006/10/20 04:40:49  slava
 * o moved FILE_ReadI32L, FILE_ReadI32B etc. from s_file.c to s_futil.c
 *
 * Revision 1.46  2006/10/13 23:32:25  slava
 * o added FILE_GetName() function which returns NULL if doesn't find non-empty
 *   file name in the the chain of streams, unlike FILE_Name() which returns an
 *   empty string in such case.
 *
 * Revision 1.45  2006/10/13 23:02:06  slava
 * o fixed newly introduced bug in FILE_Name
 * o added ASSERT in FILE_Putc that character isn't zero
 *
 * Revision 1.44  2006/10/13 16:14:22  slava
 * o count number of bytes written to and read from the stream
 *
 * Revision 1.43  2006/03/26 05:50:50  slava
 * o added a bunch of utilities for reading and writing binary data with byte
 *   order conversion
 *
 * Revision 1.42  2006/03/25 07:23:27  slava
 * o added FILE_ReadAll, FILE_WriteAll, FILE_SkipAll and FILE_CopyN functions
 *
 * Revision 1.41  2005/06/13 12:10:01  slava
 * o fixed a problem with FILE_ReadData which was doing an extra
 *   zero-byte read after reading the requested amount of data. It
 *   makes no difference with file-based streams, but socket-based
 *   stream can block on a zero-byte read.
 *
 * Revision 1.40  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.39  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.38  2004/10/20 00:40:13  slava
 * o use BoolValue macro to convert int into Bool
 *
 * Revision 1.37  2004/04/08 01:44:18  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.36  2004/04/01 18:46:37  slava
 * o don't reference PlainFileIO in kernel mode
 *
 * Revision 1.35  2003/12/26 15:21:40  slava
 * o FILE_Getc should cast char to int without extending the sign to ensure
 *   that a successfully read char cannot be confused with EOF
 *
 * Revision 1.34  2003/06/05 02:42:25  slava
 * o added FILE_PushBack and FILE_Ungetc functions
 *
 * Revision 1.33  2003/01/08 17:37:48  slava
 * o allow to pass NULL path argument to FILE_Init. In that case, FILE_Name
 *   will return the name of the target file. This rule recursively applies
 *   to the target file as well.
 *
 * Revision 1.32  2003/01/08 17:15:43  slava
 * o removed ZIP_GetTarget and WRAP_GetTarget functions, replaced them
 *   with generic FILE_Target
 *
 * Revision 1.31  2002/12/30 21:44:10  slava
 * o added FILE_IsFileIO function
 *
 * Revision 1.30  2002/08/27 11:45:11  slava
 * o added FILE_Copy
 *
 * Revision 1.29  2002/08/01 03:45:15  slava
 * o added another parameter for FILE_ReadData - maximum number of bytes
 *   to read. If this parameter is negative, the function behaves the
 *   same way as before - reads the file until either end-of-file or
 *   out-of-memory condition.
 *
 * Revision 1.28  2002/05/22 05:00:27  slava
 * o fixed gcc compilation warning
 *
 * Revision 1.27  2002/05/22 04:07:06  slava
 * o renamed STRBUF_Read into FILE_ReadLine and moved it from s_strbuf
 *   to s_file module. Also, added FILE_ReadData utility
 *
 * Revision 1.26  2001/12/25 04:33:33  slava
 * o fixed a stupid bug in FILE_Skip which made it much less efficient 
 *   (although didn't break it...)
 *
 * Revision 1.25  2001/12/24 17:18:31  slava
 * o added FILE_Skip
 *
 * Revision 1.24  2001/12/23 21:17:00  slava
 * o moved each file I/O implemenetation into a separate file. This prevents
 *   unnecessary linkage with zlib and socket library and makes executables
 *   smaller. Most linkers are not very good in removing dead references
 *
 * Revision 1.23  2001/12/23 20:17:41  slava
 * o made gzipped I/O work on Windows CE
 *
 * Revision 1.22  2001/12/21 01:49:52  slava
 * o fixed compilation warnings in release build
 *
 * Revision 1.21  2001/12/20 10:44:31  slava
 * o port to Windows CE
 *
 * Revision 1.20  2001/10/15 04:33:52  slava
 * o fixed a similar problem with the "splitter" stream
 * o cleaned up FILE_IS_OPEN/FILE_IS_ATTACHED mess
 *
 * Revision 1.19  2001/10/15 04:09:20  slava
 * o fixed a bug in the "wrapper" stream. It shouldn't close the target
 *   stream until its free() callback is invoked. Otherwise, a perfectly
 *   legal sequence of FILE_* calls (involving FILE_Finish followed by
 *   FILE_Detach) performed on a "wrapped" file may cause a crash
 *
 * Revision 1.18  2001/10/10 16:00:31  slava
 * o added output stream "splitter"
 *
 * Revision 1.17  2001/10/09 18:15:45  slava
 * o "wrapper" bug fix
 *
 * Revision 1.16  2001/10/09 15:29:52  slava
 * o changed semantics of WRAP_Indent(). First, it now manipulates
 *   something that I call "indentation level". The actual number of
 *   positions is indentation level multiplied by the indentation step,
 *   the latter is defined when wrapper is created. Second, WRAP_Indent()
 *   now *changes* the current indentation level, typically by +1 or -1,
 *   rather then *sets* it. The indentation level can be set by calling
 *   WRAP_SetIndent()
 *
 * Revision 1.15  2001/10/08 05:17:51  slava
 * o eliminated some strict gcc warnings (mostly shadow declarations)
 *
 * Revision 1.14  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.13  2001/05/18 22:36:57  slava
 * o THIS_FILE variable is no longer being used
 *
 * Revision 1.12  2001/04/21 17:04:19  slava
 * o use SOCKET_Connect() function to connect a socket
 *
 * Revision 1.11  2001/04/13 05:53:51  slava
 * o fixed compilation warnings on Solaris
 *
 * Revision 1.10  2001/04/12 07:55:31  slava
 * o added simple "text wrapper" stream. Obvously, it should only be
 *   applied to text streams. The "wrapper" also supports indentation.
 *   This should simplify formatting of the text files and also solve
 *   the problem with some editors which cannot handle long lines,
 *   Microsoft Visual Studio (2K limit) being one of them. Since this
 *   functionality is hidden behind the File interface, if is very
 *   simple to switch between "wrapped" and "non-wrapped" modes.
 *
 * Revision 1.9  2001/03/11 03:24:05  slava
 * o fixed a comment
 *
 * Revision 1.8  2001/03/11 03:22:13  slava
 * o replaced FILE_SetFlags() with FILE_SetFlag() and FILE_ClrFlag()
 *
 * Revision 1.7  2001/03/11 02:47:43  slava
 * o added user-settable flags
 *
 * Revision 1.6  2001/01/13 16:05:41  slava
 * o added FILE_VaPrintf() function
 *
 * Revision 1.5  2001/01/12 00:30:53  slava
 * o renamed FILE_Socket() into FILE_AttachToSocket()
 * o added FILE_Detach() and FILE_AttachToFile()
 *
 * Revision 1.4  2001/01/09 14:49:36  slava
 * o the Close functionality has been separated from Free in order to
 *   properly implement Finish semantics. The original implementation
 *   of FILE_Finish was no good.
 *
 * Revision 1.3  2001/01/06 05:03:37  slava
 * o added support for network socket I/O via generic interface
 *
 * Revision 1.2  2001/01/05 02:27:12  slava
 * o minor cleanup
 *
 * Revision 1.1  2001/01/03 09:18:23  slava
 * o new file I/O support for transparent access to plain or compressed
 *   files, sockets, etc.
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
