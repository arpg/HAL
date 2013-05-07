/*
 * $Id: s_xml.c,v 1.28 2007/01/27 01:47:15 slava Exp $
 *
 * Copyright (C) 2001-2007 by Slava Monich
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

#include "s_mem.h"
#include "s_xmlp.h"
#include "s_util.h"
#include "s_vector.h"

/* Empty XML attribute set */
const XMLAttr xmlEmptyAttr = {0};

/* Macros */
#define _XML_AttrNameAt(a,i) ((a)->storage + (a)->off[2*(i)])
#define _XML_AttrValueAt(a,i) ((a)->storage + (a)->off[2*(i)+1])

/*==========================================================================*
 *              A T T R I B U T E S
 *==========================================================================*/

/**
 * Copies the attribute set. If this function fails, the destination attribute
 * set is left unchanged.
 */
Bool XML_AttrCopy(XMLAttr * dest, const XMLAttr * src)
{
    XMLAttr bak = *dest;
    memset(dest, 0, sizeof(*dest));
    if (src->n == 0) {
        XML_AttrDestroy(&bak);
        return True;
    } else {
        ASSERT(src->size > 0);
        dest->size = src->size;
        dest->n = src->n;
        dest->storage = (Char*)MEM_Alloc(src->size);
        if (dest->storage) {
            dest->size = src->size;
            memcpy(dest->storage, src->storage, src->size);
            dest->off = (int*)MEM_Alloc((2*src->n)*sizeof(int));
            if (dest->off) {
                memcpy(dest->off, src->off, (2*src->n)*sizeof(int));
                XML_AttrDestroy(&bak);
                return True;
            }
            MEM_Free(dest->storage);
        }
    }
    *dest = bak;
    return False;
}

/**
 * Clones the attribute set
 */
XMLAttr * XML_AttrClone(const XMLAttr * src)
{
    XMLAttr * dest = MEM_New(XMLAttr);
    if (dest) {
        memset(dest, 0, sizeof(*dest));
        if (XML_AttrCopy(dest, src)) {
            return dest;
        }
        MEM_Free(dest);
    }
    return NULL;
}

/**
 * Destroys the attribute set
 */
void XML_AttrDestroy(XMLAttr * atts)
{
    MEM_Free(atts->off);
    MEM_Free(atts->storage);
    memset(atts, 0, sizeof(*atts));
}

/**
 * Deletes dynamically allocated attribute set
 */
void XML_AttrDelete(XMLAttr * atts)
{
    ASSERT(atts != &xmlEmptyAttr);
    if (atts && atts != &xmlEmptyAttr) {
        XML_AttrDestroy(atts);
        MEM_Free(atts);
    }
}

/**
 * Returns number of attributes
 */
int XML_AttrCount(const XMLAttr * atts)
{
    ASSERT(atts->n >= 0);
    return atts->n;
}

/**
 * Returns the value of the attribute specified by name
 */
Str XML_AttrValue(const XMLAttr * atts, Str name)
{
    int i;
    for (i=0; i<atts->n; i++) {
        if (StrCmp(_XML_AttrNameAt(atts,i),name) == 0) {
            return _XML_AttrValueAt(atts,i);
        }
    }
    return NULL;
}

/**
 * Returns the name of the attribute at specified position
 */
Str XML_AttrNameAt(const XMLAttr * atts, int pos)
{
    ASSERT(pos >= 0 && pos < atts->n);
    return _XML_AttrNameAt(atts,pos);
}

/**
 * Returns the value of the attribute at specified position
 */
Str XML_AttrValueAt(const XMLAttr * atts, int pos)
{
    ASSERT(pos >= 0 && pos < atts->n);
    return _XML_AttrValueAt(atts,pos);
}

/**
 * Returns True if and only if the specified attribute exists and has
 * exactly the specified value (comparison is case sensitive).
 */
Bool XML_MatchAttr(const XMLAttr * a, Str name, Str value)
{
    Str attrValue = XML_AttrValue(a, name);
    if (attrValue) {
        return value && StrCmp(value, attrValue) == 0;
    } else {
        return (value == NULL);
    }
}

/*==========================================================================*
 *              U T I L I T I E S 
 *==========================================================================*/

/**
 * Converts a nibble to a hex character
 */
STATIC Char XML_ToHex(int nibble)
{
    /** A table of hex digits */
    static Char hexDigit[] = {
        TEXT('0'),TEXT('1'),TEXT('2'),TEXT('3'),
        TEXT('4'),TEXT('5'),TEXT('6'),TEXT('7'),
        TEXT('8'),TEXT('9'),TEXT('A'),TEXT('B'),
        TEXT('C'),TEXT('D'),TEXT('E'),TEXT('F')
    };
    return hexDigit[(nibble & 0xF)];
}

/**
 * XMLizes a string by escaping special characters.
 * Returns the second argument if string does not need to be 
 * escaped. If string needs to be escaped, returns pointer
 * to the string buffer (the first parameter) or NULL if the
 * first parameter is NULL or STRBUF_Append() failed (i.e.
 * running out of memory)
 */
Str XML_Escape2(StrBuf * sb, Str s, Bool escapeNonPrint)
{
    Bool escape = False;
    ASSERT(s);
    if (sb) STRBUF_Clear(sb);
    if (s) {
        Str p = s;
        while (*p) {
            Char buf[10];
            Char c = *p++;
            Str replace = NULL;         /* replacement string */

            switch (c) {
            case TEXT('&') :            /* Ampersand */
                replace = TEXT("&amp;");
                break;
            case TEXT('<') :            /* Less than */
                replace = TEXT("&lt;");
                break;
            case TEXT('>') :            /* Greater than */
                replace = TEXT("&gt;");
                break;
            case TEXT('\"') :           /* Double quote */
                replace = TEXT("&quot;");
                break;
            case TEXT('\'') :           /* Single quote */
                replace = TEXT("&apos;");
                break;
            default:
#ifdef UNICODE
                if (!escapeNonPrint || (c<0x80 && IsPrint(c))) {
#else /* UNICODE */
                if (!escapeNonPrint || (c>0 && IsPrint(c))) {
#endif /* UNICODE */
                    /* normal printable character */
                    break;
                } else {
#ifdef UNICODE
                    if (c < 0x0100) {
#endif /* UNICODE */
                        /* 8-bit Unicode (US-ASCII) */
                        int len = 0;
                        buf[len++] = TEXT('&');
                        buf[len++] = TEXT('#');
                        buf[len++] = TEXT('x');
                        buf[len++] = XML_ToHex(c >> 4);
                        buf[len++] = XML_ToHex(c);
                        buf[len++] = TEXT(';');
                        buf[len] = 0;

                        replace = buf;
#ifdef UNICODE
                    } else {

                        /* 16-bit Unicode */
                        int len = 0;
                        buf[len++] = TEXT('&');
                        buf[len++] = TEXT('#');
                        buf[len++] = TEXT('x');
                        buf[len++] = XML_ToHex(c >> 12);
                        buf[len++] = XML_ToHex(c >>  8);
                        buf[len++] = XML_ToHex(c >>  4);
                        buf[len++] = XML_ToHex(c);
                        buf[len++] = TEXT(';');
                        buf[len] = 0;

                        replace = buf;
                    }
#endif /* UNICODE */
                }
            }

            if (replace) {
                if (!escape) {
                    escape = True;
                    if (sb) {
                        /* copy non-escaped characters */
                        size_t n = p-s-1;
                        if (n>0 && !STRBUF_AppendN(sb, s, n)) {
                            return NULL;
                        }
                    } else {
                        /* need string buffer */
                        return NULL;
                    }
                }
                if (!STRBUF_Append(sb, replace)) {
                    return NULL;
                }
            } else {
                if (escape) {
                    if (!STRBUF_AppendChar(sb, c)) {
                        return NULL;
                    }
                }
            }
        }
    }
    return (escape ? sb->s : s);
}

/**
 * XMLizes a string by escaping special characters.
 * Returns the second argument if string does not need to be 
 * escaped. If string needs to be escaped, returns pointer
 * to the string buffer (the first parameter) or NULL if the
 * first parameter is NULL or STRBUF_Append() failed (i.e.
 * running out of memory)
 */
Str XML_Escape(StrBuf * sb, Str s)
{
    return XML_Escape2(sb, s, True);
}

/**
 * Writes <tag to the output stream
 */
Bool XML_StartTag(File * out, Str tag)
{
    return (FILE_Puts(out, TEXT("<")) && 
            FILE_Puts(out, tag));
}

/**
 * Writes /> to the output stream, possibly terminating the line
 */
Bool XML_EndTag(File * out, Bool eol)
{
    return eol ? FILE_Puts(out, TEXT("/>\n")) : 
                 FILE_Puts(out, TEXT("/>"));
}

/**
 * Writes <tag> to the output stream, possibly terminating the line.
 * If line is terminated and the stream is a wrapping stream, increments
 * the indentation level.
 */
Bool XML_OpenTag(File * out, Str tag, Bool eol)
{
    Bool ok = False;
    Bool wrap = WRAP_IsEnabled(out);
    if (FILE_Puts(out, TEXT("<"))) {
        if (wrap) WRAP_Enable(out, False);
        if (FILE_Puts(out, tag)) {
            if (eol) {
                ok = FILE_Puts(out, TEXT(">\n"));
                if (wrap) WRAP_Indent(out, +1);
            } else {
                ok = FILE_Puts(out, TEXT(">"));
            }
        }
    }
    if (wrap) WRAP_Enable(out, True);
    return ok;
}

/**
 * Writes </tag> to the output stream, possibly terminating the line.
 * If the output stream is a wrapping stream, decrements the indentation
 * level.
 */
Bool XML_CloseTag(File * out, Str tag, Bool eol)
{
    Bool ok = False;
    Bool wrap = WRAP_IsEnabled(out);
    if (wrap) WRAP_Indent(out, -1);
    if (FILE_Puts(out, TEXT("<"))) {
        if (wrap) WRAP_Enable(out, False);
        if (FILE_Puts(out, TEXT("/")) && 
            FILE_Puts(out, tag)) {
            FILE_Puts(out, eol ? TEXT(">\n") : TEXT(">"));
            ok = True;
        }
    }
    if (wrap) WRAP_Enable(out, True);
    return ok;
}

/**
 * Writes an attribute to the output stream
 */
Bool XML_WriteAttr(File * out, Str attr, Str value)
{
    Bool ok = False;
    Str escValue;
    StrBuf64 buf;
    STRBUF_InitBufXXX(&buf);
    escValue = XML_Escape2(&buf.sb, value, True);
    if (escValue) {
        if (FILE_Puts(out, TEXT(" ")) && 
            FILE_Puts(out, attr) && 
            FILE_Puts(out, TEXT("="))) {

            /* disable wrapping while writing attribute value */
            Bool wrap = WRAP_IsEnabled(out);
            if (wrap) WRAP_Enable(out, False);
            
            /* write the attribute value */
            if (FILE_Puts(out, TEXT("\"")) &&
                FILE_Puts(out, escValue) &&
                FILE_Puts(out, TEXT("\""))) {
                ok = True;
            }

            /* re-enable wrapping */
            if (wrap) WRAP_Enable(out, True);
        }
    }
    STRBUF_Destroy(&buf.sb);
    return ok;
}


/**
 * Writes an attribute to the output stream. Does not escape non-printable
 * characters.
 */
Bool XML_WriteAttrNoEsc(File * out, Str attr, Str value)
{
    Bool ok = False;
    Str escValue;
    StrBuf64 buf;
    STRBUF_InitBufXXX(&buf);
    escValue = XML_Escape2(&buf.sb, value, False);
    if (escValue) {
        if (FILE_Puts(out, TEXT(" ")) && 
            FILE_Puts(out, attr) && 
            FILE_Puts(out, TEXT("="))) {

            /* disable wrapping while writing attribute value */
            Bool wrap = WRAP_IsEnabled(out);
            if (wrap) WRAP_Enable(out, False);
            
            /* write the attribute value */
            if (FILE_Puts(out, TEXT("\"")) &&
                FILE_Puts(out, escValue) &&
                FILE_Puts(out, TEXT("\""))) {
                ok = True;
            }

            /* re-enable wrapping */
            if (wrap) WRAP_Enable(out, True);
        }
    }
    STRBUF_Destroy(&buf.sb);
    return ok;
}

/**
 * Writes a double floating point attribute to the output stream
 */
Bool XML_WriteDoubleAttr(File * out, Str attr, double value)
{
    Bool ok = False;
    if (FILE_Puts(out, TEXT(" ")) && 
        FILE_Puts(out, attr) && 
        FILE_Puts(out, TEXT("="))) {
        Str text;
        StrBuf64 buf;
        STRBUF_InitBufXXX(&buf);

        text = STRING_FormatDouble(&buf.sb, value);
        if (text) {

            /* disable wrapping while writing attribute value */
            Bool wrap = WRAP_IsEnabled(out);
            if (wrap) WRAP_Enable(out, False);
        
            /* write the attribute value */
            ok = FILE_Putc(out, '"') &&
                 FILE_Puts(out, text) &&
                 FILE_Putc(out, '"');

            /* re-enable wrapping */
            if (wrap) WRAP_Enable(out, True);
        }
        STRBUF_Destroy(&buf.sb);
    }
    return ok;
}

/**
 * Writes a single precision floating point attribute to the output stream
 */
Bool XML_WriteFloatAttr(File * out, Str attr, float value)
{
    Bool ok = False;
    if (FILE_Puts(out, TEXT(" ")) && 
        FILE_Puts(out, attr) && 
        FILE_Puts(out, TEXT("="))) {
        Str text;
        StrBuf32 buf;
        STRBUF_InitBufXXX(&buf);

        text = STRING_FormatFloat(&buf.sb, value);
        if (text) {

            /* disable wrapping while writing attribute value */
            Bool wrap = WRAP_IsEnabled(out);
            if (wrap) WRAP_Enable(out, False);
        
            /* write the attribute value */
            ok = FILE_Putc(out, '"') &&
                 FILE_Puts(out, text) &&
                 FILE_Putc(out, '"');

            /* re-enable wrapping */
            if (wrap) WRAP_Enable(out, True);
        }
        STRBUF_Destroy(&buf.sb);
    }
    return ok;
}

/**
 * Writes an integer attribute to the output stream
 */
Bool XML_WriteIntAttr(File * out, Str attr, int value)
{
    Bool ok = False;
    if (FILE_Puts(out, TEXT(" ")) && 
        FILE_Puts(out, attr) && 
        FILE_Puts(out, TEXT("="))) {

        /* disable wrapping while writing attribute value */
        Bool wrap = WRAP_IsEnabled(out);
        if (wrap) WRAP_Enable(out, False);
        
        /* write the attribute value */
        ok = (FILE_Printf(out, TEXT("\"%d\""), value) > 0);

        /* re-enable wrapping */
        if (wrap) WRAP_Enable(out, True);
    }
    return ok;
}

/**
 * Writes an 64-bit integer attribute to the output stream
 */
Bool XML_WriteI64Attr(File * out, Str attr, I64s value)
{
    Bool ok = False;
    if (FILE_Puts(out, TEXT(" ")) && 
        FILE_Puts(out, attr) && 
        FILE_Puts(out, TEXT("="))) {

        /* disable wrapping while writing attribute value */
        Bool wrap = WRAP_IsEnabled(out);
        if (wrap) WRAP_Enable(out, False);
        
        /* write the attribute value */
        ok = (FILE_Printf(out,TEXT("\"")TEXT(I64S_FORMAT)TEXT("\""),value)>0);

        /* re-enable wrapping */
        if (wrap) WRAP_Enable(out, True);
    }
    return ok;
}

/**
 * Writes an unsigned 64-bit integer attribute to the output stream
 */
Bool XML_WriteU64Attr(File * out, Str attr, I64u value)
{
    Bool ok = False;
    if (FILE_Puts(out, TEXT(" ")) && 
        FILE_Puts(out, attr) && 
        FILE_Puts(out, TEXT("="))) {

        /* disable wrapping while writing attribute value */
        Bool wrap = WRAP_IsEnabled(out);
        if (wrap) WRAP_Enable(out, False);
        
        /* write the attribute value */
        ok = (FILE_Printf(out,TEXT("\"")TEXT(I64U_FORMAT)TEXT("\""),value)>0);

        /* re-enable wrapping */
        if (wrap) WRAP_Enable(out, True);
    }
    return ok;
}

/**
 * Writes an unsigned integer attribute to the output stream
 */
Bool XML_WriteUIntAttr(File * out, Str attr, unsigned int value)
{
    Bool ok = False;
    if (FILE_Puts(out, TEXT(" ")) && 
        FILE_Puts(out, attr) && 
        FILE_Puts(out, TEXT("="))) {

        /* disable wrapping while writing attribute value */
        Bool wrap = WRAP_IsEnabled(out);
        if (wrap) WRAP_Enable(out, False);
        
        /* write the attribute value */
        ok = (FILE_Printf(out, TEXT("\"%u\""), value) > 0);

        /* re-enable wrapping */
        if (wrap) WRAP_Enable(out, True);
    }
    return ok;
}

/**
 * Writes an integer attribute to the output stream as a hex number
 */
Bool XML_WriteHexAttr(File * out, Str attr, int value)
{
    Bool ok = False;
    if (FILE_Puts(out, TEXT(" ")) && 
        FILE_Puts(out, attr) && 
        FILE_Puts(out, TEXT("="))) {

        /* disable wrapping while writing attribute value */
        Bool wrap = WRAP_IsEnabled(out);
        if (wrap) WRAP_Enable(out, False);
        
        /* write the attribute value */
        ok = (FILE_Printf(out, TEXT("\"%X\""), value) > 0);

        /* re-enable wrapping */
        if (wrap) WRAP_Enable(out, True);
    }
    return ok;
}

/**
 * Does nothing, always returns True. Parameters are ignored. Can be used as 
 * an XMLEndTag callback for statically allocated XMLTag structures.
 */
Bool XML_DontFreeTagCB(XMLTag * tag, XMLTag * parent)
{
    UNREF(tag);
    UNREF(parent);
    return True;
}

/*
 * HISTORY:
 *
 * $Log: s_xml.c,v $
 * Revision 1.28  2007/01/27 01:47:15  slava
 * o removed STATIC from XML_Escape2 declaration
 *
 * Revision 1.27  2007/01/27 02:43:28  slava
 * o added XML_Escape2 and XML_WriteAttrNoEsc functions
 *
 * Revision 1.26  2006/10/05 17:11:49  slava
 * o added XML_WriteFloatAttr function
 *
 * Revision 1.25  2006/03/29 19:21:05  slava
 * o fixed newly introduced bug in XML_AttrValue
 *
 * Revision 1.24  2006/03/28 23:47:16  slava
 * o updated XML parsing internals. New public functions: XML_AttrClone,
 *   XML_AttrDelete and XML_AttrCopy.
 *
 * Revision 1.23  2004/12/26 18:22:20  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.22  2004/03/17 19:58:12  slava
 * o added XML_MatchAttr function and xmlEmptyAttr constant
 *
 * Revision 1.21  2004/03/16 22:18:46  slava
 * o added XML_WriteDoubleAttr function
 *
 * Revision 1.20  2003/01/06 07:16:01  slava
 * o XML_OpenTag and XML_CloseTag take care of indentation
 * o added a number of XML_WriteXxxAttr functions for writing attributes
 *   of various types.
 *
 * Revision 1.19  2003/01/05 16:58:12  slava
 * o added XML_StartTag, XML_EndTag and XML_WriteAttr utilities
 *
 * Revision 1.18  2003/01/05 06:21:51  slava
 * o moved the functions that reference XML parser into a separate file,
 *   so that we don't have to link with expat if we aren't parsing any XML
 *
 * Revision 1.17  2003/01/05 05:00:00  slava
 * o added XML_OpenTag and XML_CloseTag
 *
 * Revision 1.16  2002/12/30 22:55:01  slava
 * o fixed gcc compilation warning (local declaration shadows a parameter)
 *
 * Revision 1.15  2002/12/30 22:49:45  slava
 * o implemented XML stream parsing mechanism that allows embedding pieces
 *   of opaque XML content into the XML stream
 * o initialize Expat to use slib memory allocator
 *
 * Revision 1.14  2002/11/29 19:03:02  slava
 * o gcc compiler gets the credit for this bug fix (use of uninitialized
 *   variable)
 *
 * Revision 1.13  2002/11/29 17:07:02  slava
 * o bug fix: Expat's internal buffer was being used incorrectly
 *
 * Revision 1.12  2002/11/24 19:34:35  slava
 * o fixed compilation error in release build
 *
 * Revision 1.11  2002/09/26 23:55:12  slava
 * o some optimization: obtain the buffer from Expat with the
 *   XML_GetBuffer to avoid double copying of the input.
 *
 * Revision 1.10  2002/09/23 04:46:43  slava
 * o fixed gcc compilation warning
 *
 * Revision 1.9  2002/09/23 04:44:58  slava
 * o enabled entity parsing
 * o if a parse error occurs in debug build, print error message and the
 *   location of the error
 *
 * Revision 1.8  2002/09/23 02:55:02  slava
 * o fixed a few problems in non-Windows part of EXPAT_Characters
 *
 * Revision 1.7  2002/09/23 02:09:19  slava
 * o renamed XML_RunParser into XML_ParseStream
 *
 * Revision 1.6  2002/09/23 02:07:39  slava
 * o switched to Expat release 1.95.5
 *
 * Revision 1.5  2001/12/23 18:42:55  slava
 * o fixed WinCE compilation error
 *
 * Revision 1.4  2001/10/13 01:57:44  slava
 * o don't call FILE_Read() if FILE_Eof() returns True
 *
 * Revision 1.3  2001/10/09 21:39:51  slava
 * o fixed gcc compilation warning (comparison is always 1 due to limited
 *   range of the argument)
 *
 * Revision 1.2  2001/10/09 05:56:08  slava
 * o more XML utilities
 *
 * Revision 1.1  2001/10/08 08:25:36  slava
 * o support for parsing XML files
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
