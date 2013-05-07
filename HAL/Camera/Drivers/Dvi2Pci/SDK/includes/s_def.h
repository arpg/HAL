/*
 * $Id: s_def.h,v 1.61 2010/09/25 09:55:06 slava Exp $
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

#ifndef _SLAVA_DEF_H_
#define _SLAVA_DEF_H_

#ifndef DEBUG
#  ifdef _DEBUG
#    define DEBUG 1
#  else
#    define DEBUG 0
#  endif
#endif

/* OS dependent stuff */
#include "s_os.h"
#include "s_ver.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* define 8,16,32 and 64 bit integer types */
typedef int8_t    I8s;      /*  8 bit integer */
typedef uint8_t   I8u;      /*  8 bit integer */
typedef int16_t   I16s;     /* 16 bit integer */
typedef uint16_t  I16u;     /* 16 bit integer */
typedef int32_t   I32s;     /* 32 bit integer */
typedef uint32_t  I32u;     /* 32 bit integer */
typedef int64_t   I64s;     /* 64 bit integer */
typedef uint64_t  I64u;     /* 64 bit integer */

/* string type */
typedef const Char* Str;

/* const pointer */
typedef const void * ConstPtr;

/* boolean type */
typedef enum __Bool {
    False = 0,
    True  = 1,
    MayBe = 2   /* Hey, why not? */
} Bool;

/* converts any non-zero value into True, zero into False */
#define BoolValue(x) ((x) ? True : False)

/* 
 * elapsed time in milliseconds or date in milliseconds since 
 * January 1st 1970, 00:00 GMT (which is compatible with time 
 * representation used by Java)
 */
typedef I64s Time;

/*
 * TIME_C macro should be used to initialize a Time variable with a 
 * fixed value.
 */
#define TIME_C(_t) __INT64_C(_t)

#ifdef NOTHING
#undef NOTHING
#endif
#define NOTHING ((void)0)

#define TRUE_STRING TEXT("true")
#define FALSE_STRING TEXT("false")

/* is this platform dependent ? */
#define __W(x) L##x
#define _W(x) __W(x)

/* shorter version of TEXT macro */
#define T_(quote)    TEXT(quote)

/* useful macros */
#ifndef MIN
#  define MIN(x,y) (((x)<(y))?(x):(y))
#endif /* MIN */
#ifndef MAX
#  define MAX(x,y) (((x)>(y))?(x):(y))
#endif /* MAX */
#ifndef ABS
#  define ABS(x)   (((x)>=0)?(x):(-(x)))
#endif /* ABS */
#ifndef SIGN
#  define SIGN(x)   (((x)>0)?1:(((x)<0)?(-1):0))
#endif /* SIGN */

#define COUNT(array) ((int)(sizeof(array)/sizeof(array[0])))
#define OFFSET(type,field) ((int)(((char*)&(((type *)0)->field))-((char*)0)))
#define FIELDSIZE(type,field) ((int)sizeof(((type *)0)->field))
#define NEXTOFFSET(type,field)  (OFFSET(type,field)+FIELDSIZE(type,field))
#define UNREF(param) ((void)(param))

#define WORD_ALIGN(_x)  ((((PtrWord)(_x)) + 1) & ((PtrWord)(-2)))
#define DWORD_ALIGN(_x) ((((PtrWord)(_x)) + 3) & ((PtrWord)(-4)))
#define QUAD_ALIGN(_x)  ((((PtrWord)(_x)) + 7) & ((PtrWord)(-8)))

#define _SIGNATURE32(c1,c2,c3,c4) SIGN32(c1,c2,c3,c4)
#define SIGN32(c1,c2,c3,c4) \
  (((((I32u)(c1)) & 0xff) <<  0) | \
   ((((I32u)(c2)) & 0xff) <<  8) | \
   ((((I32u)(c3)) & 0xff) << 16) | \
   ((((I32u)(c4)) & 0xff) << 24))

/* 
 * Calculate the address of the base of the structure given its type, and an
 * address of a field within the structure. 
 */
#define CAST(address,type,field) ((type *)( \
                                  (char*)(address) - \
                                  (char*)(&((type *)0)->field)))

/* debug stuff */
#ifdef ASSERT
#undef ASSERT
#endif

/* Backward compatibility macros */
#define TRACE_Stdout PRINT_Stdout
#define TRACE_Stderr PRINT_Stderr

/* PRINTF_ATTR assigns printf-like or scanf-like characteristics to the
 * declared function, and enables gcc-compatible compiler to check the
 * format string against the parameters. */
#ifdef __GNUC__
#  define PRINTF_ATTR(m,n) __attribute__((format(printf,m,n)))
#else
#  define PRINTF_ATTR(m,n) /* NOTHING */
#endif

/* Console output */
extern int PRINT_Stdout P_((const char* format, va_list va));
extern int PRINT_Stderr P_((const char* format, va_list va));
#ifdef _UNICODE
extern int PRINT_StdoutU P_((Str format, va_list va));
extern int PRINT_StderrU P_((Str format, va_list va));
#endif /* _UNICODE */
extern void PRINT_UseConsole P_((Bool enable));

#ifndef DEBUG_TRACE
#  define DEBUG_TRACE DEBUG
#endif /* DEBUG_TRACE */

#if DEBUG_TRACE

extern int  DEBUG_Trace P_((const char* format, ...) PRINTF_ATTR(1,2));
extern int  DEBUG_TraceVa P_((const char* format, va_list va));
#  ifdef _UNICODE
extern int  DEBUG_TraceU P_((Str format, ...) PRINTF_ATTR(1,2));
extern int  DEBUG_TraceVaU P_((Str format, va_list va));
#  endif /* _UNICODE */
extern void DEBUG_Assert P_((const char* msg, const char * file, long l));
extern void DEBUG_AssertFormat P_((const char* file, long l, Str fmt, ...)
                                  PRINTF_ATTR(3,4));

typedef void (*DebugAssertProc) P_((const char* msg, const char* f, long l));
extern void DEBUG_AssertHandler P_((const char* msg, const char* f, long l));
extern DebugAssertProc slibDebugAssertHandler;

#  define ASSERT(x)                       ((x) ? NOTHING : ASSMSG(#x))
#  define VERIFY(x)                       ASSERT(x)

#  define TRACE(s)                        DEBUG_Trace(TEXT("%s"),TEXT(s))
#  define TRACE0(s)                       DEBUG_Trace(TEXT("%s"),TEXT(s))
#  define TRACE1(s,p1)                    DEBUG_Trace(TEXT(s),p1)
#  define TRACE2(s,p1,p2)                 DEBUG_Trace(TEXT(s),p1,p2)
#  define TRACE3(s,p1,p2,p3)              DEBUG_Trace(TEXT(s),p1,p2,p3)
#  define TRACE4(s,p1,p2,p3,p4)           DEBUG_Trace(TEXT(s),p1,p2,p3,p4)
#  define TRACE5(s,p1,p2,p3,p4,p5)        DEBUG_Trace(TEXT(s),p1,p2,p3,p4,p5)
#  define TRACE6(s,p1,p2,p3,p4,p5,p6)     DEBUG_Trace(TEXT(s),p1,p2,p3,p4,p5,p6)
#  define TRACE7(s,p1,p2,p3,p4,p5,p6,p7)  DEBUG_Trace(TEXT(s),p1,p2,p3,p4,p5,p6,p7)

#  define DTRACE(s)                       DEBUG_Trace(TEXT("%s"),s)
#  define DTRACE0(s)                      DEBUG_Trace(TEXT("%s"),s)
#  define DTRACE1(s,p1)                   DEBUG_Trace(s,p1)
#  define DTRACE2(s,p1,p2)                DEBUG_Trace(s,p1,p2)
#  define DTRACE3(s,p1,p2,p3)             DEBUG_Trace(s,p1,p2,p3)
#  define DTRACE4(s,p1,p2,p3,p4)          DEBUG_Trace(s,p1,p2,p3,p4)
#  define DTRACE5(s,p1,p2,p3,p4,p5)       DEBUG_Trace(s,p1,p2,p3,p4,p5)
#  define DTRACE6(s,p1,p2,p3,p4,p5,p6)    DEBUG_Trace(s,p1,p2,p3,p4,p5,p6)
#  define DTRACE7(s,p1,p2,p3,p4,p5,p6,p7) DEBUG_Trace(s,p1,p2,p3,p4,p5,p6,p7)

#  define ASSMSG(s)                       DEBUG_Assert(s,__FILE__,__LINE__)
#  define ASSMSG1(s,p1)                   DEBUG_AssertFormat(__FILE__,__LINE__,TEXT(s),p1)
#  define ASSMSG2(s,p1,p2)                DEBUG_AssertFormat(__FILE__,__LINE__,TEXT(s),p1,p2)
#  define ASSMSG3(s,p1,p2,p3)             DEBUG_AssertFormat(__FILE__,__LINE__,TEXT(s),p1,p2,p3)
#  define ASSMSG4(s,p1,p2,p3,p4)          DEBUG_AssertFormat(__FILE__,__LINE__,TEXT(s),p1,p2,p3,p4)
#  define ASSMSG5(s,p1,p2,p3,p4,p5)       DEBUG_AssertFormat(__FILE__,__LINE__,TEXT(s),p1,p2,p3,p4,p5)
#  define ASSMSG6(s,p1,p2,p3,p4,p5,p6)    DEBUG_AssertFormat(__FILE__,__LINE__,TEXT(s),p1,p2,p3,p4,p5,p6)
#  define ASSMSG7(s,p1,p2,p3,p4,p5,p6,p7) DEBUG_AssertFormat(__FILE__,__LINE__,TEXT(s),p1,p2,p3,p4,p5,p6,p7)

#  define DEBUG_TRACE_ONLY(x)             x
#  define DEBUG_TRACE_PARAM(x)            x,
#  define DEBUG_TRACE_PARAM2(x)           ,x
#  define DEBUG_TRACE_UNREF(param)        ((void)(param))

#else /* DEBUG_TRACE */

#  define ASSERT(x)                       NOTHING
#  define VERIFY(x)                       (x)

#  define TRACE(s)                        NOTHING
#  define TRACE0(s)                       NOTHING
#  define TRACE1(s,p1)                    NOTHING
#  define TRACE2(s,p1,p2)                 NOTHING
#  define TRACE3(s,p1,p2,p3)              NOTHING
#  define TRACE4(s,p1,p2,p3,p4)           NOTHING
#  define TRACE5(s,p1,p2,p3,p4,p5)        NOTHING
#  define TRACE6(s,p1,p2,p3,p4,p5,p6)     NOTHING
#  define TRACE7(s,p1,p2,p3,p4,p5,p6,p7)  NOTHING

#  define DTRACE(s)                       NOTHING
#  define DTRACE0(s)                      NOTHING
#  define DTRACE1(s,p1)                   NOTHING
#  define DTRACE2(s,p1,p2)                NOTHING
#  define DTRACE3(s,p1,p2,p3)             NOTHING
#  define DTRACE4(s,p1,p2,p3,p4)          NOTHING
#  define DTRACE5(s,p1,p2,p3,p4,p5)       NOTHING
#  define DTRACE6(s,p1,p2,p3,p4,p5,p6)    NOTHING
#  define DTRACE7(s,p1,p2,p3,p4,p5,p6,p7) NOTHING

#  define ASSMSG(s)                       NOTHING
#  define ASSMSG1(s,p1)                   NOTHING
#  define ASSMSG2(s,p1,p2)                NOTHING
#  define ASSMSG3(s,p1,p2,p3)             NOTHING
#  define ASSMSG4(s,p1,p2,p3,p4)          NOTHING
#  define ASSMSG5(s,p1,p2,p3,p4,p5)       NOTHING
#  define ASSMSG6(s,p1,p2,p3,p4,p5,p6)    NOTHING
#  define ASSMSG7(s,p1,p2,p3,p4,p5,p6,p7) NOTHING

#  define DEBUG_TRACE_ONLY(x)
#  define DEBUG_TRACE_PARAM(x)
#  define DEBUG_TRACE_PARAM2(x)
#  define DEBUG_TRACE_UNREF(param)        NOTHING

#endif /* !DEBUG_TRACE */

#if DEBUG

#  define STATIC

#  define DEBUG_ONLY(x)                   x
#  define DEBUG_PARAM(x)                  x,
#  define DEBUG_PARAM2(x)                 ,x
#  define DEBUG_UNREF(param)              ((void)(param))

#  define RELEASE_ONLY(x)
#  define RELEASE_PARAM(x)
#  define RELEASE_PARAM2(x)
#  define RELEASE_UNREF(param)            NOTHING

#else /* DEBUG */

#  define STATIC          static

#  define DEBUG_ONLY(x)
#  define DEBUG_PARAM(x)
#  define DEBUG_PARAM2(x)
#  define DEBUG_UNREF(param)              NOTHING

#  define RELEASE_ONLY(x)                 x
#  define RELEASE_PARAM(x)                x,
#  define RELEASE_PARAM2(x)               ,x
#  define RELEASE_UNREF(param)            ((void)(param))

#endif /* !DEBUG */


/* some platform specific ASSERTs */

#ifdef __linux__
#  define LINUX_ASSERT(expr) ASSERT(expr) 
#else
#  define LINUX_ASSERT(expr) NOTHING
#endif /* __linux__ */

#ifdef _UNIX
#  define UNIX_ASSERT(expr) ASSERT(expr) 
#else
#  define UNIX_ASSERT(expr) NOTHING
#endif /* _UNIX */

#ifdef _WIN32
#  define WIN32_ASSERT(expr) ASSERT(expr)
#else
#  define WIN32_ASSERT(expr) NOTHING
#endif /* _WIN32 */

/* 
 * Unicode mapping for prototypes that use Str or Char types. If the UNICODE
 * settings of the application that uses the library don't match the UNICODE
 * settings used by the library when it was compiled, the application won't
 * link. That's better than debug the problem at run time.
 */
#ifdef _UNICODE

#define DEBUG_Trace                 DEBUG_TraceU
#define DEBUG_TraceVa               DEBUG_TraceVaU

/* s_base32.h */
#  define BASE32_Encode             BASE32_EncodeU
#  define BASE32_EncodeStr          BASE32_EncodeStrU
#  define BASE32_Decode             BASE32_DecodeU
#  define BASE32_StrictDecode       BASE32_StrictDecodeU

/* s_base64.h */
#  define BASE64_Encode             BASE64_EncodeU
#  define BASE64_EncodeStr          BASE64_EncodeStrU
#  define BASE64_Decode             BASE64_DecodeU
#  define BASE64_StdDecode          BASE64_StdDecodeU
#  define BASE64_SafeDecode         BASE64_SafeDecodeU

/* s_dom.h */
#  define DOM_FindFirst             DOM_FindFirstU
#  define DOM_FindLast              DOM_FindLastU
#  define DOM_FindNext              DOM_FindNextU
#  define DOM_FindPrev              DOM_FindPrevU
#  define DOM_CharData              DOM_CharDataU
#  define DOM_TagName               DOM_TagNameU
#  define DOM_RootCB                DOM_RootCBU
#  define DOM_TagsCB                DOM_TagsCBU
#  define DOM_Load                  DOM_LoadU
#  define DOM_LoadTags              DOM_LoadTagsU

/* s_file.h */
#  define FILE_Open                 FILE_OpenU
#  define FILE_Reopen               FILE_ReopenU
#  define FILE_SetParam             FILE_SetParamU
#  define FILE_SetName              FILE_SetNameU
#  define FILE_Printf               FILE_PrintfU
#  define FILE_VaPrintf             FILE_VaPrintfU
#  define FILE_Puts                 FILE_PutsU
#  define FILE_Gets                 FILE_GetsU
#  define FILE_Putc                 FILE_PutcU
#  define FILE_Getc                 FILE_GetcU
#  define FILE_Ungetc               FILE_UngetcU
#  define FILE_AttachToFile         FILE_AttachToFileU
#  define FILE_OpenURL              FILE_OpenURLU
#  define FILE_AuthURL              FILE_AuthURLU
#  define FILE_Split                FILE_SplitU
#  define FILE_Split2               FILE_Split2U
#  define FILE_IsFileIO             FILE_IsFileIOU
#  define FILE_ReadLine             FILE_ReadLineU

/* s_futil.h */
#  define FILE_DirName              FILE_DirNameU
#  define FILE_TempName             FILE_TempNameU
#  define FILE_IsFileSeparator      FILE_IsFileSeparatorU
#  define FILE_Save                 FILE_SaveU
#  define FILE_FilePart             FILE_FilePartU
#  define FILE_CanOpen              FILE_CanOpenU
#  define FILE_Exist                FILE_ExistU
#  define FILE_NonExist             FILE_NonExistU
#  define FILE_IsFile               FILE_IsFileU
#  define FILE_IsDir                FILE_IsDirU
#  define FILE_MakeUnique           FILE_MakeUniqueU
#  define FILE_MkDir                FILE_MkDirU
#  define FILE_RmDir                FILE_RmDirU
#  define FILE_List                 FILE_ListU

/* s_hash.h */
#  define hashCompareStringKey      hashCompareStringKeyU
#  define hashCaseCompareStringKey  hashCaseCompareStringKeyU
#  define stringHashProc            stringHashProcU
#  define stringCaseHashProc        stringCaseHashProcU

/* s_hist */
#  define HIST1D_Create             HIST1D_CreateU
#  define HIST1D_Init               HIST1D_InitU

/* s_math.h */
#  define BIGINT_Format             BIGINT_FormatU
#  define BIGINT_Parse              BIGINT_ParseU

/* s_md.h */
#  define DIGEST_Name               DIGEST_NameU

/* s_mem.h */
#  define MEM_DebugStat             MEM_DebugStatU
#  define MEM_DebugDump             MEM_DebugDumpU

/* s_opt.h */
#  define CMDLINE_Create            CMDLINE_CreateU
#  define CMDLINE_SetApp            CMDLINE_SetAppU
#  define CMDLINE_SetOutProc        CMDLINE_SetOutProcU
#  define CMDLINE_SetParamName      CMDLINE_SetParamNameU
#  define CMDLINE_GetApp            CMDLINE_GetAppU
#  define CMDLINE_Msg               CMDLINE_MsgU
#  define CMDLINE_AddIntOpt         CMDLINE_AddIntOptU
#  define CMDLINE_AddI64Opt         CMDLINE_AddI64OptU
#  define CMDLINE_AddDoubleOpt      CMDLINE_AddDoubleOptU
#  define CMDLINE_AddTrueOpt        CMDLINE_AddTrueOptU
#  define CMDLINE_AddFalseOpt       CMDLINE_AddFalseOptU
#  define CMDLINE_AddStrOpt         CMDLINE_AddStrOptU
#  define CMDLINE_AddOpt            CMDLINE_AddOptU
#  define CMDLINE_CreateOptGrp      CMDLINE_CreateOptGrpU
#  define CMDLINE_Parse             CMDLINE_ParseU
#  define CMDLINE_Parse1            CMDLINE_Parse1U
#  define CMDLINE_Usage             CMDLINE_UsageU

/* s_prop.h */
#  define PROP_Load                 PROP_LoadU
#  define PROP_Save                 PROP_SaveU
#  define PROP_LoadXML              PROP_LoadXMLU
#  define PROP_SaveXML              PROP_SaveXMLU
#  define PROP_WriteXML             PROP_WriteXMLU
#  define PROP_RootCB               PROP_RootCBU
#  define PROP_Get                  PROP_GetU
#  define PROP_GetInt               PROP_GetIntU
#  define PROP_GetUInt              PROP_GetUIntU
#  define PROP_GetLong              PROP_GetLongU
#  define PROP_GetULong             PROP_GetULongU
#  define PROP_GetBool              PROP_GetBoolU
#  define PROP_GetIntBool           PROP_GetIntBoolU
#  define PROP_GetDouble            PROP_GetDoubleU
#  define PROP_GetComment           PROP_GetCommentU
#  define PROP_Set                  PROP_SetU
#  define PROP_SetInt               PROP_SetIntU
#  define PROP_SetUInt              PROP_SetUIntU
#  define PROP_SetLong              PROP_SetLongU
#  define PROP_SetULong             PROP_SetULongU
#  define PROP_SetBool              PROP_SetBoolU
#  define PROP_SetDouble            PROP_SetDoubleU
#  define PROP_SetComment           PROP_SetCommentU
#  define PROP_Comment              PROP_CommentU
#  define PROP_GetLong64            PROP_GetLong64U
#  define PROP_SetLong64            PROP_SetLong64U
#  define PROP_Remove               PROP_RemoveU
#  define PROP_Examine              PROP_ExamineU
#  define PROP_ExamineAll           PROP_ExamineAllU
#  define PROP_Clear                PROP_ClearU
#  define PROP_Copy                 PROP_CopyU
#  define PROP_CopyAll              PROP_CopyAllU
#  define PROP_Merge                PROP_MergeU
#  define PROP_Extract              PROP_ExtractU
#  define PROP_Keys                 PROP_KeysU
#  define PROP_Values               PROP_ValuesU
#  define PROP_Data                 PROP_DataU

/* s_strbuf.h */
#  define STRBUF_Create             STRBUF_CreateU
#  define STRBUF_Delete             STRBUF_DeleteU
#  define STRBUF_Init               STRBUF_InitU
#  define STRBUF_InitBuf            STRBUF_InitBufU
#  define STRBUF_InitBuf2           STRBUF_InitBuf2U
#  define STRBUF_Destroy            STRBUF_DestroyU
#  define STRBUF_Length             STRBUF_LengthU
#  define STRBUF_SetLength          STRBUF_SetLengthU
#  define STRBUF_Equals             STRBUF_EqualsU
#  define STRBUF_EqualsNoCase       STRBUF_EqualsNoCaseU
#  define STRBUF_EqualsTo           STRBUF_EqualsToU
#  define STRBUF_EqualsToNoCase     STRBUF_EqualsToNoCaseU
#  define STRBUF_CharAt             STRBUF_CharAtU
#  define STRBUF_FirstChar          STRBUF_FirstCharU
#  define STRBUF_LastChar           STRBUF_LastCharU
#  define STRBUF_IndexOf            STRBUF_IndexOfU
#  define STRBUF_LastIndexOf        STRBUF_LastIndexOfU
#  define STRBUF_Find               STRBUF_FindU
#  define STRBUF_FindNoCase         STRBUF_FindNoCaseU
#  define STRBUF_FindLast           STRBUF_FindLastU
#  define STRBUF_FindLastNoCase     STRBUF_FindLastNoCaseU
#  define STRBUF_FindFrom           STRBUF_FindFromU
#  define STRBUF_FindFromNoCase     STRBUF_FindFromNoCaseU
#  define STRBUF_Replace            STRBUF_ReplaceU
#  define STRBUF_ReplaceStr         STRBUF_ReplaceStrU
#  define STRBUF_GetString          STRBUF_GetStringU
#  define STRBUF_StartsWith         STRBUF_StartsWithU
#  define STRBUF_EndsWith           STRBUF_EndsWithU
#  define STRBUF_EndsWithNoCase     STRBUF_EndsWithNoCaseU
#  define STRBUF_StartsWithNoCase   STRBUF_StartsWithNoCaseU
#  define STRBUF_ToUpperCase        STRBUF_ToUpperCaseU
#  define STRBUF_ToLowerCase        STRBUF_ToLowerCaseU
#  define STRBUF_Alloc              STRBUF_AllocU
#  define STRBUF_Clear              STRBUF_ClearU
#  define STRBUF_Erase              STRBUF_EraseU
#  define STRBUF_Trim               STRBUF_TrimU
#  define STRBUF_Copy               STRBUF_CopyU
#  define STRBUF_CopyN              STRBUF_CopyNU
#  define STRBUF_Append             STRBUF_AppendU
#  define STRBUF_AppendN            STRBUF_AppendNU
#  define STRBUF_AppendInt          STRBUF_AppendIntU
#  define STRBUF_AppendChar         STRBUF_AppendCharU
#  define STRBUF_AppendBool         STRBUF_AppendBoolU
#  define STRBUF_AppendDouble       STRBUF_AppendDoubleU
#  define STRBUF_AppendFormat       STRBUF_AppendFormatU
#  define STRBUF_AppendFormatVa     STRBUF_AppendFormatVaU
#  define STRBUF_Inflate            STRBUF_InflateU
#  define STRBUF_Insert             STRBUF_InsertU
#  define STRBUF_InsertN            STRBUF_InsertNU
#  define STRBUF_InsertChar         STRBUF_InsertCharU
#  define STRBUF_Format             STRBUF_FormatU
#  define STRBUF_FormatVa           STRBUF_FormatVaU
#  define STRBUF_AppendTime         STRBUF_AppendTimeU
#  define STRBUF_FormatTime         STRBUF_FormatTimeU

/* s_util.h */
#  define PRINT_Error               PRINT_ErrorU
#  define PRINT_ErrorVa             PRINT_ErrorVaU
#  define PRINT_Warning             PRINT_WarningU
#  define PRINT_WarningVa           PRINT_WarningVaU
#  define PRINT_Output              PRINT_OutputU
#  define PRINT_OutputVa            PRINT_OutputVaU
#  define PRINT_Verbose             PRINT_VerboseU
#  define PRINT_VerboseVa           PRINT_VerboseVaU
#  define PRINT_Stdout              PRINT_StdoutU
#  define PRINT_Stderr              PRINT_StderrU
#  define yyerror                   yyerrorU
#  define TIME_ToString             TIME_ToStringU
#  define URL_Encode                URL_EncodeU
#  define URL_Decode                URL_DecodeU
#  define STRING_Format             STRING_FormatU
#  define STRING_IndexOf            STRING_IndexOfU
#  define STRING_LastIndexOf        STRING_LastIndexOfU
#  define STRING_HashCode           STRING_HashCodeU
#  define STRING_HashCodeNoCase     STRING_HashCodeNoCaseU
#  define STRING_FormatDouble       STRING_FormatDoubleU
#  define PARSE_Bool                PARSE_BoolU
#  define PARSE_Byte                PARSE_ByteU
#  define PARSE_UByte               PARSE_UByteU
#  define PARSE_Short               PARSE_ShortU
#  define PARSE_UShort              PARSE_UShortU
#  define PARSE_Int                 PARSE_IntU
#  define PARSE_UInt                PARSE_UIntU
#  define PARSE_Long                PARSE_LongU
#  define PARSE_ULong               PARSE_ULongU
#  define PARSE_Double              PARSE_DoubleU
#  define PARSE_Long64              PARSE_Long64U
#  define PARSE_ULong64             PARSE_ULong64U

/* s_vector.h */
#  define vectorCompareString       vectorCompareStringU
#  define vectorCompareStringNoCase vectorCompareStringNoCaseU
#  define vectorEqualsString        vectorEqualsStringU
#  define vectorEqualsStringNoCase  vectorEqualsStringNoCaseU

/* s_xml.h */
#  define xmlEmptyAttr              xmlEmptyAttrU
#  define XML_AttrCount             XML_AttrCountU
#  define XML_AttrValue             XML_AttrValueU
#  define XML_AttrValueAt           XML_AttrValueAtU
#  define XML_AttrNameAt            XML_AttrNameAtU
#  define XML_ParseStream           XML_ParseStreamU
#  define XML_ParseFile             XML_ParseFileU
#  define XML_Handle                XML_HandleU
#  define XML_Escape                XML_EscapeU
#  define XML_OpenTag               XML_OpenTagU
#  define XML_CloseTag              XML_CloseTagU
#  define XML_StartTag              XML_StartTagU
#  define XML_EndTag                XML_EndTagU
#  define XML_WriteAttr             XML_WriteAttrU
#  define XML_WriteDoubleAttr       XML_WriteDoubleAttrU
#  define XML_WriteIntAttr          XML_WriteIntAttrU
#  define XML_WriteI64Attr          XML_WriteI64AttrU
#  define XML_WriteU64Attr          XML_WriteU64AttrU
#  define XML_WriteUIntAttr         XML_WriteUIntAttrU
#  define XML_WriteHexAttr          XML_WriteHexAttrU
#  define XML_MatchAttr             XML_MatchAttrU

#endif /* _UNICODE */

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_DEF_H_ */

/*
 * HISTORY:
 *
 * $Log: s_def.h,v $
 * Revision 1.61  2010/09/25 09:55:06  slava
 * o added PRINTF_ATTR macro which assigns printf-like characteristics to the
 *   declared function and enables gcc to check the format string against the
 *   parameters.
 *
 * Revision 1.60  2010/07/04 16:14:30  slava
 * o moved T_ macro from s_win32.h to s_def.h
 *
 * Revision 1.59  2010/07/04 13:44:25  slava
 * o UNICODE build issues
 *
 * Revision 1.58  2010/04/25 08:37:26  slava
 * o added RELEASE_ONLY, RELEASE_PARAM and RELEASE_PARAM2 macros
 *
 * Revision 1.57  2010/03/10 21:16:25  slava
 * o added STRBUF_EqualsNoCase and STRBUF_EqualsToNoCase functions
 *
 * Revision 1.56  2010/01/22 23:21:47  slava
 * o fixed double definition of DEBUG_UNREF macro
 *
 * Revision 1.55  2009/05/26 05:56:33  slava
 * o added DEBUG_TRACE_ONLY, DEBUG_TRACE_PARAM, DEBUG_TRACE_PARAM2 and
 *   DEBUG_TRACE_UNREF macros.
 *
 * Revision 1.54  2009/03/25 22:08:04  slava
 * o made it easier to compile release version of slib with debug trace
 *
 * Revision 1.53  2009/02/09 23:02:16  slava
 * o added FILE_AuthURL function
 *
 * Revision 1.52  2008/11/20 11:52:13  slava
 * o allow use of non-UNICODE console output functions in UNICODE build
 *
 * Revision 1.51  2008/11/20 08:26:24  slava
 * o switched NT kernel build to UNICODE
 *
 * Revision 1.50  2008/11/05 12:15:08  slava
 * o made assert handler configurable, cleaned up the assertion handling
 *   code a bit, moved system specific code to system-specific directories
 *
 * Revision 1.49  2007/06/05 18:02:01  slava
 * o added DEBUG_PARAM2 macro
 *
 * Revision 1.48  2007/03/05 02:01:56  slava
 * o added FILE_SetName() function. This function only affects the output
 *   of FILE_Name() and FILE_GetName(). It has no effect on the actual I/O.
 *
 * Revision 1.47  2006/04/19 04:57:29  slava
 * o added SIGN macro
 *
 * Revision 1.46  2006/03/27 21:52:32  slava
 * o STRING_DupU is now a function present in both Unicode and single-byte
 *   builds, and STRING_Dup is a macro pointing to either STRING_Dup8 or
 *   STRING_DupU depending on whether or not UNICODE macro is defined.
 *   This change is source code compatible with the older builds (but
 *   not binary compatible)
 *
 * Revision 1.45  2006/03/12 08:01:04  slava
 * o moved PRINT_Stdout and PRINT_Stderr declarations from s_def.h to s_util.h
 *
 * Revision 1.44  2005/10/19 22:34:21  slava
 * o added aliases for STRBUF_FindNoCase, STRBUF_FindLastNoCase and
 *   STRBUF_FindFromNoCase
 *
 * Revision 1.43  2005/10/19 21:21:19  slava
 * o fixed Unicode compilation issue
 *
 * Revision 1.42  2005/10/11 13:03:30  slava
 * o added missing Unicode aliases for recently added functions
 *
 * Revision 1.41  2005/05/31 07:02:56  slava
 * o fixed a problem with TRACE_Stdout and TRACE_Stderr macros not being
 *   defined in release build
 *
 * Revision 1.40  2005/02/20 20:31:29  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.39  2005/02/16 06:01:52  slava
 * o renamed TRACE_xxx functions and macros to PRINT_xxx. Macros are provided
 *   for backward compatibility
 *
 * Revision 1.38  2005/01/02 00:07:07  slava
 * o updated a comment
 *
 * Revision 1.37  2004/12/27 07:24:15  slava
 * o added Unicode mapping for prototypes that use Str or Char types. If the
 *   UNICODE settings of the application that uses the library don't match the
 *   UNICODE settings used by the library when it was compiled, the application
 *   won't link. That's better than debug the problem at run time.
 *
 * Revision 1.36  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.35  2004/08/18 03:50:40  slava
 * o include "s_ver.h" from "s_def.h" to make it possible to maintain backward
 *   source compatibility with older versions of slib which didn't have s_ver.h
 *
 * Revision 1.34  2004/04/08 13:49:07  slava
 * o removed the Class macro. It breaks NT kernel mode build
 *
 * Revision 1.33  2004/04/08 00:45:05  slava
 * o added BoolValue macro
 *
 * Revision 1.32  2004/03/15 20:12:13  slava
 * o added ConstPtr type
 *
 * Revision 1.31  2003/05/21 00:11:03  slava
 * o resolved the issue with the Bool type being defined in two places;
 *   in s_def.h and in s_os.h; which prevented slib headers from being
 *   compiled by the GNU compiler in C++ mode. The downside is that now
 *   s_mem.h has to be included from every file referencing slib memory
 *   allocation functions and macros, but that should not be a problem
 *   for the apps because the apps (at least mine) typically include
 *   the whole s_lib.h rather than the individual headers.
 *
 * Revision 1.30  2003/05/17 01:23:37  slava
 * o port to Mac OS X
 *
 * Revision 1.29  2003/03/18 16:52:44  slava
 * o added Unicode build configurations for Windoze
 *
 * Revision 1.28  2003/03/11 01:44:40  slava
 * o FreeBSD port
 *
 * Revision 1.27  2003/01/09 03:51:44  slava
 * o added TIME_C macro that should be used to initialize a Time variable
 *   with a fixed value.
 *
 * Revision 1.26  2002/12/30 21:38:57  slava
 * o SIGN32 macro replaces _SIGNATURE32
 *
 * Revision 1.25  2002/12/30 15:52:45  slava
 * o renamed DebugTrace -> DEBUG_Trace, DebugAssert -> DEBUG_Assert,
 *   DebugAssertFormat -> DEBUG_AssertFormat
 *
 * Revision 1.24  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.23  2001/11/24 19:37:31  slava
 * o added UNIX_ASSERT macro
 *
 * Revision 1.22  2001/10/22 00:10:36  slava
 * o recrafted memory management.
 *
 * Revision 1.21  2001/07/05 07:04:39  slava
 * o added WORD_ALIGN, DWORD_ALIGN and QUAD_ALIGN macros
 *
 * Revision 1.20  2001/06/30 16:33:04  slava
 * o added MEM_InitModule() and MEM_Shutdown() which in release build do
 *   nothing. In debug build, MEM_Shutdown() makes sure that memory mutex
 *   gets destroyed. While under Windows NT/98 or on Unix it's not a problem
 *   because all handles get closed by the system when the process dies, in
 *   some scenarios (like, NT kernel mode or perhaps Windows CE) this may
 *   cause a resource leak.
 *
 * Revision 1.19  2001/06/08 04:23:46  slava
 * o exported TRACE_Stdout() and TRACE_Stderr() functions
 * o added _W macro
 *
 * Revision 1.18  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.17  2001/05/27 11:45:51  slava
 * o port to NT kernel mode environment
 *
 * Revision 1.16  2001/05/20 17:39:44  slava
 * o added DEBUG_PARAM macro
 *
 * Revision 1.15  2001/05/20 02:45:29  slava
 * o fixed TRACE and TRACE0 macros which didn't work on Windows CE
 *
 * Revision 1.14  2001/05/18 23:26:20  slava
 * o don't undefine TRUE and FALSE macros
 *
 * Revision 1.13  2001/05/18 22:29:54  slava
 * o slib has been (partially) ported to Windows CE
 *
 * Revision 1.12  2001/04/23 09:14:15  slava
 * o added ABS macro
 *
 * Revision 1.11  2001/02/27 06:49:15  slava
 * o changed DEBUG_ONLY macro so that it expands into ABSOLUTELY NOTHING in
 *   release build
 *
 * Revision 1.10  2001/01/31 02:00:41  slava
 * o portability fixes. Now compiles on Solaris
 *
 * Revision 1.9  2001/01/03 06:57:58  slava
 * o moved CAST macro from s_queue.h to s_def.h
 *
 * Revision 1.8  2000/12/29 06:56:18  slava
 * o changed TRACE() and TRACE0() macros to allow use of percent
 *   character in the trace string without escaping
 *
 * Revision 1.7  2000/12/23 17:01:39  slava
 * o added ASSMSG1...ASSMSG7 assertions macros that allow formatting. For
 *   example, you can write something like ASSMSG1("Invalid index %d",i)
 *   instead of just ASSERT(FALSE)
 *
 * Revision 1.6  2000/12/17 16:09:47  slava
 * o added extern "C" {} so that slib could be used from a C++ program
 *
 * Revision 1.5  2000/11/05 07:07:49  slava
 * o reorganization of output functions (Verbose(), Output(), etc.)
 *
 * Revision 1.4  2000/11/01 13:25:05  slava
 * o replaced TRUE/FALSE with True/False
 *
 * Revision 1.3  2000/11/01 10:25:14  slava
 * o replaced New, NewArray and ReallocArray macros with MEM_New,
 *   MEM_NewArray and MEM_ReallocArray, respectively
 *
 * Revision 1.2  2000/11/01 03:57:53  slava
 * o made Bool a enum
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
