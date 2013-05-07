/*
 * $Id: s_opt.c,v 1.41 2010/12/22 17:57:04 slava Exp $
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

#include "s_opt.h"
#include "s_mem.h"
#include "s_util.h"
#include "s_futil.h"
#include "s_queue.h"

/*==========================================================================*
 *              C O M M A N D    L I N E
 *==========================================================================*/

/* A command keyword */
typedef struct _CmdKeyword {
    QEntry entry;               /* entry in CmdLine->keywords */
    Char * name;                /* the command name */
    Vector v;                   /* array of variants */
} CmdKeyword;

/* Group of command line options */
struct _CmdOptGrp {
    QEntry entry;               /* entry in CmdLine->groups */
    Queue opts;                 /* command line options */
    Char * name;                /* the name of the option group */
};

/* Command line parser */
struct _CmdLine {
    Str app;                    /* app name, may be NULL */
    StrBuf16 appBuf;            /* buffer for app name */
    StrBuf64 buf;               /* temporary buffer */
    CmdOutProc print;           /* output function */
    void * printCtx;            /* output function context */
    Queue opts;                 /* contains command line options */
    Queue groups;               /* contains option groups */
    Queue keywords;             /* contains command keywords */
    int minArgs;                /* minimum number of additional arguments */
    int maxArgs;                /* maximum number of additional arguments */
};

/* Supported types of command line options */
typedef enum _CmdOptType {
    OptInt = 1,                 /* option has integer parameter */
    OptUInt,                    /* option has unsigned integer parameter */
    OptI64,                     /* option has 64-bit integer parameter */
    OptFloat,                   /* option has float parameter */
    OptDouble,                  /* option has double parameter */
    OptStr,                     /* option has string parameter */
    OptTrue,                    /* boolean option */
    OptFalse,                   /* boolean option */
    OptFlag,                    /* flag option */
    OptCustom                   /* custom callback option */
} CmdOptType;

/* Command line option */
struct _CmdOpt {
    QEntry entry;               /* entry in CmdLine->opts */
    QEntry groupQ;              /* entry in CmdOptGrp->opts */
    Vector exclude;             /* options mutually exclusive with this one */
    Vector require;             /* options required by this option */
    Char optChar;               /* short option name (one character) */
    Str optName;                /* long  option name */
    Str descr;                  /* description */
    Str param;                  /* parameter name */
    CmdOptType type;            /* type of command line option */
    int flags;                  /* flags, see below */

#define CMDOPT_REPEATABLE  0x01 /* this option is repeatable */
#define CMDOPT_REQUIRED    0x02 /* this option is repeatable */
#define CMDOPT_OWN_DESCR   0x04 /* description has been copied */

    int count;                  /* how many times this option has been seen */
    union _CmdOptValue {
        int * i;                /* OptInt */
        unsigned int * u;       /* OptUInt */
        I64s * i64;             /* OptI64 */
        float * f;              /* OptFloat */
        double * d;             /* OptDouble */
        Str * s;                /* OptStr */
        Bool * b;               /* OptTrue/OptFalse */
        struct _FlagOpt {       /* OptFlag */
            int * i;            /* pointer to the int variable */
            int bits;           /* bits to set */
        } flag;
        struct _CustomOpt {     /* OptCustom */
            CmdOptProc cb;      /* callback to invoke */
            void * ctx;         /* callback's context */
        } custom;
    } value;
};

/* Context for CMDLINE_FindOpt */
typedef struct _FindOpt {
    union _FindOptName {
        Char c;
        Str s;
    } name;                     /* name of the option we are looking for */
    CmdOpt * opt;               /* the option that we have found */
} FindOpt;

/* Default representation of arguments of different types in CMDLINE_Usage */
STATIC const Str CMDLINE_NumParam = TEXT("NUMBER");
STATIC const Str CMDLINE_StrParam = TEXT("STRING");

/**
 * CmdOutProc that sends output to standard output. 
 * Context parameter is ignored.
 */
STATIC void CMDLINE_StdOutProc(void * ctx, Str str) 
{
    UNREF(ctx);
    Output(TEXT("%s"), str);
}

/**
 * CmdOutProc that sends output to a File stream. 
 * Context parameter must point to a File
 */
STATIC void CMDLINE_FileOutProc(void * ctx, Str str) 
{ 
    FILE_Puts((File*)ctx, str);
}

/**
 * Formats and prints a message. DOES NOT prepend the output with the 
 * app name. Returns number of characters written to the output stream,
 * zero if output is disabled.
 */
STATIC size_t CMDLINE_Printf(CmdLine * c, Str format, ...)
{
    size_t nchars = 0;
    if (c->print) {
        va_list va;
        va_start(va, format);
        STRBUF_FormatVa(&c->buf.sb, format, va);
        nchars = STRBUF_Length(&c->buf.sb);
        c->print(c->printCtx, STRBUF_Text(&c->buf.sb));
        va_end(va);
    }
    return nchars;
}

/**
 * Prints a string. DOES NOT prepend the output with the app name. 
 * Returns number of characters written to the output stream,
 * zero if output is disabled.
 */
STATIC size_t CMDLINE_Puts(CmdLine * c, Str str)
{
    size_t nchars = 0;
    if (c->print) {
        nchars = StrLen(str);
        c->print(c->printCtx, str);
    }
    return nchars;
}

/**
 * Formats and prints a message. Prepends the output with the app name
 * if such is defined.
 */
void CMDLINE_Msg(CmdLine * c, Str format, ...)
{
    if (c->print) {
        va_list va;
        va_start(va, format);
        STRBUF_Clear(&c->buf.sb);
        if (c->app && c->app[0]) {
            STRBUF_Append(&c->buf.sb, c->app);
            STRBUF_Append(&c->buf.sb, TEXT(": "));
        }
        STRBUF_AppendFormatVa(&c->buf.sb, format, va);
        c->print(c->printCtx, STRBUF_Text(&c->buf.sb));
        va_end(va);
    }
}

/**
 * Deletes a single command line option
 */
STATIC void CMDOPT_Delete(CmdOpt * opt)
{
    if (opt) {
        QUEUE_RemoveEntry(&opt->entry);
        QUEUE_RemoveEntry(&opt->groupQ);
        VECTOR_Destroy(&opt->exclude);
        VECTOR_Destroy(&opt->require);
        if (opt->flags & CMDOPT_OWN_DESCR) {
            MEM_Free((void*)opt->descr);
        }
        MEM_Free(opt);
    }
}

/**
 * Creates command line object 
 */
CmdLine * CMDLINE_Create(Str app)
{
    CmdLine * c = MEM_New(CmdLine);
    if (c) {
        memset(c, 0, sizeof(*c));
        c->maxArgs = -1;   /* no limit */
        c->print = CMDLINE_StdOutProc;
        STRBUF_InitBufXXX(&c->appBuf);
        STRBUF_InitBufXXX(&c->buf);
        QUEUE_Init(&c->opts);
        QUEUE_Init(&c->groups);
        QUEUE_Init(&c->keywords);
        CMDLINE_SetApp(c, app);
    }
    return c;
}

/**
 * Deletes command line object 
 */
void CMDLINE_Delete(CmdLine * c)
{
    if (c) {
        QEntry * e;
        while ((e = QUEUE_RemoveHead(&c->opts)) != NULL) {
            CmdOpt * opt = QCAST(e,CmdOpt,entry);
            CMDOPT_Delete(opt);
        }
        while ((e = QUEUE_RemoveHead(&c->groups)) != NULL) {
            CmdOptGrp * g = QCAST(e,CmdOptGrp,entry);
            ASSERT(QUEUE_IsEmpty(&g->opts));
            MEM_Free(g->name);
            MEM_Free(g);
        }
        while ((e = QUEUE_RemoveHead(&c->keywords)) != NULL) {
            CmdKeyword * k = QCAST(e,CmdKeyword,entry);
            VECTOR_Destroy(&k->v);
            MEM_Free(k->name);
            MEM_Free(k);
        }
        STRBUF_Destroy(&c->appBuf.sb);
        STRBUF_Destroy(&c->buf.sb);
        MEM_Free(c);
    }
}

/**
 * Finds a keyword by name
 */
STATIC CmdKeyword * CMDLINE_FindKeyword(CmdLine * c, Str name)
{
    if (c && name) {
        QEntry * e;
        for (e = QUEUE_First(&c->keywords); e; e = QUEUE_Next(e)) {
            CmdKeyword * k = QCAST(e,CmdKeyword,entry);
            if (!StrCmp(k->name, name) || VECTOR_Contains(&k->v, name)) {
                return k;
            }
        }
    }
    return NULL;
}

/**
 * Defines new command
 */
Bool CMDLINE_AddCmd(CmdLine * c, Str cmd)
{
    if (c && cmd) {
        CmdKeyword * k = CMDLINE_FindKeyword(c, cmd);
        ASSERT(!k);
        if (!k) {
            k = MEM_New(CmdKeyword);
            if (k) {
                memset(k, 0, sizeof(*k));
                VECTOR_Init(&k->v,0,vectorEqualsString,vectorFreeValueProc);
                k->name = STRING_Dup(cmd);
                if (k->name) {
                    QUEUE_InsertTail(&c->keywords, &k->entry);
                    return True;
                }
                VECTOR_Destroy(&k->v);
                MEM_Free(k);
            }
        }
    }
    return False;
}

/**
 * Defines a command variant (synonym)
 */
Bool CMDLINE_AddCmdVar(CmdLine * c, Str cmd, Str variant)
{
    if (variant) {
        CmdKeyword * k = CMDLINE_FindKeyword(c, cmd);
        ASSERT(k);
        if (k) {
            Char* v = STRING_Dup(variant);
            return (v && VECTOR_TryAdd(&k->v, v));
        }
    }
    return False;
}

/**
 * Defines a new command and a number of variants (synonyms)
 */
Bool CMDLINE_AddCmdVars(CmdLine * c, Str cmd, const Str * variants, int n)
{
    if (CMDLINE_AddCmd(c, cmd)) {
        Bool ok = True;
        int i;
        for (i=0; i<n; i++) {
            if (!CMDLINE_AddCmdVar(c, cmd, variants[i])) {
                ok = False;
            }
        }
        return ok;
    }
    return False;
}

/**
 * Returns the first option group, NULL if no options is defined
 */
STATIC CmdOptGrp * OPTGRP_First(CmdLine * c)
{
    QEntry * e = QUEUE_First(&c->groups);
    if (e) {
        return QCAST(e,CmdOptGrp,entry);
    }
    return NULL;
}

/**
 * Returns the next option group, NULL if it's the last one
 */
STATIC CmdOptGrp * OPTGRP_Next(CmdOptGrp * g)
{
    if (g) {
        QEntry * e = QUEUE_Next(&g->entry);
        if (e) {
            return QCAST(e,CmdOptGrp,entry);
        }
    }
    return NULL;
}

/**
 * Returns the first command line option, NULL if no options is defined
 */
STATIC CmdOpt * CMDOPT_First(CmdLine * c)
{
    QEntry * e = QUEUE_First(&c->opts);
    if (e) {
        return QCAST(e,CmdOpt,entry);
    }
    return NULL;
}

/**
 * Returns the next command line option, NULL if it's the last one
 */
STATIC CmdOpt * CMDOPT_Next(CmdOpt * opt)
{
    if (opt) {
        QEntry * e = QUEUE_Next(&opt->entry);
        if (e) {
            return QCAST(e,CmdOpt,entry);
        }
    }
    return NULL;
}

/**
 * Returns True if this option requires a parameter, False if not
 */
STATIC Bool CMDOPT_HasParam(CmdOpt * opt)
{
    switch (opt->type) {
    case OptInt:
    case OptUInt:
    case OptI64:
    case OptStr:
    case OptFloat:
    case OptDouble:
        return True;
    case OptCustom:
        return BoolValue(opt->param != NULL);
    default:
        return False;
    }
}

/** 
 * Sets app name.
 */
void CMDLINE_SetApp(CmdLine * c, Str app)
{
    if (app) {
        STRBUF_Copy(&c->appBuf.sb, FILE_FilePart(app));
#ifdef _WIN32
        if (STRBUF_Length(&c->appBuf.sb) > 4) {
            static Str exe = TEXT(".EXE");
            size_t len = STRBUF_Length(&c->appBuf.sb);
            size_t i, ext = len - 4;
            for (i=0; i<4; i++) {
                if (exe[i] != ToUpper(STRBUF_CharAt(&c->appBuf.sb,ext+i))) {
                    break;
                }
            }
            if (i == 4) {
                STRBUF_SetLength(&c->appBuf.sb,len-4);
            }
        } else {
            STRBUF_Copy(&c->appBuf.sb, FILE_FilePart(app));
        }
#endif /* _WIN32 */
        if (STRBUF_Length(&c->appBuf.sb) > 0) {
            c->app = STRBUF_Text(&c->appBuf.sb);
        } else {
            c->app = NULL;
        }
    } else {
        c->app = NULL;
        STRBUF_Clear(&c->appBuf.sb);
    }
}

/**
 * Returns app file name without path and .exe suffix on Windows 
 */
Str CMDLINE_GetApp(CmdLine * c)
{
    return c->app;
}

/**
 * Directs output to a File stream. NULL File disables the output
 */
void CMDLINE_SetOutFile(CmdLine * c, File * out)
{
    c->printCtx = out;
    c->print = (out ? NULL : CMDLINE_FileOutProc);
}

/**
 * Directs output to the specified function. NULL function disables the output
 */
void CMDLINE_SetOutProc(CmdLine * c, CmdOutProc out, void * ctx)
{
    c->printCtx = ctx;
    c->print = out;
}

/**
 * Sets parameter name for the specified option, for printing purposes. 
 * The option must have a parameter (i.e. be either string or integer option).
 * The string is NOT copied to the internal storage, so it better be allocated
 * statically.
 */
Bool CMDLINE_SetParamName(CmdOpt * opt, Str param)
{
    if (opt) {
        ASSERT(opt->type == OptCustom || CMDOPT_HasParam(opt));
        opt->param = param;
        return True;
    } else {
        return False;
    }
}

/**
 * Sets minimum number of additional argumentes (not including options
 * and their parameters) expected in the command line. 
 */
void CMDLINE_SetMinArgs(CmdLine * c, int min)
{
    c->minArgs = min;
}

/**
 * Sets maximum number of additional argumentes (not including options
 * and their parameters) expected in the command line. Negative value
 * is interpreted as no limit.
 */
void CMDLINE_SetMaxArgs(CmdLine * c, int max)
{
    c->maxArgs = max;
}

/**
 * Callback for CMDLINE_FindShortOpt
 */
STATIC Bool CMDLINE_FindShortOptCB(QEntry * e, void * ctx)
{
    CmdOpt * opt = QCAST(e,CmdOpt,entry);
    FindOpt * find = (FindOpt*)ctx;
    if (opt->optChar && opt->optChar == find->name.c) {
        find->opt = opt;
        return False;
    }
    return True;
}

/**
 * Callback for CMDLINE_FindLongOpt
 */
STATIC Bool CMDLINE_FindLongOptCB(QEntry * e, void * ctx)
{
    CmdOpt * opt = QCAST(e,CmdOpt,entry);
    FindOpt * find = (FindOpt*)ctx;
    if (opt->optName && StrCmp(opt->optName,find->name.s) == 0) {
        find->opt = opt;
        return False;
    }
    return True;
}

/* 
 * Finds short command line option. 
 */
STATIC CmdOpt * CMDLINE_FindShortOpt(CmdLine * c, Char ch)
{
    FindOpt find;
    find.name.c = ch;
    find.opt = NULL;
    QUEUE_Examine(&c->opts,CMDLINE_FindShortOptCB,&find);
    return find.opt;
}

/* 
 * Finds long command line option. 
 */
STATIC CmdOpt * CMDLINE_FindLongOpt(CmdLine * c, Str name)
{
    FindOpt find;
    find.name.s = name;
    find.opt = NULL;
    QUEUE_Examine(&c->opts,CMDLINE_FindLongOptCB,&find);
    return find.opt;
}

/* 
 * The following functions create command line options. We allow two names 
 * for the same option, a short and a long one. For example: 'p' and 
 * "pattern". Both options are absolutely interchangeable. NOTE: CmdOpt 
 * does NOT copy option names and description to the internal storage. 
 * The strings MUST be statically allocated and SHOULD never change 
 * during the lifetime of CmdOpt object. Either short or long option 
 * (but not both) may be NULL.
 */
STATIC CmdOpt * CMDLINE_Add(CmdLine * c, Char opt1, Str opt2, Str descr)
{
    CmdOpt * opt = MEM_New(CmdOpt);
    ASSERT(opt1 || opt2);
    ASSERT(!opt2 || opt2[0] != '-');  /* must not start with - */
    ASSERT(!opt1 || !CMDLINE_FindShortOpt(c,opt1));
    ASSERT(!opt2 || !CMDLINE_FindLongOpt(c,opt2));
    ASSERT(!opt2 || StrLen(opt2)>0);
    if (opt) {
        memset(opt, 0, sizeof(*opt));
        if (VECTOR_Init(&opt->exclude, 0, NULL, NULL)) {
            if (VECTOR_Init(&opt->require, 0, NULL, NULL)) {
                opt->optChar = opt1;
                opt->optName = opt2;
                opt->descr = descr;
                QUEUE_InsertTail(&c->opts, &opt->entry);
                return opt;
            }
            VECTOR_Destroy(&opt->exclude);
        }
        MEM_Free(opt);
    }
    return NULL;
}

CmdOpt * 
CMDLINE_AddIntOpt(CmdLine * c, Char opt1, Str opt2, Str descr, int * value)
{
    ASSERT(value);
    if (value) {
        CmdOpt * opt = CMDLINE_Add(c, opt1, opt2, descr);
        if (opt) {
            opt->param = CMDLINE_NumParam;
            opt->type = OptInt;
            opt->value.i = value;
            return opt;
        }
    }
    return NULL;
}

CmdOpt * 
CMDLINE_AddUIntOpt(CmdLine* c, Char opt1, Str opt2, Str d, unsigned int* u)
{
    ASSERT(u);
    if (u) {
        CmdOpt * opt = CMDLINE_Add(c, opt1, opt2, d);
        if (opt) {
            opt->param = CMDLINE_NumParam;
            opt->type = OptUInt;
            opt->value.u = u;
            return opt;
        }
    }
    return NULL;
}

CmdOpt * 
CMDLINE_AddI64Opt(CmdLine * c, Char opt1, Str opt2, Str descr, I64s * value)
{
    ASSERT(value);
    if (value) {
        CmdOpt * opt = CMDLINE_Add(c, opt1, opt2, descr);
        if (opt) {
            opt->param = CMDLINE_NumParam;
            opt->type = OptI64;
            opt->value.i64 = value;
            return opt;
        }
    }
    return NULL;
}

CmdOpt * 
CMDLINE_AddFloatOpt(CmdLine* c, Char opt1, Str opt2, Str descr, float* value)
{
    ASSERT(value);
    if (value) {
        CmdOpt * opt = CMDLINE_Add(c, opt1, opt2, descr);
        if (opt) {
            opt->param = CMDLINE_NumParam;
            opt->type = OptFloat;
            opt->value.f = value;
            return opt;
        }
    }
    return NULL;
}

CmdOpt * 
CMDLINE_AddDoubleOpt(CmdLine* c, Char opt1,Str opt2,Str descr, double* value)
{
    ASSERT(value);
    if (value) {
        CmdOpt * opt = CMDLINE_Add(c, opt1, opt2, descr);
        if (opt) {
            opt->param = CMDLINE_NumParam;
            opt->type = OptDouble;
            opt->value.d = value;
            return opt;
        }
    }
    return NULL;
}

CmdOpt * 
CMDLINE_AddTrueOpt(CmdLine * c, Char opt1, Str opt2, Str descr, Bool * value)
{
    ASSERT(value);
    if (value) {
        CmdOpt * opt = CMDLINE_Add(c, opt1, opt2, descr);
        if (opt) {
            opt->type = OptTrue;
            opt->value.b = value;
            return opt;
        }
    }
    return NULL;
}

CmdOpt * 
CMDLINE_AddFalseOpt(CmdLine * c, Char opt1, Str opt2, Str descr, Bool * value)
{
    ASSERT(value);
    if (value) {
        CmdOpt * opt = CMDLINE_Add(c, opt1, opt2, descr);
        if (opt) {
            opt->type = OptFalse;
            opt->value.b = value;
            return opt;
        }
    }
    return NULL;
}

CmdOpt * 
CMDLINE_AddFlagOpt(CmdLine * c, Char opt1, Str opt2, Str descr, int f, int* v)
{
    ASSERT(v);
    if (v) {
        CmdOpt * opt = CMDLINE_Add(c, opt1, opt2, descr);
        if (opt) {
            opt->type = OptFlag;
            opt->value.flag.i = v;
            opt->value.flag.bits = f;
            return opt;
        }
    }
    return NULL;
}

CmdOpt * 
CMDLINE_AddStrOpt(CmdLine * c, Char opt1, Str opt2, Str descr, Str * value)
{
    ASSERT(value);
    if (value) {
        CmdOpt * opt = CMDLINE_Add(c, opt1, opt2, descr);
        if (opt) {
            opt->param = CMDLINE_StrParam;
            opt->type = OptStr;
            opt->value.s = value;
            return opt;
        }
    }
    return NULL;
}

/**
 * Creates a custom option. If param is not NULL, then this option is assumed
 * to have a parameter. The callback is invoked every time this option is seen
 * on the command line. If the callback function returns False, parsing is
 * stopped and CMDLINE_Parse return False.
 */
CmdOpt * CMDLINE_AddOpt(CmdLine * c, Char opt1, Str opt2, Str descr,
                        CmdOptProc cb, void * ctx, Str param)
{
    CmdOpt * opt = CMDLINE_Add(c, opt1, opt2, descr);
    if (opt) {
        ASSERT(cb);
        opt->param = param;
        opt->type = OptCustom;
        opt->value.custom.cb = cb;
        opt->value.custom.ctx = ctx;
    }
    return opt;
}

/**
 * Makes option repeatable. By default options are not repeatable.
 * Repeatable options can be repeated on the command line even if
 * PARSE_NO_DUP flag is passed to CMDLINE_Parse
 */
Bool CMDLINE_SetRepeatable(CmdOpt * opt)
{
    if (opt) {
        opt->flags |= CMDOPT_REPEATABLE;
        return True;
    } else {
        return False;
    }
}

/**
 * Marks option as required. If this option doesn;t appear on the command
 * line, CMDLINE_Parse will fail with an error message.
 */
Bool CMDLINE_SetRequired(CmdOpt * opt)
{
    if (opt) {
        opt->flags |= CMDOPT_REQUIRED;
        return True;
    } else {
        return False;
    }
}

/*
 * Duplicates the description string passed to CMDLINE_AddXXX
 * function. This may be useful if the description was dynamically
 * allocated and needs to be deallocated before CmdOpt. By default,
 * CmdOpt assumes that all strings passed to CMDLINE_AddXXX are
 * allocated statically, and it's safe to keep pointers to it.
 */
Bool CMDLINE_DupDesc(CmdOpt * opt)
{
    if (opt->flags & CMDOPT_OWN_DESCR) {
        return True;
    } else {
        opt->descr = STRING_Dup(opt->descr);
        if (opt->descr) {
            opt->flags |= CMDOPT_OWN_DESCR;
            return True;
        } else {
            return False;
        }
    }
}

/**
 * Returns how many times this option has been used on the command line.
 * Normally, returns either 0 (option was not used) or 1 (option was used)
 * but may return a numebr greater than 1 if the option is repeatable or
 * PARSE_NO_DUP flag wasn't set
 */
int CMDLINE_GetCount(CmdOpt * opt)
{
    return opt ? opt->count : 0;
}

/**
 * Defines mutually exclusive options. If opt2 is NULL, makes opt1
 * mutually exclusive with all other (currently defined) options.
 */
Bool CMDLINE_Exclude(CmdOpt * opt1, CmdOpt * opt2)
{
    ASSERT(opt1);
    ASSERT(opt1 != opt2);
    if (opt1 && opt1 != opt2) {
        if (opt2) {
            return BoolValue((VECTOR_Contains(&opt1->exclude, opt2) ||
                              VECTOR_Add(&opt1->exclude, opt2)) &&
                             (VECTOR_Contains(&opt2->exclude, opt1) ||
                              VECTOR_Add(&opt2->exclude, opt1)));
        } else {
            CmdLine * c = CAST_QUEUE(opt1->entry.queue,CmdLine,opts);
            if (VECTOR_EnsureCapacity(&opt1->exclude, c->opts.size-1)) {
                Bool ok = True;
                CmdOpt * opt = CMDOPT_First(c);
                while (opt) {
                    if (opt != opt1) {
                        if (!CMDLINE_Exclude(opt1,opt)) {
                            break;
                        }
                    }
                    opt = CMDOPT_Next(opt);
                }
                return ok;
            }
        }
    }
    return False;
}

/**
 * Defines dependent options. opt1 requires (depends on) opt2 
 */
Bool CMDLINE_Require(CmdOpt * opt1, CmdOpt * opt2)
{
    ASSERT(opt1 && opt2);
    ASSERT(opt1 != opt2);
    if (opt1 && opt2 && opt1 != opt2) {
        if (VECTOR_Contains(&opt1->require, opt2)) {
            return True;
        } else {
            return VECTOR_Add(&opt1->require, opt2);
        }
    }
    return False;
}

/*
 * Defines an "option group". Option groups exist strictly for formatting
 * purposes. The affect the output produced by CMDLINE_Usage function
 */
CmdOptGrp * CMDLINE_CreateOptGrp(CmdLine * c, Str name)
{
    CmdOptGrp * g = MEM_New(CmdOptGrp);
    if (g) {
        memset(g, 0, sizeof(*g));
        g->name = STRING_Dup(name);
        if (name) {
            QUEUE_Init(&g->opts);
            QUEUE_InsertTail(&c->groups, &g->entry);
            return g;
        }
        MEM_Free(g);
    }
    return NULL;
}

/**
 * Adds option to the option group
 */
void CMDLINE_AddToGroup(CmdOptGrp * group, CmdOpt * opt)
{
    if (group && opt) {
        QUEUE_RemoveEntry(&opt->groupQ);
        QUEUE_InsertTail(&group->opts, &opt->groupQ);
    }
}

/**
 * Formats the options, like "-x (--exit)"
 */
STATIC Str CMDLINE_FormatOption(const CmdOpt * opt, StrBuf * sb)
{
    STRBUF_Clear(sb);
    if (opt->optChar) {
        STRBUF_Copy(sb, TEXT("-"));
        STRBUF_AppendChar(sb, opt->optChar);
        if (opt->optName) {
            STRBUF_Append(sb, TEXT(" (--"));
            STRBUF_Append(sb, opt->optName);
            STRBUF_Append(sb, TEXT(")"));
            
        }
    } else {
        ASSERT(opt->optName);
        STRBUF_Copy(sb, TEXT("--"));
        STRBUF_Append(sb, opt->optName);
    }

    return STRBUF_Text(sb);
}

/**
 * Handles a single option. Returns True of the option has been processed,
 * False if an error occured.
 *
 * opt     the option being processed
 * arg     the current argument (leading '-' characters are stripped)
 * nextArg is the next argument in the command line, NULL if there's no 
 *         next argument. If option requires an argument, but there's no 
 *         next argument, this function returns False
 * flags   flags passed to CMDLINE_Parse
 * argList array of previously seen arguments (strings) 
 * optList array of previously seen options (CmdOpt) 
 */
STATIC Bool 
CMDLINE_HandleOption(CmdOpt * opt, Str arg, Str nextArg, 
                     int flags, Vector * argList, Vector * optList)
{
    CmdLine * c = CAST_QUEUE(opt->entry.queue,CmdLine,opts);
    int i, n = VECTOR_Size(argList);
    Str prefix = ((StrLen(arg) == 1) ? TEXT("-") : TEXT("--"));

    /* check for duplicate options? */
    if (!(opt->flags & CMDOPT_REPEATABLE) && (flags & PARSE_NO_DUP)) {
        if (VECTOR_Contains(argList,arg) || VECTOR_Contains(optList,opt)) {
            if (!(flags & PARSE_SILENT)) {
                StrBuf16 buf;
                STRBUF_InitBufXXX(&buf);
                CMDLINE_Msg(c,TEXT("duplicate option %s\n"),
                    CMDLINE_FormatOption(opt, &buf.sb));
                STRBUF_Destroy(&buf.sb);
            }
            return False;
        }
    }

    /* check for mutually exclusive options */
    if (!VECTOR_IsEmpty(&opt->exclude)) {
        for (i=0; i<n; i++) {
            CmdOpt * prevOpt = (CmdOpt*)VECTOR_Get(optList, i);
            if (VECTOR_Contains(&opt->exclude,prevOpt)) {
                if (!(flags & PARSE_SILENT)) {
                    StrBuf16 buf1;
                    StrBuf16 buf2;
                    STRBUF_InitBufXXX(&buf1);
                    STRBUF_InitBufXXX(&buf2);
                    CMDLINE_Msg(c,TEXT("option %s cannot be used with %s\n"),
                        CMDLINE_FormatOption(prevOpt, &buf1.sb), 
                        CMDLINE_FormatOption(opt, &buf2.sb));
                    STRBUF_Destroy(&buf1.sb);
                    STRBUF_Destroy(&buf2.sb);
                }
                return False;
            }
        }
    }

    /* does this option require an argument? boolean options don't */
    if (opt->type == OptTrue) {
        if (opt->value.b) (*opt->value.b) = True;
    } else if (opt->type == OptFalse) {
        if (opt->value.b) (*opt->value.b) = False;
    } else if (opt->type == OptFlag) {
        if (opt->value.flag.i) (*opt->value.flag.i) |= opt->value.flag.bits;
    } else {
        if (!nextArg) {
            if (!(flags & PARSE_SILENT)) {
                Str what = opt->param;
                if (!what) what = TEXT("argument");
                CMDLINE_Msg(c,TEXT("expecting %s after %s%s\n"),
                    what,prefix,arg);
            }
            return False;
        }

        /* try to interpret the argument */
        switch (opt->type) {
        case OptInt:
            if (!PARSE_Int(nextArg, opt->value.i, 0)) {
                if (!(flags & PARSE_SILENT)) {
                    Str what = opt->param;
                    if (!what) what = TEXT("number");
                    CMDLINE_Msg(c,TEXT("expecting %s after %s%s option,")
                        TEXT_(" not '%s'\n"),what,prefix,arg,nextArg);
                }
                return False;
            }
            break;
        case OptUInt:
            if (!PARSE_UInt(nextArg, opt->value.u, 0)) {
                if (!(flags & PARSE_SILENT)) {
                    Str what = opt->param;
                    if (!what) what = TEXT("unsigned number");
                    CMDLINE_Msg(c,TEXT("expecting %s after %s%s option,")
                        TEXT_(" not '%s'\n"),what,prefix,arg,nextArg);
                }
                return False;
            }
            break;
        case OptI64:
            if (!PARSE_I64(nextArg, opt->value.i64, 0)) {
                if (!(flags & PARSE_SILENT)) {
                    Str what = opt->param;
                    if (!what) what = TEXT("number");
                    CMDLINE_Msg(c,TEXT("expecting %s after %s%s option,")
                        TEXT_(" not '%s'\n"),what,prefix,arg,nextArg);
                }
                return False;
            }
            break;
        case OptFloat:
            if (!PARSE_Float(nextArg, opt->value.f)) {
                if (!(flags & PARSE_SILENT)) {
                    Str what = opt->param;
                    if (!what) what = TEXT("number");
                    CMDLINE_Msg(c,TEXT("expecting %s after %s%s option,")
                        TEXT_(" not '%s'\n"),what,prefix,arg,nextArg);
                }
                return False;
            }
            break;
        case OptDouble:
            if (!PARSE_Double(nextArg, opt->value.d)) {
                if (!(flags & PARSE_SILENT)) {
                    Str what = opt->param;
                    if (!what) what = TEXT("number");
                    CMDLINE_Msg(c,TEXT("expecting %s after %s%s option,")
                        TEXT_(" not '%s'\n"),what,prefix,arg,nextArg);
                }
                return False;
            }
            break;
        case OptCustom:
            if (!opt->value.custom.cb(
                CAST_QUEUE(opt->entry.queue,CmdLine,opts),
                nextArg, opt->value.custom.ctx)) {
                return False;
            }
            break;
        default:
            ASSERT(opt->type == OptStr);
            (*opt->value.s) = nextArg;
        }
    }

    /* store this option */
    VECTOR_Add(argList,STRING_Dup(arg));
    VECTOR_Add(optList,opt);
    opt->count++;
    return True;
}

/**
 * Resets option use count. This is done every time before parsing the
 * command line.
 */
STATIC Bool CMDLINE_ResetOptCountCB(QEntry * e, void * ctx)
{
    CmdOpt * opt = QCAST(e,CmdOpt,entry);
    UNREF(ctx);
    opt->count = 0;
    return True;
}

/** 
 * Internal helper for all CMDLINE_Parse routines
 */
STATIC Bool CMDLINE_DoParse(CmdLine * c, Char * args[], int n, int flg,
    Str* command, Vector * unused)
{
    int i;
    int unusedArgs = 0;
    Bool ok = True;
    Vector argList, optList;
    VECTOR_Init(&argList, 0, vectorEqualsString, vectorFreeValueProc);
    VECTOR_Init(&optList, 0, NULL, NULL);
    QUEUE_Examine(&c->opts, CMDLINE_ResetOptCountCB, NULL);
    if (command) *command = NULL;
    for (i=0; i<n && ok; i++) {
        Str arg = args[i];
        CmdOpt * opt = NULL;

        /* if it does not start with -, then it's not an option */
        if (arg[0] == '-') {

            /* if it starts with --, then it better be a long option */
            if (arg[1] == '-') {
                opt = CMDLINE_FindLongOpt(c, arg + 2);
                if (opt) {
                    arg += 2;
                } else {
                    if (!(flg & PARSE_SILENT)) {
                        CMDLINE_Msg(c,TEXT("unknown option %s\n"),arg);
                    }
                    ok = False;
                    break;
                }
            } else {

                /* try to handle this as a bunch of short options */
                int j = 1;
                Char s[2];
                s[1] = 0;
                while (arg[j]) {
                    opt = CMDLINE_FindShortOpt(c, arg[j]);
                    if (opt) {
                        Str next = NULL;
                        s[0] = arg[j];
                        if (!arg[j+1] && (i+1)<n) next = args[i+1];
                        ok = CMDLINE_HandleOption(opt,s,next,flg,
                            &argList,&optList);
                        if (ok) {
                            j++; /* switch to the next letter */
                            if (CMDOPT_HasParam(opt)) {
                                ASSERT(!arg[j]); /* must be last letter */
                                i++;
                            }
                        } else {
                            break;
                        }
                    } else {
                        if (!(flg & PARSE_SILENT)) {
                            CMDLINE_Msg(c,TEXT("unknown option -%c\n"),arg[j]);
                        }
                        ok = False;
                        break;
                    }
                }

                /* will break the loop if ok isn't True */
                continue;
            }
        }

        if (opt) {
            Str nextArg = NULL;
            if ((i+1)<n) nextArg = args[i+1];
            ok = CMDLINE_HandleOption(opt,arg,nextArg,flg,&argList,&optList);
            if (ok && CMDOPT_HasParam(opt)) i++;
        } else if (arg[0] == '-') {
            if (!(flg & PARSE_SILENT)) {
                CMDLINE_Msg(c,TEXT("unknown option %s\n"),arg);
            }
            ok = False;
            break;
        } else {
            CmdKeyword * k;
            if (command && (k = CMDLINE_FindKeyword(c, arg)) != NULL) {
                *command = k->name;
                if (unused) {
                    int j;
                    for (j = i+1; j<n; j++) {
                        VECTOR_Add(unused, args[j]);
                    }
                }
                break;
            } else {
                if (c->maxArgs >= 0 && unusedArgs == c->maxArgs) {
                    if (!(flg & PARSE_SILENT)) {
                        if (c->maxArgs == 0) {
                            CMDLINE_Msg(c,
                                TEXT("unexpected argument %s\n"),arg);
                        } else {
                            CMDLINE_Msg(c,
                                TEXT("too many command line parameters\n"));
                        }
                    }
                    ok = False;
                    break;
                }
                if (unused) VECTOR_Add(unused,arg);
            }
            unusedArgs++;
        }
    }
    if (ok && unusedArgs < c->minArgs) {
        if (!(flg & PARSE_SILENT)) {
            CMDLINE_Msg(c,TEXT("missing command line parameter%s\n"),
                (c->maxArgs == 1) ? TEXT("") : TEXT("(s)"));
        }
        ok = False;
    }

    /* check the dependencies between the options */
    if (ok) {
        for (i=0; i<VECTOR_Size(&optList); i++) {
            /* don't do the same thing twice */
            CmdOpt * opt = (CmdOpt*)VECTOR_Get(&optList,i);
            if (VECTOR_IndexOf(&optList,opt) == i) {
                if (!VECTOR_ContainsAll(&optList,&opt->require)) {
                    /* take the first one and print it */
                    CmdOpt * req = (CmdOpt*)VECTOR_Get(&opt->require,0);
                    Str badArg = (Str)VECTOR_Get(&argList,i);
                    Str prefix = ((StrLen(badArg)==1)?TEXT("-"):TEXT("--"));
                    if (!(flg & PARSE_SILENT)) {
                        if (req->optChar) {
                            CMDLINE_Msg(c,TEXT("option %s%s requires -%c\n"),
                                prefix, badArg, req->optChar);
                        } else {
                            CMDLINE_Msg(c,TEXT("option %s%s requires --%s\n"),
                                prefix, badArg, req->optName);
                        }
                    }
                    ok = False;
                    break;
                }
            }
        }
    }

    VECTOR_Destroy(&argList);
    VECTOR_Destroy(&optList);
    return ok;
}


/** 
 * Parses command line. You can use CMDLINE_SetOutFile or CMDLINE_SetOutProc
 * to forward error messages anywhere you like. The 'unused' vector (if not 
 * NULL), receives the additional command line arguments that were not
 * interpreted as options or their parameters.
 *
 * The same option may appear several time in the same command line. The
 * default behavior is that the value of the next option overrides the 
 * previous one. You can override this behavior by specifying PARSE_NO_DUP
 * flag. Repeatable options are always allowed more than once regardless
 * of PARSE_NO_DUP flag.
 */
Bool CMDLINE_Parse(CmdLine* c, Char* args[], int n, int flg, Vector* unused)
{
    return CMDLINE_DoParse(c, args, n, flg, NULL, unused);
}

/* 
 * Equivalent to CMDLINE_Parse, assumes that there may be no more than
 * one argument on the command line. If the argument has been specified,
 * then arg points to the specified argument when this function returns.
 * Otherwise, arg is set to NULL. Whether or not this argument is requred
 * is set by CMDLINE_SetMinArgs call. Set min args to 1 if parameter is
 * required, to zero if it's optional. CMDLINE_SetMaxArgs calls made prior
 * to this call are ignored. Max args is assumed to be 1 for the duration
 * of this call.
 */
Bool CMDLINE_Parse1(CmdLine* c, Char* args [], int nargs, int flags, Str* arg)
{
    Bool ok;
    Vector params;
    int maxArgs, minArgs;

    if (arg) *arg = NULL;
    VECTOR_Init(&params, 0, NULL, NULL);

    /* why would you want to set minArgs or maxArgs to more than one? */
    ASSERT(c->maxArgs <= 1); 
    ASSERT(c->minArgs <= 1); 
    maxArgs = c->maxArgs;
    minArgs = c->minArgs;
    if (c->minArgs > 1) c->minArgs = 1;
    c->maxArgs = 1;

    /* run the parser */
    ok = CMDLINE_Parse(c, args, nargs, flags, &params);
    if (ok && !VECTOR_IsEmpty(&params) && arg) {
        /* copy the result */
        *arg = (Str)VECTOR_Get(&params,0);
    }

    /* restore minArgs and maxArgs, just in case */
    c->minArgs = minArgs;
    c->maxArgs = maxArgs;
    VECTOR_Destroy(&params);
    return ok;
}

/* 
 * Parses command line, stops at a "command" keyword. You can use
 * CMDLINE_SetOutFile or CMDLINE_SetOutProc to forward error messages
 * anywhere you like. The 'cmdargs' vector (if not  NULL), receives all
 * additional command line arguments that follow the "command" keyword.
 * The 'command' parameter points to the "command" keyword that stopped
 * parsing. If PARSE_NO_DUP is set, then only repeatable options are
 * allowed on the command line.
 *
 * This function allows parsing svn/cvs style command line that has a
 * syntax like this:
 *
 * myapp [global options] command [command options]
 *
 * After you have analized the command, you can create another parser
 * for command specific options and continue parsing the options that
 * follow the command keyword. All known command keywords must be added
 * to the parser with CMDLINE_AddCmd function. Note that even if user
 * specifies a command variant (added with CMDLINE_AddCmdVar), the the
 * 'command' parameter still points to the full name of the command.
 */
Bool CMDLINE_ParseCmd(CmdLine * c, Char * args [], int nargs, int flags,
    Str * command, Vector * cmdargs)
{
    Str tmp = NULL;
    if (!command) command = &tmp;
    return CMDLINE_DoParse(c, args, nargs, flags, command, cmdargs);
}

/**
 * Prints the usage statement for a single option
 */
STATIC void 
USAGE_Opt(
    CmdLine * c, 
    CmdOpt * opt, 
    StrBuf * sb, 
    Str optPrefix, 
    Bool shortOpt, 
    size_t colWidth, 
    int maxcol)
{
    size_t pos;
    STRBUF_Copy(sb, optPrefix);
    if (opt->optChar && opt->optName) {
        STRBUF_AppendChar(sb,'-');
        STRBUF_AppendChar(sb,opt->optChar);
        STRBUF_Append(sb,TEXT(", --"));
        STRBUF_Append(sb,opt->optName);
    } else if (opt->optChar) {
        STRBUF_AppendChar(sb,'-');
        STRBUF_AppendChar(sb,opt->optChar);
    } else {
        if (shortOpt) STRBUF_Append(sb,TEXT("    "));
        STRBUF_Append(sb,TEXT("--"));
        STRBUF_Append(sb,opt->optName);
    }
    if (opt->param) {
        STRBUF_AppendChar(sb,' ');
        STRBUF_Append(sb,opt->param);
    }
    STRBUF_Inflate(sb,colWidth,' ');
    pos = CMDLINE_Puts(c,STRBUF_Text(sb));

    if (opt->descr) {
        Str p;
        size_t len;
        Char * percent;
        StrBuf descrBuf;
        STRBUF_Init(&descrBuf);

        /* if description string contains %, assume that the caller
         * wants to include the current (presumably, default) value
         * of the parameter in the description text */
        switch (opt->type) {
        case OptInt:
        case OptUInt:
        case OptI64:
        case OptFloat:
        case OptDouble:
        case OptStr:
            percent = StrChr(opt->descr,'%');
            if (percent) {
                switch (opt->type) {
                case OptInt:
                    STRBUF_Format(&descrBuf, opt->descr, *opt->value.i);
                    break;
                case OptUInt:
                    STRBUF_Format(&descrBuf, opt->descr, *opt->value.u);
                    break;
                case OptI64:
                    STRBUF_Format(&descrBuf, opt->descr, *opt->value.i64);
                    break;
                case OptFloat:
                    STRBUF_Format(&descrBuf, opt->descr, *opt->value.f);
                    break;
                case OptDouble:
                    STRBUF_Format(&descrBuf, opt->descr, *opt->value.d);
                    break;
                case OptStr:
                    STRBUF_Format(&descrBuf, opt->descr, *opt->value.s);
                    break;
                default:
                    ASSERT(False);
                    break;
                }
                p = STRBUF_Text(&descrBuf);
                len = STRBUF_Length(&descrBuf);
                break;
            }
            /* no break */
        default:
            p = opt->descr;
            len = StrLen(p);
            break;
        }        
        
        while (*p && IsSpace(*p)) p++;

        /* can just print the whole thing? */
        if (maxcol <= 0 || (pos+len) <= (size_t)maxcol) {
            CMDLINE_Printf(c,TEXT("%s\n"),p);
        } else {
            int nwords = 0;
            while (*p) {
                STRBUF_Clear(sb);
                if (!nwords) {
                    /* looking for the first word */
                    while(*p && IsSpace(*p)) p++;
                    while(*p && !IsSpace(*p))STRBUF_AppendChar(sb,*p++);
                    pos += CMDLINE_Puts(c,STRBUF_Text(sb));
                    nwords++;
                } else {
                    /* scan next word */
                    Str save = p;
                    while(*p && IsSpace(*p))STRBUF_AppendChar(sb,*p++);
                    while(*p && !IsSpace(*p))STRBUF_AppendChar(sb,*p++);
                    if ((pos + STRBUF_Length(sb)) < (size_t)maxcol) {
                        pos += CMDLINE_Puts(c,STRBUF_Text(sb));
                    } else {
                        CMDLINE_Printf(c,TEXT("\n"));
                        STRBUF_Clear(sb);
                        STRBUF_Inflate(sb, colWidth,' ');
                        pos = CMDLINE_Printf(c,STRBUF_Text(sb));
                        nwords = 0;
                        p = save;

                        /* can just print the rest? */
                        while(*p && IsSpace(*p)) p++;
                        len = StrLen(p);
                        if ((pos+len) <= (size_t)maxcol) {
                            CMDLINE_Puts(c,p);
                            break;
                        }
                    }
                }
            }
            /* terminate the last line */
            CMDLINE_Printf(c,TEXT("\n"));
        }
        STRBUF_Destroy(&descrBuf);
    }
}

/* 
 * Prints usage summary. If the maxcol parameter is zero, the text will be
 * wrapped to fit into the terminal window. Negative maxcol turns off the
 * wrapping.  You can use CMDLINE_SetOutFile or CMDLINE_SetOutProc to forward
 * the output anywhere you like.
 */
void CMDLINE_Usage(CmdLine * c, Str extra, int maxcol)
{
    /* write Usage: stuff only if we have app name */
    if (c->print && c->app) {
        CmdOpt * opt;
        Str prefix;
        int optCount = 0;
        static const Str usage = TEXT("Usage: ");
        size_t pos = CMDLINE_Printf(c,TEXT("%s%s "),usage,c->app);
        size_t prefixLen = StrLen(usage) + STRBUF_Length(&c->appBuf.sb) + 1;

        StrBuf64 optBuf;
        StrBuf32 prefixBuf;
        StrBuf * sb = &optBuf.sb;
        STRBUF_InitBufXXX(&optBuf);
        STRBUF_InitBufXXX(&prefixBuf);
        STRBUF_Inflate(&prefixBuf.sb, prefixLen,' ');
        prefix = STRBUF_Text(&prefixBuf.sb);

        /* use full screen width? */
        if (maxcol == 0) {
            maxcol = 80;
            IO_TermSize(NULL, &maxcol);
            maxcol--;
        }

        /* first print required options */
        opt = CMDOPT_First(c);
        while (opt) {
            if (opt->flags & CMDOPT_REQUIRED) {
                STRBUF_Clear(sb);
                /* print one variant for required options, preferably short */
                if (opt->optChar) {
                    STRBUF_AppendChar(sb, '-');
                    STRBUF_AppendChar(sb,opt->optChar);
                } else {
                    STRBUF_Append(sb,TEXT("--"));
                    STRBUF_Append(sb,opt->optName);
                }
                if (opt->param) {
                    STRBUF_AppendChar(sb, ' ');
                    STRBUF_Append(sb,opt->param);
                }
                STRBUF_AppendChar(sb, ' ');

                /* break the line is necessary */
                if ((maxcol > 0) && (optCount > 0) && 
                   ((pos + STRBUF_Length(sb)) >= (size_t)maxcol)) {
                    CMDLINE_Printf(c,TEXT("\n"));
                    pos = CMDLINE_Printf(c, prefix);
                    optCount = 0;
                }

                /* print this option */
                pos += CMDLINE_Printf(c, STRBUF_Text(sb));
                optCount++;
            }

            /* switch to the next option */
            opt = CMDOPT_Next(opt);
        }

        /* then print options that are not required */
        opt = CMDOPT_First(c);
        while (opt) {
            if (!(opt->flags & CMDOPT_REQUIRED)) {
                STRBUF_Clear(sb);
                /* print both variants for non-required options */
                STRBUF_AppendChar(sb, '[');
                if (opt->optChar && opt->optName) {
                    STRBUF_AppendChar(sb, '-');
                    STRBUF_AppendChar(sb, opt->optChar);
                    STRBUF_Append(sb, TEXT(", --"));
                    STRBUF_Append(sb, opt->optName);
                } else if (opt->optChar) {
                    STRBUF_AppendChar(sb, '-');
                    STRBUF_AppendChar(sb,opt->optChar);
                } else {
                    STRBUF_Append(sb, TEXT("--"));
                    STRBUF_Append(sb, opt->optName);
                }
                if (opt->param) {
                    STRBUF_AppendChar(sb, ' ');
                    STRBUF_Append(sb, opt->param);
                }
                STRBUF_Append(sb, TEXT("] "));

                /* break the line is necessary */
                if ((maxcol > 0) && (optCount > 0) && 
                   ((pos + STRBUF_Length(sb)) >= (size_t)maxcol)) {
                    CMDLINE_Printf(c,TEXT("\n"));
                    pos = CMDLINE_Printf(c, prefix);
                    optCount = 0;
                }

                /* print this option */
                pos += CMDLINE_Printf(c, STRBUF_Text(sb));
                optCount++;
            }

            /* switch to the next option */
            opt = CMDOPT_Next(opt);
        }

        /* print extra arguments */
        if (extra) {

            /* break the line is necessary */
            if ((maxcol > 0) && (optCount > 0) && 
               ((pos + StrLen(extra)) >= (size_t)maxcol)) {
                CMDLINE_Printf(c,TEXT("\n%s"), STRBUF_Text(&prefixBuf.sb));
            }

            /* print extra arguments */
            CMDLINE_Puts(c, extra);
        }

        /* terminate the last line */
        CMDLINE_Printf(c, TEXT("\n"));

        /* cleanup */
        STRBUF_Destroy(&optBuf.sb);
        STRBUF_Destroy(&prefixBuf.sb);
    }
}

/* 
 * Prints full help. If the maxcol parameter is zero, the text will be wrapped
 * to fit into the terminal window. Negative maxcol turns off the wrapping. 
 * You can use CMDLINE_SetOutFile or CMDLINE_SetOutProc to forward the output
 * anywhere you like.
 */
void CMDLINE_Help(CmdLine * c, Str extra, int maxcol)
{
    if (c->print && !QUEUE_IsEmpty(&c->opts)) {
        Str optPrefix = TEXT("  ");
        CmdOpt * opt;
        CmdOptGrp * grp;
        StrBuf64 buf;
        Bool shortOpt = False, defaultGroup = False, otherGroup = False;
        size_t colWidth = 0;

        STRBUF_InitBufXXX(&buf);

        /* use full screen width? */
        if (maxcol == 0) {
            maxcol = 80;
            IO_TermSize(NULL, &maxcol);
            maxcol--;
        }

        /* print usage summary first */
        CMDLINE_Usage(c, extra, maxcol);

        /* calculate the length of the longest option */
        opt = CMDOPT_First(c);
        while (opt && !shortOpt) {
            if (opt->optChar) shortOpt = True;
            opt = CMDOPT_Next(opt);
        }
        opt = CMDOPT_First(c);
        while (opt) {
            size_t len = 0;
            if (shortOpt) {
                len += 2;
            }
            if (opt->optName) {
                len += StrLen(opt->optName) + 2;
                if (shortOpt) len += 2;
            }
            if (opt->param) {
                len += StrLen(opt->param) + 1;
            }
            if (len > colWidth) {
                colWidth = len;
            }
            opt = CMDOPT_Next(opt);
        }
        colWidth += StrLen(optPrefix) + 2;

        /* first describe the options not associated with any group */
        opt = CMDOPT_First(c);
        while (opt) {
            if (!opt->groupQ.queue) {
                if (!defaultGroup) {
                    CMDLINE_Printf(c,TEXT("Options:\n"));
                    defaultGroup = True;
                }
                USAGE_Opt(c,opt,&buf.sb,optPrefix,shortOpt,colWidth,maxcol);
            }
            opt = CMDOPT_Next(opt);
        }

        /* then print other option group */
        grp = OPTGRP_First(c);
        while (grp) {
            QEntry * e = QUEUE_First(&grp->opts);
            if (e) {
                if (defaultGroup || otherGroup) CMDLINE_Printf(c,TEXT("\n"));
                CMDLINE_Printf(c,TEXT("%s:\n"),grp->name);
                otherGroup = True;
            }
            while (e) {
                opt = QCAST(e,CmdOpt,groupQ);
                USAGE_Opt(c,opt,&buf.sb,optPrefix,shortOpt,colWidth,maxcol);
                e = QUEUE_Next(e);
            }
            grp = OPTGRP_Next(grp);
        }

        STRBUF_Destroy(&buf.sb);
    }
}

/*
 * HISTORY:
 *
 * $Log: s_opt.c,v $
 * Revision 1.41  2010/12/22 17:57:04  slava
 * o added CMDLINE_AddFlagOpt function
 *
 * Revision 1.40  2010/12/21 19:12:00  slava
 * o fixed gcc compilation error (missing return type)
 *
 * Revision 1.39  2010/12/20 20:16:34  slava
 * o added CMDLINE_AddCmd, CMDLINE_AddCmdVar, CMDLINE_AddCmdVars and
 *   CMDLINE_ParseCmd functions which allow parsing of cvs/svn-like
 *   command lines (common options, followed by a command and then
 *   command-specific options)
 *
 * Revision 1.38  2009/12/13 12:45:25  slava
 * o added CMDLINE_AddUIntOpt
 *
 * Revision 1.37  2009/05/23 10:23:04  slava
 * o more 32-bit warnings fixed... Damn! Is this ever going to end?
 *
 * Revision 1.36  2009/05/23 10:20:48  slava
 * o more 64-bit warnings fixed...
 *
 * Revision 1.35  2009/05/23 10:05:06  slava
 * o continuing to fight compilation warnings...
 *
 * Revision 1.34  2009/04/13 18:25:07  slava
 * o reduced number of warnings in 64-bit build
 *
 * Revision 1.33  2008/11/22 20:01:38  slava
 * o ignore NULL option in CMDLINE_AddToGroup
 *
 * Revision 1.32  2008/07/27 13:52:26  slava
 * o added CMDLINE_AddFloatOpt
 *
 * Revision 1.31  2006/12/01 16:27:19  slava
 * o added support for required options (CMDLINE_SetRequired)
 *
 * Revision 1.30  2006/11/26 21:05:02  slava
 * o if description string contains %, assume that the caller wants to
 *   include the current (presumably, default) value of the parameter
 *   in the description text
 *
 * Revision 1.29  2006/10/20 04:56:45  slava
 * o cleanup. moved file related utilities (most if not all of them implemented
 *   in s_futil.c) into a separate header file, s_futil.h. This may break
 *   compilation of the sources that include individual slib header files
 *   instead of including s_lib.h
 *
 * Revision 1.28  2006/10/01 19:52:33  slava
 * o added CMDLINE_DupDesc function (makes internal copy of the description)
 * o added CMDLINE_Help function. It now does what CMDLINE_Usage used to do.
 *   CMDLINE_Usage now prints only the usage summary. Also, fixed erroneous
 *   comment in the header file.
 *
 * Revision 1.27  2006/03/28 23:44:37  slava
 * o handle NULL CmdOpt parameters
 *
 * Revision 1.26  2005/10/11 01:09:26  slava
 * o CMDLINE_Usage fix for "no wrap" mode
 *
 * Revision 1.25  2005/03/11 22:29:34  slava
 * o added PARSE_SILENT option
 *
 * Revision 1.24  2005/02/20 20:31:30  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.23  2005/02/09 23:44:50  slava
 * o changed an error message
 *
 * Revision 1.22  2005/02/08 01:08:08  slava
 * o fixed bug in CMDLINE_Parse1
 *
 * Revision 1.21  2005/02/07 18:27:57  slava
 * o added CMDLINE_Parse1
 *
 * Revision 1.20  2005/01/02 16:14:00  slava
 * o changed some parameters of vector functions from VElement to VElementC
 *
 * Revision 1.19  2004/12/26 18:22:19  slava
 * o support for CodeWarrior x86 compiler
 *
 * Revision 1.18  2004/11/17 08:45:35  slava
 * o fixed couple minor parsing issues
 * o made the output nicer
 *
 * Revision 1.17  2004/04/08 01:44:18  slava
 * o made it compilable with C++ compiler
 *
 * Revision 1.16  2004/02/03 20:30:23  slava
 * o minor usage formatting issues
 *
 * Revision 1.15  2003/10/23 06:42:00  slava
 * o if IO_TermSize fails, assume 80 columns terminal
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
 * Revision 1.13  2003/01/30 07:33:08  slava
 * o support for repeatable options
 * o changed CMDLINE_AddStrOpt prototype
 *
 * Revision 1.12  2003/01/26 21:14:22  slava
 * o added CMDLINE_AddDoubleOpt
 *
 * Revision 1.11  2003/01/24 05:36:07  slava
 * o added CMDLINE_AddI64Opt (an option with 64-bit integer argument)
 *
 * Revision 1.10  2003/01/09 14:44:01  slava
 * o more informative error message
 *
 * Revision 1.9  2003/01/07 21:43:36  slava
 * o added "option groups"
 *
 * Revision 1.8  2003/01/02 03:48:20  slava
 * o CMDLINE_Usage can automatically detect number of columns on the
 *   terminal and adjust wrapping accordingly
 *
 * Revision 1.7  2002/09/15 20:19:11  slava
 * o removed unused static function
 *
 * Revision 1.6  2002/06/01 04:59:38  slava
 * o fixed compile error under Windows
 *
 * Revision 1.5  2002/05/31 06:49:28  slava
 * o minor optimization
 *
 * Revision 1.4  2002/05/31 04:17:37  slava
 * o fixed formatting bugs in CMDLINE_Usage
 * o CMDLINE_Exclude and CMDLINE_Require now return boolean status
 * o CMDLINE_Exclude takes NULL as the second parameter. That makes the
 *   option specified by the first parameter mutually exclusive with
 *   all other currently defined options.
 *
 * Revision 1.3  2002/05/29 08:22:02  slava
 * o slightly changed CMDLINE_Parse prototype
 *
 * Revision 1.2  2002/05/29 06:54:43  slava
 * o fixed compilation warning
 *
 * Revision 1.1  2002/05/29 06:49:42  slava
 * o parser for command line options
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
