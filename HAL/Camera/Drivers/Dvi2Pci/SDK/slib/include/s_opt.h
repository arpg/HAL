/*
 * $Id: s_opt.h,v 1.19 2010/12/22 17:57:04 slava Exp $
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

#ifndef _SLAVA_CMDLINE_OPT_H_
#define _SLAVA_CMDLINE_OPT_H_

#include "s_def.h"
#include "s_file.h"
#include "s_vector.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*
 * Command line parser
 */

/* Opaque structures */
typedef struct _CmdLine    CmdLine;
typedef struct _CmdOpt     CmdOpt;
typedef struct _CmdOptGrp  CmdOptGrp;

/* Function that receives the output */
typedef void (*CmdOutProc) P_((void * ctx, Str str));

/* callback for a custom option. Returns False to stop parsing */
typedef Bool (*CmdOptProc) P_((CmdLine * c, Str param, void * ctx));

/* Create/destroy command line object */
extern CmdLine * CMDLINE_Create P_((Str app));
extern void CMDLINE_Delete P_((CmdLine * c));

/* Set command line parser options */
extern void CMDLINE_SetApp P_((CmdLine * c, Str app));
extern void CMDLINE_SetMinArgs P_((CmdLine * c, int min));
extern void CMDLINE_SetMaxArgs P_((CmdLine * c, int max));
extern void CMDLINE_SetOutFile P_((CmdLine * c, File * out));
extern void CMDLINE_SetOutProc P_((CmdLine * c, CmdOutProc out, void * ctx));
extern Bool CMDLINE_SetParamName P_((CmdOpt * opt, Str param));

/* Utilities */
extern Str  CMDLINE_GetApp P_((CmdLine * c));
extern void CMDLINE_Msg P_((CmdLine * c, Str format, ...) PRINTF_ATTR(2,3));

/* 
 * Command line options. We allow two names for the same option, a short 
 * and a long one. For example: 'p' and "pattern". Both options are 
 * absolutely interchangeable. NOTE: CmdOpt does NOT copy option names
 * and description to the internal storage. The strings MUST be statically
 * allocated and SHOULD never change during the lifetime of CmdOpt object.
 * Either short or long option (but not both) may be NULL.
 *
 * Boolean options take no arguments. The different between CMDLINE_AddTrueOpt
 * and CMDLINE_AddFalseOpt is that the option defined by CMDLINE_AddTrueOpt
 * sets the value to True, while CMDLINE_AddFalseOpt set it to False, as
 * you might have guessed.
 *
 * The same option may appear several time in the same command line. The
 * default behavior is that the value of the next option overrides the 
 * previous one. You can override this behavior by specifying PARSE_NO_DUP
 * flag when you call CMDLINE_Parse.
 */
extern CmdOpt * CMDLINE_AddIntOpt P_((CmdLine * c, Char optChar, Str longOpt,
                                      Str descr, int * value));
extern CmdOpt * CMDLINE_AddUIntOpt P_((CmdLine * c, Char optChar, Str longOpt,
                                      Str descr, unsigned int * value));
extern CmdOpt * CMDLINE_AddI64Opt P_((CmdLine * c, Char optChar, Str longOpt,
                                      Str descr, I64s * value));
extern CmdOpt * CMDLINE_AddFloatOpt P_((CmdLine * c,Char optChar, Str longOpt,
                                        Str descr, float * value));
extern CmdOpt * CMDLINE_AddDoubleOpt P_((CmdLine * c,Char optChar,Str longOpt,
                                      Str descr, double * value));
extern CmdOpt * CMDLINE_AddTrueOpt P_((CmdLine * c, Char optChar, Str longOpt, 
                                       Str descr, Bool * value));
extern CmdOpt * CMDLINE_AddFalseOpt P_((CmdLine * c, Char optChar,Str longOpt,
                                        Str descr, Bool * value));
extern CmdOpt * CMDLINE_AddFlagOpt P_((CmdLine * c, Char optChar, Str longOpt,
                                      Str descr, int flag, int * value));
extern CmdOpt * CMDLINE_AddStrOpt P_((CmdLine * c, Char optChar, Str longOpt, 
                                      Str descr, Str * value));
extern CmdOpt * CMDLINE_AddOpt P_((CmdLine * c, Char optChar, Str longOpt, 
                                   Str descr, CmdOptProc cb, void * ctx,
                                   Str param));

/* Adds a "command" keyword. Used by CMDLINE_ParseCmd */
extern Bool CMDLINE_AddCmd P_((CmdLine * c, Str cmd));
extern Bool CMDLINE_AddCmdVar P_((CmdLine * c, Str cmd, Str variant));
extern Bool CMDLINE_AddCmdVars P_((CmdLine*c, Str cmd,const Str*vars,int n));

/* By default options are not required */
extern Bool CMDLINE_SetRequired P_((CmdOpt * opt));

/* By default options are not repeatable */
extern Bool CMDLINE_SetRepeatable P_((CmdOpt * opt));
extern int  CMDLINE_GetCount P_((CmdOpt * opt));

/*
 * Duplicates the description string passed to CMDLINE_AddXXX
 * function. This may be useful if the description was dynamically
 * allocated and needs to be deallocated before CmdOpt. By default,
 * CmdOpt assumes that all strings passed to CMDLINE_AddXXX are
 * allocated statically, and it's safe to keep pointers to it.
 */
extern Bool CMDLINE_DupDesc P_((CmdOpt * opt));

/* Defines mutually exclusive options. */
extern Bool CMDLINE_Exclude P_((CmdOpt * opt1, CmdOpt * opt2));

/* Defines dependent options. opt1 requires (depends on) opt2 */
extern Bool CMDLINE_Require P_((CmdOpt * opt1, CmdOpt * opt2));

/*
 * Defines an "option group". Option groups exist strictly for formatting
 * purposes. The affect the output produced by CMDLINE_Usage function
 */
extern CmdOptGrp * CMDLINE_CreateOptGrp P_((CmdLine * c, Str name));
extern void CMDLINE_AddToGroup P_((CmdOptGrp * group, CmdOpt * opt));

/* 
 * Parses command line. You can use CMDLINE_SetOutFile or CMDLINE_SetOutProc
 * to forward error messages anywhere you like. The 'unused' vector (if not 
 * NULL), receives the additional command line arguments that were not
 * interpreted as options or their parameters. If PARSE_NO_DUP is set, then
 * only repeatable options are allowed on the command line.
 */
extern Bool CMDLINE_Parse P_((CmdLine * c, Char * args [], int nargs,
                              int flags, Vector * unused));
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
extern Bool CMDLINE_Parse1 P_((CmdLine * c, Char * args [], int nargs,
                              int flags, Str * arg));

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
 * specifies a command variant (added with CMDLINE_AddCmdVariant), the
 * 'command' parameter still points to the full name of the command.
 */
extern Bool CMDLINE_ParseCmd P_((CmdLine * c, Char * args [], int nargs,
    int flags, Str * command, Vector * cmdargs));

/* Flags for CMDLINE_Parse */
#define PARSE_NO_DUP    0x0001  /* disallow duplicate options */
#define PARSE_SILENT    0x0002  /* don't print error messages */

/* 
 * Print usage summary (CMDLINE_Usage) or full help (CMDLINE_Help) which
 * includes usage summary as well. If the maxcol parameter is zero, the
 * text will be wrapped to fit into the terminal window. Negative maxcol
 * turns off the wrapping. You can use CMDLINE_SetOutFile/Proc function
 * to forward the output anywhere you like.
 */
extern void CMDLINE_Usage P_((CmdLine * c, Str extra, int maxcol));
extern void CMDLINE_Help P_((CmdLine * c, Str extra, int maxcol));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_CMDLINE_OPT_H_ */

/*
 * HISTORY:
 *
 * $Log: s_opt.h,v $
 * Revision 1.19  2010/12/22 17:57:04  slava
 * o added CMDLINE_AddFlagOpt function
 *
 * Revision 1.18  2010/12/20 20:16:34  slava
 * o added CMDLINE_AddCmd, CMDLINE_AddCmdVar, CMDLINE_AddCmdVars and
 *   CMDLINE_ParseCmd functions which allow parsing of cvs/svn-like
 *   command lines (common options, followed by a command and then
 *   command-specific options)
 *
 * Revision 1.17  2010/09/25 09:55:06  slava
 * o added PRINTF_ATTR macro which assigns printf-like characteristics to the
 *   declared function and enables gcc to check the format string against the
 *   parameters.
 *
 * Revision 1.16  2009/12/13 12:45:25  slava
 * o added CMDLINE_AddUIntOpt
 *
 * Revision 1.15  2008/07/27 13:52:25  slava
 * o added CMDLINE_AddFloatOpt
 *
 * Revision 1.14  2006/12/01 16:27:19  slava
 * o added support for required options (CMDLINE_SetRequired)
 *
 * Revision 1.13  2006/10/01 19:52:33  slava
 * o added CMDLINE_DupDesc function (makes internal copy of the description)
 * o added CMDLINE_Help function. It now does what CMDLINE_Usage used to do.
 *   CMDLINE_Usage now prints only the usage summary. Also, fixed erroneous
 *   comment in the header file.
 *
 * Revision 1.12  2006/03/28 23:44:36  slava
 * o handle NULL CmdOpt parameters
 *
 * Revision 1.11  2005/03/11 22:29:34  slava
 * o added PARSE_SILENT option
 *
 * Revision 1.10  2005/02/07 18:27:56  slava
 * o added CMDLINE_Parse1
 *
 * Revision 1.9  2004/12/27 07:05:00  slava
 * o fixed function prototype that didn't use the P_ macro
 *
 * Revision 1.8  2003/01/30 07:33:07  slava
 * o support for repeatable options
 * o changed CMDLINE_AddStrOpt prototype
 *
 * Revision 1.7  2003/01/26 21:14:22  slava
 * o added CMDLINE_AddDoubleOpt
 *
 * Revision 1.6  2003/01/24 05:36:07  slava
 * o added CMDLINE_AddI64Opt (an option with 64-bit integer argument)
 *
 * Revision 1.5  2003/01/07 21:43:36  slava
 * o added "option groups"
 *
 * Revision 1.4  2002/12/17 04:01:35  slava
 * o fixed a typo
 *
 * Revision 1.3  2002/05/31 04:13:47  slava
 * o CMDLINE_Exclude and CMDLINE_Require should return a status
 *
 * Revision 1.2  2002/05/29 08:22:02  slava
 * o slightly changed CMDLINE_Parse prototype
 *
 * Revision 1.1  2002/05/29 06:49:42  slava
 * o parser for command line options
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * compile-command: "make -C .."
 * End:
 */
