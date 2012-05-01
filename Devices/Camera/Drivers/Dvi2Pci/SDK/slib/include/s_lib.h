/*
 * $Id: s_lib.h,v 1.26 2009/11/17 00:14:24 slava Exp $
 *
 * Copyright (C) 2000-2009 by Slava Monich
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

#ifndef _SLAVA_LIB_H_
#define _SLAVA_LIB_H_

#include "s_base32.h"
#include "s_base64.h"
#include "s_bitset.h"
#include "s_buf.h"
#include "s_cs.h"
#include "s_dom.h"
#include "s_event.h"
#include "s_file.h"
#include "s_futil.h"
#include "s_hash.h"
#include "s_hist.h"
#include "s_itr.h"
#include "s_lib.h"
#include "s_lock.h"
#include "s_math.h"
#include "s_md.h"
#include "s_mem.h"
#include "s_mutex.h"
#include "s_opt.h"
#include "s_prop.h"
#include "s_queue.h"
#include "s_random.h"
#include "s_ring.h"
#include "s_rwlock.h"
#include "s_stack.h"
#include "s_strbuf.h"
#include "s_thread.h"
#include "s_util.h"
#include "s_vector.h"
#include "s_ver.h"
#include "s_wkq.h"
#include "s_xml.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*
 * SLIB_InitModules() and SLIB_Init() do the same thing, the only 
 * difference is that Win32 version of SLIB_Init() initializes 
 * Winsock, while SLIB_InitModules() does not. On any other platform
 * these two functions are absolutely equivalent.
 */
extern void SLIB_Init P_((void));
extern void SLIB_InitModules P_((void));
extern void SLIB_Shutdown P_((void));

#ifdef __cplusplus
} /* end of extern "C" */
#endif  /* __cplusplus */

#endif /* _SLAVA_LIB_H_ */

/*
 * HISTORY:
 *
 * $Log: s_lib.h,v $
 * Revision 1.26  2009/11/17 00:14:24  slava
 * o introducing generic synchronization API. This is a pretty destructive
 *   checkin, it's not 100% backward compatible. The Lock type is now a
 *   generic lock. What used to be called Lock is now RWLock. Sorry.
 *
 * Revision 1.25  2008/09/03 09:24:35  slava
 * o added ring buffer object
 *
 * Revision 1.24  2006/10/20 04:56:44  slava
 * o cleanup. moved file related utilities (most if not all of them implemented
 *   in s_futil.c) into a separate header file, s_futil.h. This may break
 *   compilation of the sources that include individual slib header files
 *   instead of including s_lib.h
 *
 * Revision 1.23  2004/08/18 02:52:01  slava
 * o added support for BASE32 encoding
 *
 * Revision 1.22  2004/07/29 17:09:37  slava
 * o message digest framework
 *
 * Revision 1.21  2004/07/19 22:55:11  slava
 * o moved BASE64 encoding functions from s_util to s_base64 module
 *
 * Revision 1.20  2004/03/25 04:01:51  slava
 * o added SLIB_VERSION macro
 *
 * Revision 1.19  2003/12/04 04:48:55  slava
 * o replaced _LINUX_KERNEL_ with _LINUX_KERNEL for consistency with other
 *   platform defines such as _UNIX, _WIN32, _NT_KERNEL etc.
 *
 * Revision 1.18  2003/11/30 02:49:57  slava
 * o port to Linux kernel mode environment
 *
 * Revision 1.17  2003/11/08 20:47:43  slava
 * o added iterators
 *
 * Revision 1.16  2003/05/21 17:36:34  slava
 * o added extern "C" block for C++ compilers
 *
 * Revision 1.15  2003/05/21 00:17:25  slava
 * o include s_mem.h
 *
 * Revision 1.14  2003/01/29 07:33:20  slava
 * o include s_dom.h
 *
 * Revision 1.13  2002/08/12 05:13:27  slava
 * o use (void) rather than () to declare functions that have
 *   no arguments. The problem with () is that compilers interpret
 *   it as an *unspecified* argument list, but (void) clearly
 *   indicates that function takes no arguments. Improves compile
 *   time error checking
 *
 * Revision 1.12  2002/05/29 06:49:42  slava
 * o parser for command line options
 *
 * Revision 1.11  2002/05/22 04:24:42  slava
 * o include s_strbuf.h
 *
 * Revision 1.10  2001/12/28 00:41:16  slava
 * o added BitSet functions
 *
 * Revision 1.9  2001/11/27 06:06:58  slava
 * o added "work queue" module
 *
 * Revision 1.8  2001/11/26 07:59:55  slava
 * o added s_math module
 *
 * Revision 1.7  2001/11/25 01:29:02  slava
 * o added SLIB_Init(), SLIB_InitModules() and SLIB_Shutdown() functions
 *
 * Revision 1.6  2001/10/08 08:25:36  slava
 * o support for parsing XML files
 *
 * Revision 1.5  2001/05/30 04:42:13  slava
 * o from now on this code is distributed under BSD-style license which
 *   permits unlimited redistribution and use in source and binary forms
 *
 * Revision 1.4  2001/01/12 06:52:53  slava
 * o support for histogramming
 *
 * Revision 1.3  2001/01/03 09:18:23  slava
 * o new file I/O support for transparent access to plain or compressed
 *   files, sockets, etc.
 *
 * Revision 1.2  2000/12/31 02:55:47  oleg
 * o added Stack to slib
 *
 * Revision 1.1  2000/08/19 04:48:58  slava
 * o initial checkin
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
