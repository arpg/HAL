/*
 * $Id: s_ver.h,v 1.56 2010/12/18 16:57:05 slava Exp $
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

#ifndef _SLAVA_VERSION_H_
#define _SLAVA_VERSION_H_

/* NOTE: we have room for micro version, but not using it yet */
#define SLIB_MAKE_VERSION(major,minor) (((major)<<16)|(((minor)&0xff)<<8))

/* 
 * SLIB_VERSION macro defines the current version of SLIB.
 *
 * NOTE: stable (released) builds have even minor versions, e.g. 1.18, 1.20,
 * etc. Development builds (such as the ones that you get directly from CVS)
 * have odd minor versions, e.g. 1.19, 1.21 and so on. The SLIB_VERSION
 * macro first appeared in SLIB release 1.18
 */
#define SLIB_VERSION_MAJOR  1
#define SLIB_VERSION_MINOR  71
#define SLIB_VERSION SLIB_MAKE_VERSION(SLIB_VERSION_MAJOR,SLIB_VERSION_MINOR)

/* SLIB version as a string */
#define __SLIB_VERSION_TEXT(x) #x
#define _SLIB_VERSION_TEXT(x) __SLIB_VERSION_TEXT(x)
#define SLIB_VERSION_TEXT \
    TEXT(_SLIB_VERSION_TEXT(SLIB_VERSION_MAJOR)) TEXT(".") \
    TEXT(_SLIB_VERSION_TEXT(SLIB_VERSION_MINOR))

#endif /* _SLAVA_VERSION_H_ */

/*
 * HISTORY:
 *
 * $Log: s_ver.h,v $
 * Revision 1.56  2010/12/18 16:57:05  slava
 * o version 1.71 (development build)
 *
 * Revision 1.55  2010/12/18 16:53:39  slava
 * o version 1.70
 *
 * Revision 1.54  2010/06/26 10:52:06  slava
 * o version 1.69 (development build)
 *
 * Revision 1.53  2010/06/26 09:27:43  slava
 * o version 1.68
 *
 * Revision 1.52  2010/02/09 08:38:02  slava
 * o version 1.67 (development build)
 *
 * Revision 1.51  2010/01/22 23:24:25  slava
 * o version 1.66
 *
 * Revision 1.50  2009/11/17 00:15:57  slava
 * o version 1.65 (development build)
 *
 * Revision 1.49  2009/11/16 23:54:22  slava
 * o version 1.64
 *
 * Revision 1.48  2009/09/26 19:59:00  slava
 * o version 1.63
 *
 * Revision 1.47  2009/05/23 09:20:50  slava
 * o version 1.62
 *
 * Revision 1.46  2009/04/09 21:07:27  slava
 * o version 1.61 (development build)
 *
 * Revision 1.45  2009/03/25 22:10:08  slava
 * o vesion 1.60
 *
 * Revision 1.44  2008/12/17 12:06:18  slava
 * o version 1.59 (development build)
 *
 * Revision 1.43  2008/12/12 15:17:51  slava
 * o version 1.58
 *
 * Revision 1.42  2008/11/05 14:23:16  slava
 * o version 1.57 (development build)
 *
 * Revision 1.41  2008/11/05 12:18:42  slava
 * o version 1.56
 *
 * Revision 1.40  2008/09/24 12:02:05  slava
 * o version 1.55 (development build)
 *
 * Revision 1.39  2008/09/04 08:25:00  slava
 * o version 1.54
 *
 * Revision 1.38  2008/03/03 11:02:49  slava
 * o version 1.53 (development build)
 *
 * Revision 1.37  2008/03/02 09:46:17  slava
 * o version 1.52
 *
 * Revision 1.36  2007/07/12 19:28:07  slava
 * o version 1.51 (development build)
 *
 * Revision 1.35  2007/06/27 21:54:53  slava
 * o version 1.50
 *
 * Revision 1.34  2007/05/20 22:07:27  slava
 * o fixed SLIB_VERSION_TEXT macro to produce wide string in Unicode build
 *
 * Revision 1.33  2007/05/01 00:36:19  slava
 * o version 1.49 (development build)
 *
 * Revision 1.32  2007/04/29 18:35:10  slava
 * o version 1.48
 *
 * Revision 1.31  2007/02/07 03:56:49  slava
 * o added SLIB_VERSION_TEXT macro
 *
 * Revision 1.30  2006/11/11 17:18:02  slava
 * o version 1.47 (development build)
 *
 * Revision 1.29  2006/11/11 17:14:50  slava
 * o version 1.46
 *
 * Revision 1.28  2006/04/15 18:46:38  slava
 * o version 1.45 (development build)
 *
 * Revision 1.27  2006/04/13 12:22:53  slava
 * o version 1.44
 *
 * Revision 1.26  2005/10/02 01:36:10  slava
 * o version 1.43 (development build)
 *
 * Revision 1.25  2005/10/01 23:22:02  slava
 * o version 1.42
 *
 * Revision 1.24  2005/05/31 06:39:06  slava
 * o version 1.41 (development build)
 *
 * Revision 1.23  2005/05/31 06:13:41  slava
 * o version 1.40
 *
 * Revision 1.22  2005/01/11 23:39:11  slava
 * o version 1.39 (development build)
 *
 * Revision 1.21  2005/01/11 23:34:47  slava
 * o version 1.38
 *
 * Revision 1.20  2004/12/26 18:47:09  slava
 * o version 1.37 (development build)
 *
 * Revision 1.19  2004/12/26 18:35:08  slava
 * o version 1.36
 *
 * Revision 1.18  2004/11/28 02:49:09  slava
 * o version 1.35 (development build)
 *
 * Revision 1.17  2004/11/27 17:00:04  slava
 * o version 1.34
 *
 * Revision 1.16  2004/08/29 13:43:55  slava
 * o version 1.33 (development version)
 *
 * Revision 1.15  2004/08/29 13:28:36  slava
 * o version 1.32
 *
 * Revision 1.14  2004/08/12 01:15:20  slava
 * o version 1.31 (development version)
 *
 * Revision 1.13  2004/08/12 00:14:09  slava
 * o version 1.30
 *
 * Revision 1.12  2004/07/21 01:35:03  slava
 * o version 1.29 (development version)
 *
 * Revision 1.11  2004/07/21 01:29:50  slava
 * o version 1.28
 *
 * Revision 1.10  2004/06/22 06:01:58  slava
 * o version 1.27 (development build)
 *
 * Revision 1.9  2004/06/22 05:50:31  slava
 * o version 1.26
 *
 * Revision 1.8  2004/05/28 13:03:38  slava
 * o version 1.25 (development build)
 *
 * Revision 1.7  2004/05/28 12:08:22  slava
 * o version 1.24
 *
 * Revision 1.6  2004/04/19 08:31:46  slava
 * o version 1.23 (development build)
 *
 * Revision 1.5  2004/04/19 08:11:17  slava
 * o version 1.22
 *
 * Revision 1.4  2004/04/04 16:05:42  slava
 * o bumped version to 1.21
 *
 * Revision 1.3  2004/04/04 15:50:44  slava
 * o version 1.20
 *
 * Revision 1.2  2004/04/01 13:53:34  slava
 * o bumped version to 1.19
 *
 * Revision 1.1  2004/03/25 04:01:51  slava
 * o added SLIB_VERSION macro
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
