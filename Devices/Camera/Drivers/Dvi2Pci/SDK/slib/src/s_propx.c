/*
 * $Id: s_propx.c,v 1.2 2005/02/20 20:31:30 slava Exp $
 *
 * Copyright (C) 2000-2005 by Slava Monich
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

#include "s_xml.h"
#include "s_prop.h"

/**
 * Read properties from the XML file specified by name. This functions
 * has been extracted from s_prop.c in order to avoid having to link with 
 * the XML parser unless we actually parse some XML.
 */
Bool PROP_LoadXML(Prop * prop, Str fname, IODesc io)
{
    Bool success = False;
    File * in = FILE_Open(fname, READ_TEXT_MODE, io);
    PROP_Clear(prop);
    if (in) {
        XMLTag * root = XML_Handle(in, PropRootTag, PROP_RootCB, prop);
        if (root) {
            success = True;
            root->endTag(root, NULL);
        }
        FILE_Close(in);
    } else {
        TRACE1("cannot open file %s\n",fname);
    }
    return success;
}

/*
 * HISTORY:
 *
 * $Log: s_propx.c,v $
 * Revision 1.2  2005/02/20 20:31:30  slava
 * o porting SLIB to EPOC gcc compiler
 *
 * Revision 1.1  2003/01/05 17:03:14  slava
 * o support for reading properties from an XML file. The PROP_LoadXML
 *   function has to be in a separate file to avoid having to link the
 *   XML parser even if we are not doing any XML parsing.
 *
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
