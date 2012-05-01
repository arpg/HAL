/*
 * $Id: s_md.c,v 1.3 2009/04/09 21:17:17 slava Exp $
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

#include "s_mdp.h"

/*==========================================================================*
 *              M E S S A G E    D I G E S T
 *==========================================================================*/

/**
 * Re-initializes the digest. After this function returns, the digest is ready
 * for new updates
 */
void DIGEST_Init(Digest * d)
{
    d->type->init(d);
    d->flags = MD_INITIAL;
}

/**
 * Returns the name of the digest algorithm
 */
Str DIGEST_Name(const Digest * d)
{
    return d->type->name;
}

/**
 * Returns the size of the message digest buffer. This is the miniumal size of
 * the buffer passed to DIGEST_Finish
 */
int DIGEST_Size(const Digest * d)
{
    return d->type->size;
}

/**
 * Updates the digest with new data.
 */
void DIGEST_Update(Digest * d, const void * data, size_t size)
{
    ASSERT(!(d->flags & MD_FINISHED));
    d->type->update(d, data, size);
}

/**
 * Finish the digest and return the result. The size of the output buffer
 * must not be less than the value returned by the DIGEST_Size function.
 */
void DIGEST_Finish(Digest * d, void * out)
{
    ASSERT(!(d->flags & MD_FINISHED));
    d->type->finish(d, out);
    d->flags |= MD_FINISHED;
}

/**
 * Deletes the digest object.
 */
void DIGEST_Delete(Digest * d)
{
    if (d) d->type->free(d);
}

/*
 * HISTORY:
 *
 * $Log: s_md.c,v $
 * Revision 1.3  2009/04/09 21:17:17  slava
 * o user size_t instead of int where it's appropriate
 *
 * Revision 1.2  2004/07/31 18:14:19  slava
 * o added DIGEST_Name function
 *
 * Revision 1.1  2004/07/29 17:09:37  slava
 * o message digest framework
 *
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
