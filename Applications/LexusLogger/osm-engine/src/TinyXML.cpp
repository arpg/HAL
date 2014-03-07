#include <TinyXML.h>

namespace rpg {

	bool TiXmlBase::condenseWhiteSpace = true;

	const int TiXmlBase::utf8ByteTable[256] =
	{
		//	0	1	2	3	4	5	6	7	8	9	a	b	c	d	e	f
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x00
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x10
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x20
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x30
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x40
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x50
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x60
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x70	End of ASCII range
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x80 0x80 to 0xc1 invalid
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0x90
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0xa0
			1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	// 0xb0
			1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	// 0xc0 0xc2 to 0xdf 2 byte
			2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	// 0xd0
			3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	3,	// 0xe0 0xe0 to 0xef 3 byte
			4,	4,	4,	4,	4,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1	// 0xf0 0xf0 to 0xf4 4 byte, 0xf5 and higher invalid
	};

	const char* TiXmlBase::errorString[ TIXML_ERROR_STRING_COUNT ] =
	{
		"No error",
		"Error",
		"Failed to open file",
		"Memory allocation failed.",
		"Error parsing Element.",
		"Failed to read Element name",
		"Error reading Element value.",
		"Error reading Attributes.",
		"Error: empty tag.",
		"Error reading end tag.",
		"Error parsing Unknown.",
		"Error parsing Comment.",
		"Error parsing Declaration.",
		"Error document empty.",
		"Error null (0) or unexpected EOF found in input stream.",
		"Error parsing CDATA.",
		"Error when TiXmlDocument added to document, because TiXmlDocument can only be at the root.",
	};

	// Note tha "PutString" hardcodes the same list. This
	// is less flexible than it appears. Changing the entries
	// or order will break putstring.
	TiXmlBase::Entity TiXmlBase::entity[ NUM_ENTITY ] =
	{
		{ "&amp;",  5, '&' },
		{ "&lt;",   4, '<' },
		{ "&gt;",   4, '>' },
		{ "&quot;", 6, '\"' },
		{ "&apos;", 6, '\'' }
	};

	void TiXmlBase::ConvertUTF32ToUTF8( unsigned long input, char* output, int* length )
	{
		const unsigned long BYTE_MASK = 0xBF;
		const unsigned long BYTE_MARK = 0x80;
		const unsigned long FIRST_BYTE_MARK[7] = { 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC };

		if (input < 0x80)
			*length = 1;
		else if ( input < 0x800 )
			*length = 2;
		else if ( input < 0x10000 )
			*length = 3;
		else if ( input < 0x200000 )
			*length = 4;
		else
			{ *length = 0; return; }	// This code won't covert this correctly anyway.

		output += *length;

		// Scary scary fall throughs.
		switch (*length)
		{
			case 4:
				--output;
				*output = (char)((input | BYTE_MARK) & BYTE_MASK);
				input >>= 6;
			case 3:
				--output;
				*output = (char)((input | BYTE_MARK) & BYTE_MASK);
				input >>= 6;
			case 2:
				--output;
				*output = (char)((input | BYTE_MARK) & BYTE_MASK);
				input >>= 6;
			case 1:
				--output;
				*output = (char)(input | FIRST_BYTE_MARK[*length]);
		}
	}


}
