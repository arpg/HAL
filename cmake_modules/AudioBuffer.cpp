/*
* AudioBuffer.cpp
*
*  Created on: Feb 6, 2012
*      Author: nima
*/
#include <stdio.h>
#include <string.h>
#include "AudioBuffer.h"

using namespace std;

namespace Mochaccino {

AudioBuffer::AudioBuffer(int frameCount, int channelCount)
{
	m_nTotalLength = frameCount * channelCount;
	m_pData = new short[m_nTotalLength];
	m_nChannelCount = channelCount;
	m_nCurrentLength = 0;
}
AudioBuffer::~AudioBuffer()
{
	delete m_pData;
}

void AudioBuffer::AppendSamples(int amplitude, int count)
{
	for( int i = 0 ; i < count ; i++)
	{
		for(int j = 0 ; j < m_nChannelCount ; j++ )
		{
			m_pData[m_nCurrentLength] = amplitude;
			m_nCurrentLength++;
		}
	}
}

void AudioBuffer::CopyData(char *pData, int *length)
{
	//each short is 2 bytes
	*length = m_nCurrentLength*2;

	//copy the data over into the array and return the length
	memcpy(pData,(char*)m_pData,*length);
}

void AudioBuffer::FillRemainder(int amplitude)
{
	for( int i = m_nCurrentLength ; i < m_nTotalLength ; i++)
	{
		m_pData[m_nCurrentLength] = amplitude;
		m_nCurrentLength++;
	}
}
}
