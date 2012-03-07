/*
 * AudioBuffer.h
 *
 *  Created on: Feb 6, 2012
 *      Author: nima
 */

#ifndef AUDIOBUFFER_H_
#define AUDIOBUFFER_H_

namespace Mochaccino {

class AudioBuffer {
public:
	AudioBuffer(int frameCount, int channelCount);
	void AppendSamples(int amplitude, int count);
	void CopyData(char *pData, int *length);
	void FillRemainder(int amplitude);
	virtual ~AudioBuffer();


private:
	int m_nCurrentLength;
	int m_nChannelCount;
	int m_nTotalLength;
	short *m_pData;
};

} /* namespace Mochaccino */
#endif /* AUDIOBUFFER_H_ */
