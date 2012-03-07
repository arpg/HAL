/*
 * Ppm.h
 *
 *  Created on: Feb 6, 2012
 *      Author: nima
 */

#ifndef PPM_H_
#define PPM_H_

#include <boost/thread.hpp>

namespace Mochaccino {

class Ppm : public boost::mutex{
public:
	void Start();
	void Stop();

	Ppm(int rate = 192000, int nAudioChannels = 2,int nPwmChannels = 6, int amplitude = 32760);

	virtual ~Ppm();
    void SetChannel(int id, int value);

private:
    ao_device *m_pDevice;
	static const short SHORT_BYTES = 2;
	unsigned int m_nRate;
	int m_nAudioChannels;
	int m_nPwmChannels;
	int m_nAmplitude;
    int m_nFrames;
	double m_dMicrosec;
    double *m_pChannels;
	char *m_pFrameBuffer;
	bool m_bIsStarted;
    bool m_bFlag;

	boost::thread *m_pAudioThread;

    int m_nWindowCounter;


	void Initialize();
	static void ThreadHandler(Ppm *arg);
	void ThreadFunction();
	void GetPpm(char *buffer, int *pLength);

};

} /* namespace Mochaccino */
#endif /* PPM_H_ */
