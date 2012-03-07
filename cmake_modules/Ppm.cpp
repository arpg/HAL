/*
 * Ppm.cpp
 *
 *  Created on: Feb 6, 2012
 *      Author: nima
 */

#include <iostream>
//#include <alsa/asoundlib.h>
#include <ao/ao.h>
#include "MochaException.h"
#include <boost/thread.hpp>
#include <boost/signals2/mutex.hpp>
#include "AudioBuffer.h"
#include "Ppm.h"


namespace Mochaccino {

Ppm::Ppm(int rate /*= 192000*/, int nAudioChannels /* = 2 */,int nPwmChannels /* = 6 */, int amplitude /* = 32760 */)
{
    m_bFlag = 0;

	m_nRate = rate;
	m_nAudioChannels = nAudioChannels;
	m_nPwmChannels = nPwmChannels;
	m_nAmplitude = amplitude;

	//calculate the number of frames
    //m_nFrames = (int)(0.0220 * rate * m_nAudioChannels); // 22 or 22.5ms in samples, rounded up
    m_nFrames = (int)(0.0090 * rate * m_nAudioChannels); // 22 or 22.5ms in samples, rounded up
    m_dMicrosec = rate / 10000.0; // 192 = 1ms, 19.2 = 0.1ms or 1mis @ 192khz

	//initialize the pwm and frame buffers
	m_pChannels = new double[m_nPwmChannels];
	//zero the channels
	for(int i = 0 ; i < m_nPwmChannels; i++)
	{
        m_pChannels[i] = 50;
	}

	m_pFrameBuffer = new char[m_nFrames*nAudioChannels*SHORT_BYTES];

	m_bIsStarted = false;
}

Ppm::~Ppm() {
	delete m_pChannels;
	delete m_pFrameBuffer;

	if(m_bIsStarted)
		Stop();


}

void Ppm::Start()
{
	//do not start again
	if( m_bIsStarted == true )
		throw MochaException("The PPM thread has already started.");

	//first init the driver
	Initialize();



	//now call the thread function
	m_pAudioThread = new boost::thread(Ppm::ThreadHandler, this);

	m_bIsStarted = true;
}

void Ppm::Stop()
{
	if( m_bIsStarted == false )
		throw MochaException("No PPM thread is running.");

	//stop the audio thread
	m_pAudioThread->interrupt();
	m_pAudioThread->join();

	m_bIsStarted = false;

	//clear the handles
    //snd_pcm_drain(m_pHandle);
    //snd_pcm_close(m_pHandle);
}

void Ppm::SetChannel(int id, int value)
{
    //try to lock the thread
    lock();

    if(value>100)
        value = 100;
    else if( value < 0 )
        value = 0;

    //set the appropriate channel (the user must ensure this is not out of bounds)
    m_pChannels[id] = value;
    m_bFlag = true;

    unlock();
}

void Ppm::GetPpm(char *buffer, int *pLength)
{
	//create the helper class
	AudioBuffer period(m_nFrames, 2);

	//add the stop/start section (4 microseconds)
    period.AppendSamples(-m_nAmplitude, (int)(4 * m_dMicrosec));

	//append the channel data
	for(int i = 0; i < m_nPwmChannels ; i++)
	{
		//add the PPM base
        period.AppendSamples(m_nAmplitude, (int)(7 * m_dMicrosec));
		//add the PPM signal
        lock();
        period.AppendSamples(m_nAmplitude, (int)((m_pChannels[i] * 0.75 / 100) * 10 * m_dMicrosec));
        unlock();
		//add the PPM signal end
        period.AppendSamples(-m_nAmplitude, (int)(4 * m_dMicrosec));
	}

	//Complete the PPM signal with leading blank
    period.FillRemainder(m_nAmplitude);

	//get the data from the helper class
    period.CopyData(buffer,pLength);
}


void Ppm::Initialize()
{
    ao_sample_format format;

    ao_initialize();

    //setup the default driver
    int default_driver = ao_default_driver_id();

    //prepare the driver
    memset(&format, 0, sizeof(format));
    format.bits = 16;
    format.channels = 2;
    format.rate = m_nRate;
    format.byte_format = AO_FMT_LITTLE;

    //open the driver
    ao_option option;
    option.key = "buffer_time";
    option.value = "0";
    option.next = NULL;
    m_pDevice = ao_open_live(default_driver, &format, &option);
    if (m_pDevice == NULL) {
        fprintf(stderr, "Error opening device.\n");
        return;
    }
}

void Ppm::ThreadHandler(Ppm *arg)
{
	arg->ThreadFunction();
}

void Ppm::ThreadFunction()
{
	while (1)
    {
		boost::this_thread::interruption_point();

        int length;
        GetPpm(m_pFrameBuffer, &length);

        //play whatever the GetPpm function gave us
        ao_play(m_pDevice, m_pFrameBuffer,length);

        //sleep while the sound card plays this. This specifies the
        //frequency of the ppm period
        usleep(20000);
	}
}



} /* namespace Mochaccino */
