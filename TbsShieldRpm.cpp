#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <Arduino.h> 
#include "TbsShieldRpm.h"

const int ips1Pin = A0;
const int ips2Pin = A1;
const int ips3Pin = A2;
const int ips4Pin = A3;

/// <summary>
///  This is the smoothing factor used for the 1st order discrete Low-Pass filter
///  The cut-off frequency fc = fs * K/(2*PI*(1-K))
/// </summary>
const int LowPassFilterCoefDiv = 10; // was 10n// 0.1; original was *, due to int now /

TbsShieldRpm::TbsShieldRpm() :
		m_isFilteringInitialized(false),
		m_noiseMaxAmplitude(0)
{
}

void TbsShieldRpm::Setup()
{
	// sensor setup
	analogRead(ips1Pin);
	analogRead(ips2Pin);
	analogRead(ips3Pin);
	analogRead(ips4Pin);

	ClearSamples();
}

int TbsShieldRpm::ReadAllSamples(bool isIgnoringCycles)
{
	int channelsReady = 0;
	Sample sample;
    Sample optimalFiltered;

	// read values, back to back for close spatial time
    //
    sample.ips[0] = analogRead(ips1Pin);
    sample.ips[1] = analogRead(ips2Pin);
    sample.ips[2] = analogRead(ips3Pin);
    sample.ips[3] = analogRead(ips4Pin);

	// filtering
	//
	if (!m_isFilteringInitialized)
	{
		InitFilteringOnFirstSample(sample);
	}

	// calc optimal filtered
	for (int channelIndex = 0; channelIndex < ChannelCount; ++channelIndex)
	{
		int value = FastLowAmplitudeNoiseFilter(sample.ips[channelIndex], m_prevOptimal.ips[channelIndex]);

		optimalFiltered.ips[channelIndex] = value;

		if (isIgnoringCycles)
		{
			// just track min and max
			m_sampleCycle[channelIndex].Include( value );
		}
		else
		{
			if (m_sampleCycle[channelIndex].Track( m_prevOptimal.ips[channelIndex], value ))
			{
				channelsReady++;
			}
		}
	}

	m_prevOptimal = optimalFiltered;
	return channelsReady;
}

void TbsShieldRpm::ClearSamples()
{
	m_isFilteringInitialized = false;
	for (int channelIndex = 0; channelIndex < ChannelCount; ++channelIndex)
	{
		m_sampleCycle[channelIndex].Clear();
		m_sampleCycle[channelIndex].SetTriggers(m_maxCycle, m_centerCycle, m_minCycle);
	}
}

void TbsShieldRpm::InitFilteringOnFirstSample(const Sample& sample)
{
	m_prevOptimal = sample;

	m_isFilteringInitialized = true;
}

void TbsShieldRpm::CalibrateAtZeroWithSamples()
{
	int noiseRange = 0;
	for (int channelIndex = 0; channelIndex < ChannelCount; ++channelIndex)
	{
		m_sampleCalibrate.ips[channelIndex] = SampleAverage(channelIndex);
		noiseRange = max( noiseRange, SampleMax(channelIndex) - SampleMin(channelIndex) );
	}
	m_noiseMaxAmplitude = noiseRange / 2;
}

void TbsShieldRpm::CalibrateCyclesWithSamples()
{
	long sum = 0;

	m_maxCycle = 32767; // looking for the lowest of the max
	m_minCycle = -32768; // looking for the highest of the min

	for (int channelIndex = 0; channelIndex < ChannelCount; ++channelIndex)
	{
		sum += SampleAverage(channelIndex);
		m_maxCycle = min( m_maxCycle, SampleMax(channelIndex));
		m_minCycle = max( m_minCycle, SampleMin(channelIndex));
	}
	m_centerCycle = sum / ChannelCount;
}

/// <summary>
/// discrete low-magnitude fast low-pass filter used to remove noise from raw accelerometer while allowing fast trending on high amplitude changes
/// </summary>
/// <param name="newInputValue">New input value (latest sample)</param>
/// <param name="priorOutputValue">The previous (n-1) output value (filtered, one sampling period ago)</param>
/// <returns>The new output value</returns>
int TbsShieldRpm::FastLowAmplitudeNoiseFilter(int newInputValue, int priorOutputValue)
{
    int newOutputValue = newInputValue;
    int diff = newInputValue - priorOutputValue;
    if (abs(diff) < m_noiseMaxAmplitude)
    { 
		// Simple low-pass filter
        newOutputValue = priorOutputValue + diff / LowPassFilterCoefDiv;
    }
    return newOutputValue;
}