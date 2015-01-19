#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <Arduino.h> 
#include "TbsShieldRpm.h"

const uint8_t ips1Pin = A0;
const uint8_t ips2Pin = A1;
const uint8_t ips3Pin = A2;
const uint8_t ips4Pin = A3;

/// <summary>
///  This is the smoothing factor used for the 1st order discrete Low-Pass filter
///  The cut-off frequency fc = fs * K/(2*PI*(1-K))
/// </summary>
const int16_t LowPassFilterCoefDiv = 10; // was 10n// 0.1; original was *, due to int now /

TbsShieldRpm::TbsShieldRpm(uint8_t channelCount) :
		m_isFilteringInitialized(false),
		m_noiseMaxAmplitude(0),
        m_channelCount(channelCount),
        m_prevOptimal(channelCount),
        m_sampleCalibrate(channelCount)
{
    m_sampleCycle = new SampleCycle[channelCount];
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

uint8_t TbsShieldRpm::ReadAllSamples(bool isIgnoringCycles)
{
    uint8_t channelsReady = 0;
    Sample sample(m_channelCount);
    Sample optimalFiltered(m_channelCount);

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
	for (uint8_t channelIndex = 0; channelIndex < m_channelCount; ++channelIndex)
	{
		int16_t value = FastLowAmplitudeNoiseFilter(sample.ips[channelIndex], m_prevOptimal.ips[channelIndex]);

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
	for (uint8_t channelIndex = 0; channelIndex < m_channelCount; ++channelIndex)
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
	int16_t noiseRange = 0;
	for (uint8_t channelIndex = 0; channelIndex < m_channelCount; ++channelIndex)
	{
		m_sampleCalibrate.ips[channelIndex] = SampleAverage(channelIndex);
		noiseRange = max( noiseRange, SampleMax(channelIndex) - SampleMin(channelIndex) );
	}
	m_noiseMaxAmplitude = noiseRange / 2;
}

void TbsShieldRpm::CalibrateCyclesWithSamples()
{
	int32_t sum = 0;

	m_maxCycle = 32767; // looking for the lowest of the max
	m_minCycle = -32768; // looking for the highest of the min

	for (uint8_t channelIndex = 0; channelIndex < m_channelCount; ++channelIndex)
	{
		sum += SampleAverage(channelIndex);
		m_maxCycle = min( m_maxCycle, SampleMax(channelIndex));
		m_minCycle = max( m_minCycle, SampleMin(channelIndex));
	}
	m_centerCycle = sum / m_channelCount;
}

/// <summary>
/// discrete low-magnitude fast low-pass filter used to remove noise from raw accelerometer while allowing fast trending on high amplitude changes
/// </summary>
/// <param name="newInputValue">New input value (latest sample)</param>
/// <param name="priorOutputValue">The previous (n-1) output value (filtered, one sampling period ago)</param>
/// <returns>The new output value</returns>
int16_t TbsShieldRpm::FastLowAmplitudeNoiseFilter(int16_t newInputValue, int16_t priorOutputValue) const
{
    int16_t newOutputValue = newInputValue;
    int16_t diff = newInputValue - priorOutputValue;
    if (abs(diff) < m_noiseMaxAmplitude)
    { 
		// Simple low-pass filter
        newOutputValue = priorOutputValue + diff / LowPassFilterCoefDiv;
    }
    return newOutputValue;
}