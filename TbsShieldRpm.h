#ifndef TbsShieldRpm_h
#define TbsShieldRpm_h

#include <Arduino.h>

//-------------------------------------------------------------------
// use pattern for cycle read:
//if (ReadSamplesCycles())
//{
//	for (int index = 0; index < ChannelCount; index++)
//	{
//		if (SampleForCycleReady(index))
//		{
//			int value = SampleAverageKpa100(index);
//		}
//	}
//}
//		
// use pattern for calibrate:
// for some number of ms
//      ClearSamples();
//      ReadSamples();
// then
//      CalibrateWithSamples();
//
//-------------------------------------------------------------------
class TbsShieldRpm
{
public:
	const static int ChannelCount = 4;
	const static unsigned long kPa1000PerAdcUnit = 54; // 0.05425347 per ADC unit;
	const static int adcOffset = 20; // 0.1v-4.6v, 

	TbsShieldRpm();

	void Setup();

	void ReadSamples()
	{
		ReadAllSamples(true);
	}
	int ReadSamplesCycle()
	{
		return ReadAllSamples(false);
	}

	void ClearSamples();
	void CalibrateCyclesWithSamples();
	void CalibrateAtZeroWithSamples();


	long SampleMinKpa100( int channel )
	{
		return ConvertToKpa100(m_sampleCalibrate.ips[channel] - SampleMin(channel));
	}
	long SampleMaxKpa100( int channel )
	{
		return ConvertToKpa100(m_sampleCalibrate.ips[channel] - SampleMax(channel));
	}
	long SampleAverageKpa100( int channel )
	{
		return ConvertToKpa100(m_sampleCalibrate.ips[channel] - SampleAverage(channel));
	}

	
	bool SampleForCycleReady( int channel )
	{
		return m_sampleCycle[channel].IsNewCycle;
	}
	int SampleWidth( int channel )
	{
		return m_sampleCycle[channel].Count;
	}
	long SampleMin( int channel )
	{
		return m_sampleCycle[channel].MinValue;
	}
	long SampleMax( int channel )
	{
		return m_sampleCycle[channel].MaxValue;
	}
	long SampleAverage( int channel )
	{
		int count = m_sampleCycle[channel].Count;
		if (count > 0)
		{
			return m_sampleCycle[channel].Sum / count;
		}
		return 0;
	}
	
private:
	// types
	struct Sample
	{
		int ips[ChannelCount];
	};
  
	struct SampleCycle
	{
		long Sum;
		long Count;

		int MinValue;
		int MaxValue;

		int TopTrigger;
		int BottomTrigger;

		bool IsWaitingOnUpCurve; // looking for samples going up
		bool IsNewCycle;

		void SetTriggers(  int top, int center, int bottom )
		{
			TopTrigger = center + (top - center) * 2 / 3;
			BottomTrigger = center - (center - bottom) * 2 / 3;
		}

		void Clear()
		{
			Sum = 0;
			Count = 0;

			MinValue = 32767;
			MaxValue = -32768;

			IsWaitingOnUpCurve = true;
			IsNewCycle = false;
		}

		void Include( int value )
		{
			Sum += value;
			Count++;
			MinValue = min(MinValue, value);
			MaxValue = max(MaxValue, value);
		}

		bool Track( int lastValue, int newValue )
		{
			bool isValueReady = false;

			if (IsNewCycle)
			{
				Clear();
			}
			
			//if (newValue < 100 || newValue > 923)
			//{
			//	return false; // value ignore as outside sensor range
			//}

			Include( newValue );

			if (IsWaitingOnUpCurve)
			{
				if (newValue > TopTrigger)
				{
					IsWaitingOnUpCurve = false;
				}
			}
			else
			{
				if (newValue < BottomTrigger)
				{
					if (Count > 0)
					{
						isValueReady = true;
					}
					IsNewCycle = true;
				}
			}
			return isValueReady;
		}
	};

	// members
	long ConvertToKpa100( long adcValue )
	{
		// cap readings as adcValue has been offset due to calibration
		adcValue = max(0L, min(1023L, adcValue));
		long kPa100Value = (adcValue * kPa1000PerAdcUnit) / 10L;
		return kPa100Value;
	}

	void InitFilteringOnFirstSample(const Sample& sample);

	int FastLowAmplitudeNoiseFilter(int newInputValue, int priorOutputValue);
	int ReadAllSamples(bool isIgnoringCycles);

	// variables
	SampleCycle m_sampleCycle[ChannelCount];
	Sample m_prevOptimal;
	bool m_isFilteringInitialized;

	int m_maxCycle;
	int m_centerCycle;
	int m_minCycle;

	Sample m_sampleCalibrate;  // sensor reading at normal air pressure (no vacume)
	int m_noiseMaxAmplitude; // calculated at normal air pressure
};

#endif