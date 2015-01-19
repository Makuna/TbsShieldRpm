#ifndef TbsShieldRpm_h
#define TbsShieldRpm_h

#include <Arduino.h>

//-------------------------------------------------------------------
// use pattern for cycle read:
//if (ReadSamplesCycles())
//{
//	for (uint8_t index = 0; index < ChannelCount; index++)
//	{
//		if (SampleForCycleReady(index))
//		{
//			int16_t value = SampleAverageKpa100(index);
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
	const static uint32_t kPa1000PerAdcUnit = 54; // 0.05425347 per ADC unit;
	const static uint16_t adcOffset = 20; // 0.1v-4.6v, 

	TbsShieldRpm( uint8_t channelCount );

	void Setup();

    uint8_t ChannelCount() const
    {
        return m_channelCount;
    }

	void ReadSamples()
	{
		ReadAllSamples(true);
	}
    uint8_t ReadSamplesCycle()
	{
		return ReadAllSamples(false);
	}

	void ClearSamples();
	void CalibrateCyclesWithSamples();
	void CalibrateAtZeroWithSamples();


	int32_t SampleMinKpa100( uint8_t channel ) const
	{
		return ConvertToKpa100(m_sampleCalibrate.ips[channel] - SampleMin(channel));
	}
	int32_t SampleMaxKpa100( uint8_t channel ) const
	{
		return ConvertToKpa100(m_sampleCalibrate.ips[channel] - SampleMax(channel));
	}
	int32_t SampleAverageKpa100( uint8_t channel ) const 
	{
		return ConvertToKpa100(m_sampleCalibrate.ips[channel] - SampleAverage(channel));
	}

	
	bool SampleForCycleReady( uint8_t channel ) const
	{
		return m_sampleCycle[channel].IsNewCycle;
	}
	int SampleWidth( uint8_t channel ) const
	{
		return m_sampleCycle[channel].Count;
	}
	int32_t SampleMin( uint8_t channel ) const
	{
		return m_sampleCycle[channel].MinValue;
	}
	int32_t SampleMax( uint8_t channel ) const
	{
		return m_sampleCycle[channel].MaxValue;
	}
	int32_t SampleAverage( uint8_t channel ) const
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
        Sample(uint8_t channelCount)
        {
            ips = new int16_t[channelCount];
        }
        ~Sample()
        {
            delete[] ips;
        }
		int16_t* ips;
	};
  
	struct SampleCycle
	{
		int32_t Sum;
		uint16_t Count;

		int16_t MinValue;
		int16_t MaxValue;

		int16_t TopTrigger;
		int16_t BottomTrigger;

		bool IsWaitingOnUpCurve; // looking for samples going up
		bool IsNewCycle;

		void SetTriggers(  int16_t top, int16_t center, int16_t bottom )
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

		void Include( int16_t value )
		{
			Sum += value;
			Count++;
			MinValue = min(MinValue, value);
			MaxValue = max(MaxValue, value);
		}

		bool Track( int16_t lastValue, int16_t newValue )
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
	int32_t ConvertToKpa100( int32_t adcValue ) const
	{
		// cap readings as adcValue has been offset due to calibration
		adcValue = max(0L, min(1023L, adcValue));
		int32_t kPa100Value = (adcValue * kPa1000PerAdcUnit) / 10L;
		return kPa100Value;
	}

	void InitFilteringOnFirstSample(const Sample& sample);

	int16_t FastLowAmplitudeNoiseFilter(int16_t newInputValue, int16_t priorOutputValue) const;
    uint8_t ReadAllSamples(bool isIgnoringCycles);

	// variables
    const uint8_t m_channelCount;
    
    SampleCycle* m_sampleCycle;
	Sample m_prevOptimal;
	bool m_isFilteringInitialized;

	int16_t m_maxCycle;
	int16_t m_centerCycle;
	int16_t m_minCycle;

	Sample m_sampleCalibrate;  // sensor reading at normal air pressure (no vacume)
	int16_t m_noiseMaxAmplitude; // calculated at normal air pressure
};

#endif