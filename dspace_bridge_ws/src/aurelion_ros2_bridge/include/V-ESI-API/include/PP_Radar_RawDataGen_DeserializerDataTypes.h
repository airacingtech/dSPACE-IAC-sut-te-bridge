#pragma once

#include <cstdint>
#include <iostream>
#include <Deserializer/OptixSensorBaseHeader.h>

namespace dSPACE
{
	namespace PPRadarRawDataGenDeserializer
	{
		static constexpr uint32_t FormatIdentifier = 0xABCDEF21;

		//ver 3: Deserializer unification
		//ver 4: Version handling
		static constexpr DeserializerBase::FDeserializerVersion Version =
		{
			4,	// Major
			0	// Minor
		};

#pragma pack(push, 1)
		struct FAdcSample
		{
			float Real = 0.0f;
			float Imag = 0.0f;

			//for unknown reason, clang needs ctors for initializing this type
			inline FAdcSample() = default;
			inline FAdcSample(float Real, float Imag) : Real(Real), Imag(Imag) {}

			friend std::ostream& operator<<(std::ostream& Os, const FAdcSample& Sample)
			{
				Os << Sample.Real << "+" << Sample.Imag << "i";
				return Os;
			}
		};

		struct FRawDataConfig
		{
			uint32_t NumRxas = 0;
			uint32_t NumChirps = 0;
			uint32_t NumSamples = 0;
			bool bReadComplexSamples = true;
		};

		struct FRadarRawDataGenHeader : public DeserializerBase::FOptixSensorBaseHeader
		{
			FRawDataConfig Conf;
		};
#pragma pack(pop)
	}
}