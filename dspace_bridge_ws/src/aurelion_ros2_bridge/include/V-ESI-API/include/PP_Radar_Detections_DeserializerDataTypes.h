#pragma once

#include "PP_Radar_Detections_Base_Detection.h"
#include <Deserializer/OptixSensorBaseHeader.h>

namespace dSPACE
{
	namespace PPRadarDetectionsDeserializer
	{
		static constexpr uint32_t FormatIdentifier = 0xABCDEF20;

		//ver 4: Deserializer unification
		//ver 5: Instance IDs
		//ver 6: Simulation time + version handling
		static constexpr DeserializerBase::FDeserializerVersion Version =
		{
			6,	// Major
			0	// Minor
		};

#pragma pack(push, 1)
		struct FDetection : public PPRadarDetectionsBase::FDetection
		{
			//just use default fields.
		};

		struct FRadarDetectionsHeader : public DeserializerBase::FOptixSensorBaseHeader
		{
			uint16_t NumDetections{};
		};
#pragma pack(pop)
	}
}