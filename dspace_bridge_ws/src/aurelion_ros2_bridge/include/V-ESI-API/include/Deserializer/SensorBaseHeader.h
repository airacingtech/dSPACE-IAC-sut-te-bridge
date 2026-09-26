#pragma once

#include <cstdint>

namespace dSPACE
{
	namespace DeserializerBase
	{

#pragma pack(push, 1) 
		struct FDeserializerVersion
		{
			uint16_t Major;
			uint16_t Minor;
		};

		struct FSensorBaseHeader
		{
			uint32_t FormatIdentifier;	// Unique deserializer identifier

			FDeserializerVersion Version;			// Version to match serialized data to correct deserializer.

			double SimulationTime;		// Simulation time in s
		};
#pragma pack(pop)
	}
}