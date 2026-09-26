#pragma once

#include <cstdint>
#include "SensorBaseHeader.h"

namespace dSPACE
{
	namespace DeserializerBase
	{
#pragma pack(push, 1)
		struct FOptixSensorBaseHeader : FSensorBaseHeader{
			uint32_t DataLength;
		};
#pragma pack(pop)
	}
}
