#pragma once

#include <cstdint>
#include <vector>
#include <ostream>
#include <iomanip>

#include <SensorUtilities/OptixSensor/OptixMaterial.h>
#include <SensorUtilities/OptixSensor/SemanticSegmentationIds.h>
#include <Deserializer/OptixSensorBaseHeader.h>

namespace dSPACE
{
	namespace PPUltrasonicPropagationPathsDeserializer
	{
		static constexpr uint32_t FormatIdentifier = 0xABBBBB11;

		//ver 1: initial version
		static constexpr DeserializerBase::FDeserializerVersion Version =
		{
			1,	// Major
			0	// Minor
		};

#pragma pack(push, 1) //pack structs to enable fast std::memcpy from img data without padding risk
		struct FUltrasonicPropagationPathHeader : DeserializerBase::FOptixSensorBaseHeader
		{
			uint32_t NumReceivers;
		};

		// Vector of three floats.
		struct FVec3f
		{
			float X, Y, Z;

			friend std::ostream& operator<<(std::ostream& o, const FVec3f& d)
			{
				o << '[' << d.X << ',' << d.Y << ',' << d.Z << ']';
				return o;
			}
		};

		// Stores a single interaction point of an ultrasonic path.
		struct FCompositeUltrasonicFrameReceivedDataPathHop
		{
			FVec3f Position; //XYZ-Position of this hop in the coordinate system of the receiving RX - Sensor of the ray path, in meters. (Note: Unlike raytracer output, these are cartesian coordinates and not spherical coordinates)
			materialId_t MaterialId; //The material ID of the hit surface for this hop
			FSemanticSegmentationIds GroundTruthData; //Semantic Segmentation IDs of this hop.

			friend std::ostream& operator<<(std::ostream& o, const FCompositeUltrasonicFrameReceivedDataPathHop& d)
			{
				o << "\t\t\tPosition: " << d.Position << '\n';
				o << "\t\t\tMaterial ID: " << d.MaterialId << '\n';
				o << "\t\t\tClass ID: " << d.GroundTruthData.ClassID << '\n';
				o << "\t\t\tInstance ID: " << d.GroundTruthData.InstanceID << '\n';

				return o;
			}
		};
#pragma pack(pop)

		// Stores a single ultrasonic path, including all interaction points.
		struct FCompositeUltrasonicFrameReceivedDataPath
		{
			std::vector<FCompositeUltrasonicFrameReceivedDataPathHop> Hops;	// a list of all reflection points (hops) of this propagation path. Does not include emitter and receiver.
			float SoundIntensityWattPerSquaremeter;							// the sound intensity in Watt per Squaremeter
			float PathLengthMeter;											// the total path length in meter
			uint8_t EmitterId;												// the id of the emitter of the path
			uint8_t ReceiverId;												// the id of the receiver of the path

			friend std::ostream& operator<<(std::ostream& o, const FCompositeUltrasonicFrameReceivedDataPath& d)
			{
				o << "\t\tSource Emitter Id: " << unsigned(d.EmitterId) << '\n';
				o << "\t\tReceiving Receiver Id: " << unsigned(d.ReceiverId) << '\n';
				o << "\t\tSound Intensity: " << d.SoundIntensityWattPerSquaremeter << "W/m2\n";
				o << "\t\tPath Length: " << d.PathLengthMeter << "m\n";
				o << "\t\tHop Count: " << d.Hops.size() << '\n';
				for (size_t i = 0; i < d.Hops.size(); ++i)
				{
					o << "\t\tData Of Hop " << i << '\n';
					o << d.Hops[i] << '\n';
				}
				return o;
			}
		};

		// Stores all ultrasonic data received by a specific receiver.
		struct FCompositeUltrasonicFrameReceivedData
		{
			std::vector<FCompositeUltrasonicFrameReceivedDataPath> Paths;

			friend std::ostream& operator<<(std::ostream& o, const FCompositeUltrasonicFrameReceivedData& d)
			{
				o << "\tPath Count: " << d.Paths.size() << '\n';
				for (size_t i = 0; i < d.Paths.size(); ++i)
				{
					o << "\tData Of Path " << i << '\n';
					o << d.Paths[i] << '\n';
				}
				return o;
			}
		};

		// Stores an entire ultrasonic data frame.
		struct FCompositeUltrasonicFrame
		{
			std::vector<FCompositeUltrasonicFrameReceivedData> ReceivedData;

			friend std::ostream& operator<<(std::ostream& o, const FCompositeUltrasonicFrame& d)
			{
				o << "Receiver Count: " << d.ReceivedData.size() << '\n';

				for (size_t i = 0; i < d.ReceivedData.size(); ++i)
				{
					o << "Data Of Receiver " << i << '\n';
					o << d.ReceivedData[i] << '\n';
				}
				return o;
			}
		};
	};
}