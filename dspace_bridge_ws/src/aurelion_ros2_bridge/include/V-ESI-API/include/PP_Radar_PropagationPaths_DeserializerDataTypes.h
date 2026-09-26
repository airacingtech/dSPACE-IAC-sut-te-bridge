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
	namespace PPRadarPropagationPathsDeserializer
	{
		static constexpr uint32_t FormatIdentifier = 0xABCDEF22;

		//ver 8: Deserializer unification
		//ver 9: Version handling
		static constexpr DeserializerBase::FDeserializerVersion Version =
		{
			9,	// Major
			0	// Minor
		};

#pragma pack(push, 1) //pack structs to enable fast std::memcpy from img data without padding risk
		struct FRadarPropagationPathHeader : public DeserializerBase::FOptixSensorBaseHeader
		{
			uint32_t NumRxas;
		};

		// Vector of three floats.
		struct FVec3f
		{
			float X, Y, Z;

			friend std::ostream& operator<<(std::ostream& Os, const FVec3f& Vec)
			{
				Os << '[' << Vec.X << ',' << Vec.Y << ',' << Vec.Z << ']';
				return Os;
			}
		};

		// A single interaction point of a radar path.
		struct FCompositeRadarFrameRxaDataPathHop
		{
			FVec3f Position; //XYZ-Position of this hop in the coordinate system of the receiving RX - Antenna of the ray path, in meters. (Note: Unlike raytracer output, these are cartesian coordinates and not spherical coordinates)
			FVec3f Velocity; //XYZ-Velocity of this hop relative to the receiving antenna in meters/second. Please note that the values are calculated for the antenna coordinate system.
			materialId_t MaterialId; //The material ID of the hit surface for this hop
			FSemanticSegmentationIds GroundTruthData; //Semantic Segmentation IDs of this hop.

			friend std::ostream& operator<<(std::ostream& Os, const FCompositeRadarFrameRxaDataPathHop& Hop)
			{
				Os << "\t\t\tPosition: " << Hop.Position << '\n';
				Os << "\t\t\tVelocity: " << Hop.Velocity << '\n';
				Os << "\t\t\tMaterial ID: " << Hop.MaterialId << '\n';
				Os << "\t\t\tClass ID: " << Hop.GroundTruthData.ClassID << '\n';
				Os << "\t\t\tInstance ID: " << Hop.GroundTruthData.InstanceID << '\n';

				return Os;
			}
		};
#pragma pack(pop)

		struct FComplexFloat
		{
			float Real, Imag;

			friend std::ostream& operator<<(std::ostream& Os, const FComplexFloat& Value)
			{
				Os << Value.Real << "+" << Value.Imag << "i";
				return Os;
			}
		};

		struct FComplexEField
		{
			FComplexFloat X, Y, Z;

			friend std::ostream& operator<<(std::ostream& Os, const FComplexEField& Efield)
			{
				Os << "(";
				Os << Efield.X;
				Os << " | ";
				Os << Efield.Y;
				Os << " | ";
				Os << Efield.Z;
				Os << ")";

				return Os;
			}
		};

		struct FEFieldsFromExcitation
		{
			FComplexEField EFieldFromHExcitation, EFieldFromVExcitation;

			friend std::ostream& operator<<(std::ostream& Os, const FEFieldsFromExcitation& Efields)
			{
				Os << "[";
				Os << Efields.EFieldFromHExcitation;
				Os << " - ";
				Os << Efields.EFieldFromVExcitation;
				Os << "]";
				return Os;
			}

		};

		// Stores a single radar path, including all interaction points.
		struct FCompositeRadarFrameRxaDataPath
		{
			std::vector<FCompositeRadarFrameRxaDataPathHop> Hops; //A list of all reflection points (hops) of this propagation path. Does not include TX- and RX-Antenna. 
			FEFieldsFromExcitation EFields;
			float Length; //see raytracer channel impulse response output.
			float DopplerSpeed; //see raytracer channel impulse response output.
			uint8_t SourceTxAntennaId; //see raytracer channel impulse response output.

			friend std::ostream& operator<<(std::ostream& Os, const FCompositeRadarFrameRxaDataPath& Path)
			{
				Os << "\t\teFields: " << Path.EFields << '\n';
				Os << "\t\tPath Length: " << Path.Length << '\n';
				Os << "\t\tTotal Doppler Speed: " << Path.DopplerSpeed << '\n';
				Os << "\t\tHop Count: " << Path.Hops.size() << '\n';
				for (size_t i = 0; i < Path.Hops.size(); ++i)
				{
					Os << "\t\tHop Data " << i << '\n';
					Os << Path.Hops[i] << '\n';
				}
				return Os;
			}
		};

		// Stores all radar data received by a specific RX-Antenna.
		struct FCompositeRadarFrameRxaData
		{
			std::vector<FCompositeRadarFrameRxaDataPath> Paths;

			friend std::ostream& operator<<(std::ostream& Os, const FCompositeRadarFrameRxaData& Data)
			{
				Os << "\tPath Count: " << Data.Paths.size() << '\n';
				for (size_t i = 0; i < Data.Paths.size(); ++i)
				{
					Os << "\tPath Data " << i << '\n';
					Os << Data.Paths[i] << '\n';
				}
				return Os;
			}
		};

		// Stores an entire radar data frame.
		struct FCompositeRadarFrame
		{
			std::vector<FCompositeRadarFrameRxaData> RxaData;

			friend std::ostream& operator<<(std::ostream& Os, const FCompositeRadarFrame& Frame)
			{
				Os << "RXA Count: " << Frame.RxaData.size() << '\n';

				for (size_t i = 0; i < Frame.RxaData.size(); ++i)
				{
					Os << "RXA Data " << i << '\n';
					Os << Frame.RxaData[i] << '\n';
				}
				return Os;
			}
		};
	}
}
