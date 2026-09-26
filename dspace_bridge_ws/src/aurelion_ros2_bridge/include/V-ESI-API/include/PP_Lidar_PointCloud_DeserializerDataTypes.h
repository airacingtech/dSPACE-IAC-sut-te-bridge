#pragma once

#include <cstdint>
#include <vector>

#include <SensorUtilities/OptixSensor/SemanticSegmentationIds.h>
#include <Deserializer/OptixSensorBaseHeader.h>

namespace dSPACE
{
	namespace PPLidarPointCloudDeserializer
	{
		static constexpr uint32_t FormatIdentifier = 0xBCDEF20;

		//ver 4: Deserializer unification
		//ver 5: Version handling
		//ver 5.1: Fix header parsing
		static constexpr DeserializerBase::FDeserializerVersion Version =
		{
			5,	// Major
			1	// Minor
		};

#pragma pack(push, 1)
		struct FPointCloudLidarPoint
		{
			float Normal[3];			// normal vector of surface hit, normalized
			float Azimuth;				// in degree, clockwise (!)
			float Elevation;			// in degree, pointing up
			float Distance;				// in m
			float RelativeVelocity;	// in m/s, projected in ray direction
			float Reflectivity;			// in [0, 1]
			float OpticalPower;		// rescaled to [0,1]. Multiplication with transmit power yields receive power.
			float TimeOffset;			// in ms, offset from simulation time

			FSemanticSegmentationIds GroundTruthData; //Semantic Segmentation IDs of this point.
			uint32_t RayID;
			uint16_t MaterialID;
		};
#pragma pack(pop)

#pragma pack(push, 1)
		struct FPointCloudOutputModes
		{
			bool bNormal;
			bool bAzimuth;
			bool bElevation;
			bool bDistance;
			bool bRelativeVelocity;
			bool bReflectivity;
			bool bOpticalPower;
			bool bTimeOffset;

			bool bMaterialID;
			bool bClassID;
			bool bInstanceID;
			bool bRayID;

			size_t GetLidarPointSize() const
			{
				size_t size = 0;

				size += bNormal ? sizeof(FPointCloudLidarPoint::Normal) : 0;
				size += bAzimuth ? sizeof(FPointCloudLidarPoint::Azimuth) : 0;
				size += bElevation ? sizeof(FPointCloudLidarPoint::Elevation) : 0;
				size += bDistance ? sizeof(FPointCloudLidarPoint::Distance) : 0;
				size += bTimeOffset ? sizeof(FPointCloudLidarPoint::TimeOffset) : 0;
				size += bRelativeVelocity ? sizeof(FPointCloudLidarPoint::RelativeVelocity) : 0;
				size += bReflectivity ? sizeof(FPointCloudLidarPoint::Reflectivity) : 0;
				size += bOpticalPower ? sizeof(FPointCloudLidarPoint::OpticalPower) : 0;
				size += bMaterialID ? sizeof(FPointCloudLidarPoint::MaterialID) : 0;
				size += bClassID ? sizeof(FPointCloudLidarPoint::GroundTruthData.ClassID) : 0;
				size += bInstanceID ? sizeof(FPointCloudLidarPoint::GroundTruthData.InstanceID) : 0;
				size += bRayID ? sizeof(FPointCloudLidarPoint::RayID) : 0;

				return size;
			}
		};
#pragma pack(pop)

#pragma pack(push, 1) 
		struct FLidarPointCloudHeader : DeserializerBase::FOptixSensorBaseHeader
		{
			uint32_t Checksum;								// CRC32 checksum if enabled, otherwise zero.
			FPointCloudOutputModes Output;					// Defines what kind of information is valid to read per point.
			uint32_t NumPoints;								// Number of points within frame.
			uint32_t MaxNumOfPoints;						// Maximal possible number of points under current configuration (max number of rays and returns per ray).
		};
#pragma pack(pop)

		struct FCompositeLidarFrame
		{
			std::vector<FPointCloudLidarPoint> LidarPoints;
		};
	}
}