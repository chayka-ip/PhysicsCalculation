// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ConstantsHM.h"
#include "Engine/DataAsset.h"
#include "PhysicsCache/PhysCacheDataStructs.h"
#include "PhysicsCacheParams_DataAsset.generated.h"

/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UPhysicsCacheParams_DataAsset : public UDataAsset
{
	GENERATED_BODY()
	
public:
	UPhysicsCacheParams_DataAsset()
	{
		StartLinearVelocity.Speed.Min = 0.0f;
		StartLinearVelocity.Speed.Max = 220.0f;
		StartLinearVelocity.Speed.Delta = 10.0f;

		StartLinearVelocity.AngleXZ.Min = 0.0f;
		StartLinearVelocity.AngleXZ.Max = 80.0f;
		StartLinearVelocity.AngleXZ.Delta = 2.0f;

		//

		StartAngularVelocity.Speed.Min = 0.0f;
		StartAngularVelocity.Speed.Max = 2000.0f;
		StartAngularVelocity.Speed.Delta = 50.0f;
		
	};
	
public:

	UPROPERTY(EditAnywhere)
	FPhysCacheTimeData TimeData;
	
	// Distance in cm
	UPROPERTY(EditAnywhere)
	FPhysCacheDistanceLimits DistanceLimits;
	
	// Speed: km/h; angle: degrees
	UPROPERTY(EditAnywhere)
	FPhysCacheStartVelocity StartLinearVelocity;

	// Speed: Rate/minute; angle: degrees
	UPROPERTY(EditAnywhere)
	FPhysCacheStartVelocity StartAngularVelocity;

	// todo: clamp velocity, location by delta change -> to prevent junk data calculation
	// todo: make limit for linear-angular velocities combined; if something is high - decrease other 

public:
	static float GetLinearVelocityScale(){return KmHourToCmSec;}
	static float GetAngularVelocityScale(){return RpMToRadSec;}
	
public:
	FVector GetLinearVelocityFromIterations(int SpeedIndex, int XZAngleIndex, int XYAngleIndex) const;
	FVector GetAngularVelocityFromIterations(int SpeedIndex, int XZAngleIndex, int XYAngleIndex) const;
	TArray<FVector> GetAllStartLinearVelocities() const {return StartLinearVelocity.GetAllVectors(GetLinearVelocityScale());}
	TArray<FVector> GetAllStartAngularVelocities() const {return StartAngularVelocity.GetAllVectors(GetAngularVelocityScale());}
	TArray<FDetailedVector> GetAllStartLinearVelocitiesDetailed() const {return StartLinearVelocity.GetAllVectorsDetailed(GetLinearVelocityScale());} 
	TArray<FDetailedVector> GetAllStartAngularVelocitiesDetailed() const {return StartAngularVelocity.GetAllVectorsDetailed(GetAngularVelocityScale());} 
};
