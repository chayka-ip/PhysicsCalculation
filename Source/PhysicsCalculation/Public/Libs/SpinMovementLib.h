// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/PhysTransform.h"
#include "HMStructs/CustomVectorCurve.h"
#include "ImpulseDistribution/ImpulseReconstructed.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "PhysicsCache/BallLaunchCache.h"
#include "SpinMovementLib.generated.h"

class UAdvancedPhysicsComponent;
/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API USpinMovementLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	static FBallLaunchCache_Data CalculateBallLaunchCache(UAdvancedPhysicsComponent* PC);
	static void CalculateDataFromLaunchParams(UAdvancedPhysicsComponent* PC, const FBallLaunchParams& LaunchParams, FBallLaunchCache_Data& OutData);
	static FImpulseReconstructed GetImpulseFromBallLaunchParams(UAdvancedPhysicsComponent* PC, const FBallLaunchParams& P, FVector COM, FVector BaseVector=FVector::ForwardVector);
	static FCustomVectorCurve CalculateSpinTrajectory(UAdvancedPhysicsComponent* Obj, const FImpulseReconstructed& ImpulseData, FVector COM, float TimeStep, int NumSteps, FPhysTransform& TLaunch);

	UFUNCTION(BlueprintCallable)
	static FBallLaunchVerticalDistribution CalculateVerticalDistributionFromTrajectory(UAdvancedPhysicsComponent* PC, const FCustomVectorCurve& Curve);
	
	UFUNCTION(BlueprintCallable)
	static FCustomVectorCurve CalculateSpinTrajectory(UAdvancedPhysicsComponent* Obj, const FBallLaunchParams& P, FVector COM, float TimeStep, int NumSteps, FPhysTransform& TLaunch);

	static uint8 GetVerticalLevelIndexFromDistanceZ(float DistanceZ, float LevelWidth);
};
