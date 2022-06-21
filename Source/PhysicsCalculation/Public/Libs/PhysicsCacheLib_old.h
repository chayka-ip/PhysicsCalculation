// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/PhysTransform.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "PhysicsCache/PhysCachedTrajectory.h"
#include "PhysicsCacheLib_old.generated.h"

struct FPhysCachedTrajectoryFull;
class UCustomPhysicsBaseComponent;
class UAdvancedPhysicsComponent;
/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UPhysicsCacheLib_old : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	static void CleanUpTrajectoryData(UAdvancedPhysicsComponent* Target);
	static void GetIdenticalTrajectoryIndices(const TArray<FPhysCachedTrajectoryFull>& Array, int StartIndex, TArray<int>& InOutSkipIndices);
	
	
	static void CalculatePhysicsCache(UAdvancedPhysicsComponent* Target, const TArray<UCustomPhysicsBaseComponent*> OtherObj);
	static void CalculateCachedTrajectory(UAdvancedPhysicsComponent* Target, const TArray<UCustomPhysicsBaseComponent*> OtherObj,
											const FPhysTransform& TStart, FPhysCachedTrajectoryFull& OutTrajectory);
	static FPhysTransform GetStartTransformForCalculation(float Radius, const FVector& LV, const FVector& AV);
	static bool CanContinueCalculation(UAdvancedPhysicsComponent* Target, const FPhysTransform& T);
	
};
