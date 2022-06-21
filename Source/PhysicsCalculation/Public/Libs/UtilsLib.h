// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "PhysicsCache/PhysCachedTrajectory.h"
#include "UtilsLib.generated.h"

struct FPhysTransform;
struct FGeneratorTimeBased;
class UCustomPhysicsProcessor;
class UCustomPhysicsBaseComponent;
/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UUtilsLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	static UCustomPhysicsProcessor* GetPhysicsProcessor();
	static void SubscribeToPhysicsProcessor(UCustomPhysicsBaseComponent* Obj);

	static void UpdateTimeBasedGenerator(FGeneratorTimeBased& G, float DeltaTime);

public:
	UFUNCTION(BlueprintPure)
	static FDetailedVector MakeDetailedVector(FVector V);
	UFUNCTION(BlueprintPure)
	static FDetailedVector MakeDetailedVectorFromFloat(float Magnitude, float Angle_XY, float Angle_XZ);
	static TArray<FVector> LocationsFromPhysTransformArray(const TArray<FPhysTransform>& Array);
};
