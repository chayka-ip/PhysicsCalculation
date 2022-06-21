// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ParabolicMotion.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "ParabolicMotionLib.generated.h"

UCLASS()
class PHYSICSCALCULATION_API UParabolicMotionLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
    
public:
    static FVector ComputeLocationOnTime(float InitialSpeed, float LaunchAngleXZ, float Time, float x_0=0.0f, float z_0=0.0f);

    UFUNCTION(BlueprintCallable)
    static TArray<FVector> ComputeParabolicMotionTrajectory(FParabolicCompData P);

    // z = 0 is assumed as ground level
    static void ComputeParabolicMotion(float LaunchSpeed, float LaunchAngleXZ, float x_0, float z_0, FParabolicMotion& Result);
    static float ComputeFlightTime(float LaunchSpeed, float LaunchAngleXZ, float z_0);
    static float ComputeMaxHeightTime(float LaunchSpeed, float LaunchAngleXZ);
    static bool GetRequiredLaunchSpeedForAngle(float LaunchAngleXZ, float x, float z, float& Speed);
    UFUNCTION(BlueprintPure)
    static float GetRequiredLaunchSpeedForAngle(float LaunchAngleXZ, float x, float z);
    UFUNCTION(BlueprintPure)
    static bool GetRequiredLaunchAnglesForSpeed(float LaunchSpeed, float x, float z, bool bAccept90Deg, float& MinAngle, float& MaxAngle);
    static bool ValidateAngle90DegPrevention(float A1, float A2, float& OutA1, float& OutA2);

};

