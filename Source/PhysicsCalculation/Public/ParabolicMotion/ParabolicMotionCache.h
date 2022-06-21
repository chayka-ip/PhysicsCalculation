// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ParabolicMotionCurve.h"
#include "ParabolicMotionCache.generated.h"

USTRUCT()
struct FParabolicMotionCache_Data
{
	GENERATED_BODY()

public:

	// angle is a key
	UPROPERTY()
	TMap<int, FParabolicMotionCurve> Data = {};

	// Launch Speed [cm/sec] is key; angle is value
	UPROPERTY()
	FRuntimeFloatCurve MaxLaunchCurve;

public:
	void AddLaunchAngleData(int Angle, const FParabolicMotionCurve& Curve) {Data.Add(Angle, Curve);}
	void ComputeMinMaxLaunchAngle(int& Min, int& Max) const;
};

USTRUCT()
struct FParabolicCurveSelection
{
	GENERATED_BODY()

public:
	FParabolicMotionCurve* Min = nullptr;
	FParabolicMotionCurve* Max = nullptr;
	float Alpha = 0.0f;

public:
	bool IsValidMin() const {return Min != nullptr;}
	bool IsValidMax() const {return Max != nullptr;}
	bool AreValidBoth() const {return IsValidMin() && IsValidMax();}
	bool IsInvalid() const {return !IsValidMin() && !IsValidMax();}

public:
	float GetParabolicMultiplierValue(float x) const;
	float GetParabolicMaxDistance(float LaunchSpeed) const;
};

/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UParabolicMotionCache : public UObject
{
	GENERATED_BODY()

public:
	UPROPERTY()
	FParabolicMotionCache_Data Data;

	UPROPERTY()
	int MinLaunchAngle = 0;

	UPROPERTY()
	int MaxLaunchAngle = 0;
	
public:
	void Initialize(const FParabolicMotionCache_Data& data);
	
public:
	TMap<int, FParabolicMotionCurve>* GetAngleMap() {return &Data.Data;}
	FParabolicCurveSelection GetParabolicCurvesByAngle(float Angle);
	
public:
	UFUNCTION(BlueprintPure)
	float GetParabolicMultiplierForAngle(float LaunchAngleXZ, float x);

	UFUNCTION(BlueprintPure)
	float GetParabolicMaxDistanceForAngle(float LaunchAngleXZ, float LaunchSpeed);

public:
	UFUNCTION(BlueprintPure)
	float GetRequiredLaunchSpeedForAngle(float LaunchAngleXZ, float x, float z);

	UFUNCTION(BlueprintPure)
	FRuntimeFloatCurve GetMultiplierCurveForAngle(int LaunchAngleXZ);

	UFUNCTION(BlueprintPure)
	float GetAngleMaxDistanceForLaunchSpeed(float LaunchSpeed);
	
public:
	UFUNCTION(BlueprintPure)
	int GetMinLaunchAngle() const {return MinLaunchAngle;}
	UFUNCTION(BlueprintPure)
	int GetMaxLaunchAngle() const {return MaxLaunchAngle;}
	UFUNCTION(BlueprintPure)
	void GetMinMaxLaunchAngleCached(int& Min, int& Max) const;

};
