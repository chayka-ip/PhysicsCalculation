// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ImpulseDistributionCache.generated.h"

USTRUCT()
struct FImpulseDistributionCache_Data
{
	GENERATED_BODY()

	/*
	 * Linear velocity divided by angular velocity [cm/rad]
	 */
	UPROPERTY()
	FRuntimeFloatCurve AngleDistribution;
};

/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UImpulseDistributionCache : public UObject
{
	GENERATED_BODY()

public:
	
	UPROPERTY()
	FImpulseDistributionCache_Data Data;
	
public:

	const FRichCurve* AngleDistribution() const {return Data.AngleDistribution.GetRichCurveConst();}
	
public:
	UFUNCTION(BlueprintPure)
	float GetVelocityCoefficientForAngle(float Angle) const {return AngleDistribution()->Eval(Angle);}

	UFUNCTION(BlueprintPure)
	float GetImpulseAngleFromVelocityRatio(float Ratio) const;
	
	UFUNCTION(BlueprintPure)
	float GetImpulseAngleFromVelocities(FVector LinearVelocity, FVector AngularVelocity) const;
};
