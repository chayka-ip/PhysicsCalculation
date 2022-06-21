// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/SimpleMatrix3.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "InertiaLib.generated.h"

UENUM()
enum EShape
{
	Sphere,
	Ball,
};
/**
 * Check if Cm / M system have an effect on simulation
 */
UCLASS()
class PHYSICSCALCULATION_API UInertiaLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
public:
	static float GetCentralAxisInertiaCoefficient(EShape Object);
	static float ComputeInertia(float Mass,  float R,  float K);
	static float GetCentralAxisInertia( float Mass,  float R,  EShape ObjectType);
	static FSimpleMatrix3 GetCentralAxisInertiaTensor( float Mass,  float R, EShape Object, bool GetInverted);
};
