// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/PSI_Data.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "AerodynamicsSimulation.generated.h"

struct FPhysTransform;
struct FPhysRigidBodyParams;
struct FBodyAerodynamics;
struct FSideforceBaseGenerator;
class UCustomPhysicsComponent;
/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UAerodynamicsSimulation : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
    
public:
    /*
     * use independent location offset because generator introduces crucial error during recalculation from location to velocity
     */
    static void CalculateAerodynamicImpact(FPSI_Data& Data, FVector& OutLinearVelocity, FVector& OutAngularVelocity, FVector& locationOffset);

protected:
    static FVector CalculateAerodynamicForce(const FPhysRigidBodyParams& RbParams, const FVector& LinearVelocity, const FVector& AngularVelocity);
    static FVector ComputeSphereAirDragForce(float CrossSectionArea, float AirDensity, FVector LinearVelocity);
    static FVector ComputeSphereLiftForceIdeal(float Radius, float AirDensity, FVector LinearVelocity, FVector AngularVelocity);

public:
    static FVector GetOffsetToApplyWithTimeSkip(FSideforceBaseGenerator& S, FVector ForwardVector, FVector RightVector, FVector UpVector,
                                                float DeltaTime, float SkipTime, float VelocityImpactAlpha);

protected:
    static void CalculateSideForceImpact(float DeltaTime, float SkipTime, FBodyAerodynamics& ADP, const FPhysTransform& T, FVector& OutAngularVelocity, FVector& locationOffset);

public:
    UFUNCTION(BlueprintPure)
    static FVector ComputeMagnusAccelerationForBody(const UCustomPhysicsComponent* Body);
    
    UFUNCTION(BlueprintPure)
    static FVector ComputeMagnusAccelerationForBodyForVelocity(const UCustomPhysicsComponent* Body, FVector LV, FVector AV);

};
