// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/PhysRigidBodyParams.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "IntegratorTestLib.generated.h"

/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UIntegratorTestLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable)
    static FVector GetIntegratedLocation(const FPhysTransform& TIn, float Time, const FPhysRigidBodyParams& Params);
    
    UFUNCTION(BlueprintCallable)
    static FPhysTransform GetIntegratedTransform(const FPhysTransform& TIn, float DeltaTime, const FPhysRigidBodyParams& RbParams);

    // UFUNCTION(BlueprintCallable)
    static TArray<FPhysTransform> GetIntegralTrajectory(const FPhysTransform& TIn, float Time, int NumSimSteps, const FPhysRigidBodyParams& Params);

    UFUNCTION(BlueprintCallable)
    static TArray<FVector> GetIntegralTrajectoryPoints(const FPhysTransform& TIn, float Time, int NumSimSteps, const FPhysRigidBodyParams& Params);
};
