// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

#include "BallGroundMovementData.h"
#include "Common/PhysEnums.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Common/PhysPredict.h"
#include "PhysicsEngine/PhysicsSettingsEnums.h"

#include "PhysicsUtils.generated.h"

class UCustomPhysicsBaseComponent;
class UCustomPhysicsComponent;
/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UPhysicsUtils : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
    
    static EPhysicsCombineMode GetCombineModeFromDefaultPhysics(EFrictionCombineMode::Type Mode);
    static EPhysicsCombineMode SelectPhysCombineMode(EPhysicsCombineMode A, EPhysicsCombineMode B);
    
    static float CombinePhysValue(float A, float B, EPhysicsCombineMode Mode);

    UFUNCTION(BlueprintCallable)
    static FPhysTransform DiffTransform(const FPhysTransform& A, const FPhysTransform& B);

    UFUNCTION(BlueprintPure)
    static FPhysTransform DiffTransformPure(const FPhysTransform& A, const FPhysTransform& B);

    UFUNCTION(BlueprintCallable)
    static FTransform PhysTransformToTransform(UPARAM(ref) const FPhysTransform PhysTransform, FVector Scale = FVector(1,1,1));

    UFUNCTION(BlueprintPure)
    static FTransform PhysTransformToTransformPure(UPARAM(ref) const FPhysTransform PhysTransform, const FVector Scale = FVector(1,1,1));

    UFUNCTION(BlueprintCallable)
    static  FPhysTransform TransformToPhysTransform(UPARAM(ref) const FTransform Transform, const FVector LinearVelocity=FVector::ZeroVector, const FVector AngularVelocity=FVector::ZeroVector);

    UFUNCTION(BlueprintPure)
    static  FPhysTransform TransformToPhysTransformPure(UPARAM(ref) const FTransform Transform, const FVector LinearVelocity=FVector::ZeroVector, const FVector AngularVelocity=FVector::ZeroVector);

    UFUNCTION(BlueprintCallable)
    static FPhysTransform TLerp(const FPhysTransform& A, const FPhysTransform& B, float Alpha);

    UFUNCTION(BlueprintCallable)
    static FBallGroundMovementData ComputeBallGroundMovementData(const FPhysRigidBodyParams& P, float Time_Step);

    static float GetBallInitialVelocityFromEndVelocity(FBallGroundMovementData P, float EndVelocity, float Time);
    // static float GetInitialVelocityDistancePassedVelocity2(FPhysRigidBodyParams P, float DistanceToPass, float Time);
    // static float GetDistancePassedOnGroundWithStartVelocityAfterTime(FPhysRigidBodyParams P, float StartVelocity, float Time, float TimeStep);

    static float GetTotalMassInv(UCustomPhysicsBaseComponent* A, UCustomPhysicsBaseComponent* B);
    static float GetRestitutionFromBodies(UCustomPhysicsBaseComponent* A, UCustomPhysicsBaseComponent* B);
    static float GetFrictionFromBodies(UCustomPhysicsBaseComponent* A, UCustomPhysicsBaseComponent* B, EPhysicsCombineMode Mode);
};
