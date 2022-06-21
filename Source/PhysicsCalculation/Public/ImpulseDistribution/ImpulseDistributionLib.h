// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ImpulseDistributionCache.h"
#include "ImpulseReconstructed.h"
#include "Components/AdvancedPhysicsComponent.h"
#include "Kick/KickSpinRange.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "ImpulseDistributionLib.generated.h"

/**
 * todo: we can reconstruct orient quat from impulse:
 * impulse is forward vector; find angle to plane XY; make Z vector; make right vector
 */
UCLASS()
class PHYSICSCALCULATION_API UImpulseDistributionLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:

    static FImpulseDistributionCache_Data CalculateImpulseDistribution(UAdvancedPhysicsComponent* Obj);
    
    UFUNCTION(BlueprintCallable)
    static FRuntimeFloatCurve GetImpulseDistributionCurve(UAdvancedPhysicsComponent* Obj);

    UFUNCTION(BlueprintPure)
    static float GetImpulseDistributionRatioForSphere(UAdvancedPhysicsComponent* Obj, float ImpulseMagnitude, float AngleToNormalDeg);
    static bool CheckAngle90Deg(FVector LinearVelocity, FVector AngularVelocity);

    static FVector GetImpulseApplyLocationFromVelocity(UAdvancedPhysicsComponent* PC, FVector LinearVelocity, FVector AngularVelocity, FVector COM);
    static FVector GetImpulseFromLinearVelocity(UAdvancedPhysicsComponent* Obj, FVector LinearVelocity, FVector ApplyLocation, FVector COM);
    static FVector GetImpulseFromVelocities(UAdvancedPhysicsComponent* Obj, FVector LinearVelocity, FVector AngularVelocity, FVector ApplyLocation, FVector COM);

    UFUNCTION(BlueprintPure)
    static FImpulseReconstructed ReconstructImpulseFromVelocities(UAdvancedPhysicsComponent* PC, FVector LinearVelocity, FVector AngularVelocity, FVector COM);

    UFUNCTION(BlueprintPure)
    static FImpulseReconstructed ReconstructImpulseFromImpactTransform(UAdvancedPhysicsComponent* Obj, const FPhysTransform& T);

    UFUNCTION(BlueprintPure)
    static FImpulseReconstructed GetImpulseFromParabolicAndSpin(UAdvancedPhysicsComponent* PC, FVector ParabolicVelocity, FVector ApplyLocation, FVector COM, float FrontSpin, float SideSpin, bool bRemoveVelocity);

    /*
     * Used for impulse
     * Right Vector - top/back spin; negative value is top spin, positive - back spin
     * Up Vector - side spin
     */
    UFUNCTION(BlueprintPure)
    static FQuat GetImpulseQuatFromParabolicVelocityAndSpin(FVector V, float FrontSpin=0.0f, float SideSpin=0.0f);

    UFUNCTION(BlueprintPure)
    static void GetSpinFromImpulseQuatAndParabolicVelocity(FVector V, FQuat Q, float& FrontSpin, float& SideSpin);

    static FImpulseReconstructed RotateImpulseZAxis(const FImpulseReconstructed& ImpulseData, const FVector& COM, float AngleDeg);

    static TArray<FQuat> GetImpulseQuatArray(FVector ParabolicVelocity, const FKickSpinRange& SpinRange, bool bFlipSideSpin);
};
