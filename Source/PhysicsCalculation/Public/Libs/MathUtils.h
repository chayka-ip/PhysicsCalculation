// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "MathUtils.generated.h"

UENUM()
enum EPCVectorComp
{
    X,
    Y,
    Z
};


/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UMathUtils : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
    
    static bool IsRestVectorComponentsZero(const FVector& V, EPCVectorComp ExcludeComponent);
    static float AngleBetweenVectorsDeg(FVector V1, FVector V2);
    static float GetSphereRadiusFromScale(const FVector V);
    static  FVector GetVectorTangentComponent(const FVector &V, const FVector &N);

    static  float GetVectorMaxRelativeScale(const FVector &A, const FVector &B);
    static  float GetVectorRelativeScaleAtoB(const FVector &A, const FVector &B);
    
	//Spherical interpolation from A to B
    static FQuat Slerp(const FQuat &A, const FQuat &B, float Alpha);
    static FQuat GetDeltaRotation(const FQuat& Target, const FQuat& Initial);

    static void ApplyAngularVelocityToRotation(const FVector& AngularVelocity, const float DeltaTime, FQuat& InOutOrientation);

    UFUNCTION(BlueprintPure)
    static FQuat AngularVelocityToSpinFromRotator(const FRotator& Orientation, const FVector AngularVelocity);
    UFUNCTION(BlueprintPure)
    static FQuat AngularVelocityToSpin(const FQuat& Orientation, const FVector AngularVelocity);
    UFUNCTION(BlueprintCallable)
    static void BP_ApplyAngularVelocityToRotation(FVector AngularVelocity, float DeltaTime, FQuat InOrientation, FQuat& OutOrientation);

    static float SplitTimeToSubstepsAndFraction(float Value, float SimulationDeltaTime, int& NumSteps);
    static float GetSimulationRationalTimeFullStep(float Time, float SimulationDeltaTime);
    static float GetSimulationRationalTimeLeftoverPart(float Time, float TimeLeftBeforeNextPredict);
    static float GetRatio(float A, float B);
    
    static void ExtractDataFromTimeForPrediction(float Time, float TimeBeforeNextPredict, float SimStep, int& NumStepsToUpdate,
                                                    int& NumStepsToGet, float& Alpha, float& NewTimeBeforeNextPredict);

    // returns fractional time left after last integral step reached
    static float CalculateSkipTimeOffset(float SkipTime, float TimeBeforeNextPredict, float SimStep, int& NumStepsToSkip);

    /*
     * NumStepsToSkip - how much steps should be skip from the first 
     * NumStepsToGet - how much steps  be taken after skip point in the future
     */
    static void GetTimeExtractionDataForPredictWithSkip(float DeltaTime, float SkipTime, float InitialTimeBeforePredict, float SimStep,
                                                        int& NumStepsToSkip, int& NumStepsToGet, float& InitialTimeFraction, float& FutureTimeFraction);

    static void CalculateFractionSegmentMultipliers(float InitialTimeFraction, float FutureTimeFraction, float DeltaTime, float SimStep,
                                                    float& InitialPartMultiplier, float& FuturePartMultiplier);

    static void CalculateTimeExtractionDataForPredictWithSkip(float DeltaTime, float SkipTime, float InitialTimeBeforePredict, float SimStep,
                                                              int& NumStepsToSkip, int& NumStepsToGet,
                                                              float& InitialPartMultiplier, float& FuturePartMultiplier);
};
