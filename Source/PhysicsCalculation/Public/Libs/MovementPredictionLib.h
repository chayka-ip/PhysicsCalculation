// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/PhysTransform.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "MovementPredictionLib.generated.h"

struct FPhysPredict;
/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UMovementPredictionLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
    
    static void PredictionGetExtractionDataFromTime(const FPhysPredict& P, float Time, float SimStep, int& NumStepsToUpdate, int& NumStepsToGet, float& Alpha, float& NewTimeBeforeNextPredict);
    static void GetPredictedTransformAfterInterval(FPhysTransform& InOutT, FPhysPredict& P, int NumStepsToGet, float Alpha, bool bPreciseType);
    // SimStep is Simulation Step Time (sec)
    static void PredictGetNextPreciseTransformAndUpdateStruct(FPhysTransform& InOutT, FPhysPredict& P, float Time, float SimStep);
    static FPhysTransform GetPrecisePredictedTransform(const FPhysTransform& TCurrent, FPhysPredict& P, float Time, float SimStep);
    static FPhysTransform GetRoughPredictedTransform(FPhysPredict& P, float Time);
    
};
