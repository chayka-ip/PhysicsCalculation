// Fill out your copyright notice in the Description page of Project Settings.


#include "Libs/MovementPredictionLib.h"

#include "Common/PhysPredict.h"
#include "Libs/PhysicsUtils.h"

void UMovementPredictionLib::PredictionGetExtractionDataFromTime(const FPhysPredict& P, float Time, float SimStep, int& NumStepsToUpdate,
                                                                 int& NumStepsToGet, float& Alpha, float& NewTimeBeforeNextPredict)
{
    const float TimeBNP = P.GetTimeBeforeNextPredict();
    UMathUtils::ExtractDataFromTimeForPrediction(Time, TimeBNP, SimStep, NumStepsToUpdate, NumStepsToGet, Alpha, NewTimeBeforeNextPredict);
}

void UMovementPredictionLib::GetPredictedTransformAfterInterval(FPhysTransform& InOutT, FPhysPredict& P, int NumStepsToGet, float Alpha,
    bool bPreciseType)
{
    if (NumStepsToGet >= 0)
    {
        if(bPreciseType)
        {
            InOutT = *P.GetPrecisePredictedTransformPtrAfterStepCount(NumStepsToGet);
        }
        else
        {
            InOutT = *P.GetRoughPredictedTransformPtrAfterStepCount(NumStepsToGet);
        }
    }

    if(Alpha > 0.0f)
    {
        FPhysTransform TLerpTarget;
        const int TargetIndex = NumStepsToGet + 1;
        if(bPreciseType)
        {
            TLerpTarget = *P.GetPrecisePredictedTransformPtrAfterStepCount(TargetIndex);
        }
        else
        {
            TLerpTarget = *P.GetRoughPredictedTransformPtrAfterStepCount(TargetIndex);
        }
        InOutT = UPhysicsUtils::TLerp(InOutT, TLerpTarget, Alpha);
    }
}

void UMovementPredictionLib::PredictGetNextPreciseTransformAndUpdateStruct(FPhysTransform& InOutT, FPhysPredict& P, float Time, float SimStep)
{
    int NumStepsToUpdate;
    int NumStepsToGet;
    float Alpha;
    float NewTimeBeforeNextPredict;

    PredictionGetExtractionDataFromTime(P, Time, SimStep, NumStepsToUpdate, NumStepsToGet, Alpha, NewTimeBeforeNextPredict);
    GetPredictedTransformAfterInterval(InOutT, P, NumStepsToGet, Alpha, true);

    P.AdvanceByTime(NewTimeBeforeNextPredict, NumStepsToUpdate);
}

FPhysTransform UMovementPredictionLib::GetPrecisePredictedTransform(const FPhysTransform& TCurrent, FPhysPredict& P, float Time, float SimStep)
{
    int NumStepsToUpdate;
    int NumStepsToGet;
    float Alpha;
    float NewTimeBeforeNextPredict;
    
    PredictionGetExtractionDataFromTime(P, Time, SimStep, NumStepsToUpdate, NumStepsToGet, Alpha, NewTimeBeforeNextPredict);

    auto TOut = TCurrent;
    GetPredictedTransformAfterInterval(TOut, P, NumStepsToGet, Alpha, true);
    
    return  TOut;
}

FPhysTransform UMovementPredictionLib::GetRoughPredictedTransform(FPhysPredict& P, float Time)
{
    int NumStepsToUpdate;
    int NumStepsToGet;
    float Alpha;
    float NewTimeBeforeNextPredict;

    const float SimStep = P.GetRoughSimulationTimeStep();
    
    PredictionGetExtractionDataFromTime(P, Time, SimStep, NumStepsToUpdate, NumStepsToGet, Alpha, NewTimeBeforeNextPredict);

    auto TOut = *P.GetFirstRoughPredictedTransformPtr();
    GetPredictedTransformAfterInterval(TOut, P, NumStepsToGet, Alpha, false);

    return TOut;
}
