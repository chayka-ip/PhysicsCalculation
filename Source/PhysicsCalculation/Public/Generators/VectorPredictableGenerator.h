#pragma once

#include "CoreMinimal.h"
#include "GeneratorTimeBased.h"
#include "HandyMathLibrary.h"
#include "NativeLib.h"
#include "VectorPredictableGenerator.generated.h"

USTRUCT(BlueprintType)
struct FVectorPredictableGenerator : public FGeneratorTimeBased
{
    GENERATED_BODY()

public:

    UPROPERTY(BlueprintReadOnly)
    TArray<FVector> PredictionArray;

public:
    bool HasData() const {return PredictionArray.Num() > 0;}
    bool HasNotLessStepsThan(int NumSteps) const {return HasData() && PredictionArray.Num() >= NumSteps;}
    
    virtual void Init() override
    {
        const int StepsCount = GetSimulationStepsCount();
        for (int i = 0; i < StepsCount; ++i)
        {
            PredictionArray.Add(ProduceVector());
        }
    }

    virtual void UpdateOneStep() override
    {
        PredictionArray.RemoveAt(0);
        PredictionArray.Add(ProduceVector());
    }
    
    virtual FVector ProduceVector()
    {
        checkNoEntry()
        return {};
    }
    
    virtual FVector ProduceRandomVector(float x, float y, float z) const
    {
        const float X = UHM::FRandRangePlusMinus(x);
        const float Y = UHM::FRandRangePlusMinus(y);
        const float Z = UHM::FRandRangePlusMinus(z);
        return FVector(X, Y, Z);
    }
    
    FVector GetFirstPredictedVector() const
    {
        return HasData() ? PredictionArray[0] : FVector::ZeroVector;
    }

    FVector GetProducedVectorAfterStepCount(int N)
    {
        return *Native::ArrayGetElementInBoundsSafe(N, PredictionArray, FVector::ZeroVector);
    }

};