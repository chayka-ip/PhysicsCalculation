#pragma once

#include "CoreMinimal.h"
#include "AerodynamicsSimulation.h"
#include "Generators/VectorPredictableGenerator.h"
#include "SideforceGenerator.generated.h"

USTRUCT(BlueprintType)
struct FSideforceBaseGenerator : public FVectorPredictableGenerator
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float OffsetImpactAlpha = 1.0f;
    
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float VelocityAlphaThreshold = 0.01f;
    
public:

    virtual FVector GetProperVectorAfterStepCount(FVector ForwardVector, FVector RightVector, FVector UpVector, int N)
    {
        const FVector P = GetProducedVectorAfterStepCount(N);
        const FVector F = ForwardVector * P.X;
        const FVector R = RightVector * P.Y;
        const FVector U = UpVector * P.Z;
        return F + R + U;
    }

    virtual FVector GetVectorToApply(FVector ForwardVector, FVector RightVector, FVector UpVector, float DeltaTime, float SkipTime, float VelocityImpactAlpha)
    {
        const bool bSmallImpact = VelocityImpactAlpha <= VelocityAlphaThreshold;
        if(bSmallImpact || !bEnabled) return FVector::ZeroVector;
        return UAerodynamicsSimulation::GetOffsetToApplyWithTimeSkip(*this, ForwardVector, RightVector, UpVector, DeltaTime, SkipTime, VelocityImpactAlpha);
    }
};

USTRUCT(BlueprintType)
struct FLocationOffsetGenerator : public FSideforceBaseGenerator
{
    GENERATED_BODY()

public:

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float MaxHorizontalDistance = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float MaxVerticalDistance = 0.0f;

public:
    virtual FVector ProduceVector() override {return ProduceRandomVector(MaxHorizontalDistance, MaxVerticalDistance, 0.0f);}

    static float GetHorizontalValue(const FVector &V) {return V.X;}
    static float GetVerticalValue(const FVector &V) {return V.Y;}

    virtual FVector GetProperVectorAfterStepCount(FVector ForwardVector, FVector RightVector, FVector UpVector, int N) override
    {
        const FVector P = GetProducedVectorAfterStepCount(N);
        const FVector H = RightVector * GetHorizontalValue(P);
        const FVector V = UpVector * GetVerticalValue(P);
        return  H + V;
    }
};

USTRUCT(BlueprintType)
struct FAngularVelocityGenerator : public FSideforceBaseGenerator
{
    GENERATED_BODY()
    
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float MaxAxisValue;

public:
    virtual FVector ProduceVector() override {return ProduceRandomVector(MaxAxisValue, MaxAxisValue, MaxAxisValue);}

};