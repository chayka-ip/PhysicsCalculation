// Fill out your copyright notice in the Description page of Project Settings.


#include "Libs/MathUtils.h"
#include "HandyMathLibrary.h"
#include "Kismet/KismetMathLibrary.h"

bool UMathUtils::IsRestVectorComponentsZero(const FVector& V, const EPCVectorComp ExcludeComponent)
{
    switch (ExcludeComponent)
    {
    case X: return FMath::IsNearlyZero(V.Y) && FMath::IsNearlyZero(V.Z);
    case Y: return FMath::IsNearlyZero(V.X) && FMath::IsNearlyZero(V.Z);
    case Z: return FMath::IsNearlyZero(V.X) && FMath::IsNearlyZero(V.Y);
    }
    return false;
}

float UMathUtils::AngleBetweenVectorsDeg(FVector V1, FVector V2)
{
    if (V1.IsZero() || V2.IsZero())
        return 0;

    const float DotProd = FVector::DotProduct(V1.GetSafeNormal(), V2.GetSafeNormal());
    return UKismetMathLibrary::DegAcos(DotProd);
}

float UMathUtils::GetSphereRadiusFromScale(const FVector V)
{
    return V.X  * 50;
}

FVector UMathUtils::GetVectorTangentComponent(const FVector &V, const FVector &N)
{
    const FVector Projection = V.ProjectOnTo(N.GetSafeNormal());
    if(Projection.ContainsNaN()) return V;
    return V - Projection;
}

float UMathUtils::GetVectorMaxRelativeScale(const FVector& A, const FVector& B)
{
    const float a = A.Size();
    const float b = B.Size();

    const float Min = FMath::Min(a, b);
    const float Max = FMath::Max(a, b);
    
    return FMath::IsNearlyZero(Min, 0.001f) ? 0.0f : Max/Min;
}

float UMathUtils::GetVectorRelativeScaleAtoB(const FVector& A, const FVector& B)
{
    const float a = A.Size();
    const float b = B.Size();

    if(FMath::IsNearlyZero(a) || FMath::IsNearlyZero(b)) return 0.0f;
    return a/b;
    
}

FQuat UMathUtils::Slerp(const FQuat& A, const FQuat& B, float Alpha)
{
    return FQuat::Slerp(A, B, Alpha);
}

FQuat UMathUtils::GetDeltaRotation(const FQuat& Target, const FQuat& Initial)
{
    FQuat Q = Initial;
    Q.EnforceShortestArcWith(Target);
    return Target * Q.Inverse();
}

void UMathUtils::ApplyAngularVelocityToRotation(const FVector &AngularVelocity, const float DeltaTime, FQuat& InOutOrientation)
{
    InOutOrientation += AngularVelocityToSpin(InOutOrientation, AngularVelocity) * DeltaTime;
    InOutOrientation.Normalize();
}

FQuat UMathUtils::AngularVelocityToSpinFromRotator(const FRotator& Orientation, const FVector AngularVelocity)
{
    return AngularVelocityToSpin(Orientation.Quaternion(), AngularVelocity);
}

FQuat UMathUtils::AngularVelocityToSpin(const FQuat& Orientation, const FVector AngularVelocity)
{
    const FVector V = 0.5 * AngularVelocity;
    return  FQuat(V.X, V.Y, V.Z,0) * Orientation;
}

void UMathUtils::BP_ApplyAngularVelocityToRotation(FVector AngularVelocity, float DeltaTime, FQuat InOrientation, FQuat& OutOrientation)
{
    OutOrientation = InOrientation;
    ApplyAngularVelocityToRotation(AngularVelocity, DeltaTime, OutOrientation);
}

float UMathUtils::SplitTimeToSubstepsAndFraction(float Value, float SimulationDeltaTime, int& NumSteps)
{
    return UHM::SplitFloatsIntWithLeftOver(Value, NumSteps, SimulationDeltaTime);
}

float UMathUtils::GetSimulationRationalTimeFullStep(float Time, float SimulationDeltaTime)
{
    return FMath::Clamp(Time / SimulationDeltaTime, 0.0f, 1.0f);
}

float UMathUtils::GetSimulationRationalTimeLeftoverPart(float Time, float TimeLeftBeforeNextPredict)
{
    return FMath::Clamp(UHM::SafeDivision(Time, TimeLeftBeforeNextPredict), 0.0f, 1.0f);
}

float UMathUtils::GetRatio(float A, float B)
{
    if(FMath::IsNearlyZero(B)) return 0.0f;
    const float f  = UHM::SafeDivision(A, B);
    return UHM::FClampZeroOne(f);
}

void UMathUtils::ExtractDataFromTimeForPrediction(float Time, float TimeBeforeNextPredict, float SimStep, int& NumStepsToUpdate,
                                                  int& NumStepsToGet, float& Alpha, float& NewTimeBeforeNextPredict)
{
    NumStepsToUpdate = 0;
    NumStepsToGet = 0;

    const float TimeBNP = TimeBeforeNextPredict;
    if(Time <= TimeBNP)
    {
        Alpha = GetSimulationRationalTimeLeftoverPart(Time, TimeBNP);
        NewTimeBeforeNextPredict = TimeBNP - Time;
    }
    else
    {
        Time -= TimeBNP;
        const float Fraction = SplitTimeToSubstepsAndFraction(Time, SimStep, NumStepsToUpdate);
        Alpha = GetSimulationRationalTimeFullStep(Fraction, SimStep);

        NewTimeBeforeNextPredict = FMath::Clamp(SimStep - Fraction, 0.0f, SimStep);
        NumStepsToUpdate += 1;
    }

    NumStepsToGet = NumStepsToUpdate - 1;
}

float UMathUtils::CalculateSkipTimeOffset(float SkipTime, float TimeBeforeNextPredict, float SimStep, int& NumStepsToSkip)
{
    NumStepsToSkip = 0;
    
    const float CorrectedTime = SkipTime - TimeBeforeNextPredict;
    if(CorrectedTime > 0.0f)
    {
        const float f =  SplitTimeToSubstepsAndFraction(CorrectedTime, SimStep, NumStepsToSkip);
        NumStepsToSkip += 1;
        return f;
    }
    
    return TimeBeforeNextPredict;    
}

void UMathUtils::GetTimeExtractionDataForPredictWithSkip(float DeltaTime, float SkipTime, float InitialTimeBeforePredict, float SimStep,
                                                         int& NumStepsToSkip, int& NumStepsToGet,float& InitialTimeFraction, float& FutureTimeFraction)
{
    /*
    * Zero time describes entity's current state, so TimeBeforeNextPredict should be accounted;
    * This is because entity should not generate new prediction step until second step is reached;
    */
    check(InitialTimeBeforePredict < SimStep && SimStep > 0.0f)

    NumStepsToSkip = -1;
    NumStepsToGet = -1;
    InitialTimeFraction = 0.0f;
    FutureTimeFraction = 0.0f;

    // exact offset to begin calculation for DeltaTime given
    InitialTimeFraction = CalculateSkipTimeOffset(SkipTime, InitialTimeBeforePredict, SimStep, NumStepsToSkip);

    const bool NoSkipSteps = NumStepsToSkip < 1;
    const bool BigInitialTime = InitialTimeFraction >= DeltaTime;

    if(NoSkipSteps && BigInitialTime)
    {
        InitialTimeFraction = DeltaTime;
        return;
    }

    int _;
    float __, FutureTimeBeforeNextPredict = 0.0f; // time that will left from end point to further prediction step

    ExtractDataFromTimeForPrediction(DeltaTime, InitialTimeFraction, SimStep, _, NumStepsToGet, __,
                    FutureTimeBeforeNextPredict);

    if(NumStepsToGet >= 0)
    {
        FutureTimeFraction = SimStep - FutureTimeBeforeNextPredict;
    }
}

void UMathUtils::CalculateFractionSegmentMultipliers(float InitialTimeFraction, float FutureTimeFraction, float DeltaTime, float SimStep,
                                                     float& InitialPartMultiplier, float &FuturePartMultiplier)
{
    const float InitialRemainingPortion = 1.0f - UHM::SafeDivision(SimStep - InitialTimeFraction, SimStep);
    const float DeltaTimeRatioToInitial = GetRatio(DeltaTime, InitialTimeFraction);
    
    InitialPartMultiplier = InitialRemainingPortion * DeltaTimeRatioToInitial;
    FuturePartMultiplier = GetRatio(FutureTimeFraction, SimStep);
}

void UMathUtils::CalculateTimeExtractionDataForPredictWithSkip(float DeltaTime, float SkipTime, float InitialTimeBeforePredict, float SimStep,
    int& NumStepsToSkip, int& NumStepsToGet, float& InitialPartMultiplier, float& FuturePartMultiplier)
{
    float InitialTimeFraction, FutureTimeFraction = 0.0f;
    
    GetTimeExtractionDataForPredictWithSkip(DeltaTime, SkipTime, InitialTimeBeforePredict, SimStep,
                                            NumStepsToSkip, NumStepsToGet, InitialTimeFraction, FutureTimeFraction);

    CalculateFractionSegmentMultipliers(InitialTimeFraction, FutureTimeFraction, DeltaTime, SimStep,
                                        InitialPartMultiplier, FuturePartMultiplier);
}
