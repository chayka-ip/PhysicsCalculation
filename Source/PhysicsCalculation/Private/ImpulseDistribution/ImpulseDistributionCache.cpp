// Fill out your copyright notice in the Description page of Project Settings.


#include "ImpulseDistribution/ImpulseDistributionCache.h"

#include "HandyMathCurveLibrary.h"

float UImpulseDistributionCache::GetImpulseAngleFromVelocityRatio(float Ratio) const
{
    const float RatioOnZero = AngleDistribution()->Eval(0.0f);
    const float RatioOn90 = AngleDistribution()->Eval(90.0f);

    if(Ratio >= RatioOnZero) return 0.0f;
    if(Ratio <= RatioOn90) return 90.0f;

    float Angle;
    const bool bFound = UHMC::GetCurveNthTimeWhenValueEquals(Data.AngleDistribution, Ratio, Angle, 0);
    check(bFound)
    return Angle;
}

float UImpulseDistributionCache::GetImpulseAngleFromVelocities(FVector LinearVelocity, FVector AngularVelocity) const
{
    const float LV = LinearVelocity.Size();
    const float AV = AngularVelocity.Size();

    const bool bZeroLV = FMath::IsNearlyZero(LV);
    const bool bZeroAV = FMath::IsNearlyZero(AV);

    if(bZeroLV && bZeroAV) return 0.0f;
    if(!bZeroLV && bZeroAV) return 0.0f;
    if(bZeroLV && !bZeroAV) return 90.0f;

    const float Ratio = LV / AV;
    return GetImpulseAngleFromVelocityRatio(Ratio);
}
