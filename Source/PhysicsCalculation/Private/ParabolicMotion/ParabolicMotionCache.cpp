// Fill out your copyright notice in the Description page of Project Settings.

#include "ParabolicMotion/ParabolicMotionCache.h"
#include "debug.h"
#include "HandyMathArrayLibrary.h"
#include "HandyMathLibrary.h"
#include "ParabolicMotion/ParabolicMotionLib.h"

void FParabolicMotionCache_Data::ComputeMinMaxLaunchAngle(int& Min, int& Max) const
{
    TArray<int> Keys;
    Data.GetKeys(Keys);
    UHMA::IntArrayGetMinMax(Keys, Min, Max);
}

float FParabolicCurveSelection::GetParabolicMultiplierValue(float x) const
{
    if(IsInvalid())
    {
        PrintToLog("Can't get parabolic multiplier");
        return 1.0f;
    }
    if(AreValidBoth())
    {
        const float MinVal = Min->GetMultiplierForDistance(x);
        const float MaxVal = Max->GetMultiplierForDistance(x);
        return UHM::GetValueFromFloatRangeByAlphaFromMin(MinVal, MaxVal, Alpha);
    }
    return IsValidMin() ? Min->GetMultiplierForDistance(x) : Max->GetMultiplierForDistance(x);
}

float FParabolicCurveSelection::GetParabolicMaxDistance(float LaunchSpeed) const
{
    if(IsInvalid()) return 0.0f;
    if(AreValidBoth())
    {
        const float MinVal = Min->GetMaxDistanceForLaunchSpeed(LaunchSpeed);
        const float MaxVal = Max->GetMaxDistanceForLaunchSpeed(LaunchSpeed);
        return UHM::GetValueFromFloatRangeByAlphaFromMin(MinVal, MaxVal, Alpha);
    }
    return IsValidMin() ? Min->GetMaxDistanceForLaunchSpeed(LaunchSpeed) : Max->GetMaxDistanceForLaunchSpeed(LaunchSpeed);
}

void UParabolicMotionCache::Initialize(const FParabolicMotionCache_Data& data)
{
    Data = data;
    Data.ComputeMinMaxLaunchAngle(MinLaunchAngle, MaxLaunchAngle);    
}

FParabolicCurveSelection UParabolicMotionCache::GetParabolicCurvesByAngle(float Angle)
{
    FParabolicCurveSelection Out;

    int AMin;
    int AMax;
    UHM::FloatToIntRange(Angle, AMin, AMax);

    Out.Min = GetAngleMap()->Find(AMin);
    Out.Max = GetAngleMap()->Find(AMax);
    Out.Alpha = UHM::GetRatioFromPositiveRange(Angle, AMin, AMax);
    
    check(!Out.IsInvalid())
    return Out;
}

float UParabolicMotionCache::GetParabolicMultiplierForAngle(float LaunchAngleXZ, float x)
{
    const auto Sel =  GetParabolicCurvesByAngle(LaunchAngleXZ);
    return Sel.GetParabolicMultiplierValue(x);
}

float UParabolicMotionCache::GetParabolicMaxDistanceForAngle(float LaunchAngleXZ, float LaunchSpeed)
{
    const auto Sel =  GetParabolicCurvesByAngle(LaunchAngleXZ);
    return Sel.GetParabolicMaxDistance(LaunchSpeed);
}

float UParabolicMotionCache::GetRequiredLaunchSpeedForAngle(float LaunchAngleXZ, float x, float z)
{
    const float BaseSpeed = UParabolicMotionLib::GetRequiredLaunchSpeedForAngle(LaunchAngleXZ, x, z);
    const float Mul = GetParabolicMultiplierForAngle(LaunchAngleXZ, x);
    return Mul * BaseSpeed;
}

FRuntimeFloatCurve UParabolicMotionCache::GetMultiplierCurveForAngle(int LaunchAngleXZ)
{
    const auto Item = GetAngleMap()->Find(LaunchAngleXZ);
    if(Item) return Item->MultiplierCurve;
    PrintToLog("Multiplier curve not found");
    return {};
}

float UParabolicMotionCache::GetAngleMaxDistanceForLaunchSpeed(float LaunchSpeed)
{
    return Data.MaxLaunchCurve.GetRichCurve()->Eval(LaunchSpeed);
}

void UParabolicMotionCache::GetMinMaxLaunchAngleCached(int& Min, int& Max) const
{
    Min = GetMinLaunchAngle();
    Max = GetMaxLaunchAngle();
}

