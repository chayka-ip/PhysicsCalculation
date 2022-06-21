#include "DataAssets/SpinMovementParams_DataAsset.h"
#include "HandyMathLibrary.h"

void FSpinMovementParams::Validate() const
{
    LaunchSpeed.Validate();
    LaunchAngle.Validate();
    FrontSpinAngle.Validate();
    SideSpinAngle.Validate();
    TargetParams.Validate();
        
    check(MaxLaunchAngleCurve)
    check(MaxSideSpinCurve)
    check(MaxBackSpinCurve)
    check(MaxSideSpinCurve)
}

TArray<float> FSpinMovementParams::GetLaunchSpeedCmSec() const
{
    TArray<float> Out = LaunchSpeed.GetValueArrayFullRange();

    for (int i = 0; i < Out.Num(); ++i)
    {
        Out[i] = UHM::FKmph2CMSec(Out[i]);
    }
    
    return Out;
}

float FSpinMovementParams::GetMaxLaunchSpeedCmSec() const
{
    return UHM::FKmph2CMSec(LaunchSpeed.GetMax());
}

int FSpinMovementParams::GetLaunchSpeedSnappedToGrid_KMpH(float speed_kmph) const
{
    TArray<float> Values = LaunchSpeed.GetValueArrayFullRange();

    for (int i = 1; i < Values.Num() - 1; ++i)
    {
        const float V = Values[i];
        const float VPrev = Values[i - 1];
        const bool bMid = speed_kmph >= VPrev && speed_kmph < V;
        if(bMid) return VPrev;
    }

    return LaunchSpeed.GetMax();
}

bool FSpinMovementParams::CanUseLaunchAngle(float launch_speed, float Angle) const
{
    const float AngleMax = MaxLaunchAngleCurve->GetFloatValue(launch_speed);
    return Angle <= AngleMax;
}

bool FSpinMovementParams::CanUseFrontSpin(float launch_speed, float Spin) const
{
    const bool bPositiveSpin = Spin >= 0.0f;
    const auto Curve = bPositiveSpin ? MaxBackSpinCurve : MaxTopSpinCurve;
    const float AbsMax = FMath::Abs(Curve->GetFloatValue(launch_speed));
    const float AbsSpin = FMath::Abs(Spin);
    return AbsSpin <= AbsMax;
}

bool FSpinMovementParams::CanUseSideSpin(float launch_speed, float Spin) const
{
    const float AbsMax = FMath::Abs(MaxSideSpinCurve->GetFloatValue(launch_speed));
    const float AbsSpin = FMath::Abs(Spin);
    return AbsSpin <= AbsMax;
}

bool FSpinMovementParams::CanUseLaunchParams(const FBallLaunchParams& P) const
{
    const float launch_speed = UHM::FCMSec2Kmph(P.LaunchSpeed);
    const bool bOkAngle = CanUseLaunchAngle(launch_speed, P.LaunchAngle);
    const bool bOkFrontSpin = CanUseLaunchAngle(launch_speed, P.FrontSpinAngle);
    const bool bOkSideSpin = CanUseLaunchAngle(launch_speed, P.SideSpinAngle);
    const bool bOK = bOkAngle && bOkFrontSpin && bOkSideSpin;
    return bOK;
}

TArray<float> FSpinMovementParams::GetTargetVerticalDistanceValues() const
{
    return TargetParams.GetValueArrayFullRange();
}

TArray<FFloatMinMax> FSpinMovementParams::GetTargetVerticalLevels() const
{
    TArray<FFloatMinMax>  Out;

    const float HalfDelta = TargetParams.GetHalfDelta();
    auto L = GetTargetVerticalDistanceValues();
    for (const float Center : L)
    {
        const float Min = Center - HalfDelta;
        const float Max = Center + HalfDelta;
        auto Level = FFloatMinMax(Min, Max);
        Out.Add(Level);
    }
        
    return Out;
}

USpinMovementParams_DataAsset::USpinMovementParams_DataAsset()
{
    Data.LaunchSpeed.SetMinMaxDelta(40.0f, 220.0f, 10.0f);
    Data.LaunchAngle.SetMinMaxDelta(0.0f, 70.0f, 1.0f);
    Data.FrontSpinAngle.SetMinMaxDelta(-10.0f, 10.0f, 1.0f);
    Data.SideSpinAngle.SetMinMaxDelta(0.0f, 20.0f, 1.0f);
    Data.TargetParams.SetMinMaxDelta(10.0f, 230.0f, 20.0f);
}
