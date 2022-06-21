#pragma once

#include "CoreMinimal.h"
#include "HMStructs/FloatMinMaxDelta.h"
#include "HandyMathLibrary.h"
#include "SpinCacheIterCheckData.generated.h"

USTRUCT(BlueprintType)
struct PHYSICSCALCULATION_API FSpinCacheIterCheckData
{
    GENERATED_BODY()

    // raw vector not corrected to XZ plane
    UPROPERTY(BlueprintReadWrite)
    FVector VToTarget = FVector::ZeroVector;
    // KMpH
    UPROPERTY(BlueprintReadWrite)
    FFloatMinMaxDelta LaunchSpeed;
    UPROPERTY(BlueprintReadWrite)
    FFloatMinMaxDelta LaunchAngle;
    UPROPERTY(BlueprintReadWrite)
    FFloatMinMaxDelta FrontSpinAngle;
    UPROPERTY(BlueprintReadWrite)
    FFloatMinMaxDelta SideSpinAngle;

public:
    void SetVectorToTarget(const FVector& V){VToTarget = V;}
    void SetMinLaunchSpeed_KMpH(float V){LaunchSpeed.SetMin(V);}
    void SetMinLaunchSpeed_CmSec(float V)
    {
        const float kmph = UHM::FCMSec2Kmph(V);
        SetMinLaunchSpeed_KMpH(kmph);
    }
    void SetLaunchSpeedData_KMpH(float min, float max, float delta)
    {
        LaunchSpeed = FFloatMinMaxDelta(min, max, delta);
    }
    void SetLaunchAngleData(float min, float max, float delta)
    {
        LaunchAngle = FFloatMinMaxDelta(min, max, delta);
    }
    void SetFrontSpinAngleData(float min, float max, float delta)
    {
        FrontSpinAngle = FFloatMinMaxDelta(min, max, delta);
    }
    void SetSideSpinAngleData(float min, float max, float delta)
    {
        SideSpinAngle = FFloatMinMaxDelta(min, max, delta);
    }
    
public:
    /*
    void FixIterationParams()
    {
        const bool bZeroDeltaLaunchSpeed = LaunchSpeed.IsNearlyZeroDeltaMinMax();
        const bool bZeroDeltaLaunchAngle = LaunchAngle.IsNearlyZeroDeltaMinMax();
        const bool bZeroDeltaFrontSpin = FrontSpinAngle.IsNearlyZeroDeltaMinMax();
        const bool bZeroDeltaSideSpin = SideSpinAngle.IsNearlyZeroDeltaMinMax();

        if(bZeroDeltaLaunchSpeed) LaunchSpeed.AddToMax(1.0f);
        if(bZeroDeltaLaunchAngle)LaunchAngle.AddToMax(1.0f);
        if(bZeroDeltaFrontSpin) FrontSpinAngle.AddToMax(1.0f);
        if(bZeroDeltaSideSpin) SideSpinAngle.AddToMax(1.0f);
    }
    */
    
public:
    FVector GetVectorToTarget() const {return VToTarget;}

    float GetMinLaunchAngle() const {return LaunchAngle.Min;}
    float GetMaxLaunchAngle() const {return LaunchAngle.Max;}
    float GetLaunchAngleDelta() const {return LaunchAngle.Delta;}

    float GetMinFrontSpinAngle() const {return FrontSpinAngle.Min;}
    float GetMaxFrontSpinAngle() const {return FrontSpinAngle.Max;}
    float GetFrontSpinAngleDelta() const {return FrontSpinAngle.Delta;}

    float GetMinSideSpinAngle() const {return SideSpinAngle.Min;}
    float GetMaxSideSpinAngle() const {return SideSpinAngle.Max;}
    float GetSideSpinAngleDelta() const {return SideSpinAngle.Delta;}

    float GetMinLaunchSpeed_KMpH() const {return LaunchSpeed.Min;}
    float GetMaxLaunchSpeed_KMpH() const {return LaunchSpeed.Max;}
    float GetLaunchSpeedDelta_KMpH() const {return LaunchSpeed.Delta;}

};
