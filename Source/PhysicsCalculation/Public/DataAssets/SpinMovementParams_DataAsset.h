// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "HMStructs/FloatMinMaxDelta.h"
#include "PhysicsCache/BallLaunchParamsItem.h"
#include "SpinMovementParams_DataAsset.generated.h"

USTRUCT(BlueprintType)
struct PHYSICSCALCULATION_API FSMValue : public FFloatMinMaxDelta
{
    GENERATED_BODY()

public:
    void Validate() const
    {
        check(GetMin() <= GetMax())
        check(GetDelta() > 0.0f)
    }
};

USTRUCT(BlueprintType)
struct PHYSICSCALCULATION_API FSpinMovementParams
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=SimulationParams)
    float SimulationStep = 0.015f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=SimulationParams)
    float SimulationTimeSec = 20.0f;
    
    // Speed in KMpH;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Params)
    FSMValue LaunchSpeed;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Params)
    FSMValue LaunchAngle;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Params)
    FSMValue FrontSpinAngle;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Params)
    FSMValue SideSpinAngle;

    // min-max is vertical range of target; delta is change step for iteration
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=TargetParams)
    FSMValue TargetParams;
    
    // launch speed kmph is key
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Curves)
    UCurveFloat* MaxLaunchAngleCurve = nullptr;

    // launch speed kmph is key
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Curves)
    UCurveFloat* MaxTopSpinCurve = nullptr;

    // launch speed kmph is key
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Curves)
    UCurveFloat* MaxBackSpinCurve = nullptr;

    // launch speed kmph is key
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Curves)
    UCurveFloat* MaxSideSpinCurve = nullptr;
    
public:
    void Validate() const;

    TArray<float> GetLaunchSpeedCmSec() const;
    float GetMaxLaunchSpeedCmSec() const;
    int GetMinLaunchSpeedKMpH() const{return LaunchSpeed.GetMin();}
    int GetMaxLaunchSpeedKMpH() const{return LaunchSpeed.GetMax();}
    int GetLaunchSpeedDelta_KMpH() const {return LaunchSpeed.GetDelta();}
    int GetMinLaunchAngle() const {return LaunchAngle.GetMin();}
    int GetMaxLaunchAngle() const {return LaunchAngle.GetMax();}
    int GetLaunchAngleDelta() const {return LaunchAngle.GetDelta();}
    int GetFrontSpinAngleDelta() const {return FrontSpinAngle.GetDelta();}
    int GetSideSpinAngleDelta() const {return SideSpinAngle.GetDelta();}
    int GetLaunchSpeedSnappedToGrid_KMpH(float speed_kmph) const;

    bool CanUseLaunchAngle(float launch_speed, float Angle) const;
    bool CanUseFrontSpin(float launch_speed, float Spin) const;
    bool CanUseSideSpin(float launch_speed, float Spin) const;
    bool CanUseLaunchParams(const FBallLaunchParams& P) const;

    int GetComputationNumSteps() const {return SimulationTimeSec / SimulationStep;}

public:
    float GetVerticalLevelWidth() const {return TargetParams.GetDelta();}
    TArray<float> GetTargetVerticalDistanceValues() const;
    TArray<FFloatMinMax> GetTargetVerticalLevels() const;
};

/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API USpinMovementParams_DataAsset : public UDataAsset
{
	GENERATED_BODY()

    USpinMovementParams_DataAsset();

public:
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FSpinMovementParams Data;

public:
    void Validate() const {Data.Validate();}

};
