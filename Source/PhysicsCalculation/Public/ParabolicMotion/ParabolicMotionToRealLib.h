// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ParabolicMotionCurve.h"
#include "Common/PhysTransform.h"
#include "Common/TrajectoryStepComparison.h"
#include "HMStructs/CustomVectorCurve.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "ParabolicMotionToRealLib.generated.h"

class UParabolicMotionCache;
class UAdvancedPhysicsComponent;

struct FParabolicMotionCache_Data;
/**
 * Purpose is to obtain coefficient that connects parabolic motion for specified angle with real trajectory
 */

USTRUCT(BlueprintType)
struct FPMCompData
{
    GENERATED_BODY()
    
public:
    UPROPERTY(BlueprintReadWrite)
    FVector TargetLocation = FVector::ZeroVector;
    UPROPERTY(BlueprintReadWrite)
    FVector LaunchLocation = FVector::ZeroVector;
    UPROPERTY(BlueprintReadWrite)
    float LaunchAngle = 0.0f;
    UPROPERTY(BlueprintReadWrite)
    float TimeStep = 0.05f;
    UPROPERTY(BlueprintReadWrite)
    int NumSteps = 100;
    UPROPERTY(BlueprintReadWrite)
    float LocationTolerance = 1.0f;
    UPROPERTY(BlueprintReadWrite)
    float MultiplierChangeStep = 0.05f;
    
public:
    FVector GetVectorToTargetRelative() const {return TargetLocation - LaunchLocation;}
    
public:
    void ChangeLaunchLocation(FVector V){LaunchLocation = V;}
    
    void Init(FVector target_location, FVector launch_location, float launch_angle, float time_step, int num_steps, float location_tolerance, float mul_change_step)
    {
        TargetLocation = target_location;
        LaunchLocation = launch_location;
        LaunchAngle = launch_angle;
        TimeStep = time_step;
        NumSteps = num_steps;
        LocationTolerance = location_tolerance;
        MultiplierChangeStep = mul_change_step;
    }
};

UCLASS()
class PHYSICSCALCULATION_API UParabolicMotionToRealLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
    static float CalculateDirectAngle(FVector TargetLocation, FVector LaunchLocation);
    
    UFUNCTION(BlueprintCallable)
    static UParabolicMotionCache* ComputeParabolicMultipliers(UAdvancedPhysicsComponent* Obj);
    static FParabolicMotionCache_Data ComputeParabolicMultipliersData(UAdvancedPhysicsComponent* Obj);
    
    static FParabolicMotionCurve GetParabolicMultiplierCurveForLaunchAngle(UAdvancedPhysicsComponent* Obj, FVector TargetLocation, float LaunchAngle,
                                                                        const TArray<FVector>& LaunchLocations);

    static FPMCompData CreateParabolicCompParams(UAdvancedPhysicsComponent* Obj, FVector TargetLocation, FVector LaunchLocation, float LaunchAngle); 

    UFUNCTION(BlueprintCallable)
    static float GetParabolicMultiplier(UAdvancedPhysicsComponent* Obj, const FPMCompData& CompData, float& LaunchSpeed, float& MaxDistance);
    static float GetParabolicMultiplier_Body(UAdvancedPhysicsComponent* Obj, const FPMCompData& CompData, const FPhysTransform& T_Base, float BaseMultiplier = 1.0f);
    static float ComputeMultiplierChangeValue(float DistanceDelta, float StepDelta, float MulIncreaseStep);

    static FTrajectoryStepComparison GetComparisonBaseToOneStep(UAdvancedPhysicsComponent* Obj, const FPhysTransform& T_Base, float TimeStep, int NumSteps, float BaseMultiplier, float MultiplierStep);
    static void DrawCurveTrajectory(UObject* Obj, FCustomVectorCurve Curve,  FLinearColor Color);

    UFUNCTION(BlueprintCallable)
    static int GetLaunchAngleMaxDistance(UAdvancedPhysicsComponent* Obj, float LaunchSpeed, float TimeStep, int NumSteps, int MinAngle, int MaxAngle);

    static FCustomVectorCurve CalculateMotionCurveFromTransform(UAdvancedPhysicsComponent* Obj, const FPhysTransform& T, float TimeStep, int NumSteps);
    static FCustomVectorCurve CalculateParabolicMotionCurve(UAdvancedPhysicsComponent* Obj, const FVector& LinearVelocity, const FVector& LaunchLocation, float TimeStep, int NumSteps);
    
    UFUNCTION(BlueprintCallable)
    static FRuntimeFloatCurve CalculateMaxLaunchAngleCurve(UAdvancedPhysicsComponent* Obj, int MinLaunchAngle, int MaxLaunchAngle);

    //don't use this anymore
    // UFUNCTION(BlueprintCallable)
    static void CalculateSpinTrajectories(UAdvancedPhysicsComponent* Obj, float LaunchSpeed, float LaunchAngle, float LaunchAngleXY, float FrontSpinAngle, float SideSpinAngle);

    // UFUNCTION(BlueprintCallable)
    static void TestTrajectoryCalcTime(UAdvancedPhysicsComponent* Obj, float LaunchSpeed, float LaunchAngle, float MaxFrontSpinAngle, float MaxSideSpinAngle, int NumSpeedIterations, int
                                       NumAngleIterations, TArray<FVector>& Out);

    static  bool GetTrajectoryGroundLocation(const FCustomVectorCurve& Trajectory, FVector& OutLocation, bool bCheckForTrue);
};
