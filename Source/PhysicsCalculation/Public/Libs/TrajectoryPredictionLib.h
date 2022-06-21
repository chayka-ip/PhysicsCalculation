// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/TrajectoryData.h"
#include "Components/CustomPhysicsComponent.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "TrajectoryPredictionLib.generated.h"

USTRUCT()
struct FTrajectoryDataCompData
{
    GENERATED_BODY()

    float SimStep = 0.1f;
    int NumSteps = 0;

    FVector Impulse = FVector::ZeroVector;
    FVector ApplyLocation = FVector::ZeroVector;

    FVector Target = FVector::ZeroVector;
    float ApplicableDistanceToTarget = 25.0f;
    float TrajectoryFarCheckDeclineAngleOffset = 15.0f;
    float VelocityComparisonMultiplier = 1.05f;

    float FrontSpinDecreaseMul = 0.95f;
    float SideSpinDecreaseMul = 0.98f;

    bool  bLimitSpinVector = true;
    float FrontSpinLimitRPM = 800.0f;
    float SideSpinLimitRPM = 1100.0f;

    float LimitMinZ = -150.0f;
    
    bool bRemoveQuatXY = false;
    bool bRemoveQuatXZ = false;

    bool bDebug = false;

public:
    // use same value because XZ correction is computed independently
    float GetCorrectionLimitXZ() const {return  ApplicableDistanceToTarget;}
    float GetCorrectionLimitXZ_Individual() const {return  FMath::Sqrt(ApplicableDistanceToTarget * 0.5f);}
    float GetVelocityComparisonMultiplier() const {return VelocityComparisonMultiplier;}
    float GetVelocityComparisonMultiplierStep() const {return GetVelocityComparisonMultiplier() - 1.0f;}
    float GetFrontSpinDecreaseMul() const {return FrontSpinDecreaseMul;}
    float GetSideSpinDecreaseMul() const {return SideSpinDecreaseMul;}
    float GetFrontSpinLimitRadSec() const {return UHM::FRpm2RadSec(FrontSpinLimitRPM);}
    float GetSideSpinLimitRadSec() const {return UHM::FRpm2RadSec(SideSpinLimitRPM);}

public:
    void Validate() const
    {
        constexpr float Zero = 0.0f;
        constexpr float One = 1.0f;

        check(NumSteps > Zero)
        check(ApplicableDistanceToTarget > Zero)
        check(FrontSpinLimitRPM > Zero)
        check(SideSpinLimitRPM > Zero)
        
        check(GetVelocityComparisonMultiplier() > One);
        check(GetFrontSpinDecreaseMul() < One); 
        check(GetSideSpinDecreaseMul() < One); 
    }
};


/**
 * 
 */
class UAdvancedPhysicsComponent;
UCLASS()
class PHYSICSCALCULATION_API UTrajectoryPredictionLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
    static FTrajectoryData CalculateCorrectedTrajectoryDataFromImpulse(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData);
    static FTrajectoryData CalculateTrajectoryDataFromImpulse(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData);
    static FTrajectoryData GetCorrectedTrajectoryData(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData, const FTrajectoryData& PrevData);
    static bool CanTrajectoryReachTarget(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData, const FTrajectoryData& PrevData, FVector& OutClosestPoint, float&
                                         OutDistanceDeltaXY);
    static FTrajectoryData GetCorrectedTrajectoryFromReachable(const FTrajectoryDataCompData& CompData, const FTrajectoryData& PrevData, FVector ClosestPoint);
    static FTrajectoryData GetCorrectedTrajectoryFromUnreachable(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData, const FTrajectoryData& PrevData, float DistanceDeltaXY);
    static FVector GetTrajectoryFarPointInBaseDirection(FVector Target, FVector Origin, FVector ClosestPoint, const FTrajectoryData& Data, float AngleOffset);
    static float ComputeMultiplierChangeValue(float DistanceDelta, float StepDelta, float MulIncreaseStep);
    static float GetTrajectoryDeltaX(UAdvancedPhysicsComponent* Obj, const FTrajectoryData& BaseData, FVector Target, FVector Origin, float TimeStep, int NumSteps, float VelocityMul);
    static float GetTrajectoryValidDeltaX(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData, const FTrajectoryData& PrevData, FTrajectoryData& OutData);
    static FCustomVectorCurve ComputeTrajectoryFromTransform(UCustomPhysicsComponent* Obj, const FPhysTransform& T, float TimeStep, int NumSteps);

    static FVector LimitSpinVector(const FVector& V, float FrontSpinLimit, float SideSpinLimit);
    static FVector ScaleSpinVector(const FVector& V, float FrontSpinMul, float SideSpinMul);
    static void ScaleTransformAngularVelocity(FPhysTransform& T, float FrontSpinMul, float SideSpinMul);

    static bool IsTopSpin(float FrontSpinAngle);
};

