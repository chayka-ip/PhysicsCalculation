#pragma once

#include "CoreMinimal.h"
#include "PhysTransform.h"
#include "HMStructs/CustomVectorCurve.h"
#include "TrajectoryData.generated.h"

USTRUCT(BlueprintType)
struct FTrajectoryData
{
    GENERATED_BODY()
    
    UPROPERTY(BlueprintReadOnly)
    FPhysTransform TOrigin;
    
    UPROPERTY(BlueprintReadOnly)
    FCustomVectorCurve Trajectory;
    
    UPROPERTY(BlueprintReadOnly)
    FVector Target = FVector::ZeroVector;
    
    UPROPERTY(BlueprintReadOnly)
    FRuntimeFloatCurve DistanceToTarget;

public:
    void Init(const FPhysTransform& T, const FCustomVectorCurve& trajectory, FVector target)
    {
        SetTransform(T);
        SetTarget(target);
        SetTrajectory(trajectory, true);
    }
    
public:
    void SetTarget(FVector V){Target = V;}
    void SetTransform(const FPhysTransform& T) { TOrigin = T;}
    void SetTrajectory(const FCustomVectorCurve& T, bool bCalculateDistance)
    {
        Trajectory = T;
        if(bCalculateDistance)
        {
            DistanceToTarget = Trajectory.GetDistanceToVector(Target);
        }
    }

    float GetMinDistance() const {return UHMC::GetFloatCurveMinValue(DistanceToTarget);}
    float GetTimeOfMinDistance() const
    {
        float Time;
        UHMC::GetCurveNthTimeWhenValueEquals(DistanceToTarget, GetMinDistance(), Time);
        return Time;
    }

    FVector GetVectorOfMinDistance() const
    {
        const float Time = GetTimeOfMinDistance();
        return Trajectory.GetVectorValue(Time);
    }
    TArray<FVector> GetTrajectoryPoints() const {return Trajectory.GetVectorKeys();}
    
public:

    void RotateByQuat(const FQuat& QLinear, const FQuat& QAngular)
    {
        Trajectory.RotateByQuat(QLinear);
        TOrigin.RotateLinearVelocity(QLinear);
        TOrigin.RotateAngularVelocity(QAngular);
        DistanceToTarget = Trajectory.GetDistanceToVector(Target);
    }

    void RotateByAngleXY(float AngleDeg)
    {
        const FQuat Q = UHMR::QuatZAxisAngleDeg(AngleDeg);
        RotateByQuat(Q, Q);
    }
};