#include "Components/PhysicsComponent.h"
#include "DataAssets/ParabolicCacheParams_DataAsset.h"
#include "DataAssets/PhysicsCache_DataAsset.h"
#include "HMStructs/ValidatedInt.h"


float UPhysicsComponent::GetKickPredictTimeStep() const
{
    // return GetRoughPredictSimStep();
    return ParabolicCacheParams->ComputationTimeStep;
}

FTrajectoryCompData UPhysicsComponent::GetTrajectoryCompData(FVector TargetLocation, int NumSimSteps, float DesiredDistanceToTarget, float InterruptDistanceAddition) const
{
    FTrajectoryCompData Data;

    const FVector DirectVectorToTarget = TargetLocation - GetCurrentLocation();
    const float InterruptDistance = DirectVectorToTarget.Size() + InterruptDistanceAddition;
    
    Data.SimStep = GetKickPredictTimeStep();
    Data.TargetLocation = TargetLocation;
    Data.NumSimSteps = NumSimSteps;
    Data.DesiredDistanceToTarget = DesiredDistanceToTarget;
    Data.InterruptDistance = InterruptDistance;
    Data.InterruptLocationMinZ = 0.0f;
    
    return Data;
}

bool UPhysicsComponent::CalculateParabolicMinMaxLaunchAngle(float MaxSpeed, float AngleMin, float AngleMax, FVector VToTarget, FFloatMinMax& Out) const
{
    const int MinAngle = FMath::CeilToInt(AngleMin);
    const int MaxAngle = FMath::CeilToInt(AngleMax);

    const FVector V = UHMV::ConvertVectorToPlaneXZ(VToTarget);
    const float x = V.X;
    const float z = V.Z;

    constexpr int invalid_angle = -100;
    FValidatedIntMinMax ResultData;
    ResultData.SetInvalidValue(invalid_angle);
    
    for (int angle = MinAngle; angle <= MaxAngle; ++angle)
    {
        const float Speed = PhysicsCache->ParabolicMotionCache->GetRequiredLaunchSpeedForAngle(angle, x, z);
        const bool bSpeedOk = Speed <= MaxSpeed;

        if(bSpeedOk)
        {
            ResultData.IsInvalidMin() ? ResultData.SetMin(angle) : ResultData.SetMax(angle);
        }

        const bool B1 = !bSpeedOk && ResultData.AreValidBoth();
        const bool CanInterrupt = B1;
        
        if(CanInterrupt) break;
    }

    if (ResultData.IsValidMinOnly())
    {
        ResultData.SetMax(ResultData.GetMin());
    }

    if(ResultData.AreValidBoth())
    {
        Out.Min = ResultData.GetMin();
        Out.Max = ResultData.GetMax();
        return true;
    }
    
    return  false;
}

float UPhysicsComponent::GetRealRequiredParabolicSpeedForAngle(float LaunchAngle, FVector VToTarget) const
{
    const auto ParabolicCache = PhysicsCache->ParabolicMotionCache;
    VToTarget = UHMV::ConvertVectorToPlaneXZ(VToTarget);
    const float x = VToTarget.X;
    const float z = VToTarget.Z;
    return  ParabolicCache->GetRequiredLaunchSpeedForAngle(LaunchAngle, x, z);
}

