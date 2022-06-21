#pragma once

#include "CoreMinimal.h"
#include "AdvancedPhysicsComponent.h"
#include "Common/TracjectoryCompData.h"
#include "HMStructs/FloatMinMax.h"
#include "PhysicsComponent.generated.h"

UCLASS()
class PHYSICSCALCULATION_API UPhysicsComponent : public UAdvancedPhysicsComponent
{
    GENERATED_BODY()

public:
    virtual float GetKickPredictTimeStep() const;

    UFUNCTION(BlueprintPure)
    FTrajectoryCompData GetTrajectoryCompData(FVector TargetLocation, int NumSimSteps, float DesiredDistanceToTarget, float InterruptDistanceAddition) const;

public:
    bool CalculateParabolicMinMaxLaunchAngle(float MaxSpeed, float AngleMin, float AngleMax, FVector VToTarget, FFloatMinMax& Out) const;
    float GetRealRequiredParabolicSpeedForAngle(float LaunchAngle, FVector VToTarget) const;
};


