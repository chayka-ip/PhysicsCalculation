#pragma once

#include "CoreMinimal.h"
#include "HandyMathLibrary.h"
#include "Kick/KickSpinRange.h"
#include "KickTypeSettings.generated.h"

USTRUCT(BlueprintType)
struct FKickTypeSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FKickSpinRange SpinRange;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    int LaunchSpeedDeltaMultiplier = 1;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float BaseAngleMultiplier = 1.0f;
    
    // distance xy (in M) to target mapped to z coordinate offset from target z (in cm)
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UCurveFloat* VerticalDistanceTolerance = nullptr;

    float DefaultVerticalDistanceTolerance = 50.0f;
    
public:
    float GetVerticalDistanceTolerance(float DistanceXY_cm) const
    {
        const float DistanceM = UHM::FCM2M(DistanceXY_cm);
        const auto Curve = VerticalDistanceTolerance;
        return Curve ? Curve->GetFloatValue(DistanceM) : DefaultVerticalDistanceTolerance;
    }
};
