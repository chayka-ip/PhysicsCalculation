#pragma once

#include "CoreMinimal.h"
#include "ParabolicMotionCurve.generated.h"

USTRUCT()
struct FParabolicMotionCurve
{
    GENERATED_BODY()

    UPROPERTY()
    float LaunchAngle = 0.0f;
    
    UPROPERTY()
    FRuntimeFloatCurve MultiplierCurve;

    UPROPERTY()
    FRuntimeFloatCurve MaxDistanceCurve;

public:
    float GetMultiplierForDistance(float x) const
    {
        return MultiplierCurve.GetRichCurveConst()->Eval(x, 1.0f);
    }
    float GetMaxDistanceForLaunchSpeed(float Speed)
    {
        return MaxDistanceCurve.GetRichCurve()->Eval(Speed, 0.0f);
    }
};
