#pragma once

#include "CoreMinimal.h"
#include "HMStructs/CustomVectorCurve.h"
#include "TrajectoryStepComparison.generated.h"

USTRUCT()
struct FTrajectoryStepComparison
{
    GENERATED_BODY()

    float X_BaseWhenZeroZ = 0.0f;
    float X_OneStepWhenZeroZ = 0.0f;

    // applicable to use +/- 1 step of multiplier change 
    float StepDeltaZ = 0.0f;

    FCustomVectorCurve CurveBase;
    FCustomVectorCurve CurveFirstIter;
    
public:
    float GetBaseXWhenZeroZ() const {return X_BaseWhenZeroZ;}
    float GetStepDeltaX() const {return FMath::Abs(X_OneStepWhenZeroZ - X_BaseWhenZeroZ);}
    float GetStepDeltaZ() const {return FMath::Abs(StepDeltaZ);}
};