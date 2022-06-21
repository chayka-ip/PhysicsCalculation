#pragma once

#include "CoreMinimal.h"
#include "PhysPredictSettings.generated.h"

USTRUCT(BlueprintType)
struct FPhysPredictSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, meta = (ClampMin = "0.1", ClampMax = "1.5", UIMin = "0.1", UIMax = "1.5"))
    float PrecisePredictTimeSec = 1.0f; 

    UPROPERTY(EditAnywhere, meta = (ClampMin = "0.5", ClampMax = "10.0", UIMin = "0.5", UIMax = "10.0"))
    float RoughPredictTimeSec = 8.0;

    UPROPERTY(EditAnywhere, meta = (ClampMin = "1.0", ClampMax = "10.0", UIMin = "1.0", UIMax = "10.0"))
    float RoughPredictionTimeScale = 2.0f;

    UPROPERTY(EditAnywhere, meta = (ClampMin = "1.0", ClampMax = "10.0", UIMin = "1.0", UIMax = "10.0"))
    float RoughPredictUpdateRate = 2.0f;

    //DEPRECATED (remove if old phys iteration will be removed) or make revision
    /*
     * Allowed difference between predicted transform and transform being set during movement in prediction mode.
     * Basically this parameter is used only to prevent recomputation when object is rest on the ground.
     */
    // UPROPERTY(EditAnywhere)
    float TransformTolerance = 15.0f;
    // UPROPERTY(EditAnywhere)
    float DefaultTolerance = 5.0f;
    // UPROPERTY(EditAnywhere)
    float GroundVelocityThreshold = 200.0f;
};