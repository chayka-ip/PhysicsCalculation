#pragma once

#include "CoreMinimal.h"
#include "TracjectoryCompData.generated.h"

USTRUCT(BlueprintType)
struct FTrajectoryCompData
{
    GENERATED_BODY()

public:
    UPROPERTY(BlueprintReadWrite)
    float SimStep = 0.0001f;
    UPROPERTY(BlueprintReadWrite)
    int NumSimSteps = 0;

    UPROPERTY(BlueprintReadWrite)
    FVector TargetLocation = FVector::ZeroVector;
    
    // Radius around target where point treated as applicable
    UPROPERTY(BlueprintReadWrite)
    float DesiredDistanceToTarget = 0.0f;

    // terminates computation

    UPROPERTY(BlueprintReadWrite)
    float InterruptDistance = 0.0f;

    UPROPERTY(BlueprintReadWrite)
    float InterruptLocationMinZ = 0.0f;
    
};
