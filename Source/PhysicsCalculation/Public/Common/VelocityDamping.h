#pragma once

#include "CoreMinimal.h"
#include "VelocityDamping.generated.h"

USTRUCT(BlueprintType)
struct FVelocityDamping
{
    GENERATED_BODY()
	
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bEnabled = true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Value = 0.0f;
};