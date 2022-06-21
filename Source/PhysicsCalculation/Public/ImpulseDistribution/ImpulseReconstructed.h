#pragma once

#include "CoreMinimal.h"
#include "ImpulseReconstructed.generated.h"

USTRUCT(BlueprintType)
struct FImpulseReconstructed
{
    GENERATED_BODY()

    FImpulseReconstructed(){}
    FImpulseReconstructed(const FVector& impulse, const FVector& apply_location)
    {
        Impulse = impulse;    
        ApplyLocation = apply_location;
    }

public:
    
    UPROPERTY(BlueprintReadWrite)
    FVector Impulse = FVector::ZeroVector;
    
    UPROPERTY(BlueprintReadWrite)
    FVector ApplyLocation = FVector::ZeroVector;

};