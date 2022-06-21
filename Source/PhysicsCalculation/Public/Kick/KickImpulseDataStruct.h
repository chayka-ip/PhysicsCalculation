#pragma once

#include "CoreMinimal.h"
#include "KickImpulseDataStruct.generated.h"

USTRUCT()
struct FKickImpulseDataStruct
{
    GENERATED_BODY()

    FVector Impulse = FVector::ZeroVector;
    FVector ApplyLocation = FVector::ZeroVector;
    // used to reconstruct spin values 
    FQuat QImpulse = FQuat::Identity;
    // applied before impulse to shift body position (mostly to detach from ground)
    FVector InitialBodyLocationOffset = FVector::ZeroVector;
};