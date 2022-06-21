#pragma once

#include "CoreMinimal.h"
#include "KickParabolicComp.generated.h"

USTRUCT()
struct FKickParabolicComp
{
    GENERATED_BODY()

    float MinLaunchAngle = 0.0f;
    float MaxLaunchAngle = 0.0f;
    float MinLaunchSpeedCmSec = 0.0f;
};
