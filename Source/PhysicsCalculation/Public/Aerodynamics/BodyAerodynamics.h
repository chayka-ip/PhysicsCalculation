#pragma once

#include "CoreMinimal.h"
#include "Sideforce.h"
#include "Aerodynamics/AirDrag.h"

#include "BodyAerodynamics.generated.h"

USTRUCT(BlueprintType)
struct FOptionalMultiplier
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bEnabled;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Value = 1.0f;
};

USTRUCT(BlueprintType)
struct FBodyAerodynamics
{
    GENERATED_BODY()

public:

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FOptionalMultiplier MagnusImpact;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FOptionalMultiplier AirDragImpact;
        
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FAirDrag AirDrag;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FSideforce Sideforce;

    void Init()
    {
        Sideforce.Init();
    }
};