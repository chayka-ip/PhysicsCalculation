#pragma once

#include "CoreMinimal.h"
#include "DataFromTime.generated.h"

USTRUCT(BlueprintType)
struct FDataFromTime
{
    GENERATED_BODY()

public:
    
    UPROPERTY(BlueprintReadOnly)
    int NumStepsToSkip = -1;

    UPROPERTY(BlueprintReadOnly)
    int NumStepsToGet = -1;

    UPROPERTY(BlueprintReadOnly)
    float InitialPartMultiplier = 0.0f;

    UPROPERTY(BlueprintReadOnly)
    float FuturePartMultiplier = 0.0f;

    UPROPERTY(BlueprintReadOnly)
    int InitialStartIndex = -1;

    UPROPERTY(BlueprintReadOnly)
    int IntegerStartIndex = -1;

    UPROPERTY(BlueprintReadOnly)
    int FutureStartIndex = -1;

    UPROPERTY(BlueprintReadOnly)
    bool HasInitialPart = false;

    UPROPERTY(BlueprintReadOnly)
    bool HasIntegerPart = false;
    
    UPROPERTY(BlueprintReadOnly)
    bool HasFuturePart = false;
public:

    FDataFromTime(){}
    FDataFromTime(float DeltaTime, float SkipTime, float SimStep, float InitialTimeBeforePredict);

};