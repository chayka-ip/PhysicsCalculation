// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "ParabolicCacheParams_DataAsset.generated.h"

/**
 * There is no need to compute very far distances for big angles because this is not applicable for the game
 * todo: tune launch distances for different angle ranges
 */
UCLASS()
class PHYSICSCALCULATION_API UParabolicCacheParams_DataAsset : public UDataAsset
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    int LaunchAngleMin = 1.0f;

    // 80 deg is tested and computable for now
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    int LaunchAngleMax = 80.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    int LaunchAngleStep = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FVector TargetLocation = FVector(6000.0f, 0.0f, 100.0f);

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float LaunchPositionStepOffsetX = 250.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float LaunchPositionOffsetZ = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float ComputationTimeStep = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float ComputationTimeSec = 100.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float ComputationLocationTolerance = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float MultiplierChangeStep = 0.05f;

public:
    TArray<FVector> GetLaunchLocations(float OffsetZ) const;
    TArray<int> GetLaunchAngles() const;
    int GetComputationNumSteps() const {return ComputationTimeSec / ComputationTimeStep;}
};
