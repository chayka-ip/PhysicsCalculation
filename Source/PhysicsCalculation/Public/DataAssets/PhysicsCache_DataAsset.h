// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "ImpulseDistribution/ImpulseDistributionCache.h"
#include "InGameSavable/IngameSavable_Interface.h"
#include "ParabolicMotion/ParabolicMotionCache.h"
#include "PhysicsCache/BallLaunchCache.h"
#include "PhysicsCache_DataAsset.generated.h"

/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UPhysicsCache_DataAsset : public UDataAsset, public IIngameSavable_Interface
{
    GENERATED_BODY()
public:

    UPROPERTY()
    FParabolicMotionCache_Data ParabolicMotionCache_Data;

    UPROPERTY(BlueprintReadOnly)
    UParabolicMotionCache* ParabolicMotionCache = nullptr;

    UPROPERTY()
    FImpulseDistributionCache_Data ImpulseDistributionCache_Data;

    UPROPERTY(BlueprintReadOnly)
    UImpulseDistributionCache* ImpulseDistributionCache = nullptr;

    UPROPERTY()
    FBallLaunchCache_Data BallLaunchCache_Data;

    UPROPERTY(BlueprintReadOnly)
    UBallLaunchCache* BallLaunchCache = nullptr;
    
public:
    void Refresh();
    void MakeImpulseDistributionCacheObj();
    void MakeParabolicMotionCacheObj();
    void MakeSpinMovementCacheObj();
    
public:
    virtual UObject* GetSelf() override {return this;}
    void Reset();
};
