// Fill out your copyright notice in the Description page of Project Settings.

#include "DataAssets/PhysicsCache_DataAsset.h"

void UPhysicsCache_DataAsset::Refresh()
{
    MakeParabolicMotionCacheObj();
    MakeImpulseDistributionCacheObj();
    MakeSpinMovementCacheObj();
}

void UPhysicsCache_DataAsset::MakeParabolicMotionCacheObj()
{
    ParabolicMotionCache = NewObject<UParabolicMotionCache>();
    ParabolicMotionCache->Initialize(ParabolicMotionCache_Data);
}

void UPhysicsCache_DataAsset::MakeSpinMovementCacheObj()
{
    BallLaunchCache = NewObject<UBallLaunchCache>();
    BallLaunchCache->Data = BallLaunchCache_Data;
}

void UPhysicsCache_DataAsset::MakeImpulseDistributionCacheObj()
{
    ImpulseDistributionCache = NewObject<UImpulseDistributionCache>();
    ImpulseDistributionCache->Data = ImpulseDistributionCache_Data;
}

void UPhysicsCache_DataAsset::Reset()
{
    
}

