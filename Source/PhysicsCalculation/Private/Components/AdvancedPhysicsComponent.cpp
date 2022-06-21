// Fill out your copyright notice in the Description page of Project Settings.

#include "Components/AdvancedPhysicsComponent.h"
#include "Components/CustomPhysicsProcessor.h"
#include "DataAssets/ParabolicCacheParams_DataAsset.h"
#include "DataAssets/PhysicsCache_DataAsset.h"
#include "DataAssets/SpinMovementParams_DataAsset.h"

void UAdvancedPhysicsComponent::Initialize()
{
    Super::Initialize();
}

void UAdvancedPhysicsComponent::Validate() const
{
    Super::Validate();
    check(PhysicsCache)
    check(ParabolicCacheParams)
    check(SpinMovementParams)

    SpinMovementParams->Validate();
}

void UAdvancedPhysicsComponent::ComputePhysicsCache()
{
    PhysicsProcessor->RecalculatePhysicsCache(this);
}

TArray<FVector> UAdvancedPhysicsComponent::CalculateTrajectoryPoints(const FDetailedVector& LinearVelocity, const FDetailedVector& AngularVelocity)
{
    const auto T = PhysicsProcessor->CalculateCachedTrajectory(this, LinearVelocity, AngularVelocity);
    return T.GetNoCollisionTrajectoryPoints();
}
