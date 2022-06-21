// Fill out your copyright notice in the Description page of Project Settings.

#include "Components/CustomPhysicsProcessor.h"
#include "Components/AdvancedPhysicsComponent.h"
#include "DataAssets/PhysicsCache_DataAsset.h"
#include "ImpulseDistribution/ImpulseDistributionLib.h"
#include "Libs/PhysicsCacheLib_old.h"
#include "Libs/SpinMovementLib.h"
#include "ParabolicMotion/ParabolicMotionToRealLib.h"

void UCustomPhysicsProcessor::OnSecondTick()
{
    // return;
    for (const auto Obj : Objects)
    {
        if(const auto T = Cast<UAdvancedPhysicsComponent>(Obj))
        {
            if(T->ShouldRecalculateCacheOnInit()) RecalculatePhysicsCache(T);
            T->PhysicsCache->Refresh();
        }
    }
}

void UCustomPhysicsProcessor::RecalculatePhysicsCache(UAdvancedPhysicsComponent* T)
{
    if(T->bCalculateTrajectoryCache) CalculatePhysicsCacheTrajectories(T);
    if(T->bCalculateParabolicMotionCache) RecalculateParabolicMotionCache(T);
    if(T->bCalculateImpulseDistributionCache) RecalculateImpulseDistributionCache(T);
    if(T->bCalculateSpinCache) RecalculateSpinMovementCache(T);
}

TArray<UCustomPhysicsBaseComponent*> UCustomPhysicsProcessor::GetObjectsEnabledForCacheComputations(const TArray<UCustomPhysicsBaseComponent*>& ExcludedObjects)
{
    TArray<UCustomPhysicsBaseComponent*> Out;
    for (auto Obj : Objects)
    {
        const bool B1 = !ExcludedObjects.Contains(Obj);
        const bool B2 = Obj->IsEnabledForPhysicsCaching();
        if(B1 && B2) Out.Add(Obj);
    }
    return Out;
}

void UCustomPhysicsProcessor::CalculatePhysicsCacheTrajectories(UAdvancedPhysicsComponent* Target)
{
    if(!IsInitialized() && false) return;
    
    const auto ExcludedTarget = TArray<UCustomPhysicsBaseComponent*>{Cast<UCustomPhysicsBaseComponent>(Target)};
    const auto EnvObjects = GetObjectsEnabledForCacheComputations(ExcludedTarget);

    const auto Cache = Target->PhysicsCache;
    Cache->Reset();
    
    UPhysicsCacheLib_old::CalculatePhysicsCache(Target, EnvObjects);
    UPhysicsCacheLib_old::CleanUpTrajectoryData(Target);
    
    Cache->Save();
}

void UCustomPhysicsProcessor::RecalculateParabolicMotionCache(UAdvancedPhysicsComponent* Target)
{
    const auto Cache = Target->PhysicsCache;
    Cache->ParabolicMotionCache_Data = UParabolicMotionToRealLib::ComputeParabolicMultipliersData(Target);
    Cache->Save();
}

void UCustomPhysicsProcessor::RecalculateImpulseDistributionCache(UAdvancedPhysicsComponent* Target)
{
    const auto Cache = Target->PhysicsCache;
    Cache->ImpulseDistributionCache_Data = UImpulseDistributionLib::CalculateImpulseDistribution(Target);
    Cache->Save();
}

void UCustomPhysicsProcessor::RecalculateSpinMovementCache(UAdvancedPhysicsComponent* Target)
{
    const auto Cache = Target->PhysicsCache;
    Cache->BallLaunchCache_Data = USpinMovementLib::CalculateBallLaunchCache(Target);
    Cache->Save();
}

FPhysCachedTrajectoryFull UCustomPhysicsProcessor::CalculateCachedTrajectory(UAdvancedPhysicsComponent* Target, const FDetailedVector& LinearVelocity,
                                                                             const FDetailedVector& AngularVelocity)
{
    FPhysCachedTrajectoryFull Trajectory;
    const auto T = UPhysicsCacheLib_old::GetStartTransformForCalculation(Target->GetRadius(), LinearVelocity.GetV(), AngularVelocity.GetV());
    UPhysicsCacheLib_old::CalculateCachedTrajectory(Target, {}, T, Trajectory);
    return Trajectory;
}



