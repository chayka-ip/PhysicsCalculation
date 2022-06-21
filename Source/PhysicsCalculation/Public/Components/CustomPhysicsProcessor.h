// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/CustomPhysicsProcessorBase.h"
#include "PhysicsCache/PhysCachedTrajectory.h"
#include "CustomPhysicsProcessor.generated.h"

struct FDetailedVector;
/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UCustomPhysicsProcessor : public UCustomPhysicsProcessorBase
{
	GENERATED_BODY()

protected:
    virtual void OnSecondTick() override;
    TArray<UCustomPhysicsBaseComponent*> GetObjectsEnabledForCacheComputations(const TArray<UCustomPhysicsBaseComponent*>& ExcludedObjects);
        
public:
    void RecalculatePhysicsCache(UAdvancedPhysicsComponent* T);
    void CalculatePhysicsCacheTrajectories(UAdvancedPhysicsComponent* Target);
    static void RecalculateParabolicMotionCache(UAdvancedPhysicsComponent* Target);
    static void RecalculateImpulseDistributionCache(UAdvancedPhysicsComponent* Target);
    static void RecalculateSpinMovementCache(UAdvancedPhysicsComponent* Target);

    FPhysCachedTrajectoryFull CalculateCachedTrajectory(UAdvancedPhysicsComponent* Target, const FDetailedVector& LinearVelocity,
                                                        const FDetailedVector& AngularVelocity);
};
