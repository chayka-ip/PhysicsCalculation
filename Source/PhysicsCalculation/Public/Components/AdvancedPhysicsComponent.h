// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/CustomPhysicsComponent.h"
#include "DataAssets/PhysicsCacheParams_DataAsset.h"
#include "AdvancedPhysicsComponent.generated.h"

class USpinMovementParams_DataAsset;
class UParabolicCacheParams_DataAsset;
class UPhysicsCache_DataAsset;
class UPhysicsCacheParams_DataAsset;
/**
 * 
 */
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class PHYSICSCALCULATION_API UAdvancedPhysicsComponent : public UCustomPhysicsComponent
{
	GENERATED_BODY()

public:

    UPROPERTY(EditAnywhere)
    bool bComputePhysicsCacheOnStart = false;
    UPROPERTY(EditAnywhere)
    bool bRecalculateCache = false;
    // UPROPERTY(EditAnywhere)
    bool bCalculateTrajectoryCache = false;
    UPROPERTY(EditAnywhere)
    bool bCalculateParabolicMotionCache = false;
    UPROPERTY(EditAnywhere)
    bool bCalculateImpulseDistributionCache = false;
    UPROPERTY(EditAnywhere)
    bool bCalculateSpinCache = false;
    
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UPhysicsCacheParams_DataAsset* PhysicsCacheParams = nullptr;
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UPhysicsCache_DataAsset* PhysicsCache = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UParabolicCacheParams_DataAsset* ParabolicCacheParams = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    USpinMovementParams_DataAsset* SpinMovementParams = nullptr;

    
public:
    virtual void Initialize() override;

protected:
    virtual void Validate() const override;

public:
    bool ShouldRecalculateCacheOnInit() const {return bComputePhysicsCacheOnStart && bRecalculateCache;}
    UFUNCTION(BlueprintCallable)
    void ComputePhysicsCache();

    UFUNCTION(BlueprintCallable)
    TArray<FVector> CalculateTrajectoryPoints(const FDetailedVector& LinearVelocity, const FDetailedVector& AngularVelocity);

public:
    UFUNCTION(BlueprintPure)
    int GetVelocityEstimatedCount(){return PhysicsCacheParams->StartLinearVelocity.GetAllVectors().Num();}
    
};
