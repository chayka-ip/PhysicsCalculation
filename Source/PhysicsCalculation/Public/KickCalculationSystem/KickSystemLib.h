// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/PhysicsPredictionCurveWithMeta.h"
#include "ImpulseDistribution/ImpulseReconstructed.h"
#include "Kick/KickCompResultTmp.h"
#include "Kick/KickSpecialPredictionCompData.h"
#include "Kick/KickSpinRange.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "PhysicsCache/BallLaunchParamsItem.h"
#include "Structs/KickCompData.h"
#include "KickSystemLib.generated.h"

class UPhysicsComponent;
class UPhysicsComponentBall;

USTRUCT(BlueprintType)
struct FKickCalculatedData
{
	GENERATED_BODY()

public:

	UPROPERTY(BlueprintReadOnly)
	FVector LinearVelocity = FVector::ZeroVector;
	
	UPROPERTY(BlueprintReadOnly)
	FVector AngularVelocity = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly)
	bool bValid = false;
	
public:
	void Init(const FVector& LV, const FVector& AV)
	{
		LinearVelocity = LV;
		AngularVelocity = AV;
		bValid = true;
	}
};

USTRUCT(BlueprintType)
struct FKickJunks
{
	GENERATED_BODY()
	
public:
	UPROPERTY(BlueprintReadOnly)
	float MaxSpeed_MinAngle = 0.0f;
	UPROPERTY(BlueprintReadOnly)
	float MaxSpeed_MaxAngle = 0.0f;
	UPROPERTY(BlueprintReadOnly)
	bool bValidMaxSpeedAngles = false;

	UPROPERTY(BlueprintReadOnly)
	bool  bSameMinMaxAngle_MaxSpeed = false;
	UPROPERTY(BlueprintReadOnly)
	float LaunchSpeed_DEG45  = 0.0f;
	UPROPERTY(BlueprintReadOnly)
	float LaunchSpeed_DirectAngle = 0.0f;
	
};

/*
 * we can use general computation method for kick; difference originated only in FKickCompData where basic settings are created
 */
UCLASS()
class PHYSICSCALCULATION_API UKickSystemLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable)
	static FImpulseReconstructed CalculateKickImpulse(UPhysicsComponent* PC, const FKickCompData& Data);
	static FKickCompResultTmp KickComputeParabolicWithSpin_prev(UPhysicsComponent* PC, const FKickCompData& Data);
	static FKickCompResultTmp KickComputePossibleImpact(UPhysicsComponent* PC, const FKickCompData& Data);
	static void DisplayComputedTrajectories(UPhysicsComponent* PC, const FKickCompData& Data, const FKickCompResultTmp& Result);

	static FKickSpecialPredictionImpulseData GetKickSpecialPredictionData(UPhysicsComponent* PC, FVector ParabolicVelocity, const FKickSpinRange& KickSpinRange, bool bFlipSideSpin);
	static void CalculateSpecialPrediction(UPhysicsComponent* PC, const FSpecialPrediction_ExtraData& ExtraData,
	                                       const FPhysCurveSpecialPrediction& PredictionData, const FPhysTransform& Impact, FKickCompResultTmp& Out);

	static FKickCompResultTmp GetKickComputedTrajectories(UPhysicsComponent* PC, const FKickSpecialPredictionImpulseData& Data,
	                                                      const FSpecialPrediction_Limits& Limits, const FSpecialPrediction_ExtraData& ExtraData);
	static FKickCompResultTmp GetKickComputedTrajectoriesFromImpactArray(UPhysicsComponent* PC,
	                                                                     const FSpecialPrediction_Limits& Limits, const FSpecialPrediction_ExtraData& ExtraData, const TArray<FPhysTransform>& TArray);
	static FKickCompResultTmp RotateImpactTransformsToKickTarget(UPhysicsComponent* PC, const FKickCompResultTmp& Data, FVector KickTarget);

	static TArray<FBallLaunchParams> MakeLaunchParamsArray(UPhysicsComponent* PC, const FSpinCacheIterCheckData& Data);
	static TArray<FPhysTransform> CalculateApplicableImpactDataCacheBased(UPhysicsComponent* PC, const TArray<FBallLaunchParams>& Data, FVector VToTarget);

	UFUNCTION(BlueprintPure)
	static FPhysTransform BallLaunchParamsToImpact(UPhysicsComponent* PC, const FBallLaunchParams& P, FVector VToTarget, bool bRemoveVelocities);
};
