#pragma once

#include "CoreMinimal.h"
#include "HandyMathLibrary.h"
#include "HandyMathVectorLibrary.h"
#include "SpinCacheIterCheckData.h"
#include "Common/PhysicsPredictionCurveWithMeta.h"
#include "HMStructs/FloatMinMax.h"
#include "Kick/KickSpinRange.h"
#include "KickCompData.generated.h"


USTRUCT(BlueprintType)
struct FKickAlphaSet
{
	GENERATED_BODY()
	
	/*
	 * Selects launch angle from allowed range
	 * range: [0; 1]
	 */
	UPROPERTY(BlueprintReadWrite, meta=(UIMin="0.0", UIMax="1.0"))
	float LaunchAngleAlpha = 0.2f;

	/*
	 * positive - back spin; negative - top spin;
	 * range: [-1; 1]
	 */
	UPROPERTY(BlueprintReadWrite, meta=(UIMin="-1.0", UIMax="1.0"))
	float FrontSpinAlpha = 0.0f;

	/*
	 * positive value - curvature to the left
	 * range: [-1; 1]
	 */
	UPROPERTY(BlueprintReadWrite, meta=(UIMin="-1.0", UIMax="1.0"))
	float SideSpinAlpha = 0.0f;

public:
	void RandomizeLaunchAngleAlpha(float min=0.0f, float max=1.0f) {LaunchAngleAlpha = FMath::RandRange(min, max);}
	void RandomizeFrontSpinAlpha(float min=-1.0f, float max=1.0f) {FrontSpinAlpha = FMath::RandRange(min, max);}
	void RandomizeSideSpinAlpha(float min=-1.0f, float max=1.0f) {SideSpinAlpha = FMath::RandRange(min, max);}
	static FKickAlphaSet GetRandomSet()
	{
		FKickAlphaSet Out;
		Out.RandomizeLaunchAngleAlpha();
		Out.RandomizeFrontSpinAlpha();
		Out.RandomizeSideSpinAlpha();
		return Out;
	}

public:
	FString ToString() const
	{
		const auto LA = FString::SanitizeFloat(LaunchAngleAlpha);
		const auto FS = FString::SanitizeFloat(FrontSpinAlpha);
		const auto SS = FString::SanitizeFloat(SideSpinAlpha);
		return  "LA: " + LA + " | FS: " + FS + " | SS: " + SS;
	}
};

USTRUCT(BlueprintType)
struct FKickCompData
{
	GENERATED_BODY()

	// todo: remove soon
	// cm/sec
	UPROPERTY(BlueprintReadWrite)
	float MaxSpeed = UHM::FKmph2CMSec(220);

	UPROPERTY(BlueprintReadWrite)
	FKickAlphaSet AlphaSet;

	// todo: remove soon
	UPROPERTY(BlueprintReadWrite)
	FKickSpinRange SpinRange;

	UPROPERTY(BlueprintReadWrite)
	FSpinCacheIterCheckData CacheIterCheckData;

	UPROPERTY(BlueprintReadWrite)
	FSpecialPrediction_Limits PredictionLimits;

	UPROPERTY(BlueprintReadWrite)
	FSpecialPrediction_ExtraData PredictionExtraData;
	
	UPROPERTY(BlueprintReadWrite)
	bool bRandomizeSideSpinFlip = true;

	UPROPERTY(BlueprintReadWrite)
	int LaunchSpeedDelta_Multiplier = 12;
	
public:
	UPROPERTY(BlueprintReadWrite)
	bool bDrawResultTrajectories = false;

	UPROPERTY(BlueprintReadWrite)
	bool bRotateResultTrajectories = false;
	
	UPROPERTY(BlueprintReadWrite)
	float DrawTime = 1.0f;
	
public:
	void SetMaxSpeed(float V){MaxSpeed = V;}
	void SetAlphaSet(const FKickAlphaSet&  alpha_set){AlphaSet = alpha_set;}
	void SetLaunchSpeedDeltaMultiplier(float V){LaunchSpeedDelta_Multiplier = V;}
	void SetSpinRange(const FKickSpinRange& spin_range){SpinRange = spin_range;}
	void SetPredictionLimits(const FSpecialPrediction_Limits& limits){PredictionLimits = limits;}
	void SetPredictionExtraData(const FSpecialPrediction_ExtraData& data){PredictionExtraData = data;}
	void SetCacheIterCheckData(const FSpinCacheIterCheckData& P){CacheIterCheckData = P;}
	
public:
	int GetLaunchSpeedDeltaMultiplier() const {return LaunchSpeedDelta_Multiplier;}
	float GetLaunchAngleAlpha() const {return AlphaSet.LaunchAngleAlpha;}
	float GetDistanceToTargetXY() const {return GetDistanceToTarget(true);}
	float GetDistanceToTarget(bool bXY) const {return bXY ? GetVectorToTarget().Size2D() : GetVectorToTarget().Size();}
	float GetDirectLaunchAngle() const {return UHMV::GetAngleVectorToPlaneXY(GetVectorToTarget());}
	int GetDirectLaunchAngleRounded() const {return FMath::CeilToInt(GetDirectLaunchAngle());}
	float GetEffectiveFrontSpinAngle() const
	{
		const float Min = SpinRange.FrontSpin.GetMin();
		const float Max = SpinRange.FrontSpin.GetMax();
		return UHM::GetValueFromFloatRangeBySignedAlpha(Min, Max, AlphaSet.FrontSpinAlpha);
	}
	float GetEffectiveSideSpinAngle() const
	{
		const float Min = SpinRange.SideSpin.GetMin();
		const float Max = SpinRange.SideSpin.GetMax();
		return UHM::GetValueFromFloatRangeBySignedAlpha(Min, Max, AlphaSet.SideSpinAlpha);
	}
	FVector GetVectorToTarget() const {return CacheIterCheckData.GetVectorToTarget();}
};