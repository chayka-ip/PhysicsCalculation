#pragma once

#include "CoreMinimal.h"
#include "PhysCachedTrajectory.h"
#include "PhysCacheDataStructs.generated.h"

USTRUCT()
struct FPhysCacheTimeData
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere,  meta=(UIMin="0.0"))
	float MaxSimulationTime = 1.0f;

	UPROPERTY(EditAnywhere,  meta=(UIMin="0.0001"))
	float SimulationDeltaTime = 0.1f;

public:
	int GetMaxSimulationStepsCount() const {return FMath::CeilToInt(MaxSimulationTime / SimulationDeltaTime);}

};

/*
 * Hit is intended to be calculated along X Axis as World Forward axis
 */
USTRUCT()
struct FPhysCacheDistanceLimits
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	bool bEnabled = true;
	
	// Distance on X axis; cm
	UPROPERTY(EditAnywhere,  meta=(UIMin="0.0"))
	float MaxDistanceX  = 10000.0f;

	// +/- distance on Y Axis; cm	
	UPROPERTY(EditAnywhere,  meta=(UIMin="0.0"))
	float MaxDistanceY  = 8000.0f;

	//Positive height; cm
	UPROPERTY(EditAnywhere,  meta=(UIMin="0.0"))
	float MaxDistanceZ  = 4000.0f;

	//Negative height; Defines max offset for hit when ball is in air ; meters	
	UPROPERTY(EditAnywhere,  meta=(UIMax="0.0"))
	float MaxDistanceNegZ  = -250.0f;

public:
	bool IsLocationApplicable(const FVector& V) const
	{
		const bool B1 = FMath::Abs(V.X) <= MaxDistanceX;
		const bool B2 = FMath::Abs(V.Y) <= MaxDistanceY;
		const bool B3 = V.Z <= MaxDistanceZ && V.Z >= MaxDistanceNegZ;
		const bool B =  B1 && B2 && B3;
		return B;
	}
};

USTRUCT()
struct  FSpeedMinMaxDelta
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere,  meta=(UIMin="0.0"))
	float Min = 0.0f;
	UPROPERTY(EditAnywhere,  meta=(UIMin="0.0"))
	float Max = 0.0f;
	UPROPERTY(EditAnywhere,  meta=(UIMin="0.0001"))
	float Delta = 0.001f;

	int GetNumSegments() const {return UHM::GetNumSegmentsFromFloatRange(Min, Max, Delta);}
	float GetFloatValueOnRangeSegment(int Index) const {return UHM::GetFloatValueOnRangeSegment(Min, Max, Delta, Index);}
};

USTRUCT()
struct  FAngleMinMaxDelta
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	float Min = 0.0f;
	UPROPERTY(EditAnywhere)
	float Max = 0.0f;
	UPROPERTY(EditAnywhere,  meta=(UIMin="0.0001"))
	float Delta = 0.001f;

	int GetNumSegments() const {return UHM::GetNumSegmentsFromFloatRange(Min, Max, Delta);}
	float GetFloatValueOnRangeSegment(int Index) const {return UHM::GetFloatValueOnRangeSegment(Min, Max, Delta, Index);}
};

USTRUCT()
struct FPhysCacheStartVelocity
{
	GENERATED_BODY()

public:

	UPROPERTY(EditAnywhere)
	FSpeedMinMaxDelta Speed;

	UPROPERTY(EditAnywhere)
	FAngleMinMaxDelta AngleXY;

	UPROPERTY(EditAnywhere)
	FAngleMinMaxDelta AngleXZ;

public:
	FVector GetVectorFromIterations(int SpeedIndex, int XZAngleIndex, int XYAngleIndex, float Scale=1.0f) const;
	TArray<FVector> GetAllVectors(float Scale=1.0f) const;
	TArray<FDetailedVector> GetAllVectorsDetailed(float Scale=1.0f) const;
	static bool HasSimilarDetailedVector(const TArray<FDetailedVector>& Array, const FVector& V);
};