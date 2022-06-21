#pragma once

#include "CoreMinimal.h"
#include "PhysicsPredictionCurve.h"
#include "PhysicsPredictionCurveWithMeta.generated.h"

class UCustomPhysicsComponent;

USTRUCT(BlueprintType)
struct FFloatLimitValue
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bLimit = false;
    UPROPERTY(BlueprintReadWrite)
    float Value = 0.0f;

public:
    float GetValue() const {return  Value;}
    bool IsLimit() const {return bLimit;}
    void Disable(){bLimit = false;}
    void Enable(float value)
    {
        bLimit = true;
        Value = value;
    }
};

USTRUCT(BlueprintType)
struct FSpecialPrediction_Limits
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadWrite)
    FFloatLimitValue Limit_MinZ;
    UPROPERTY(BlueprintReadWrite)
    FFloatLimitValue Limit_MaxZ;
    UPROPERTY(BlueprintReadWrite)
    FFloatLimitValue Limit_DistanceFromStartXY;

public:
    bool HasLimitZ() const {return Limit_MinZ.IsLimit() || Limit_MaxZ.IsLimit();}
    void EnableCoordinateLimits(float distance_from_start_xy, float limit_z_max, float limit_z_min=0.0f)
    {
        Limit_MinZ.Enable(limit_z_min);
        Limit_MaxZ.Enable(limit_z_max);
        Limit_DistanceFromStartXY.Enable(distance_from_start_xy);
    }
    void DisableCoordinateLimits()
    {
        Limit_MinZ.Disable();
        Limit_MaxZ.Disable();
        Limit_DistanceFromStartXY.Disable();
    }
};

USTRUCT(BlueprintType)
struct FSpecialPrediction_ExtraData
{
    GENERATED_BODY()
    
    UPROPERTY(BlueprintReadWrite)
    bool bStoreLocationOnDistance = false;

    UPROPERTY(BlueprintReadWrite)
    float DistanceToStoreZ = 0.0f;

    UPROPERTY(BlueprintReadWrite)
    float AllowedDeviationZ = 0.0f;

    UPROPERTY(BlueprintReadWrite)
    float TargetZ = 0.0f;

    UPROPERTY(BlueprintReadWrite)
    bool bApplyAllResults = false;

    UPROPERTY(BlueprintReadWrite)
    bool bStoreLocationVectors = false;

public:
    void EnableStoreLocationVectors(){bStoreLocationVectors = true;}
    void DisableStoreLocationVectors(){bStoreLocationVectors = false;}
    
public:
    float GetDistanceToStoreZ() const {return DistanceToStoreZ;}
    float GetAllowedDeviationZ() const {return AllowedDeviationZ;}
    float GetMinZ() const {return TargetZ - AllowedDeviationZ;}
    float GetMaxZ() const {return TargetZ + AllowedDeviationZ;}
    bool IsStoringLocationOnDistance() const {return bStoreLocationOnDistance;}
    void DisableLocationStorageOnDistance(){bStoreLocationOnDistance = false;}
    void EnableLocationStorageOnDistance(float distance,  float target_z, float allowed_deviation)
    {
        bStoreLocationOnDistance = true;
        DistanceToStoreZ = distance > 0.0f ? distance : 0.0f;
        TargetZ = target_z;
        SetAllowedDerivationZ(allowed_deviation);
    }
    void SetAllowedDerivationZ(float V){AllowedDeviationZ = FMath::Abs(V);}
};

USTRUCT(BlueprintType)
struct FPhysCurveSpecialPrediction
{
    GENERATED_BODY()
    
    UPROPERTY(BlueprintReadWrite)
    FPhysTransform T;

    UPROPERTY(BlueprintReadWrite)
    float DeltaTime = 0.000001f;
    UPROPERTY(BlueprintReadWrite)
    int NumMaxIterations = 5000;

    UPROPERTY(BlueprintReadWrite)
    FSpecialPrediction_Limits Limits;

    UPROPERTY(BlueprintReadWrite)
    FSpecialPrediction_ExtraData ExtraData;
    
public:
    void SetInitialTransform(const FPhysTransform& Transform){T= Transform;}
    FPhysTransform GetInitialTransform() const {return T;}
    
    void SetDeltaTime(float Time){DeltaTime = Time;}
    float GetDeltaTime() const {return DeltaTime;}

    void SetMaxIterations(int N){NumMaxIterations = N;}
    int GetMaxIterations() const {return NumMaxIterations;}

};

USTRUCT()
struct FPhysicsPredictionCurveWithMeta
{
    GENERATED_BODY()

    // filled if requested
    UPROPERTY()
    TArray<FVector> Locations = {};
    
public:
    bool bVectorCached = false;
    FVector VCached = FVector::ZeroVector;

public:
    bool IsVectorCached() const {return bVectorCached;}
    float GetCachedVectorZ() const {return VCached.Z;}
    FVector GetCachedVector() const {return VCached;}
    void SetLocationCached(FVector v)
    {
        bVectorCached = true;
        VCached = v;
    }

    void AddLocationToArray(const FVector& V){Locations.Add(V);}
};
