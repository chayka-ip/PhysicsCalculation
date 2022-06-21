#pragma once

#include "CoreMinimal.h"

#include "PhysConstrains.generated.h"

USTRUCT(BlueprintType)
struct FClampLimit
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere)
    bool bClamp = false;

    UPROPERTY(EditAnywhere)
    float Value = 0.0f;
};

USTRUCT(BlueprintType)
struct FVelocityClamp
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere)
    FClampLimit Min;

    UPROPERTY(EditAnywhere)
    FClampLimit Max;

    void ClampVelocity(FVector& V) const
    {
        V = GetVelocityClamped(V);
    }
    
    FVector GetVelocityClamped(const FVector &V) const
    {
        if(V.IsNearlyZero()) return FVector::ZeroVector;
        const float size = V.Size();
        const float min = Min.bClamp ? Min.Value : 0.0f;
        const float max = Max.bClamp ? Max.Value : size;

        if(Min.bClamp && size <= min) return FVector::ZeroVector;
        if(Max.bClamp && size > max) return max * V.GetSafeNormal();
        return V;
    }
};

USTRUCT(BlueprintType)
struct FPhysConstrains
{
    GENERATED_BODY()

    FPhysConstrains()
    {
        AngularVelocity.Min.bClamp = true;
        AngularVelocity.Min.Value = 0.5f;
    }
public:

    UPROPERTY(EditAnywhere)
    FVelocityClamp LinearVelocity;

    UPROPERTY(EditAnywhere)
    FVelocityClamp AngularVelocity;
    
    void ClampVelocity(FVector &Linear, FVector Angular) const
    {
        LinearVelocity.ClampVelocity(Linear);
        AngularVelocity.ClampVelocity(Angular);
    }
};

USTRUCT(BlueprintType)
struct FPhysRendering
{
    GENERATED_BODY()

    FPhysRendering()
    {
        MaxAngularVelocity.bClamp =true;
        MaxAngularVelocity.Value = 15.0f;
    }
    
public:
    UPROPERTY(EditAnywhere)
    FClampLimit MaxAngularVelocity;

    FVector GetAngularVelocityClamped(const FVector& V) const
    {
        return V.GetClampedToSize(0.0f, MaxAngularVelocity.Value);
    }
};