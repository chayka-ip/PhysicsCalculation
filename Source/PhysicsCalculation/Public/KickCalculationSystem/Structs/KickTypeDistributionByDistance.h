#pragma once

#include "CoreMinimal.h"
#include "HandyMathLibrary.h"
#include "KickCalculationSystem/KickEnums.h"
#include "KickTypeDistributionByDistance.generated.h"

USTRUCT(BlueprintType)
struct FKickPowerDistributionFromAlpha
{
    GENERATED_BODY()

    FKickPowerDistributionFromAlpha(){}
    
    FKickPowerDistributionFromAlpha(float weak_max, float mid_max)
    {
        SetWeakMid(weak_max, mid_max);
    }

public:
    
    // equivalent of short kick
    UPROPERTY(EditAnywhere, meta=(UIMin="0.0", UIMax="1.0"))
    float WeakMax = 0.0f;

    // equivalent of mid kick
    UPROPERTY(EditAnywhere, meta=(UIMin="0.0", UIMax="1.0"))
    float MidMax = 0.0f;

public:
    void SetWeakMid(float weak_max, float mid_max)
    {
        WeakMax = weak_max;
        MidMax = mid_max;
    }
    void Validate() const
    {
        constexpr float min = 0.0f;
        constexpr float max = 1.0f;
        check(FMath::IsWithinInclusive(WeakMax, min, max));
        check(FMath::IsWithinInclusive(MidMax, min, max));
        check(WeakMax <= MidMax)
    }

public:
    EKickTypeByPower GetKickPowerTypeByAlpha(float Alpha) const
    {
        if(Alpha <= WeakMax) return EKTP_Short;
        if(Alpha <= MidMax) return EKTP_Mid;
        return EKTP_Long;
    }
};


USTRUCT(BlueprintType)
struct FKickTypeDistributionByDistance
{
    GENERATED_BODY()

public:
    FKickTypeDistributionByDistance()
    {
        ShortDistancePowerLimits.SetWeakMid(0.15f, 0.25f);
        MidDistancePowerLimits.SetWeakMid(0.0f, 0.3f);
        LongDistancePowerLimits.SetWeakMid(0.0f, 0.0f);
    }
    
    // in m
    UPROPERTY(EditAnywhere, meta=(UIMin="0.0", UIMax="50.0"))
    float ShortKickMaxDistance = 15.0f;

    // in m
    UPROPERTY(EditAnywhere, meta=(UIMin="0.0", UIMax="50.0"))
    float MidKickMaxDistance = 25.0f;

    UPROPERTY(EditAnywhere)
    FKickPowerDistributionFromAlpha ShortDistancePowerLimits;

    UPROPERTY(EditAnywhere)
    FKickPowerDistributionFromAlpha MidDistancePowerLimits;

    UPROPERTY(EditAnywhere)
    FKickPowerDistributionFromAlpha LongDistancePowerLimits;
    
public:
    void Validate() const
    {
        check(ShortKickMaxDistance < MidKickMaxDistance)
        check(ShortKickMaxDistance > 0.0f)
        ShortDistancePowerLimits.Validate();
        MidDistancePowerLimits.Validate();
        LongDistancePowerLimits.Validate();
    }
    
public:
    FKickPowerDistributionFromAlpha* GetPowerDistributionByKickType(EKickTypeByPower T)
    {
        if(T == EKTP_Short) return &ShortDistancePowerLimits;
        if(T == EKTP_Mid) return &MidDistancePowerLimits;
        if(T == EKTP_Long) return &LongDistancePowerLimits;
        return nullptr;
    }
    EKickTypeByPower GetKickPowerTypeByDistanceCM(float Distance) const
    {
        const float M = UHM::FCM2M(Distance);
        if(M <= ShortKickMaxDistance) return EKTP_Short;
        if(M <= MidKickMaxDistance) return EKTP_Mid;
        return EKTP_Long;
    }
    EKickTypeByPower CalculateKickPowerType(float DistanceToTarget, float KickAlpha)
    {
        const auto TypeByDistance = GetKickPowerTypeByDistanceCM(DistanceToTarget);
        const auto PowerLimits = GetPowerDistributionByKickType(TypeByDistance);
        return PowerLimits->GetKickPowerTypeByAlpha(KickAlpha);
    }
};