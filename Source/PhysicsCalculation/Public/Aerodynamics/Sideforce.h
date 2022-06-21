#pragma once

#include "CoreMinimal.h"
#include "ConstantsHM.h"
#include "Common/PhysTransform.h"
#include "Curves/CurveVector.h"
#include "debug.h"
#include "SideforceGenerator.h"

#include "Sideforce.generated.h"

/*
 * Produces force to change object path;
 * We want to make it predicable during amount of time
 *
 * This is not the actual side force from real physics (in real world side force is basically Lift (or Magnus) force)
 * Main purpose of this structure - simulate knuckling effect when ball moving fast with small rotation amount
 */

USTRUCT(BlueprintType)
struct FSideforce
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    bool bEnabled;

    /*
     * Used to calculate total impact coefficient in range [0, 1]
     * X - Linear velocity magnitude (km/h)
     * Y - Angular velocity magnitude (rate/min)
     *
     * Resulting impact produced by multiplication of both values
     */
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UCurveVector* ActivationCurve;
    
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FLocationOffsetGenerator LocationOffsetGenerator;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FAngularVelocityGenerator AngularVelocityGenerator;
    
public:
    void Init()
    {
        LocationOffsetGenerator.Init();
        AngularVelocityGenerator.Init();
    }
    
    void Update(float DeltaTime)
    {
        LocationOffsetGenerator.Update(DeltaTime);
        AngularVelocityGenerator.Update(DeltaTime);
    }

    float GetLinearVelocityContributionToAlpha(float VelocityKmPerH) const
    {
        return ActivationCurve ? ActivationCurve->GetVectorValue(VelocityKmPerH).X : 0.0f;
    }

    float GetAngularVelocityContributionToAlpha(float VelocityRatePerMin) const
    {
        return  ActivationCurve ? ActivationCurve->GetVectorValue(VelocityRatePerMin).Y : 0.0f;
    }

    /*
     * Linear velocity: cm/s
     * Angular velocity: rad/s
     */
    float GetImpactAlphaFromVelocity(FVector LinearVelocity, FVector AngularVelocity) const
    {
        if(ActivationCurve)
        {
            const float LV = LinearVelocity.Size() * CmSecToKmHour;
            const float AV = AngularVelocity.Size() * RadSecToRpM;

            const float MLV = GetLinearVelocityContributionToAlpha(LV);
            const float MAV = GetAngularVelocityContributionToAlpha(AV);

            return FMath::Clamp(MLV * MAV, 0.0f, 1.0f);
        }
        
        PrintToLog("Location Offset Generator: Activation Curve is not set!");
        return 1.f;
    }
    
    void GetOffsetsToApply(const FPhysTransform &T, float DeltaTime, float SkipTime, FVector& LocationOffset, FVector& AngularVelocity)
    {
        const FVector F = T.Orientation.GetForwardVector();
        const FVector R = T.Orientation.GetRightVector();
        const FVector U = T.Orientation.GetUpVector();
        const float VelocityImpact = GetImpactAlphaFromVelocity(T.LinearVelocity, T.AngularVelocity);
        
        LocationOffset = LocationOffsetGenerator.GetVectorToApply(F, R, U, DeltaTime, SkipTime, VelocityImpact);
        AngularVelocity = AngularVelocityGenerator.GetVectorToApply(F, R, U, DeltaTime, SkipTime, VelocityImpact);
    }
};