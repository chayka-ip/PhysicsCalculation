#pragma once

#include "CoreMinimal.h"
#include "PhysTransform.h"
#include "HMStructs/CustomVectorCurve.h"
#include "HMStructs/QuaternionCurve.h"
#include "PhysicsPredictionCurve.generated.h"

USTRUCT()
struct FPhysicsPredictionCurve
{
    GENERATED_BODY()

public:

    FCustomVectorCurve LocationCurve;
    FCustomVectorCurve LinearVelocityCurve;
    FCustomVectorCurve AngularVelocityCurve;
    FQuaternionCurve   OrientationCurve;

public:
    void Reset()
    {
        LocationCurve.Reset();
        LinearVelocityCurve.Reset();
        AngularVelocityCurve.Reset();
        OrientationCurve.Reset();
    }
    void UpdateOrAddKey(float Time, const FPhysTransform &T)
    {
        LocationCurve.UpdateOrAddKey(Time, T.Location);
        LinearVelocityCurve.UpdateOrAddKey(Time, T.LinearVelocity);
        AngularVelocityCurve.UpdateOrAddKey(Time, T.AngularVelocity);
        OrientationCurve.UpdateOrAddKey(Time, T.Orientation);
    }
    void InsertKeyToEnd(float Time, const FPhysTransform &T)
    {
        LocationCurve.InsertKeyToEnd(Time, T.Location);
        LinearVelocityCurve.InsertKeyToEnd(Time, T.LinearVelocity);
        AngularVelocityCurve.InsertKeyToEnd(Time, T.AngularVelocity);
        OrientationCurve.InsertKeyToEnd(Time, T.Orientation);
    }
    FPhysTransform GetTransformValue(float Time) const
    {
        const FVector Location = LocationCurve.GetVectorValue(Time);
        const FVector LinearVelocity = LinearVelocityCurve.GetVectorValue(Time);
        const FVector AngularVelocity = AngularVelocityCurve.GetVectorValue(Time);
        const FQuat Orientation = OrientationCurve.GetQuatValue(Time);
        return FPhysTransform(Location, Orientation, LinearVelocity, AngularVelocity);
    }

    FPhysTransform LastKey() const
    {
        const FVector Location = LocationCurve.LastKey();
        const FVector LinearVelocity = LinearVelocityCurve.LastKey();
        const FVector AngularVelocity = AngularVelocityCurve.LastKey();
        const FQuat Orientation = OrientationCurve.LastKey();
        return FPhysTransform(Location, Orientation, LinearVelocity, AngularVelocity);
    }
    
    TArray<FVector> GetAllLocations() const {return LocationCurve.GetVectorKeys();}
public:
    int GetNumKeys() const {return LocationCurve.GetNumKeys();}
    bool HasKeys() const {return LocationCurve.HasKeys();}
    
};
