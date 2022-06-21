#pragma once

#include "CoreMinimal.h"
#include "HandyMathRotatorLibrary.h"
#include "Libs/MathUtils.h"
#include "PhysTransform.generated.h"

USTRUCT(BlueprintType)
struct FPhysTransform
{
    GENERATED_USTRUCT_BODY()

    FPhysTransform()
    {
        Location = FVector::ZeroVector;
        LinearVelocity = FVector::ZeroVector;
        AngularVelocity = FVector::ZeroVector;
        Orientation = FQuat::Identity;
    }
    
    FPhysTransform(const FVector InLocation, const FQuat InOrientation, const FVector LinearVel, const FVector AngularVel)
    {
        Location = InLocation;
        Orientation = InOrientation;
        LinearVelocity = LinearVel;
        AngularVelocity = AngularVel;
    }

    FPhysTransform(const FTransform &Transform, const FVector LinearVel=FVector::ZeroVector, const FVector AngularVel=FVector::ZeroVector)
    {
        Location = Transform.GetLocation();
        Orientation = Transform.GetRotation();
        LinearVelocity = LinearVel;
        AngularVelocity = AngularVel;
    }

public:
    
    UPROPERTY(BlueprintReadWrite)
    FVector Location = FVector::ZeroVector;;

    UPROPERTY(BlueprintReadWrite)
    FVector LinearVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadWrite)
    FVector AngularVelocity = FVector::ZeroVector;

    UPROPERTY(BlueprintReadWrite)
    FQuat Orientation = FQuat::Identity; // obtained from FRotator->Quaternion()

public:
    void ScaleLinearVelocity(float V){LinearVelocity *= V;}
    void ScaleAngularVelocity(float V){AngularVelocity *= V;}
    void RotateLinearVelocity(const FQuat& Q)
    {
        LinearVelocity = Q.RotateVector(LinearVelocity);
    }
    void RotateAngularVelocity(const FQuat& Q)
    {
        AngularVelocity = Q.RotateVector(AngularVelocity);
    }

public:
    
    FTransform ToTransform(const FVector &Scale) const
    {
        return FTransform(Orientation, Location, Scale);
    }

    FORCEINLINE FPhysTransform operator-(const FPhysTransform& T) const
    {
        return FPhysTransform(Location - T.Location, UMathUtils::GetDeltaRotation(Orientation, T.Orientation),
            LinearVelocity - T.LinearVelocity, AngularVelocity - T.AngularVelocity);
    }
    
    FORCEINLINE bool operator==(const FPhysTransform& T) const
    {
        const bool bL = Location.Equals(T.Location);
        const bool bV = LinearVelocity.Equals(T.LinearVelocity);
        const bool bA = AngularVelocity.Equals(T.AngularVelocity);
        const bool bQ = Orientation.Equals(T.Orientation); 
        return bL && bV && bA && bQ;
    }

    bool EqualsNoOrientCheck(const FPhysTransform& T, float Tolerance) const
    {
        const bool bL = Location.Equals(T.Location, Tolerance);
        const bool bV = LinearVelocity.Equals(T.LinearVelocity, Tolerance);
        const bool bA = AngularVelocity.Equals(T.AngularVelocity, Tolerance);
        return bL && bV && bA;
    }
    
    bool Equals(const FPhysTransform& T, float Tolerance) const
    {
        const bool bE = EqualsNoOrientCheck(T, Tolerance);
        const bool bQ = Orientation.Equals(T.Orientation); 
        return bE && bQ;
    }

    FString ToString(bool bL=true, bool bV=true, bool bA=false, bool bQ=false) const
    {
        FString L = " L: " + Location.ToString();
        FString V = " V: " + LinearVelocity.ToString();
        FString A = " A: " + AngularVelocity.ToString();
        FString Q = " Q: " + UHMR::QuatToString(Orientation);

        FString Res = "";
        if(bL) Res += L;
        if(bV) Res += V;
        if(bA) Res += A;
        if(bQ) Res += Q;
        return Res;
    }

    bool ContainsNAN() const
    {
        const bool B1 = Location.ContainsNaN();
        const bool B2 = LinearVelocity.ContainsNaN();
        const bool B3 = AngularVelocity.ContainsNaN();
        const bool B4 = Orientation.ContainsNaN();
        return B1 || B2 || B3 || B4;
    }
};